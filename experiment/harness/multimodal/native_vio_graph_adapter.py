#!/usr/bin/env python3
"""Convert actual Kimera-VIO body pose graphs to camera pose graphs.

This is a coordinate adapter, not an estimator. It reads only native
pose_graph_incremental, rejects every caller other than --expected-caller,
and preserves the native keys, timestamps, robot IDs, edge types and frame
names. No SDK/LiDAR odometry or simulator ground truth is subscribed.

The installed schema was read using rosmsg show -r on 2026-11-02:
  PoseGraph: Header header; PoseGraphNode[] nodes; PoseGraphEdge[] edges
  Node: Header header; int32 robot_id; uint64 key; geometry_msgs/Pose pose
  Edge: Header header; uint64 key_from/key_to; int32 robot_from/robot_to/type;
        geometry_msgs/Pose pose; float64[36] covariance

T_BC maps camera coordinates into body coordinates. Nodes: T_WC=T_WB*T_BC;
edges: T_CiCj=inv(T_BC)*T_BiBj*T_BC. Covariance uses a right perturbation
and explicit ordering (default ROS translation then rotation):
Sigma_camera=Ad(inv(T_BC))*Sigma_body*Ad(inv(T_BC)).T. The pinned native
visualizer leaves covariance all zero (TODO); zero remains zero/unknown.
With this simulation's zero lever arm, left/right perturbation conventions
give the same block rotation for edges. Nonzero lever arms require that the
chosen perturbation convention matches the producer.

A single native key-zero bootstrap graph precedes the original first graph
to preserve its actual key-zero timestamp for this pinned Multi-LCD code.
Input is buffered until an output subscriber is connected. Buffer overflow,
unexpected source, or a missing native key zero fails closed, without drops.
"""
import argparse
from collections import deque
import copy
import json
import math
import threading
import time

import numpy as np


SIM_T_BODY_CAMERA = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                              [0., -1., 0., 0.], [0., 0., 0., 1.]])


def validate_transform(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('Calibration must be a finite 4x4 T_body_camera')
    if not np.allclose(value[3], [0., 0., 0., 1.], atol=1e-12):
        raise ValueError('Calibration has an invalid homogeneous last row')
    rotation = value[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or
            abs(np.linalg.det(rotation)-1.) > 1e-8):
        raise ValueError('Calibration rotation must belong to SO(3)')
    return value.copy()


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def pose_matrix(pose):
    quaternion = pose.orientation
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(position).all() or not np.isfinite(q).all() or abs(norm-1.) > 1e-3:
        raise ValueError('Native graph contains a nonfinite pose or nonunit quaternion')
    x, y, z, w = q/norm
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result = np.eye(4); result[:3, :3] = rotation; result[:3, 3] = position
    return result


def set_pose(pose, transform):
    rotation, position = transform[:3, :3], transform[:3, 3]
    # Largest-component extraction remains stable near 180-degree rotations.
    candidates = np.array([1+rotation[0, 0]-rotation[1, 1]-rotation[2, 2],
        1-rotation[0, 0]+rotation[1, 1]-rotation[2, 2],
        1-rotation[0, 0]-rotation[1, 1]+rotation[2, 2], 1+np.trace(rotation)])
    index = int(np.argmax(candidates)); q = np.zeros(4)
    q[index] = .5*math.sqrt(max(0., candidates[index])); divisor = 4*q[index]
    if divisor <= 1e-12:
        raise ValueError('Cannot extract a quaternion from rotation')
    if index == 3:
        q[:3] = [rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0], rotation[1, 0]-rotation[0, 1]]
        q[:3] /= divisor
    elif index == 0:
        q[1:] = [(rotation[0, 1]+rotation[1, 0])/divisor,
                 (rotation[0, 2]+rotation[2, 0])/divisor,
                 (rotation[2, 1]-rotation[1, 2])/divisor]
    elif index == 1:
        q[0] = (rotation[0, 1]+rotation[1, 0])/divisor
        q[2] = (rotation[1, 2]+rotation[2, 1])/divisor
        q[3] = (rotation[0, 2]-rotation[2, 0])/divisor
    else:
        q[0] = (rotation[0, 2]+rotation[2, 0])/divisor
        q[1] = (rotation[1, 2]+rotation[2, 1])/divisor
        q[3] = (rotation[1, 0]-rotation[0, 1])/divisor
    q /= np.linalg.norm(q)
    if q[3] < 0: q = -q
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


def adjoint(transform, order='translation_rotation'):
    rotation, translation = transform[:3, :3], transform[:3, 3]
    result = np.zeros((6, 6)); result[:3, :3] = rotation; result[3:, 3:] = rotation
    result[:3, 3:] = skew(translation) @ rotation
    if order == 'rotation_translation':
        permutation = [3, 4, 5, 0, 1, 2]
        result = result[np.ix_(permutation, permutation)]
    elif order != 'translation_rotation':
        raise ValueError('Unsupported covariance order')
    return result


def rotate_covariance(values, jacobian):
    values = np.asarray(values, dtype=float)
    if values.shape != (36,) or not np.isfinite(values).all():
        raise ValueError('Edge covariance must have 36 finite elements')
    covariance = values.reshape(6, 6)
    if not np.allclose(covariance, covariance.T, atol=1e-10, rtol=1e-8):
        raise ValueError('Native edge covariance is not symmetric')
    if np.linalg.eigvalsh(covariance).min() < -1e-10:
        raise ValueError('Native edge covariance is not positive semidefinite')
    result = jacobian @ covariance @ jacobian.T
    return ((result+result.T)/2).reshape(-1).tolist()


class GraphConverter:
    def __init__(self, robot_id, body_camera=SIM_T_BODY_CAMERA,
                 covariance_order='translation_rotation'):
        self.robot_id = int(robot_id)
        self.body_camera = validate_transform(body_camera)
        self.camera_body = np.linalg.inv(self.body_camera)
        self.jacobian = adjoint(self.camera_body, covariance_order)
        self.covariance_order = covariance_order

    def convert(self, message):
        output = copy.deepcopy(message)
        for node in output.nodes:
            if int(node.robot_id) != self.robot_id:
                raise ValueError('Native local graph contains another robot node')
            set_pose(node.pose, pose_matrix(node.pose) @ self.body_camera)
        for edge in output.edges:
            if int(edge.robot_from) != self.robot_id or int(edge.robot_to) != self.robot_id:
                raise ValueError('Native local graph contains another robot edge')
            if int(edge.type) not in (0, 1, 3):
                raise ValueError('Only ODOM, LOOPCLOSE and REJECTED_LOOPCLOSE SE(3) edges are supported')
            set_pose(edge.pose, self.camera_body @ pose_matrix(edge.pose) @ self.body_camera)
            edge.covariance = rotate_covariance(edge.covariance, self.jacobian)
        return output

    def bootstrap(self, converted):
        nodes = [node for node in converted.nodes if int(node.robot_id) == self.robot_id and int(node.key) == 0]
        if len(nodes) != 1:
            raise ValueError('First native graph must contain exactly one actual key-zero node; start adapter before native graph')
        node = nodes[0]
        if node.header.stamp.to_nsec() <= 0:
            raise ValueError('Native key-zero stamp must be nonzero')
        bootstrap = copy.deepcopy(converted)
        bootstrap.nodes = [copy.deepcopy(node)]; bootstrap.edges = []
        bootstrap.header.stamp = copy.deepcopy(node.header.stamp)
        return bootstrap


class OrderedOutput:
    """A bounded lossless queue; bootstrap and original graph remain ordered."""
    def __init__(self, converter, expected_caller, max_pending=512, bootstrap_spacing=.05):
        self.converter, self.expected_caller = converter, expected_caller
        self.max_pending, self.bootstrap_spacing = max_pending, bootstrap_spacing
        self.pending = deque(); self.lock = threading.RLock()
        self.received = self.published = self.bootstrap_count = self.zero_covariances = 0
        self.node_zero_stamp_ns = None; self.last_input_stamp_ns = None
        self.last_publish_wall = None; self.last_input_wall = None
        self.first_caller = None; self.first_native_graph_stamp_ns = None
        self.bootstrap_prepared = False; self.bootstrap_published_wall = None

    def ingest(self, message, caller, now=None):
        if caller != self.expected_caller:
            raise ValueError('Rejected graph caller %r; expected native VIO caller %r' % (caller, self.expected_caller))
        converted = self.converter.convert(message)
        with self.lock:
            addition = []
            if not self.bootstrap_prepared:
                bootstrap = self.converter.bootstrap(converted)
                addition.append(('bootstrap', bootstrap))
            addition.append(('native_graph', converted))
            if len(self.pending)+len(addition) > self.max_pending:
                raise RuntimeError('Lossless native graph buffer overflow; no graph was dropped')
            if not self.bootstrap_prepared:
                self.node_zero_stamp_ns = addition[0][1].header.stamp.to_nsec()
                self.first_native_graph_stamp_ns = message.header.stamp.to_nsec()
                self.first_caller = caller; self.bootstrap_prepared = True
            self.pending.extend(addition); self.received += 1
            self.last_input_stamp_ns = message.header.stamp.to_nsec()
            self.last_input_wall = time.time() if now is None else now
            self.zero_covariances += sum(not any(edge.covariance) for edge in message.edges)

    def drain_one(self, connected, publish, now=None):
        now = time.time() if now is None else now
        with self.lock:
            if not connected or not self.pending: return False
            if self.bootstrap_published_wall is not None and now-self.bootstrap_published_wall < self.bootstrap_spacing:
                return False
            kind, message = self.pending[0]
            publish(message)  # Remove only after the publisher accepts it.
            self.pending.popleft(); self.published += 1; self.last_publish_wall = now
            if kind == 'bootstrap':
                self.bootstrap_count += 1; self.bootstrap_published_wall = now
            return True

    def diagnostics(self):
        with self.lock:
            return {'native_graphs_received': self.received, 'graphs_published': self.published,
                'pending_graphs': len(self.pending), 'bootstrap_count': self.bootstrap_count,
                'native_key_zero_stamp_ns': self.node_zero_stamp_ns,
                'first_native_graph_header_stamp_ns': self.first_native_graph_stamp_ns,
                'last_native_graph_header_stamp_ns': self.last_input_stamp_ns,
                'last_input_receive_wall': self.last_input_wall, 'last_output_publish_wall': self.last_publish_wall,
                'actual_first_input_caller': self.first_caller,
                'zero_unknown_covariance_edges_preserved': self.zero_covariances}


def runtime_schema(kind):
    return {'type': kind._type, 'md5sum': kind._md5sum,
            'slots': list(kind.__slots__), 'slot_types': list(kind._slot_types)}


def main():
    import rospy
    from pose_graph_tools_msgs.msg import PoseGraph, PoseGraphNode, PoseGraphEdge
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--expected-caller', required=True)
    parser.add_argument('--input-topic')
    parser.add_argument('--output-topic')
    parser.add_argument('--calibration-json', help='JSON array: calibrated 4x4 T_body_camera; default is explicit simulation calibration')
    parser.add_argument('--covariance-order', choices=('translation_rotation', 'rotation_translation'), default='translation_rotation')
    parser.add_argument('--max-pending', type=int, default=512)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.max_pending < 4: parser.error('--max-pending must be at least 4')
    if not args.expected_caller.startswith('/') or any(word in args.expected_caller.lower() for word in ('sdk', 'lidar', 'lio_')):
        parser.error('--expected-caller must explicitly identify the native VIO node, not SDK/LiDAR')
    calibration = SIM_T_BODY_CAMERA if args.calibration_json is None else json.loads(args.calibration_json)
    converter = GraphConverter(args.robot_id, calibration, args.covariance_order)
    prefix = '/sim_%d/' % args.robot_id
    input_topic = args.input_topic or prefix+'kimera_vio_ros/pose_graph_incremental'
    output_topic = args.output_topic or prefix+'kimera_camera_adapter/pose_graph_incremental'
    if input_topic == output_topic: parser.error('Input and output topics must differ')
    rospy.init_node('sim_%d_native_vio_camera_graph_adapter' % args.robot_id, disable_signals=False)
    output = rospy.Publisher(output_topic, PoseGraph, queue_size=args.max_pending, latch=False)
    diagnostics_output = rospy.Publisher(prefix+'kimera_camera_adapter/diagnostics', String, queue_size=1, latch=True)
    sequence = OrderedOutput(converter, args.expected_caller, args.max_pending)
    fatal = {'error': None}; schema = {'graph': runtime_schema(PoseGraph), 'node': runtime_schema(PoseGraphNode), 'edge': runtime_schema(PoseGraphEdge)}

    def fail(error):
        fatal['error'] = str(error)
        rospy.logfatal('Native VIO camera graph adapter: %s', error)
        publish_diagnostics(None)
        rospy.signal_shutdown(str(error))

    def receive(message):
        try:
            caller = (getattr(message, '_connection_header', None) or {}).get('callerid')
            sequence.ingest(message, caller)
        except Exception as error:
            fail(error)

    def drain(_):
        try:
            sequence.drain_one(output.get_num_connections() > 0, output.publish)
        except Exception as error:
            fail(error)

    def publish_diagnostics(_):
        report = {'adapter_role': 'coordinate_transform_of_actual_native_Kimera_VIO_graph',
            'estimator': False, 'uses_simulator_ground_truth': False,
            'uses_sdk_or_lidar_odometry': False, 'input_topic': input_topic, 'output_topic': output_topic,
            'expected_input_caller': args.expected_caller, 'output_caller': rospy.get_name(),
            'T_body_camera': converter.body_camera.tolist(),
            'calibration_source': 'explicit_simulation_sensor_extrinsic' if args.calibration_json is None else 'explicit_calibration_argument',
            'covariance_order': args.covariance_order, 'covariance_perturbation': 'right',
            'unknown_zero_covariance_is_not_confidence': True,
            'schema': schema, 'output_subscribers': output.get_num_connections(),
            'bootstrap_delivery': 'single native key-zero followed by ordered TCPROS graph publication; no application-level acknowledgment',
            'fatal_error': fatal['error'], **sequence.diagnostics()}
        diagnostics_output.publish(String(data=json.dumps(report, allow_nan=False)))

    subscriber = rospy.Subscriber(input_topic, PoseGraph, receive, queue_size=args.max_pending, buff_size=2**24, tcp_nodelay=True)
    drain_timer = rospy.Timer(rospy.Duration(.01), drain)
    diagnostics_timer = rospy.Timer(rospy.Duration(1.), publish_diagnostics)
    publish_diagnostics(None)
    rospy.loginfo('Native body→camera adapter ready; source=%s; waiting for native graph and output subscriber', args.expected_caller)
    rospy.spin()
    subscriber.unregister(); drain_timer.shutdown(); diagnostics_timer.shutdown()
    if fatal['error']: raise SystemExit(2)


if __name__ == '__main__':
    main()
