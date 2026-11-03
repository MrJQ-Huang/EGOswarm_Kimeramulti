#!/usr/bin/env python3
"""Thin ROS adapters for the pinned native Voxblox -> PGMO geometry chain.

Geometry production is native tsdf_server, voxblox_mesh_frontend_node and PGMO
7a67674b677468e6c28152596c9aae66cd24ee4b. This file only converts calibrated RGB-D
to optical point messages, interpolates native VIO for TF, preserves native graph
keys/measurements, converts optimized camera poses back to body poses, and exposes
native mesh vertices for display. It does not reconstruct or deform a map.

Start before VIO and sensors in a fresh epoch. Local mode intentionally does not
consume distributed results. Joint mode withholds PGMO full meshes until an
observed native completed DPGO round supplies a complete original-key trajectory.
The pinned native messages have no shared round ID: the recorded causal window
is an observational binding, not an invented atomic optimizer/mesh receipt.
"""
import argparse
import bisect
from collections import Counter, deque
import copy
import hashlib
import io
import json
from pathlib import Path
import threading
import time

import numpy as np

from native_vio_graph_adapter import SIM_T_BODY_CAMERA, pose_matrix, set_pose, rotate_covariance
from native_imu_pose_interpolation import GyroCoverageError, GyroRotationHistory


def require_source(message, expected):
    caller = (getattr(message, '_connection_header', None) or {}).get('callerid')
    if caller != expected:
        raise ValueError('Rejected caller %r; expected %r' % (caller, expected))


def require_stamp(stamp, cutoff_ns):
    value = int(stamp.to_nsec())
    if value < cutoff_ns or value <= 0:
        raise ValueError('Zero or previous-epoch sensor/keyframe timestamp')
    return value


def frame(value):
    return str(value).lstrip('/')


def message_bytes(message):
    stream = io.BytesIO(); message.serialize(stream); return stream.getvalue()


def digest(message):
    return hashlib.sha256(message_bytes(message)).hexdigest()


def publish_preserving_cached_message(publisher, message):
    # rospy/msg.py serialize_message mutates only the top-level Header's seq
    # (and defaults a None frame_id). Native geometry/pose arrays serialize
    # read-only. Own the envelope/header without walking every mesh object.
    publisher.publish(snapshot_message_header(message))


def snapshot_message_header(message):
    output = copy.copy(message)
    if hasattr(message, 'header'): output.header = copy.copy(message.header)
    return output


def canonical_mesh_digest(message):
    """Full native mesh content hash, ignoring only ROS transport Header.seq."""
    normalized = snapshot_message_header(message)
    normalized.header.seq = 0
    return digest(normalized)


def path_geometry_digest(message):
    # DPGO recreates ROS header stamps on every cached-path retransmission.
    # Hash only numerical poses to avoid treating that timer echo as an update.
    values = [pose_matrix(stamped.pose).reshape(-1).tolist() for stamped in message.poses]
    if not values: raise ValueError('Empty native optimized path')
    return hashlib.sha256(json.dumps(values, separators=(',', ':')).encode()).hexdigest()


def image_array(message):
    encodings = {'rgb8': (np.dtype('u1'), 3), 'bgr8': (np.dtype('u1'), 3),
                 '32FC1': (np.dtype('>f4' if message.is_bigendian else '<f4'), 1)}
    if message.encoding not in encodings:
        raise ValueError('Unsupported calibrated camera encoding: '+message.encoding)
    dtype, channels = encodings[message.encoding]
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width*channels*dtype.itemsize:
        raise ValueError('Invalid image dimensions/stride')
    if len(message.data) < step*height:
        raise ValueError('Truncated image buffer')
    shape = (height, width, channels) if channels > 1 else (height, width)
    strides = (step, channels*dtype.itemsize, dtype.itemsize) if channels > 1 else (step, dtype.itemsize)
    result = np.ndarray(shape=shape, dtype=dtype, buffer=message.data, strides=strides)
    return result[..., ::-1] if message.encoding == 'bgr8' else result


def optical_points(rgb, depth, calibration, stride, minimum, maximum):
    """A pinhole sensor-message conversion; no world pose or map is involved."""
    rgb, depth = image_array(rgb), image_array(depth)
    if rgb.shape[:2] != depth.shape:
        raise ValueError('RGB/depth size mismatch')
    k = np.asarray(calibration.K, dtype=float).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or
            calibration.width != depth.shape[1] or calibration.height != depth.shape[0]):
        raise ValueError('Invalid or mismatched camera calibration')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(np.float64)
    valid = np.isfinite(z) & (z >= minimum) & (z <= maximum)
    xyz = np.column_stack(((u[valid]-k[0, 2])*z[valid]/k[0, 0],
                           (v[valid]-k[1, 2])*z[valid]/k[1, 1], z[valid])).astype('<f4')
    color = rgb[::stride, ::stride][valid].astype(np.uint32)
    packed = (color[:, 0] << 16) | (color[:, 1] << 8) | color[:, 2]
    return xyz, packed


def interpolate_transform(first, second, fraction):
    """Shortest-arc unit-quaternion interpolation, with linear translation."""
    qa = np.array([first.orientation.x, first.orientation.y, first.orientation.z, first.orientation.w])
    qb = np.array([second.orientation.x, second.orientation.y, second.orientation.z, second.orientation.w])
    ta, tb = pose_matrix(first), pose_matrix(second)
    if np.dot(qa, qb) < 0: qb = -qb
    dot = float(np.clip(np.dot(qa, qb), -1., 1.))
    if dot > .9995:
        q = qa+fraction*(qb-qa)
    else:
        theta = np.arccos(dot)
        q = (np.sin((1-fraction)*theta)*qa+np.sin(fraction*theta)*qb)/np.sin(theta)
    q /= np.linalg.norm(q)
    result_pose = copy.deepcopy(first)
    result_pose.orientation.x, result_pose.orientation.y, result_pose.orientation.z, result_pose.orientation.w = q
    result = pose_matrix(result_pose)
    result[:3, 3] = (1-fraction)*ta[:3, 3]+fraction*tb[:3, 3]
    return result


class NativeOdometryGap(ValueError):
    pass


class PoseHistory:
    def __init__(self, maximum_gap_ns, maximum_samples=2400, gyros=None):
        self.maximum_gap_ns, self.maximum_samples = maximum_gap_ns, maximum_samples
        self.gyros = gyros
        self.values = {}; self.maximum_interpolated_gap_ns = 0; self.maximum_rejected_gap_ns = 0

    def put(self, stamp, pose):
        pose_matrix(pose)
        self.values[stamp] = copy.deepcopy(pose)
        for key in sorted(self.values)[:-self.maximum_samples]: del self.values[key]

    def at(self, stamp):
        keys = sorted(self.values)
        if stamp in self.values: return pose_matrix(self.values[stamp])
        index = bisect.bisect_left(keys, stamp)
        if not index or index == len(keys): return None
        lo, hi = keys[index-1:index+1]
        if hi-lo > self.maximum_gap_ns:
            self.maximum_rejected_gap_ns = max(self.maximum_rejected_gap_ns, hi-lo)
            raise NativeOdometryGap('Cannot interpolate native VIO gap %.6fs exceeding %.6fs' %
                                    ((hi-lo)*1e-9, self.maximum_gap_ns*1e-9))
        self.maximum_interpolated_gap_ns = max(self.maximum_interpolated_gap_ns, hi-lo)
        result = interpolate_transform(self.values[lo], self.values[hi], (stamp-lo)/(hi-lo))
        if self.gyros is not None:
            try:
                result[:3, :3] = self.gyros.rotation_at(pose_matrix(self.values[lo])[:3, :3],
                    pose_matrix(self.values[hi])[:3, :3], lo, hi, stamp)
            except GyroCoverageError as error:
                if error.kind == 'future': return None
                raise
        return result


class KeyframeLedger:
    def __init__(self, robot_id, cutoff_ns):
        self.robot_id, self.cutoff_ns = robot_id, cutoff_ns
        self.nodes, self.odometry = {}, {}

    def ingest(self, message):
        for node in message.nodes:
            if int(node.robot_id) != self.robot_id: raise ValueError('Foreign node in native local graph')
            key, stamp = int(node.key), require_stamp(node.header.stamp, self.cutoff_ns)
            pose_matrix(node.pose)
            old = self.nodes.get(key)
            if old is not None and int(old.header.stamp.to_nsec()) != stamp:
                raise ValueError('Native key reused across epochs')
            self.nodes[key] = copy.deepcopy(node)
        for edge in message.edges:
            if edge.robot_from != self.robot_id or edge.robot_to != self.robot_id:
                raise ValueError('Foreign edge in native local graph')
            pose_matrix(edge.pose); rotate_covariance(edge.covariance, np.eye(6))
            if int(edge.type) != int(edge.ODOM): continue
            a, b = int(edge.key_from), int(edge.key_to)
            if b != a+1: raise ValueError('Native ODOM key sequence is not consecutive')
            stamp = require_stamp(edge.header.stamp, self.cutoff_ns)
            if b in self.nodes and stamp != self.nodes[b].header.stamp.to_nsec():
                raise ValueError('Original ODOM edge stamp differs from destination key')
            old = self.odometry.get((a, b))
            if old is not None and digest(old) != digest(edge):
                raise ValueError('Native original ODOM measurement changed for an existing key')
            self.odometry[a, b] = copy.deepcopy(edge)

    def optimized_structure(self, nodes):
        selected = sorted(nodes, key=lambda item: int(item.key))
        if not selected or [int(n.key) for n in selected] != list(range(len(selected))):
            raise ValueError('PGMO MESH requires unique complete original keys 0..N without gaps')
        for node in selected:
            if int(node.robot_id) != self.robot_id:
                raise ValueError('Foreign robot owner in native optimized graph')
            require_stamp(node.header.stamp, self.cutoff_ns); pose_matrix(node.pose)
            original = self.nodes.get(int(node.key))
            if original is not None and node.header.stamp.to_nsec() != original.header.stamp.to_nsec():
                raise ValueError('Optimized key timestamp differs from original')
        return selected

    def missing_originals(self, nodes):
        return [int(node.key) for node in nodes if int(node.key) not in self.nodes]

    def contiguous(self, nodes):
        selected = self.optimized_structure(nodes)
        if self.missing_originals(selected):
            raise ValueError('Optimized key has no native local original')
        return selected

    def require_odometry_through(self, last_key):
        if set(range(last_key+1))-set(self.nodes):
            raise ValueError('Original native body graph is missing key nodes')
        if any((key, key+1) not in self.odometry for key in range(last_key)):
            raise ValueError('Original native body graph is missing an ODOM edge; refusing to invent a connection')


class RoundLedger:
    """Observed REQUEST -> exact GNC weights -> TERMINATE, never mapper status."""
    def __init__(self, updates=25):
        self.updates = updates; self.serial = 0; self.active = None; self.completed = None
        self.latest_weights = {}; self.path_event = None; self.last_path_sha = None

    def command(self, owner, message, now):
        if int(message.publishing_robot) != owner:
            raise ValueError('DPGO command owner differs from native publisher')
        if message.command == message.REQUEST_POSE_GRAPH:
            self.serial += 1
            self.active = dict(serial=self.serial, owner=owner, cluster_id=int(message.cluster_id),
                active_robots=list(message.active_robots), request_wall=now, weight_update_walls=[],
                previous_dpgo_path_sha256=self.last_path_sha)
            self.path_event = None
        elif self.active is not None and owner == self.active['owner']:
            if message.command == message.UPDATE_WEIGHT:
                self.active['weight_update_walls'].append(now)
            elif message.command == message.HARD_TERMINATE:
                self.active = None; self.completed = None; self.path_event = None
            elif message.command == message.TERMINATE:
                if len(self.active['weight_update_walls']) == self.updates:
                    self.completed = dict(self.active, terminate_wall=now)
                else:
                    self.completed = None
                self.active = None

    def weights(self, owner, message, now):
        arrays = [message.src_robot_ids, message.src_pose_ids, message.dst_robot_ids,
                  message.dst_pose_ids, message.weights, message.fixed_weights]
        if len(set(map(len, arrays))) != 1: raise ValueError('DPGO weight array lengths differ')
        rows = [list(row) for row in zip(*arrays)]
        if not all(np.isfinite(row[4]) for row in rows): raise ValueError('Nonfinite native weight')
        self.latest_weights[owner] = dict(owner=owner, received_wall=now, rows=rows,
            sha256=hashlib.sha256(json.dumps(rows, separators=(',', ':')).encode()).hexdigest())

    def native_path(self, message, now):
        signature = path_geometry_digest(message)
        self.last_path_sha = signature
        if self.completed is None or self.active is not None: return
        if now <= self.completed['terminate_wall']: return
        # With publish_iterate=false, native PGOAgentROS writes mCachedPoses
        # only in TERMINATE. A numerically unchanged completed result remains
        # valid; cached visualization repeats are annotated, not invented
        # generations. The messages still do not expose an atomic round ID.
        self.path_event = dict(received_wall=now, sha256=signature, poses=len(message.poses),
            round_serial=self.completed['serial'], numerical_change_from_previous_cached_path=
            signature != self.completed['previous_dpgo_path_sha256'])

    def proof(self, now, require_cross):
        if self.active is not None or self.completed is None or self.path_event is None: return None
        if self.path_event['round_serial'] != self.completed['serial']: return None
        if not self.completed['terminate_wall'] < self.path_event['received_wall'] <= now: return None
        weight = self.latest_weights.get(self.completed['owner'])
        if weight is None or not self.completed['terminate_wall'] <= weight['received_wall'] <= now: return None
        accepted = [r for r in weight['rows'] if r[0] != r[2] and r[4] >= .5]
        if require_cross and not accepted: return None
        return dict(round=copy.deepcopy(self.completed), upstream_dpgo_path=dict(self.path_event),
                    weights=copy.deepcopy(weight), accepted_cross_constraints=len(accepted),
                    binding='observed_native_command_path_graph_window_no_shared_message_round_id')


def validate_mesh(message, cutoff_ns, expected_frame):
    if frame(message.header.frame_id) != frame(expected_frame): raise ValueError('Native mesh frame mismatch')
    count = len(message.vertices)
    if not count or not message.triangles: raise ValueError('Native mesh has no vertices or triangles')
    if len(message.vertex_stamps) != count: raise ValueError('Native mesh missing per-vertex timestamps')
    if message.vertex_indices and len(message.vertex_indices) != count:
        raise ValueError('Native mesh graph-index correspondence is incomplete')
    xyz = np.asarray([[p.x, p.y, p.z] for p in message.vertices], dtype='<f4')
    if not np.isfinite(xyz).all(): raise ValueError('Nonfinite native mesh vertex')
    for stamp in message.vertex_stamps: require_stamp(stamp, cutoff_ns)
    for triangle in message.triangles:
        indices = list(triangle.vertex_indices)
        if len(indices) != 3 or min(indices) < 0 or max(indices) >= count:
            raise ValueError('Native mesh triangle index outside vertex array')
    return xyz


def retag_local_mesh(message, robot_id):
    """Only the known local frontend's missing robot_id-parser metadata bug."""
    if message.ns not in ('0', str(robot_id)):
        raise ValueError('Unexpected ownership namespace from native local mesh frontend')
    # Geometry is an immutable received snapshot. Only the scalar ownership
    # field changes here; retained publications own their outer header before
    # rospy can mutate it, so sharing geometry avoids a full object walk.
    output = copy.copy(message)
    output.ns = str(robot_id)
    return output


def same_mesh_topology_and_stamps(first, second):
    return (len(first.vertices) == len(second.vertices) and
        [list(t.vertex_indices) for t in first.triangles] == [list(t.vertex_indices) for t in second.triangles] and
        [s.to_nsec() for s in first.vertex_stamps] == [s.to_nsec() for s in second.vertex_stamps])


def native_readiness_query(last_native_graph, original_key_zero):
    """No edge measurements and no new key: native PGMO path publication only."""
    if int(original_key_zero.key) != 0: raise ValueError('Readiness query requires actual native key zero')
    output = copy.deepcopy(last_native_graph)
    output.nodes = [copy.deepcopy(original_key_zero)]; output.edges = []
    return output


def optimized_camera_to_body(camera_pose):
    return pose_matrix(camera_pose) @ np.linalg.inv(SIM_T_BODY_CAMERA)


def path_error(actual, expected):
    if len(actual.poses) < len(expected.poses): return None
    translation, rotation = 0., 0.
    for measured, target in zip(actual.poses, expected.poses):
        delta = np.linalg.inv(pose_matrix(target.pose)) @ pose_matrix(measured.pose)
        translation = max(translation, float(np.linalg.norm(delta[:3, 3])))
        rotation = max(rotation, float(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1., 1.))))
    return dict(max_translation_m=translation, max_rotation_rad=rotation)


class NativeDenseBridge:
    def __init__(self, args):
        args.external_sensor_stream = getattr(args, 'external_sensor_stream', False)
        self.a = args; self.base = '/'+args.robot_name; self.lock = threading.RLock()
        self.native_input_lock = threading.Lock(); self.native_inputs = deque()
        self.native_input_max_queue = 0; self.native_input_max_delay = 0.; self.native_control_max_delay = 0.
        self.native_imu_max_delay = 0.
        self.replaceable_lock = threading.Lock(); self.replaceable_inputs = {}; self.replaceable_overwrites = Counter()
        self.optimized_candidates = deque(); self.optimized_candidate_wait = 60.; self.optimized_candidate_limit = 4
        self.missing_original_since = {}
        self.local_frame = args.robot_name+'/vio_odom'; self.optical_frame = args.robot_name+'/native_dense_optical'
        self.output_frame = self.local_frame if args.mode == 'local' else args.global_frame
        self.cutoff_ns = round(args.cutoff*1e9)
        self.gyros = (GyroRotationHistory(round(args.max_gyro_gap*1e9),
            sample_convention=args.gyro_sample_convention) if args.sensor_rotation_interpolation == 'gyro_endpoint' else None)
        self.history = PoseHistory(round(args.max_odom_gap*1e9), gyros=self.gyros)
        self.keys = KeyframeLedger(args.robot_id, self.cutoff_ns); self.rounds = RoundLedger(args.weight_updates)
        self.camera = None; self.pending_images = {}; self.graph_queue = deque(); self.mesh_graph_queue = deque()
        self.first_native_odom = None; self.original_first_pose = None; self.gauge_checked = False
        self.graph_sent = -1; self.last_graph_wall = 0.; self.last_rgbd_ns = 0; self.latest_full_mesh = None
        self.latest_full_mesh_bytes = None; self.latest_full_mesh_sha = None
        self.approved = None; self.pending_optimized = None; self.last_optimized_sha = None
        self.transaction = None; self.pgmo_initialized = False; self.last_forwarded_graph = None
        self.pgmo_graph_stats = None; self.native_body_metadata_ack = False
        self.native_body_metadata_ack_has_optimized_key_zero = False
        self.eligible = None; self.camera_graph_contract = None
        self.accepted_joint_mesh_receipt = None; self.latest_submitted_body_path_sha256 = None
        self.remesh_after = None; self.fatal = None; self.counts = Counter(); self.evidence = args.output_directory
        self.evidence.mkdir(parents=True, exist_ok=True)
        self.events = (self.evidence/'bridge_events.jsonl').open('a', buffering=1)
        self.tf = tf2_ros.TransformBroadcaster()
        topics = {'points': ('rgbd_points', PointCloud2, 1, False),
            'graph': ('pose_graph_incremental', PoseGraph, 1000, False),
            'mesh_graph': ('mesh_graph_incremental', PoseGraph, 1000, False),
            'full': ('full_mesh', KimeraPgmoMesh, 1, False),
            'path': ('approved_body_path', RosPath, 1, True),
            'native_mesh': ('native_optimized_mesh', KimeraPgmoMesh, 1, True),
            'accepted_mesh': ('accepted_joint_mesh', KimeraPgmoMesh, 1, True),
            'accepted_vertices': ('accepted_joint_vertices', PointCloud2, 1, True),
            'raw_local_vertices': ('raw_local_vertices', PointCloud2, 1, True),
            'vertices': ('display_vertices', PointCloud2, 1, True),
            'status': ('status', String, 1, True)}
        if args.external_sensor_stream: del topics['points']
        self.pub = {k: rospy.Publisher(self.base+'/native_dense/'+v[0], v[1], queue_size=v[2], latch=v[3])
                    for k, v in topics.items()}
        self.subs = []
        native = args.replay_caller if args.recorded_native_bag else self.base+'/kimera_vio_ros/kimera_vio_ros_node'
        # These callbacks retain every authenticated original immediately, before
        # contending with expensive mesh serialization/validation under lock.
        # None disables rospy's per-receive-batch queue truncation; application
        # overflow is explicit failure, never silently discarded native keys.
        self.subscribe(self.base+'/kimera_vio_ros/odometry', Odometry, self.on_odom, native, None, priority=True)
        self.subscribe(self.base+'/kimera_vio_ros/pose_graph_incremental', PoseGraph, self.on_graph, native, None, priority=True)
        sensor = args.replay_caller if args.recorded_native_bag else '/'+args.robot_name+'_multimodal_sensors'
        if not args.external_sensor_stream:
            if self.gyros is not None:
                self.subscribe(self.base+'/kimera/imu', Imu, self.on_imu, sensor, None, priority='imu')
            self.subscribe(self.base+'/kimera/rgb/camera_info', CameraInfo, self.on_camera, sensor, 2)
            self.subscribe(self.base+'/kimera/rgb/image_raw', Image, lambda m: self.on_image('rgb', m), sensor, 20)
            self.subscribe(self.base+'/kimera/depth/image_raw', Image, lambda m: self.on_image('depth', m), sensor, 20)
        frontend = self.base+'/native_mesh_frontend'
        self.subscribe(frontend+'/mesh_graph_incremental', PoseGraph, self.on_mesh_graph, frontend, 1000)
        self.subscribe(frontend+'/full_mesh', KimeraPgmoMesh, self.on_full_mesh, frontend, 1, replaceable=True)
        self.subscribe(self.base+'/native_pgmo/optimized_mesh', KimeraPgmoMesh, self.on_optimized_mesh,
                       self.base+'/native_pgmo', 1, replaceable=True)
        self.subscribe(self.base+'/native_pgmo/optimized_path', RosPath, self.on_pgmo_path,
                       self.base+'/native_pgmo', None, priority='control')
        self.subscribe(self.base+'/native_pgmo/pose_graph', PoseGraph, self.on_pgmo_graph,
                       self.base+'/native_pgmo', None, priority='control')
        if args.mode == 'joint':
            self.subscribe(self.base+'/kimera_camera_adapter/diagnostics', String, self.on_camera_graph_contract,
                '/'+args.robot_name+'_native_vio_camera_graph_adapter', 2)
            for robot in (0, 1):
                owner = '/sim_%d/dpgo_ros_node/agent' % robot
                self.subscribe('/sim_%d/dpgo_ros_node/command' % robot, Command,
                    lambda m, t, r=robot: self.rounds.command(r, m, t), owner, None,
                    priority='control', with_receive_wall=True)
                self.subscribe('/sim_%d/dpgo_ros_node/measurement_weights' % robot, RelativeMeasurementWeights,
                    lambda m, t, r=robot: self.rounds.weights(r, m, t), owner, None,
                    priority='control', with_receive_wall=True)
            self.subscribe(self.base+'/dpgo_ros_node/path', RosPath,
                lambda m, t: self.rounds.native_path(m, t), self.base+'/dpgo_ros_node/agent', None,
                priority='control', with_receive_wall=True)
            self.subscribe(self.base+'/kimera_distributed/optimized_nodes', PoseGraph, self.on_optimized_graph,
                self.base+'/distributed_loop_closure/distributed_loop_closure_node', None,
                priority='control', with_receive_wall=True)
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)

    def subscribe(self, topic, kind, callback, expected, queue, priority=False, with_receive_wall=False, replaceable=False):
        def receive(message):
            try:
                received_wall, received_monotonic = time.time(), time.monotonic()
                require_source(message, expected)
                if priority:
                    if priority is True and callback == self.on_graph:
                        if frame(message.header.frame_id) != self.a.robot_name+'/map':
                            raise ValueError('Unexpected original native graph frame at ingress')
                        for node in message.nodes:
                            if int(node.robot_id) != self.a.robot_id: raise ValueError('Foreign original robot at ingress')
                            require_stamp(node.header.stamp, self.cutoff_ns)
                        for edge in message.edges:
                            if edge.robot_from != self.a.robot_id or edge.robot_to != self.a.robot_id:
                                raise ValueError('Foreign original edge at ingress')
                            require_stamp(edge.header.stamp, self.cutoff_ns)
                    elif priority == 'imu':
                        if frame(message.header.frame_id) != self.a.robot_name+'/imu':
                            raise ValueError('Unexpected actual body gyro frame at ingress')
                        require_stamp(message.header.stamp, self.cutoff_ns)
                        omega = message.angular_velocity
                        if not np.isfinite([omega.x, omega.y, omega.z]).all():
                            raise ValueError('Nonfinite actual gyro at ingress')
                    elif priority is True:
                        if (frame(message.header.frame_id) != self.local_frame or
                                frame(message.child_frame_id) != self.a.robot_name+'/imu'):
                            raise ValueError('Unexpected native body odometry frames at ingress')
                        require_stamp(message.header.stamp, self.cutoff_ns)
                    with self.native_input_lock:
                        if len(self.native_inputs) >= 20000:
                            raise RuntimeError('Lossless native original input queue overflow; no keys discarded')
                        self.native_inputs.append((callback, message, received_wall, received_monotonic,
                            with_receive_wall, 'original' if priority is True else str(priority)))
                        self.native_input_max_queue = max(self.native_input_max_queue, len(self.native_inputs))
                    return
                if replaceable:
                    with self.replaceable_lock:
                        if topic in self.replaceable_inputs: self.replaceable_overwrites[topic] += 1
                        self.replaceable_inputs[topic] = (callback, message)
                    return
                with self.lock:
                    self.drain_native_inputs()
                    if self.fatal is None: callback(message)
            except Exception as error: self.fail(error)
        self.subs.append(rospy.Subscriber(topic, kind, receive, queue_size=queue, buff_size=2**26, tcp_nodelay=True))

    def drain_native_inputs(self):
        # Caller holds the state lock. Detach the finite batch under a separate
        # short lock so native transport callbacks never wait for mesh work.
        with self.native_input_lock:
            batch = list(self.native_inputs); self.native_inputs.clear()
        for callback, message, received_wall, received_monotonic, with_receive_wall, category in batch:
            delay = time.monotonic()-received_monotonic
            if category == 'original': self.native_input_max_delay = max(self.native_input_max_delay, delay)
            elif category == 'imu': self.native_imu_max_delay = max(self.native_imu_max_delay, delay)
            else: self.native_control_max_delay = max(self.native_control_max_delay, delay)
            if with_receive_wall: callback(message, received_wall)
            else: callback(message)
            self.counts[{'original': 'priority_native_originals_processed',
                         'imu': 'priority_actual_imu_samples_processed'}.get(category,
                         'priority_native_control_events_processed')] += 1

    def drain_replaceable_inputs(self):
        with self.replaceable_lock:
            batch = list(self.replaceable_inputs.values()); self.replaceable_inputs.clear()
        for callback, message in batch:
            self.drain_native_inputs()
            callback(message)

    def event(self, kind, **fields):
        self.events.write(json.dumps(dict(event=kind, run_id=self.a.run_id, wall=time.time(), **fields), allow_nan=False)+'\n')

    def fail(self, error):
        with self.lock:
            if self.fatal is not None: return
            # SIGINT can close rospy publishers while an already-running timer
            # finishes its callback. Only this exact closed-publisher exception
            # during actual ROS shutdown is benign; runtime failures still fail.
            if (str(error) == 'publish() to a closed topic' and
                    getattr(rospy, 'is_shutdown', lambda: False)()):
                return
            self.fatal = str(error); self.event('fatal', error=str(error)); rospy.logfatal('Native dense bridge: %s', error)
            # Keep the primary cause even if an in-flight timer encounters
            # closed publishers during rospy shutdown.
            (self.evidence/'fatal.json').write_text(json.dumps(dict(error=self.fatal, wall=time.time(),
                run_id=self.a.run_id), indent=2), encoding='utf-8')
        rospy.signal_shutdown(str(error))

    def on_odom(self, message):
        if frame(message.header.frame_id) != self.local_frame or frame(message.child_frame_id) != self.a.robot_name+'/imu':
            raise ValueError('Expected native VIO body odometry frames')
        stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        if self.first_native_odom is None: self.first_native_odom = (stamp, pose_matrix(message.pose.pose))
        self.history.put(stamp, message.pose.pose); self.counts['native_odometry'] += 1

    def on_imu(self, message):
        if frame(message.header.frame_id) != self.a.robot_name+'/imu':
            raise ValueError('Expected authenticated actual body gyro frame')
        stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        omega = message.angular_velocity
        self.gyros.put(stamp, [omega.x, omega.y, omega.z])
        self.counts['actual_gyro_samples'] += 1

    def on_graph(self, message):
        if frame(message.header.frame_id) != self.a.robot_name+'/map':
            raise ValueError('Native original body graph must retain its native sim_N/map frame declaration')
        self.keys.ingest(message)
        for node in message.nodes: self.missing_original_since.pop(int(node.key), None)
        if not self.graph_queue and self.graph_sent < 0:
            if 0 not in self.keys.nodes: raise ValueError('Start dense bridge before native key zero')
            self.original_first_pose = pose_matrix(self.keys.nodes[0].pose)
        self.graph_queue.append(copy.deepcopy(message)); self.counts['native_graphs'] += 1
        if len(self.graph_queue) > 1000: raise RuntimeError('Lossless native graph queue overflow')

    def on_camera(self, message):
        require_stamp(message.header.stamp, self.cutoff_ns)
        if frame(message.header.frame_id) != self.a.robot_name+'/camera_optical':
            raise ValueError('Unexpected calibrated optical frame')
        if self.camera is not None and (message.K != self.camera.K or message.width != self.camera.width or message.height != self.camera.height):
            raise ValueError('Camera calibration changed within epoch')
        self.camera = message

    def on_image(self, kind, message):
        stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        if frame(message.header.frame_id) != self.a.robot_name+'/camera_optical': raise ValueError('Unexpected image frame')
        if kind == 'depth' and message.encoding != '32FC1': raise ValueError('Depth must be optical-Z 32FC1 metres')
        self.pending_images.setdefault(stamp, {})[kind] = message
        for key in sorted(self.pending_images)[:-100]:
            del self.pending_images[key]; self.counts['unprocessed_rgbd_dropped'] += 1

    def on_mesh_graph(self, message):
        output = copy.deepcopy(message)
        owners = {int(n.robot_id) for n in output.nodes}
        owners.update(int(e.robot_from) for e in output.edges); owners.update(int(e.robot_to) for e in output.edges)
        if owners and owners not in ({0}, {self.a.robot_id}): raise ValueError('Mixed owner graph from local native mesh frontend')
        for node in output.nodes:
            require_stamp(node.header.stamp, self.cutoff_ns); pose_matrix(node.pose); node.robot_id = self.a.robot_id
        for edge in output.edges:
            edge.robot_from = edge.robot_to = self.a.robot_id
        self.mesh_graph_queue.append(output); self.counts['native_mesh_graphs'] += 1
        if len(self.mesh_graph_queue) > 1000: raise RuntimeError('Lossless native mesh graph queue overflow')

    def on_full_mesh(self, message):
        xyz = validate_mesh(message, self.cutoff_ns, self.local_frame)
        self.latest_full_mesh = retag_local_mesh(message, self.a.robot_id)
        self.latest_full_mesh_bytes = message_bytes(self.latest_full_mesh)
        self.latest_full_mesh_sha = hashlib.sha256(self.latest_full_mesh_bytes).hexdigest()
        self.counts['native_full_meshes'] += 1
        # The native frontend mesh is still in the VIO integration frame. Keep
        # that geometry observable independently of PGMO's global deformation.
        # Only XYZ extraction and a display ownership color happen here.
        color = 0x34D8ED if self.a.robot_id == 0 else 0xFFA04B
        self.pub['raw_local_vertices'].publish(
            self.make_cloud(xyz, color, message.header.stamp, self.local_frame))
        self.counts['raw_local_mesh_displays'] += 1

    def on_optimized_graph(self, message, received_wall=None):
        if self.transaction is not None:
            self.counts['optimized_graphs_during_mesh_transaction'] += 1; return
        nodes = self.keys.optimized_structure(message.nodes)
        if not nodes: return
        proof = self.rounds.proof(time.time(), self.a.require_cross)
        if self.camera_graph_contract is None:
            self.counts['optimized_graphs_without_verified_camera_graph_contract'] += 1; return
        code, _, states = rospy.get_master().getSystemState()
        subscribers = dict(states[1]) if code == 1 else {}
        if self.base+'/distributed_loop_closure/distributed_loop_closure_node' not in subscribers.get(
                self.base+'/kimera_camera_adapter/pose_graph_incremental', []):
            raise ValueError('Native LCD is not subscribed to the verified camera-convention graph')
        if proof is not None and self.a.robot_id not in proof['round']['active_robots']:
            self.counts['optimized_graphs_outside_active_cluster'] += 1; return
        if rospy.get_param(self.base+'/dpgo_ros_node/agent/publish_iterate', None) is not False:
            raise ValueError('Joint dense mode requires native DPGO publish_iterate=false')
        signature = digest(message)
        if (signature == self.last_optimized_sha or
                any(candidate['signature'] == signature for candidate in self.optimized_candidates)): return
        if len(self.optimized_candidates) >= self.optimized_candidate_limit:
            old = self.optimized_candidates.popleft(); self.counts['optimized_candidates_superseded'] += 1
            self.event('optimized_candidate_superseded', graph_sha256=old['signature'])
        self.optimized_candidates.append(dict(nodes=copy.deepcopy(nodes), proof=proof,
            signature=signature, raw=message_bytes(message), received_wall=received_wall or time.time(),
            received_monotonic=time.monotonic(), last_missing=None))
        if proof is None: self.counts['optimized_graphs_waiting_round_receipt'] += 1
        self.advance_optimized_candidates()

    def advance_optimized_candidates(self):
        if self.transaction is not None: return
        now = time.time(); current = self.rounds.proof(now, self.a.require_cross)
        retained = deque()
        while self.optimized_candidates:
            candidate = self.optimized_candidates.popleft(); nodes = candidate['nodes']
            if candidate['proof'] is None:
                if current is None:
                    if time.monotonic()-candidate['received_monotonic'] <= self.optimized_candidate_wait:
                        retained.append(candidate)
                    else: self.counts['optimized_candidates_expired_missing_round_receipt'] += 1
                    continue
                # Native cached poses are only committed by TERMINATE. Retain
                # independent-topic deliveries until the observed round ledger
                # catches up, while rejecting genuinely earlier stale graphs.
                if candidate['received_wall'] < current['round']['terminate_wall']-1.:
                    self.counts['optimized_candidates_predating_completed_round'] += 1; continue
                candidate['proof'] = current
            if (current is None or current['round']['serial'] != candidate['proof']['round']['serial'] or
                    current['round']['owner'] != candidate['proof']['round']['owner']):
                self.counts['optimized_candidates_obsolete_round'] += 1; continue
            if self.a.robot_id not in current['round']['active_robots']:
                self.counts['optimized_candidates_outside_active_cluster'] += 1; continue
            missing = self.keys.missing_originals(nodes)
            monotonic_now = time.monotonic()
            for key in missing: self.missing_original_since.setdefault(key, monotonic_now)
            missing_wait = max((monotonic_now-self.missing_original_since[key] for key in missing), default=0.)
            if missing and missing_wait > self.optimized_candidate_wait:
                self.counts['optimized_candidates_expired_missing_originals'] += 1
                self.event('optimized_candidate_missing_originals_expired', graph_sha256=candidate['signature'],
                    wait_s=missing_wait, missing_count=len(missing),
                    first_missing_key=missing[0], local_key_count=len(self.keys.nodes))
                continue
            if missing:
                if candidate['last_missing'] is None:
                    self.counts['optimized_candidates_waiting_originals'] += 1
                    self.event('optimized_candidate_waiting_originals', graph_sha256=candidate['signature'],
                        missing_count=len(missing), first_missing_key=missing[0], local_key_count=len(self.keys.nodes))
                candidate['last_missing'] = len(missing); retained.append(candidate); continue
            # Revalidate every exact native stamp after the original connection
            # catches up. Missing data is never fabricated, extrapolated or fit.
            self.keys.contiguous(nodes)
            if self.camera_graph_contract.get('native_key_zero_stamp_ns') != self.keys.nodes[0].header.stamp.to_nsec():
                raise ValueError('Camera graph adapter native key-zero epoch differs from dense original graph')
            self.pending_optimized = (nodes, candidate['proof'], candidate['signature'], candidate['raw'])
            self.counts['optimized_candidates_originals_verified'] += 1
        self.optimized_candidates = retained

    def on_camera_graph_contract(self, message):
        report = json.loads(message.data)
        if (report.get('adapter_role') != 'coordinate_transform_of_actual_native_Kimera_VIO_graph' or
                report.get('uses_simulator_ground_truth') is not False or
                report.get('uses_sdk_or_lidar_odometry') is not False or
                report.get('input_topic') != self.base+'/kimera_vio_ros/pose_graph_incremental' or
                report.get('output_topic') != self.base+'/kimera_camera_adapter/pose_graph_incremental' or
                report.get('expected_input_caller') != self.base+'/kimera_vio_ros/kimera_vio_ros_node' or
                report.get('fatal_error') is not None or
                not np.array_equal(np.asarray(report.get('T_body_camera')), SIM_T_BODY_CAMERA)):
            raise ValueError('Joint camera/body graph convention has not been verified against native adapter')
        self.camera_graph_contract = report

    def make_cloud(self, xyz, colors, stamp, cloud_frame):
        data = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        if len(xyz): data['x'], data['y'], data['z'] = xyz.T
        data['rgb'] = colors
        fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32 if i<3 else PointField.UINT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
        return PointCloud2(header=Header(stamp=stamp, frame_id=cloud_frame), height=1, width=len(xyz),
            fields=fields, is_bigendian=False, point_step=16, row_step=16*len(xyz), data=data.tobytes(), is_dense=True)

    def publish_sensor(self):
        # A PGMO transaction freezes its own graph/full-mesh inputs, not the
        # independent VIO -> TSDF stream. Pausing this producer used to discard
        # camera coverage while waiting for native deformation receipts.
        if getattr(self.a, 'external_sensor_stream', False): return
        if not self.gauge_checked or self.camera is None or not self.pub['points'].get_num_connections(): return
        for stamp in sorted(list(self.pending_images)):
            if stamp <= self.last_rgbd_ns or (stamp-self.last_rgbd_ns)*1e-9 < 1./self.a.cloud_rate:
                del self.pending_images[stamp]; continue
            pair = self.pending_images[stamp]
            if set(pair) != {'rgb', 'depth'}: continue
            try:
                body = self.history.at(stamp)
            except NativeOdometryGap:
                del self.pending_images[stamp]; self.counts['rgbd_omitted_native_odom_gap'] += 1
                continue
            except GyroCoverageError as error:
                # Never replace a missing inertial interval with a constant
                # angular velocity curve. Count an explicit omitted RGB-D frame.
                del self.pending_images[stamp]
                self.counts['rgbd_omitted_actual_gyro_'+error.kind] += 1
                continue
            if body is None:
                if self.history.values and stamp < min(self.history.values):
                    del self.pending_images[stamp]; self.counts['rgbd_omitted_no_native_lower_bracket'] += 1
                continue
            xyz, colors = optical_points(pair['rgb'], pair['depth'], self.camera, self.a.pixel_stride, .3, self.a.max_depth)
            camera = body @ SIM_T_BODY_CAMERA
            pose = copy.deepcopy(self.keys.nodes[0].pose); set_pose(pose, camera)
            tf = TransformStamped(header=Header(stamp=pair['rgb'].header.stamp, frame_id=self.local_frame), child_frame_id=self.optical_frame)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = camera[:3, 3]
            tf.transform.rotation = pose.orientation
            self.tf.sendTransform(tf)
            self.pub['points'].publish(self.make_cloud(xyz, colors, pair['rgb'].header.stamp, self.optical_frame))
            self.last_rgbd_ns = stamp; del self.pending_images[stamp]; self.counts['sensor_clouds'] += 1
            if self.transaction is not None: self.counts['sensor_clouds_during_mesh_transaction'] += 1
            break

    def publish_graphs(self):
        if self.transaction is not None: return
        if self.gauge_checked and self.pub['graph'].get_num_connections():
            while self.graph_queue:
                message = self.graph_queue.popleft(); output = copy.deepcopy(message)
                output.edges = [e for e in output.edges if int(e.type) == int(e.ODOM)]
                self.keys.require_odometry_through(max((int(n.key) for n in output.nodes), default=self.graph_sent))
                self.pub['graph'].publish(output)
                self.last_forwarded_graph = copy.deepcopy(output)
                self.graph_sent = max(self.graph_sent, max((int(n.key) for n in output.nodes), default=-1))
                self.last_graph_wall = time.time(); self.counts['original_body_graphs_forwarded'] += 1
        # Native mesh controls attach immediately to native unconnected body
        # metadata. Its first pose_graph response is sent only after that body
        # callback completed. Once initialized, the native queue retains a final
        # body node and accepts asynchronous increments; moving quiet is neither
        # required nor a receipt that the native callback ran.
        if self.gauge_checked and self.native_body_metadata_ack and self.graph_sent >= 0 and self.pub['mesh_graph'].get_num_connections():
            while self.mesh_graph_queue:
                self.pub['mesh_graph'].publish(self.mesh_graph_queue.popleft()); self.counts['mesh_graphs_forwarded'] += 1

    def approve_path(self):
        if self.transaction is not None or not self.pgmo_initialized or self.latest_full_mesh is None: return
        if self.pending_optimized is None or self.graph_sent < len(self.pending_optimized[0])-1: return
        if not self.pub['path'].get_num_connections(): return
        nodes, proof, signature, raw = self.pending_optimized
        current = self.rounds.proof(time.time(), self.a.require_cross)
        if current is None or current['round']['serial'] != proof['round']['serial']:
            self.pending_optimized = None; return
        path = RosPath(header=Header(stamp=rospy.Time.now(), frame_id=self.a.global_frame))
        for node in nodes:
            body = optimized_camera_to_body(node.pose)
            value = PoseStamped(header=copy.deepcopy(node.header), pose=copy.deepcopy(node.pose))
            value.header.frame_id = self.a.global_frame; value.header.seq = int(node.key); set_pose(value.pose, body)
            path.poses.append(value)
        index = self.counts['native_mesh_transactions_started']+1
        self.counts['native_mesh_transactions_started'] = index
        prefix = self.evidence/('native_transaction_%04d' % index)
        prefix.with_suffix('.rosbin').write_bytes(raw)
        metadata = dict(graph_sha256=signature, key_count=len(nodes), pose_convention='T_global_camera_to_T_global_body',
            original_key_stamps_ns=[int(n.header.stamp.to_nsec()) for n in nodes], proof=proof,
            path_sha256=digest(path), prepared_wall=time.time(), mesh_round_receipt=False,
            camera_graph_adapter_native_key_zero_stamp_ns=self.camera_graph_contract['native_key_zero_stamp_ns'])
        prefix.with_suffix('.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        # Freeze PGMO graph/full-mesh producers. Establish a native baseline on
        # the exact same fixed mesh before submitting changed priors. The local
        # TSDF stream continues; newer frontend meshes remain in the latest
        # cache and cannot replace the transaction's captured fixed mesh.
        raw_mesh = snapshot_message_header(self.latest_full_mesh)
        self.transaction = dict(phase='await_native_seed', started_wall=time.time(), phase_started_wall=time.time(), path=path,
            metadata=metadata, prefix=prefix, fixed_mesh=raw_mesh,
                    fixed_mesh_sha256=self.latest_full_mesh_sha, last_query_wall=0., echo_receipts=[],
            expected_native_body_keys=self.graph_sent+1, waiting_full_mesh_ack=True,
            last_full_mesh_send_wall=time.time(), stable_baseline_count=0)
        prefix.with_name(prefix.name+'_fixed_input_mesh.rosbin').write_bytes(self.latest_full_mesh_bytes)
        self.last_optimized_sha = signature; self.pending_optimized = None
        publish_preserving_cached_message(self.pub['full'], raw_mesh)
        self.event('native_fixed_mesh_baseline_requested', mesh_sha256=self.latest_full_mesh_sha)

    def on_pgmo_graph(self, message):
        if self.graph_sent >= 0 and 0 in self.keys.nodes:
            zeros = [n for n in message.nodes if int(n.key) == 0 and int(n.robot_id) == self.a.robot_id]
            if any(n.header.stamp.to_nsec() != self.keys.nodes[0].header.stamp.to_nsec() for n in zeros):
                raise ValueError('Native PGMO key-zero stamp differs from original epoch')
            self.native_body_metadata_ack = True
            self.native_body_metadata_ack_has_optimized_key_zero |= bool(zeros)
        self.pgmo_graph_stats = dict(nodes=len(message.nodes), edges=len(message.edges), received_wall=time.time())
        if self.transaction is not None and self.transaction['phase'] == 'await_path_echo':
            self.transaction.setdefault('query_graph_counts', []).append(dict(self.pgmo_graph_stats))

    def on_pgmo_path(self, message):
        current = self.transaction
        if current is None: return
        if current['phase'] == 'await_native_seed':
            current['native_seed_path_keys'] = len(message.poses)
            if len(message.poses) >= current['expected_native_body_keys']:
                current['phase'] = 'await_baseline'; current['phase_started_wall'] = time.time()
                current['complete_native_metadata_verified_wall'] = time.time()
                current['waiting_full_mesh_ack'] = True; current['last_full_mesh_send_wall'] = time.time()
                publish_preserving_cached_message(self.pub['full'], current['fixed_mesh'])
            return
        if current['phase'] != 'await_path_echo': return
        error = path_error(message, current['path'])
        if error is None or error['max_translation_m'] > 1e-5 or error['max_rotation_rad'] > 1e-5: return
        current['echo_receipts'].append(dict(received_wall=time.time(), path_sha256=digest(message), **error))
        # Two identical native query results plus unchanged native factor counts
        # demonstrate that the no-edge readiness queries introduced no factor.
        counts = current.get('query_graph_counts', [])
        if len(current['echo_receipts']) < 2 or len(counts) < 2: return
        if (counts[-1]['nodes'], counts[-1]['edges']) != (counts[-2]['nodes'], counts[-2]['edges']): return
        current['phase'] = 'await_deformed_mesh'; current['phase_started_wall'] = time.time()
        current['path_echo_verified_wall'] = time.time()
        publish_preserving_cached_message(self.pub['full'], current['fixed_mesh'])
        current['fixed_mesh_retriggered_wall'] = time.time()
        self.event('native_path_echo_then_fixed_mesh_retrigger', path_sha256=current['metadata']['path_sha256'],
            mesh_sha256=current['fixed_mesh_sha256'], echo_receipts=current['echo_receipts'],
            no_edge_query_factor_counts=counts[-2:])

    def transaction_tick(self):
        current = self.transaction
        if current is None: return
        if time.time()-current['phase_started_wall'] > self.a.mesh_phase_timeout:
            self.event('native_mesh_transaction_not_accepted', phase=current['phase'],
                elapsed_phase_s=time.time()-current['phase_started_wall'],
                reason='No native phase readiness/deformation proof before configured finite timeout; inputs resume and accepted snapshot stays')
            self.counts['mesh_transactions_not_accepted'] += 1; self.transaction = None; return
        if current['phase'] in ('await_native_seed', 'await_path_echo') and time.time()-current['last_query_wall'] > .25:
            # PGMO early-returns for an empty graph. A single existing native
            # node with NO edges reaches its path publisher without adding any
            # node/factor, because its trajectory has already been initialized.
            query = native_readiness_query(self.last_forwarded_graph, self.keys.nodes[0])
            self.pub['graph'].publish(query); current['last_query_wall'] = time.time()
            self.counts['no_edge_native_path_queries'] += 1
        if (current['phase'] in ('await_native_seed', 'await_baseline') and
                not current['waiting_full_mesh_ack'] and time.time()-current['last_full_mesh_send_wall'] > .2):
            # At most one same-input full-mesh request is outstanding. It
            # promotes native queued body values and then proves a stable
            # native baseline; it is never a free-running cached relay.
            current['waiting_full_mesh_ack'] = True; current['last_full_mesh_send_wall'] = time.time()
            publish_preserving_cached_message(self.pub['full'], current['fixed_mesh'])

    def on_optimized_mesh(self, message):
        xyz = validate_mesh(message, self.cutoff_ns, self.output_frame)
        if message.ns != str(self.a.robot_id): raise ValueError('Native PGMO output ownership differs from configured robot')
        serialized = None; serialized_sha = None
        def mesh_bytes():
            nonlocal serialized
            if serialized is None: serialized = message_bytes(message)
            return serialized
        def mesh_sha():
            nonlocal serialized_sha
            if serialized_sha is None: serialized_sha = hashlib.sha256(mesh_bytes()).hexdigest()
            return serialized_sha
        self.pgmo_initialized = True
        newly_accepted = False
        if self.a.mode == 'joint':
            current = self.transaction
            if current is not None:
                if not same_mesh_topology_and_stamps(message, current['fixed_mesh']): return
                if current['phase'] == 'await_native_seed':
                    current['waiting_full_mesh_ack'] = False
                    return
                if current['phase'] == 'await_baseline':
                    current['waiting_full_mesh_ack'] = False
                    previous = current.get('candidate_baseline_xyz')
                    if previous is not None and np.allclose(xyz, previous, atol=1e-7, rtol=0):
                        current['stable_baseline_count'] += 1
                    else:
                        current['stable_baseline_count'] = 1
                    current['candidate_baseline_xyz'] = xyz.copy()
                    if current['stable_baseline_count'] < 2: return
                    current['baseline_xyz'] = xyz.copy(); current['baseline_mesh_sha256'] = mesh_sha()
                    current['prefix'].with_name(current['prefix'].name+'_native_before.rosbin').write_bytes(mesh_bytes())
                    current['phase'] = 'await_path_echo'; current['phase_started_wall'] = time.time()
                    current['path_published_wall'] = time.time()
                    publish_preserving_cached_message(self.pub['path'], current['path'])
                    self.latest_submitted_body_path_sha256 = current['metadata']['path_sha256']
                    self.counts['submitted_native_paths'] += 1
                    self.event('native_path_submitted_after_fixed_mesh_baseline', **current['metadata'])
                    return
                if current['phase'] != 'await_deformed_mesh': return
                delta = np.linalg.norm(xyz-current['baseline_xyz'], axis=1)
                if not len(delta) or float(delta.max()) <= 1e-5:
                    self.counts['unchanged_native_mesh_not_deformation_proof'] += 1; return
                self.approved = current['metadata']
                self.eligible = dict(native_path_echo_verified=True, fixed_input_mesh_sha256=current['fixed_mesh_sha256'],
                    native_before_mesh_sha256=current['baseline_mesh_sha256'], native_after_mesh_sha256=mesh_sha(),
                    historical_vertices_moved=int(np.count_nonzero(delta > 1e-5)),
                    maximum_vertex_motion_m=float(delta.max()), rms_vertex_motion_m=float(np.sqrt(np.mean(delta*delta))),
                    path_echo_receipts=current['echo_receipts'], query_graph_counts=current.get('query_graph_counts', []),
                    fixed_mesh_retriggered_wall=current['fixed_mesh_retriggered_wall'],
                    same_native_input_mesh_topology_and_vertex_stamps=True, submitted_path_sha256=self.approved['path_sha256'],
                    frozen_native_body_graph_keys=current['expected_native_body_keys'],
                    complete_native_metadata_verified_wall=current['complete_native_metadata_verified_wall'],
                    same_input_native_stable_baseline_repetitions=current['stable_baseline_count'],
                    source_round_proof=self.approved['proof'], optimizer_round_binding_is_observational=True)
                current['prefix'].with_name(current['prefix'].name+'_native_after.rosbin').write_bytes(mesh_bytes())
                current['prefix'].with_name(current['prefix'].name+'_native_deformation_proof.json').write_text(
                    json.dumps(self.eligible, indent=2), encoding='utf-8')
                self.event('native_historical_mesh_deformation_measured', **self.eligible)
                self.accepted_joint_mesh_receipt = dict(run_id=self.a.run_id, cutoff_ns=self.cutoff_ns,
                    robot_id=self.a.robot_id, native_geometry_source='native_PGMO_same_fixed_mesh_deformation_transaction',
                    canonical_native_mesh_sha256=canonical_mesh_digest(message),
                    vertex_xyz_float32_le_sha256=hashlib.sha256(xyz.astype('<f4').tobytes()).hexdigest(),
                    mesh_header_stamp_ns=message.header.stamp.to_nsec(), mesh_frame=message.header.frame_id,
                    vertex_count=len(message.vertices), triangle_count=len(message.triangles),
                    vertex_stamps_uint64_le_sha256=hashlib.sha256(np.asarray([s.to_nsec() for s in message.vertex_stamps], dtype='<u8').tobytes()).hexdigest(),
                    triangles_uint32_le_sha256=hashlib.sha256(np.asarray([t.vertex_indices for t in message.triangles], dtype='<u4').tobytes()).hexdigest(),
                    optimizer_owner=self.approved['proof']['round']['owner'],
                    completed_round_serial=self.approved['proof']['round']['serial'],
                    final_native_weight_batch_sha256=self.approved['proof']['weights']['sha256'],
                    native_optimized_graph_sha256=self.approved['graph_sha256'],
                    submitted_body_path_sha256=self.approved['path_sha256'],
                    deformation_proof_file=current['prefix'].name+'_native_deformation_proof.json',
                    native_after_mesh_file=current['prefix'].name+'_native_after.rosbin',
                    completed_round_proof=self.approved['proof'],
                    native_deformation_proof=copy.deepcopy(self.eligible),
                    accepted_wall=time.time(), immutable_until_next_accepted_transaction=True,
                    exact_shared_optimizer_round_id_available=False)
                current['prefix'].with_name(current['prefix'].name+'_accepted_mesh_receipt.json').write_text(
                    json.dumps(self.accepted_joint_mesh_receipt, indent=2), encoding='utf-8')
                newly_accepted = True
                self.counts['proven_native_deformation_transactions'] += 1; self.transaction = None
            elif self.eligible is None:
                self.counts['unanchored_native_bootstrap_meshes'] += 1; return
        publish_preserving_cached_message(self.pub['native_mesh'], message)
        color = 0x34D8ED if self.a.robot_id == 0 else 0xFFA04B
        vertices = self.make_cloud(xyz, color, message.header.stamp, self.output_frame)
        self.pub['vertices'].publish(vertices)
        if newly_accepted:
            publish_preserving_cached_message(self.pub['accepted_mesh'], message)
            publish_preserving_cached_message(self.pub['accepted_vertices'], vertices)
        self.counts['native_pgmo_meshes'] += 1
        (self.evidence/'latest_native_mesh.rosbin').write_bytes(mesh_bytes())
        self.event('native_pgmo_mesh', sha256=mesh_sha(), vertices=len(xyz), triangles=len(message.triangles),
            latest_submitted_path_sha256=self.latest_submitted_body_path_sha256,
            native_deformation_proof=self.eligible,
            exact_optimizer_mesh_generation_proven=False)

    def tick(self, _):
        try:
            with self.lock:
                if self.fatal is not None or rospy.is_shutdown(): return
                self.drain_native_inputs()
                self.drain_replaceable_inputs()
                if not self.gauge_checked and 0 in self.keys.nodes:
                    stamp = self.keys.nodes[0].header.stamp.to_nsec(); at_zero = self.history.at(stamp)
                    if at_zero is not None:
                        delta = np.linalg.inv(at_zero) @ pose_matrix(self.keys.nodes[0].pose)
                        angle = np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1., 1.))
                        if np.linalg.norm(delta[:3, 3]) > .02 or angle > .02:
                            raise ValueError('Native VIO odometry/key-zero graph gauges differ; refuse silent frame relabel')
                        self.gauge_checked = True; self.event('native_local_gauge_verified', key0_stamp_ns=stamp)
                self.publish_graphs(); self.publish_sensor(); self.advance_optimized_candidates(); self.approve_path(); self.transaction_tick()
                if (self.transaction is None and self.latest_full_mesh is not None and self.native_body_metadata_ack
                        and self.graph_sent >= 0 and not self.mesh_graph_queue and self.pub['full'].get_num_connections()):
                    signature = self.latest_full_mesh_sha
                    forced = self.remesh_after is not None and time.time() >= self.remesh_after
                    if signature != getattr(self, 'last_full_sha', None) or forced:
                        publish_preserving_cached_message(self.pub['full'], self.latest_full_mesh)
                        self.last_full_sha = signature
                        self.remesh_after = None; self.counts['full_meshes_forwarded'] += 1
                report = dict(run_id=self.a.run_id, cutoff_ns=self.cutoff_ns, robot_id=self.a.robot_id, mode=self.a.mode,
                    wall=time.time(), native_local_gauge_verified=self.gauge_checked, local_keyframes=len(self.keys.nodes),
                    input_mode='sha256_pinned_recorded_native_VIO_and_sensors' if self.a.recorded_native_bag else 'live_native_VIO_and_sensors',
                    recorded_native_bag_sha256=self.a.recorded_native_bag_sha256,
                    native_vio_maximum_interpolated_gap_s=self.history.maximum_interpolated_gap_ns*1e-9,
                    native_vio_maximum_rejected_gap_s=self.history.maximum_rejected_gap_ns*1e-9,
                    maximum_permitted_native_vio_gap_s=self.a.max_odom_gap,
                    sensor_pose_interpolation=('actual gyro curve constrained by unchanged native VIO orientation endpoints; linear translation'
                        if self.gyros is not None else 'native VIO shortest-arc quaternion and linear translation; explicit SLERP comparison mode'),
                    gyro_sample_convention=self.gyros.sample_convention if self.gyros is not None else None,
                    gyro_maximum_sample_gap_s=self.gyros.maximum_sample_gap_ns*1e-9 if self.gyros is not None else None,
                    gyro_maximum_used_gap_s=self.gyros.maximum_used_gap_ns*1e-9 if self.gyros is not None else None,
                    gyro_maximum_permitted_gap_s=self.gyros.maximum_gap_ns*1e-9 if self.gyros is not None else None,
                    gyro_maximum_endpoint_residual_rad=self.gyros.maximum_endpoint_residual_rad if self.gyros is not None else None,
                    graph_forwarded_through_key=self.graph_sent, counts=dict(self.counts), fatal_error=self.fatal,
                    native_original_priority_queue_length=len(self.native_inputs),
                    native_original_priority_queue_max_length=self.native_input_max_queue,
                    native_original_max_queue_delay_s=self.native_input_max_delay,
                    native_first_body_metadata_acknowledged=self.native_body_metadata_ack,
                    native_body_metadata_ack_optimized_key_zero_seen=self.native_body_metadata_ack_has_optimized_key_zero,
                    native_body_metadata_ack_basis='fresh native pose_graph callback completion after original key0 input; optimized key0 stamp checked whenever present',
                    native_control_max_queue_delay_s=self.native_control_max_delay,
                    actual_gyro_max_queue_delay_s=self.native_imu_max_delay,
                    replaceable_full_mesh_overwrites=dict(self.replaceable_overwrites),
                    optimized_candidates_waiting=len(self.optimized_candidates),
                    optimized_candidate_wait_limit_s=self.optimized_candidate_wait,
                    optimized_candidate_missing_original_counts=[len(self.keys.missing_originals(c['nodes'])) for c in self.optimized_candidates],
                    approved_path=self.approved, local_frame=self.local_frame, output_frame=self.output_frame,
                    raw_local_vertices_topic=self.base+'/native_dense/raw_local_vertices',
                    raw_local_vertices_frame=self.local_frame,
                    raw_local_vertices_source='native_mesh_frontend/full_mesh; exact XYZ, no PGMO deformation',
                    local_tsdf_continues_during_pgmo_transaction=True,
                    sensor_stream_in_independent_process=self.a.external_sensor_stream,
                    native_deformation_eligible=self.eligible is not None if self.a.mode == 'joint' else None,
                    native_deformation_proof=self.eligible,
                    accepted_joint_mesh_receipt=self.accepted_joint_mesh_receipt,
                    latest_submitted_body_path_sha256=self.latest_submitted_body_path_sha256,
                    ordinary_native_mesh_stream_has_immutable_generation_receipt=False,
                    mesh_transaction_phase=self.transaction['phase'] if self.transaction else None,
                    mesh_transaction_phase_timeout_s=self.a.mesh_phase_timeout,
                    native_map_producers=['voxblox_ros/tsdf_server', 'kimera_pgmo_ros/voxblox_mesh_frontend_node', 'kimera_pgmo_ros/kimera_pgmo_node'],
                    adapter_not_estimator=True, consumes_ground_truth=False, consumes_lidar_or_custom_mapper=False,
                    mesh_owner_workaround='local frontend default0 -> configured robot; no geometry/key changes',
                    exact_optimizer_mesh_generation_proven=False,
                    acceptance='Native mesh deformation must be separately measured before claiming S3/S4 complete')
                self.pub['status'].publish(String(data=json.dumps(report, allow_nan=False)))
                (self.evidence/'status.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
        except Exception as error: self.fail(error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--robot-name', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--mode', choices=('local', 'joint'), default='local')
    parser.add_argument('--global-frame', default='kimera_world')
    parser.add_argument('--cloud-rate', type=float, default=2.)
    parser.add_argument('--pixel-stride', type=int, default=3)
    parser.add_argument('--max-depth', type=float, default=8.)
    parser.add_argument('--max-odom-gap', type=float, default=.6,
                        help='Pinned native odometry is keyframe-rate (~2Hz); measured 60s fixture max gap .53256s')
    parser.add_argument('--sensor-rotation-interpolation', choices=('gyro_endpoint', 'slerp'), default='gyro_endpoint',
                        help='Actual gyro curve between native keyframes; slerp is only an explicit comparison mode')
    parser.add_argument('--gyro-sample-convention', choices=('left_endpoint_interval_equivalent',),
                        default='left_endpoint_interval_equivalent',
                        help='Actual body IMU interval contract, matching the simulator and native Kimera integration')
    parser.add_argument('--max-gyro-gap', type=float, default=.05,
                        help='Omit RGB-D frames crossing larger actual IMU gaps; never silently use slerp')
    parser.add_argument('--weight-updates', type=int, default=25)
    parser.add_argument('--mesh-phase-timeout', type=float, default=30.,
                        help='Finite timeout PER native readiness/baseline/echo/deformation phase, not a total fixed freeze timeout')
    parser.add_argument('--require-cross', action='store_true')
    parser.add_argument('--external-sensor-stream', action='store_true',
                        help='Backend-only role: do not subscribe raw RGB-D/IMU or publish TSDF point clouds')
    parser.add_argument('--recorded-native-bag', type=Path,
                        help='Explicit offline native-input fixture; live source authentication remains default')
    parser.add_argument('--recorded-native-bag-sha256')
    parser.add_argument('--replay-caller', default='/native_dense_replay')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.robot_name != 'sim_%d' % args.robot_id: parser.error('Expected matching sim_N name')
    if args.cutoff <= 0 or args.cloud_rate <= 0 or args.pixel_stride < 1: parser.error('Invalid epoch/sampling')
    if args.mesh_phase_timeout <= 0 or args.max_odom_gap <= 0 or args.max_gyro_gap <= 0 or args.weight_updates <= 0:
        parser.error('Native phase timeout, odometry bracket limit and weight-update count must be positive')
    if args.recorded_native_bag:
        if args.mode != 'local': parser.error('Recorded native-input mode is only for isolated local S2 acceptance')
        if not args.replay_caller.startswith('/native_dense_replay'):
            parser.error('Explicit recorded input requires a dedicated /native_dense_replay caller')
        if not args.recorded_native_bag_sha256: parser.error('Recorded native bag SHA256 is required')
        checksum = hashlib.sha256()
        with args.recorded_native_bag.open('rb') as source:
            for block in iter(lambda: source.read(2**20), b''): checksum.update(block)
        if checksum.hexdigest() != args.recorded_native_bag_sha256:
            parser.error('Recorded native-input bag checksum differs from declared fixture')
    elif args.recorded_native_bag_sha256: parser.error('Bag path required with a recorded-input checksum')
    rospy.init_node('native_dense_bridge', anonymous=False)
    node = NativeDenseBridge(args); rospy.spin()
    if node.fatal: raise SystemExit(2)


if __name__ == '__main__':
    import rospy
    import tf2_ros
    from dpgo_ros.msg import Command, RelativeMeasurementWeights
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from kimera_pgmo_msgs.msg import KimeraPgmoMesh
    from nav_msgs.msg import Odometry, Path as RosPath
    from pose_graph_tools_msgs.msg import PoseGraph
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
    from std_msgs.msg import Header, String
    main()
