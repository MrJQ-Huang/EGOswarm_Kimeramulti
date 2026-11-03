#!/usr/bin/env python3
"""Independent local calibrated RGB-D -> native Voxblox input producer.

No graph optimization, mesh serialization, LCD callback, simulator pose, or
remote host participates in this process. Native VIO endpoints and the actual
body gyro retain the same interpolation contract as native_dense_bridge.py.
The bounded input cache omits stale frames explicitly; it never changes their
time stamps to make old geometry appear current. Geometry conversion and ROS
publication execute outside the small sensor-history lock.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import threading
import time

import numpy as np

from native_dense_bridge import (NativeOdometryGap, PoseHistory, frame,
    optical_points, require_source, require_stamp)
from native_imu_pose_interpolation import (GyroCoverageError,
    GyroRotationHistory, matrix_quaternion)
from native_vio_graph_adapter import SIM_T_BODY_CAMERA, pose_matrix


class LocalSensorState:
    """Small independently testable history/cache; caller holds its lock."""
    def __init__(self, args):
        self.a = args
        self.cutoff_ns = round(args.cutoff*1e9)
        self.local_frame = args.robot_name+'/vio_odom'
        self.optical_frame = args.robot_name+'/native_dense_optical'
        self.gyros = GyroRotationHistory(round(args.max_gyro_gap*1e9))
        self.history = PoseHistory(round(args.max_odom_gap*1e9), gyros=self.gyros)
        self.camera = None
        self.key_zero = None
        self.gauge_checked = False
        self.pending = {}
        self.last_rgbd_ns = 0
        self.counts = Counter()
        self.maximum_pending = 0
        self.maximum_input_age_s = Counter()
        self.last_received_wall = {}

    def ingest(self, kind, message, wall):
        if kind == 'key_zero':
            if not message.nodes: return
            # Native incremental graph headers need not carry a sensor stamp;
            # the original nodes are the authority, as in KeyframeLedger.
            stamp = max(require_stamp(node.header.stamp, self.cutoff_ns) for node in message.nodes)
        else:
            stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        self.maximum_input_age_s[kind] = max(self.maximum_input_age_s[kind], wall-stamp*1e-9)
        self.last_received_wall[kind] = wall
        if kind == 'odom':
            if (frame(message.header.frame_id) != self.local_frame or
                    frame(message.child_frame_id) != self.a.robot_name+'/imu'):
                raise ValueError('Unexpected native body odometry frames')
            self.history.put(stamp, message.pose.pose)
        elif kind == 'imu':
            if frame(message.header.frame_id) != self.a.robot_name+'/imu':
                raise ValueError('Unexpected actual body IMU frame')
            omega = message.angular_velocity
            self.gyros.put(stamp, [omega.x, omega.y, omega.z])
        elif kind == 'key_zero':
            if frame(message.header.frame_id) != self.a.robot_name+'/map':
                raise ValueError('Unexpected native graph frame')
            for node in message.nodes:
                if int(node.robot_id) != self.a.robot_id:
                    raise ValueError('Foreign native original graph node')
                if int(node.key) == 0:
                    require_stamp(node.header.stamp, self.cutoff_ns)
                    if self.key_zero is None: self.key_zero = copy.deepcopy(node)
                    elif (node.header.stamp.to_nsec() != self.key_zero.header.stamp.to_nsec() or
                            not np.allclose(pose_matrix(node.pose), pose_matrix(self.key_zero.pose), atol=1e-10)):
                        raise ValueError('Native key zero changed')
        elif kind == 'camera':
            if frame(message.header.frame_id) != self.a.robot_name+'/camera_optical':
                raise ValueError('Unexpected calibrated optical frame')
            if self.camera is not None and (message.K != self.camera.K or
                    message.width != self.camera.width or message.height != self.camera.height):
                raise ValueError('Camera calibration changed within epoch')
            self.camera = message
        else:
            if kind not in ('rgb', 'depth'): raise ValueError('Unexpected sensor input')
            if frame(message.header.frame_id) != self.a.robot_name+'/camera_optical':
                raise ValueError('Unexpected RGB-D frame')
            if kind == 'depth' and message.encoding != '32FC1':
                raise ValueError('Depth must be optical-Z float32 metres')
            self.pending.setdefault(stamp, {})[kind] = message
            while len(self.pending) > self.a.max_pending_rgbd:
                del self.pending[min(self.pending)]
                self.counts['rgbd_omitted_queue_limit'] += 1
            self.maximum_pending = max(self.maximum_pending, len(self.pending))
        self.counts[kind+'_received'] += 1

    def choose(self, wall):
        if not self.gauge_checked and self.key_zero is not None:
            at_zero = self.history.at(self.key_zero.header.stamp.to_nsec())
            if at_zero is not None:
                delta = np.linalg.inv(at_zero) @ pose_matrix(self.key_zero.pose)
                angle = np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1., 1.))
                if np.linalg.norm(delta[:3, 3]) > .02 or angle > .02:
                    raise ValueError('Native VIO odometry/key-zero graph gauges differ')
                self.gauge_checked = True
        for stamp in sorted(list(self.pending)):
            age = wall-stamp*1e-9
            if age > self.a.max_rgbd_age:
                del self.pending[stamp]; self.counts['rgbd_omitted_age_limit'] += 1
                continue
            if stamp <= self.last_rgbd_ns or (stamp-self.last_rgbd_ns)*1e-9 < 1./self.a.cloud_rate:
                del self.pending[stamp]; self.counts['rgbd_omitted_configured_rate'] += 1
                continue
            if not self.gauge_checked or self.camera is None: continue
            pair = self.pending[stamp]
            if set(pair) != {'rgb', 'depth'}: continue
            try: body = self.history.at(stamp)
            except NativeOdometryGap:
                del self.pending[stamp]; self.counts['rgbd_omitted_native_odom_gap'] += 1
                continue
            except GyroCoverageError as error:
                del self.pending[stamp]; self.counts['rgbd_omitted_actual_gyro_'+error.kind] += 1
                continue
            if body is None:
                if self.history.values and stamp < min(self.history.values):
                    del self.pending[stamp]; self.counts['rgbd_omitted_no_native_lower_bracket'] += 1
                continue
            # Messages are immutable after receipt. Detach references and copy
            # only the small transform; RGB-D conversion happens lock-free.
            del self.pending[stamp]
            self.last_rgbd_ns = stamp
            return stamp, pair, self.camera, body @ SIM_T_BODY_CAMERA
        return None


def packed_cloud(xyz, colors, stamp, optical_frame):
    data = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    if len(xyz): data['x'], data['y'], data['z'] = xyz.T
    data['rgb'] = colors
    fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32 if i < 3 else PointField.UINT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
    return PointCloud2(header=Header(stamp=stamp, frame_id=optical_frame), height=1, width=len(xyz),
        fields=fields, is_bigendian=False, point_step=16, row_step=16*len(xyz), data=data.tobytes(), is_dense=True)


class LocalSensorBridge:
    def __init__(self, args):
        self.a = args; self.state = LocalSensorState(args)
        self.lock = threading.Lock(); self.fatal = None
        self.timings = Counter(); self.samples = Counter(); self.last_report = 0.
        args.output_directory.mkdir(parents=True, exist_ok=True)
        self.tf = tf2_ros.TransformBroadcaster()
        self.points = rospy.Publisher('/'+args.robot_name+'/native_dense/rgbd_points', PointCloud2, queue_size=1)
        self.status = rospy.Publisher('/'+args.robot_name+'/native_dense/local_sensor_status', String, queue_size=1, latch=True)
        native = '/'+args.robot_name+'/kimera_vio_ros/kimera_vio_ros_node'
        sensor = '/'+args.robot_name+'_multimodal_sensors'
        base = '/'+args.robot_name
        topics = [('odom', base+'/kimera_vio_ros/odometry', Odometry, native, 40),
            ('key_zero', base+'/kimera_vio_ros/pose_graph_incremental', PoseGraph, native, None),
            ('imu', base+'/kimera/imu', Imu, sensor, 300),
            ('camera', base+'/kimera/rgb/camera_info', CameraInfo, sensor, 2),
            ('rgb', base+'/kimera/rgb/image_raw', Image, sensor, 8),
            ('depth', base+'/kimera/depth/image_raw', Image, sensor, 8)]
        self.subs = [rospy.Subscriber(topic, msg_type,
            lambda m, k=kind, expected=owner: self.receive(k, expected, m),
            queue_size=queue, buff_size=2**26, tcp_nodelay=True)
            for kind, topic, msg_type, owner, queue in topics]
        self.timer = rospy.Timer(rospy.Duration(.01), self.tick)

    def fail(self, error):
        if str(error) == 'publish() to a closed topic' and rospy.is_shutdown(): return
        if self.fatal is not None: return
        self.fatal = str(error)
        (self.a.output_directory/'fatal.json').write_text(json.dumps(dict(error=self.fatal,
            wall=time.time(), run_id=self.a.run_id), indent=2), encoding='utf-8')
        rospy.logfatal('Independent local sensor bridge: %s', error)
        rospy.signal_shutdown(str(error))

    def receive(self, kind, owner, message):
        try:
            require_source(message, owner)
            began = time.monotonic()
            with self.lock:
                acquired = time.monotonic()
                self.state.ingest(kind, message, time.time())
                finished = time.monotonic()
                self.timings['callback_lock_wait_max_s'] = max(self.timings['callback_lock_wait_max_s'], acquired-began)
                self.timings['callback_lock_hold_max_s'] = max(self.timings['callback_lock_hold_max_s'], finished-acquired)
        except Exception as error: self.fail(error)

    def tick(self, _):
        try:
            if self.fatal is not None or rospy.is_shutdown(): return
            began = time.monotonic()
            with self.lock:
                acquired = time.monotonic()
                chosen = self.state.choose(time.time()) if self.points.get_num_connections() else None
                finished = time.monotonic()
                self.timings['choose_lock_wait_max_s'] = max(self.timings['choose_lock_wait_max_s'], acquired-began)
                self.timings['choose_lock_hold_max_s'] = max(self.timings['choose_lock_hold_max_s'], finished-acquired)
            if chosen is not None:
                stamp, pair, calibration, camera = chosen
                conversion_start = time.monotonic()
                xyz, colors = optical_points(pair['rgb'], pair['depth'], calibration,
                    self.a.pixel_stride, .3, self.a.max_depth)
                cloud = packed_cloud(xyz, colors, pair['rgb'].header.stamp, self.state.optical_frame)
                tf = TransformStamped(header=Header(stamp=pair['rgb'].header.stamp,
                    frame_id=self.state.local_frame), child_frame_id=self.state.optical_frame)
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = camera[:3, 3]
                q = matrix_quaternion(camera[:3, :3])
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
                conversion_end = time.monotonic()
                age = time.time()-stamp*1e-9
                with self.lock:
                    self.timings['conversion_max_s'] = max(self.timings['conversion_max_s'], conversion_end-conversion_start)
                    self.timings['publish_input_age_max_s'] = max(self.timings['publish_input_age_max_s'], age)
                if age <= self.a.max_rgbd_age:
                    self.tf.sendTransform(tf); self.points.publish(cloud)
                    with self.lock:
                        self.state.counts['sensor_clouds'] += 1
                        self.samples['cloud_vertices_last'] = len(xyz)
                        self.samples['cloud_stamp_last_ns'] = stamp
                else:
                    with self.lock: self.state.counts['rgbd_omitted_conversion_age_limit'] += 1
            if time.monotonic()-self.last_report >= 1.:
                self.last_report = time.monotonic()
                with self.lock:
                    report = dict(run_id=self.a.run_id, robot_id=self.a.robot_id, cutoff_ns=self.state.cutoff_ns,
                        wall=time.time(), counts=dict(self.state.counts), timings=dict(self.timings), samples=dict(self.samples),
                        maximum_input_age_s=dict(self.state.maximum_input_age_s),
                        last_received_wall=dict(self.state.last_received_wall), pending_rgbd=len(self.state.pending),
                        maximum_pending_rgbd=self.state.maximum_pending, max_pending_rgbd=self.a.max_pending_rgbd,
                        max_rgbd_age_s=self.a.max_rgbd_age, cloud_rate_hz=self.a.cloud_rate,
                        native_local_gauge_verified=self.state.gauge_checked,
                        gyro_maximum_sample_gap_s=self.state.gyros.maximum_sample_gap_ns*1e-9,
                        gyro_maximum_used_gap_s=self.state.gyros.maximum_used_gap_ns*1e-9,
                        native_vio_maximum_interpolated_gap_s=self.state.history.maximum_interpolated_gap_ns*1e-9,
                        fatal_error=self.fatal, independent_process=True, consumes_ground_truth=False,
                        consumes_global_optimizer=False, consumes_mesh=False, stamps_preserved=True,
                        sensor_pose_interpolation='actual gyro curve constrained by unchanged native VIO endpoints; linear translation')
                serialized = json.dumps(report, allow_nan=False)
                self.status.publish(String(data=serialized))
                (self.a.output_directory/'status.json').write_text(serialized, encoding='utf-8')
        except Exception as error: self.fail(error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--robot-name', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--cloud-rate', type=float, default=2.)
    parser.add_argument('--pixel-stride', type=int, default=3)
    parser.add_argument('--max-depth', type=float, default=8.)
    parser.add_argument('--max-odom-gap', type=float, default=.6)
    parser.add_argument('--max-gyro-gap', type=float, default=.05)
    parser.add_argument('--max-rgbd-age', type=float, default=2.)
    parser.add_argument('--max-pending-rgbd', type=int, default=32)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.robot_name != 'sim_%d' % args.robot_id: parser.error('Expected matching sim_N name')
    if min(args.cutoff, args.cloud_rate, args.max_depth, args.max_odom_gap, args.max_gyro_gap,
            args.max_rgbd_age, args.max_pending_rgbd, args.pixel_stride) <= 0:
        parser.error('Epoch, rates, ages, queues and sampling arguments must be positive')
    rospy.init_node('native_local_sensor_bridge', anonymous=False)
    node = LocalSensorBridge(args); rospy.spin()
    if node.fatal: raise SystemExit(2)


if __name__ == '__main__':
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from pose_graph_tools_msgs.msg import PoseGraph
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
    from std_msgs.msg import Header, String
    main()
