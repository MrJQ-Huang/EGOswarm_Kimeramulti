#!/usr/bin/env python3
"""Independent RGB-D, instantaneous 360-degree LiDAR and IMU simulator.

Run one process on each body host. Ground-truth pose is solely a renderer input;
its explicitly named evaluation output must never be a Kimera estimator input.
The acceleration convention is audited specifically for EGO poscmd_2_odom.cpp:
that simulator puts WORLD ACCELERATION into odom.twist.angular (not gyro!).
With --body-imu, inertial synthesis is bypassed and the actual body IMU is relayed.
"""
import argparse
import copy
import json
from pathlib import Path
import threading
import time
import numpy as np
from raycast_geometry import RayScene, R_BC, rotation_xyzw, body_rate, camera_rays, lidar_rays


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--scene', type=Path, default=Path(__file__).resolve().parent.parent/'complex_scene.json')
    parser.add_argument('--width', type=int, default=320); parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--fx', type=float, default=160.); parser.add_argument('--fy', type=float, default=160.)
    parser.add_argument('--camera-rate', type=float, default=10.); parser.add_argument('--lidar-rate', type=float, default=10.)
    parser.add_argument('--camera-range', type=float, default=12.); parser.add_argument('--lidar-range', type=float, default=8.)
    parser.add_argument('--lidar-channels', type=int, default=16); parser.add_argument('--lidar-angles', type=int, default=180)
    parser.add_argument('--warmup-seconds', type=float, default=4.)
    parser.add_argument('--acceleration-source', choices=('fake-odom', 'position-command'), default='fake-odom')
    parser.add_argument('--body-imu', nargs='?', const='auto', default=None,
                        help='Relay an actual body IMU unchanged instead of differentiating odometry. No value uses /sim_ID/body/imu.')
    parser.add_argument('--publish-planner-cloud', action='store_true', help='Also send occluded LiDAR hits in world frame to EGO local cloud topic.')
    parser.add_argument('--print-calibration', action='store_true')
    parser.add_argument('--self-test', action='store_true'); parser.add_argument('--output-directory', type=Path)
    return parser.parse_args()


def calibration(args):
    cx, cy = (args.width-1)/2., (args.height-1)/2.
    transform = np.eye(4); transform[:3, :3] = R_BC
    return {'schema_version': 1, 'world': 'ROS ENU with gravity [0,0,-9.8] m/s^2',
            'body': 'FLU; colocated IMU and camera center; optical x right, y down, z forward',
            'image_size': [args.width, args.height], 'K': [[args.fx, 0., cx], [0., args.fy, cy], [0., 0., 1.]],
            'T_BC': transform.tolist(), 'T_BI': np.eye(4).tolist(), 'T_BL': np.eye(4).tolist(),
            'distortion_model': 'plumb_bob', 'D': [0., 0., 0., 0., 0.],
            'camera_rate_requested_hz': args.camera_rate, 'imu_rate_hz': 100., 'lidar_rate_requested_hz': args.lidar_rate,
            'camera_max_range_m': args.camera_range, 'lidar_max_range_m': args.lidar_range,
            'depth_encoding': '32FC1 meters along optical +Z, zero invalid',
            'exposure': 'instantaneous global shutter at exact odom header stamp; no motion blur',
            'lidar': {'channels': args.lidar_channels, 'azimuth_angles': args.lidar_angles,
                      'vertical_fov_degrees': [-25., 15.], 'scan_model': 'instantaneous, no spinning distortion'},
            'imu': {'gravity_magnitude_m_s2': 9.8, 'noise': 'deterministic ideal baseline, no added noise or bias',
                    'source': (('/sim_%d/body/imu'%args.drone_id if args.body_imu == 'auto' else args.body_imu)
                               if args.body_imu else 'legacy fake-odom derivative'),
                    'acceleration': 'unchanged actual body IMU specific force' if args.body_imu else 'R_WB.T @ (a_world - [0,0,-9.8])',
                    'angular_velocity': 'unchanged analytic actual body angular velocity' if args.body_imu else 'quaternion logarithm of consecutive body rotations / header dt'},
            'body_camera_optical_frame': 'sim_%d/camera_optical'%args.drone_id,
            'imu_frame': 'sim_%d/imu'%args.drone_id, 'lidar_frame': 'sim_%d/lidar'%args.drone_id}


class Renderer:
    def __init__(self, args):
        self.args = args
        self.scene = RayScene(json.loads(args.scene.read_text(encoding='utf-8')))
        self.camera, self.optical_z_per_m = camera_rays(args.width, args.height, args.fx, args.fy,
                                                       (args.width-1)/2., (args.height-1)/2.)
        self.lidar = lidar_rays(args.lidar_channels, args.lidar_angles)

    def camera_frame(self, position, rotation):
        directions = self.camera@(rotation@R_BC).T
        distances, points, normals, ids = self.scene.cast(position, directions, minimum=.1, maximum=self.args.camera_range)
        valid = np.isfinite(distances)
        depth = np.where(valid, distances*self.optical_z_per_m, 0.).astype('<f4')
        rgb = self.scene.colors(points, normals, ids)
        size = (self.args.height, self.args.width)
        return rgb.reshape(size+(3,)), depth.reshape(size), (valid.astype(np.uint8)*255).reshape(size)

    def lidar_frame(self, position, rotation):
        distance, points, _, ids = self.scene.cast(position, self.lidar@rotation.T, minimum=.1, maximum=self.args.lidar_range)
        valid = ids >= 0
        local = self.lidar[valid]*distance[valid, None]
        return local.astype('<f4'), points[valid].astype('<f4')


def self_test(args):
    renderer = Renderer(args)
    # The nearer opaque wall must mask the farther wall along the same ray.
    toy = {'bounds': {'x': [-10, 10], 'y': [-10, 10]}, 'obstacles': [
        {'type': 'box', 'center': [2, 0, 1], 'size': [.2, 2, 2]},
        {'type': 'box', 'center': [4, 0, 1], 'size': [.2, 2, 2]}]}
    distance, _, _, ids = RayScene(toy).cast([0, 0, 1], [[1., 0, 0], [0, 0, -1.]])
    assert np.allclose(distance, [1.9, 1.]) and ids.tolist() == [1, 0]
    near_toy = copy.deepcopy(toy); near_toy['obstacles'][0] = {'type': 'box', 'center': [.08, 0, 1], 'size': [.02, 2, 2]}
    near_distance, _, _, near_ids = RayScene(near_toy).cast([0, 0, 1], [[1., 0, 0]], minimum=.1)
    assert np.isinf(near_distance[0]) and near_ids[0] == -1
    cylinder_toy = copy.deepcopy(toy); cylinder_toy['obstacles'] = [{'type': 'cylinder', 'center': [2, 0], 'radius': .5, 'height': 2.}]
    cylinder_distance, _, _, cylinder_ids = RayScene(cylinder_toy).cast([0, 0, 1], [[1., 0, 0]])
    cap_distance, _, _, cap_ids = RayScene(cylinder_toy).cast([2, 0, 3], [[0., 0, -1.]])
    assert np.allclose(cylinder_distance, [1.5]) and cylinder_ids[0] == 1
    assert np.allclose(cap_distance, [1.]) and cap_ids[0] == 1
    assert np.allclose(R_BC@[0, 0, 1], [1, 0, 0])
    assert np.allclose(R_BC@[1, 0, 0], [0, -1, 0])
    assert np.allclose(body_rate([0, 0, 0, 1], [0, 0, np.sin(.005), np.cos(.005)], .01), [0, 0, 1.])
    scene = renderer.scene.scene; pose = scene['scenarios']['oppose']['starts'][0]
    start = time.perf_counter(); frames = [renderer.camera_frame(pose, np.eye(3)) for _ in range(3)]
    elapsed = (time.perf_counter()-start)/3
    rgb, depth, mask = frames[-1]; local, world = renderer.lidar_frame(pose, np.eye(3))
    assert np.array_equal(frames[0][0], rgb) and np.all(np.isfinite(depth))
    assert np.allclose(local+np.asarray(pose), world, atol=2e-6)
    assert np.max(np.linalg.norm(local, axis=1)) <= args.lidar_range+1e-5
    # Optical Z differs from Euclidean range away from the central pixel.
    assert renderer.optical_z_per_m[0] < .9
    native_comparison = None
    if renderer.scene.kernel:
        fallback = RayScene(scene, use_native=False)
        native_times = []; fallback_times = []; maximum_error = 0.
        for position in ([-8., -.8, 1.], [-3., 2., 1.], [6., -2., 1.6]):
            directions = renderer.camera@R_BC.T
            tick = time.perf_counter(); native = renderer.scene.cast(position, directions); native_times.append(time.perf_counter()-tick)
            tick = time.perf_counter(); python = fallback.cast(position, directions); fallback_times.append(time.perf_counter()-tick)
            assert np.array_equal(native[3], python[3]), 'Native and NumPy object IDs differ'
            valid = native[3] >= 0
            maximum_error = max(maximum_error, float(np.max(np.abs(native[0][valid]-python[0][valid]))))
        assert maximum_error < 2e-4
        native_comparison = {'views': 3, 'same_hit_ids': True, 'maximum_range_error_m': maximum_error,
                             'native_cast_mean_seconds': float(np.mean(native_times)), 'numpy_cast_mean_seconds': float(np.mean(fallback_times))}
    report = {'passed': True, 'raycast_backend': renderer.scene.backend, 'native_numpy_comparison': native_comparison,
              'camera_render_mean_seconds_local': elapsed,
              'camera_measured_fraction': float(np.mean(mask > 0)), 'lidar_return_count': len(local),
              'tests': ['nearest-surface occlusion', 'near clipping preserves occlusion', 'cylinder side and cap', 'floor range', 'FLU-to-optical axes', 'quaternion body gyro',
                        'stationary RGB repeatability', 'local/world LiDAR consistency', 'LiDAR max range', 'depth Z versus range'],
              'calibration': calibration(args)}
    if args.output_directory:
        args.output_directory.mkdir(parents=True, exist_ok=True)
        np.save(args.output_directory/'sample_depth_m.npy', depth)
        np.save(args.output_directory/'sample_lidar_local_xyz.npy', local)
        try:
            import cv2
            cv2.imwrite(str(args.output_directory/'sample_rgb.png'), rgb[:, :, ::-1])
            visualization = (255*np.clip(depth/args.camera_range, 0., 1.)).astype(np.uint8)
            cv2.imwrite(str(args.output_directory/'sample_depth.png'), visualization)
        except ImportError:
            with (args.output_directory/'sample_rgb.ppm').open('wb') as stream:
                stream.write(('P6\n%d %d\n255\n'%(args.width, args.height)).encode()); stream.write(rgb.tobytes())
        (args.output_directory/'calibration.json').write_text(json.dumps(calibration(args), indent=2), encoding='utf-8')
        (args.output_directory/'self_test.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


def run_ros(args):
    import rospy
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2, PointField
    from std_msgs.msg import Header, Bool

    rospy.init_node('sim_%d_multimodal_sensors'%args.drone_id)
    renderer = Renderer(args); prefix = '/sim_%d'%args.drone_id
    rospy.loginfo('Multimodal raycaster backend: %s', renderer.scene.backend)
    body = 'sim_%d/imu'%args.drone_id; optical = 'sim_%d/camera_optical'%args.drone_id
    lidar_frame = 'sim_%d/lidar'%args.drone_id
    publish = {key: rospy.Publisher(prefix+topic, kind, queue_size=queue, latch=latch)
               for key, topic, kind, queue, latch in [
                   ('rgb', '/kimera/rgb/image_raw', Image, 2, False),
                   ('depth', '/kimera/depth/image_raw', Image, 2, False),
                   ('mask', '/kimera/depth/measured_mask', Image, 2, False),
                   ('info', '/kimera/rgb/camera_info', CameraInfo, 2, False),
                   ('depth_info', '/kimera/depth/camera_info', CameraInfo, 2, False),
                   ('imu', '/kimera/imu', Imu, 400, False),
                   ('lidar', '/lidar/points', PointCloud2, 1, False),
                   ('truth', '/ground_truth/odometry', Odometry, 2, False),
                   ('ready', '/sensors/ready', Bool, 1, True)]}
    planner = rospy.Publisher('/drone_%d_pcl_render_node/cloud'%args.drone_id, PointCloud2, queue_size=1) if args.publish_planner_cloud else None
    publish['ready'].publish(Bool(data=False))
    lock = threading.Lock(); state = {'odom': None, 'previous': None, 'command': None, 'still_since': None, 'ready': False,
                                      'actual_imu': None, 'actual_imu_stamp_ns': 0}
    rospy.set_param(prefix+'/sensors/calibration', calibration(args))

    def command_callback(msg):
        with lock: state['command'] = msg

    def actual_imu_callback(msg):
        if msg.header.frame_id.lstrip('/') != body:
            rospy.logerr_throttle(2., 'Body IMU frame must be %s; received %s', body, msg.header.frame_id)
            return
        values = [getattr(vector, axis) for vector in (msg.linear_acceleration, msg.angular_velocity) for axis in ('x', 'y', 'z')]
        if not np.isfinite(values).all():
            rospy.logerr_throttle(2., 'Ignoring nonfinite actual body IMU')
            return
        stamp_ns = msg.header.stamp.to_nsec()
        with lock:
            if stamp_ns <= state['actual_imu_stamp_ns']:
                rospy.logwarn_throttle(2., 'Ignoring duplicate/backward actual IMU timestamp')
                return
            state['actual_imu'] = msg; state['actual_imu_stamp_ns'] = stamp_ns
        # Preserve physical measurements, covariance and exact original stamp.
        # An IMU orientation estimate is intentionally not supplied to VIO.
        if msg.orientation_covariance[0] != -1.:
            msg = copy.deepcopy(msg); msg.orientation_covariance[0] = -1.
            msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.
        publish['imu'].publish(msg)

    def update_ready(msg, omega):
        stamp = msg.header.stamp.to_sec(); v = msg.twist.twist.linear
        stationary = np.linalg.norm([v.x, v.y, v.z]) < .02 and np.linalg.norm(omega) < .03
        if not state['ready']:
            state['still_since'] = (state['still_since'] or stamp) if stationary else None
            if state['still_since'] and stamp-state['still_since'] >= args.warmup_seconds:
                state['ready'] = True; publish['ready'].publish(Bool(data=True))
                rospy.loginfo('Sensors ready after %.1f s continuous stationary warmup', args.warmup_seconds)

    def odom_callback(msg):
        q = msg.pose.pose.orientation; quaternion = np.array([q.x, q.y, q.z, q.w])
        rotation = rotation_xyzw(quaternion); stamp = msg.header.stamp.to_sec()
        with lock:
            previous = state['previous']; command = state['command']; state['odom'] = msg
            state['previous'] = (stamp, quaternion, msg.pose.pose.position)
            actual_imu = state['actual_imu']
        if args.body_imu:
            # The actual body owns all inertial kinematics. In relay mode no
            # command acceleration, odometry angular field, or q difference is
            # ever used to generate IMU measurements.
            publish['truth'].publish(msg)
            if actual_imu is None or abs((msg.header.stamp-actual_imu.header.stamp).to_sec()) > .05:
                if not state['ready']: state['still_since'] = None
                return
            w = actual_imu.angular_velocity
            update_ready(msg, np.array([w.x, w.y, w.z]))
            return
        if previous is None: return
        dt = stamp-previous[0]
        p = msg.pose.pose.position; oldp = previous[2]
        if dt <= 0 or dt > .1 or np.linalg.norm([p.x-oldp.x, p.y-oldp.y, p.z-oldp.z]) > .5:
            rospy.logwarn_throttle(2., 'IMU discontinuity: skipping sample and restarting derivative history')
            return
        omega = body_rate(previous[1], quaternion, dt)
        if args.acceleration_source == 'fake-odom':
            a = msg.twist.twist.angular; acceleration = np.array([a.x, a.y, a.z])
        else:
            if command is None: acceleration = np.zeros(3)
            else:
                a = command.acceleration; acceleration = np.array([a.x, a.y, a.z])
        specific_force = rotation.T@(acceleration-np.array([0., 0., -9.8]))
        imu = Imu(header=Header(stamp=msg.header.stamp, frame_id=body))
        imu.orientation_covariance[0] = -1.  # No truth orientation is supplied to the estimator.
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = omega.tolist()
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = specific_force.tolist()
        imu.angular_velocity_covariance = [1e-8, 0., 0., 0., 1e-8, 0., 0., 0., 1e-8]
        imu.linear_acceleration_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        publish['imu'].publish(imu)
        publish['truth'].publish(msg)
        update_ready(msg, omega)

    subscribers = [rospy.Subscriber('/drone_%d_visual_slam/odom'%args.drone_id, Odometry, odom_callback, queue_size=100, tcp_nodelay=True),
                   rospy.Subscriber('/drone_%d_planning/pos_cmd'%args.drone_id, PositionCommand, command_callback, queue_size=10, tcp_nodelay=True)]
    if args.body_imu:
        source = prefix+'/body/imu' if args.body_imu == 'auto' else args.body_imu
        if not source.startswith('/') or source == prefix+'/kimera/imu':
            raise ValueError('--body-imu must name an absolute source topic different from the VIO output topic')
        subscribers.append(rospy.Subscriber(source, Imu, actual_imu_callback, queue_size=400, tcp_nodelay=True))
        rospy.loginfo('Relaying actual body IMU from %s; odometry/command IMU synthesis disabled', source)
    k = calibration(args)['K']; info = CameraInfo(width=args.width, height=args.height, distortion_model='plumb_bob')
    info.D = [0.]*5; info.K = np.asarray(k).ravel().tolist(); info.R = np.eye(3).ravel().tolist()
    info.P = [args.fx, 0., k[0][2], 0., 0., args.fy, k[1][2], 0., 0., 0., 1., 0.]
    fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1) for index, name in enumerate(('x', 'y', 'z'))]

    def image(values, encoding, header):
        return Image(header=header, height=args.height, width=args.width, encoding=encoding, is_bigendian=False,
                     step=values.strides[0], data=np.ascontiguousarray(values).tobytes())

    def cloud(values, header):
        return PointCloud2(header=header, height=1, width=len(values), fields=fields, is_bigendian=False,
                           point_step=12, row_step=len(values)*12, is_dense=True, data=values.tobytes())

    next_camera = next_lidar = -float('inf'); last_stamp = None; frames = 0; began = time.monotonic()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        with lock: odom = copy.deepcopy(state['odom'])
        if odom is None: rate.sleep(); continue
        stamp = odom.header.stamp.to_sec()
        if last_stamp is not None and stamp < last_stamp:
            next_camera = next_lidar = -float('inf')
        last_stamp = stamp
        p = odom.pose.pose.position; q = odom.pose.pose.orientation
        position = np.array([p.x, p.y, p.z]); rotation = rotation_xyzw([q.x, q.y, q.z, q.w])
        if stamp >= next_camera:
            tick = time.monotonic(); rgb, depth, mask = renderer.camera_frame(position, rotation)
            header = Header(stamp=odom.header.stamp, frame_id=optical)
            publish['rgb'].publish(image(rgb, 'rgb8', header)); publish['depth'].publish(image(depth, '32FC1', header))
            publish['mask'].publish(image(mask, 'mono8', header)); info.header = header
            publish['info'].publish(info); publish['depth_info'].publish(info)
            next_camera = stamp+1./args.camera_rate; frames += 1
            rospy.loginfo_throttle(10., 'RGB-D %.1f ms/render, %.2f Hz actual; %d measured pixels',
                                   (time.monotonic()-tick)*1000, frames/max(time.monotonic()-began, .001), np.count_nonzero(mask))
        if stamp >= next_lidar:
            local, world = renderer.lidar_frame(position, rotation)
            publish['lidar'].publish(cloud(local, Header(stamp=odom.header.stamp, frame_id=lidar_frame)))
            if planner: planner.publish(cloud(world, Header(stamp=odom.header.stamp, frame_id=odom.header.frame_id)))
            next_lidar = stamp+1./args.lidar_rate
        rate.sleep()


if __name__ == '__main__':
    args = arguments()
    if args.print_calibration: print(json.dumps(calibration(args), indent=2))
    elif args.self_test: self_test(args)
    else: run_ros(args)
