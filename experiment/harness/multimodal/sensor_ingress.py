#!/usr/bin/env python3
"""Receive each edge sensor stream once; fan original ROS messages out locally.

Use the historical sensor node name so existing source-authenticated native
consumers keep their original contract. The edge renderer uses a different
node name and private /edge/ topics. Ground truth is relayed only to its named
evaluation topic and is never published as a VIO/SLAM estimate.
"""
import argparse
from collections import Counter
import json
import math
import threading
import time

from sensor_wire import (LANES, RGBD_KEYS, STREAMS, Receiver, WireError,
                         canonical_topic, ingress_caller, body_caller, stamp_ns, uplink_topic,
                         validate_identity)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--max-frame-age', type=float, default=2.)
    args = parser.parse_args(argv)
    if not 0 < args.cutoff < float('inf') or not .1 <= args.max_frame_age <= 10:
        parser.error('Use a finite positive cutoff and frame-age limit in [0.1,10] seconds')
    validate_identity(args.robot_id, args.run_id, round(args.cutoff*1e9))
    return args


def validate_messages(robot_id, meta, messages):
    """Validate all decoded parts before publishing any member of an RGB-D pair."""
    expected_frames = {'imu': 'sim_%d/imu' % robot_id, 'lidar': 'sim_%d/lidar' % robot_id,
                       'truth': 'world'}
    for key, message in messages.items():
        if key == 'ready': continue
        if stamp_ns(message) != meta['stamp_ns']: raise WireError('Original ROS Header differs from envelope stamp')
        expected = 'sim_%d/camera_optical' % robot_id if key in RGBD_KEYS else expected_frames[key]
        if message.header.frame_id.lstrip('/') != expected: raise WireError('Unexpected original sensor frame')
    if meta['lane'] == 'rgbd':
        rgb, depth, mask = (messages[key] for key in ('rgb', 'depth', 'mask'))
        shape = (rgb.height, rgb.width)
        if min(shape) <= 0 or any((m.height, m.width) != shape for m in (depth, mask, messages['info'], messages['depth_info'])):
            raise WireError('RGB-D/calibration dimensions differ')
        for key, encoding, pixel_bytes in (('rgb', 'rgb8', 3), ('depth', '32FC1', 4), ('mask', 'mono8', 1)):
            message = messages[key]
            if (message.encoding != encoding or message.step < message.width*pixel_bytes or
                    len(message.data) != message.height*message.step):
                raise WireError('Malformed original image layout or encoding')
    elif meta['lane'] == 'imu':
        m = messages['imu']
        if not all(math.isfinite(getattr(v, axis)) for v in (m.angular_velocity, m.linear_acceleration) for axis in ('x', 'y', 'z')):
            raise WireError('Nonfinite actual body IMU')
    elif meta['lane'] == 'lidar':
        m = messages['lidar']
        if m.point_step < 12 or m.row_step < m.point_step*m.width or len(m.data) != m.row_step*m.height:
            raise WireError('Malformed original LiDAR layout')
    elif meta['lane'] == 'truth':
        m = messages['truth']
        values = ([getattr(m.pose.pose.position, axis) for axis in ('x','y','z')] +
                  [getattr(m.pose.pose.orientation, axis) for axis in ('x','y','z','w')] +
                  [getattr(vector, axis) for vector in (m.twist.twist.linear,m.twist.twist.angular)
                   for axis in ('x','y','z')])
        if not all(math.isfinite(value) for value in values):
            raise WireError('Nonfinite actual body telemetry')


def run_ros(args):
    import rospy
    import roslib.message
    from std_msgs.msg import String, UInt8MultiArray

    rospy.init_node(ingress_caller(args.robot_id).lstrip('/'))
    cutoff_ns = round(args.cutoff*1e9)
    receiver = Receiver(args.robot_id, args.run_id, cutoff_ns, round(args.max_frame_age*1e9))
    kinds = {key: roslib.message.get_message_class(spec['type']) for key, spec in STREAMS.items()}
    if any(kind is None for kind in kinds.values()): raise RuntimeError('A ROS sensor message type is missing')
    publishers = {key: rospy.Publisher(canonical_topic(args.robot_id, spec['suffix']), kinds[key],
                    queue_size=200 if key == 'imu' else 2, latch=spec['latch'], tcp_nodelay=True)
                  for key, spec in STREAMS.items()}
    status = rospy.Publisher(canonical_topic(args.robot_id, 'sensors/ingress_status'), String, queue_size=1, latch=True)
    counts = {lane: Counter() for lane in LANES}
    lane_locks = {lane: threading.Lock() for lane in LANES}
    processing_locks = {lane: threading.Lock() for lane in LANES}

    def callback(packet, lane):
        # Serialize through publication too: a reconnected same-lane TCPROS
        # callback cannot publish sequence N+1 ahead of an accepted sequence N.
        # Each lane still has its own lock, so RGB-D cannot stall the IMU lane.
        with processing_locks[lane]:
            try:
                caller = (getattr(packet, '_connection_header', None) or {}).get('callerid')
                meta, original = receiver.receive(lane, packet.data, caller, time.time_ns())
                messages = {}
                for key, raw in original.items():
                    message = kinds[key](); message.deserialize(raw); messages[key] = message
                validate_messages(args.robot_id, meta, messages)
                if lane != 'ready' and time.time_ns()-meta['stamp_ns'] > receiver.max_age_ns:
                    raise WireError('Sensor payload became stale during decoding')
                # A typed rospy publish rewrites Header.seq. Typed publishers
                # also accept AnyMsg, whose forwarding serializer preserves the
                # original bytes while keeping the declared type/MD5 handshake.
                # Calibration precedes the paired images in the envelope order.
                for key in messages:
                    publishers[key].publish(rospy.AnyMsg().deserialize(original[key]))
                with lane_locks[lane]:
                    counts[lane]['envelopes_republished'] += 1
                    counts[lane]['last_republished_wall_ns'] = time.time_ns()
                    counts[lane]['last_republished_stamp_ns'] = meta['stamp_ns']
            except Exception as error:
                with lane_locks[lane]: counts[lane]['envelopes_rejected'] += 1
                rospy.logwarn_throttle(2., 'Sensor ingress omitted %s: %s', lane, str(error))

    subscriptions = [rospy.Subscriber(uplink_topic(args.robot_id, lane), UInt8MultiArray,
        callback, callback_args=lane, queue_size=200 if lane == 'imu' else 1,
        buff_size=2**26 if lane == 'rgbd' else 2**20, tcp_nodelay=True) for lane in LANES]
    try:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            lanes = receiver.snapshot()
            for lane in LANES:
                with lane_locks[lane]: lanes[lane].update(counts[lane])
            value = dict(run_id=args.run_id, cutoff_ns=cutoff_ns, robot_id=args.robot_id,
                         wall=time.time(), lanes=lanes, fatal_error=None,
                         max_frame_age_s=args.max_frame_age,
                         single_remote_sensor_receiver=True, raw_data_unchanged=True,
                         imu_independent_of_rgbd=True, records_raw_files=False,
                         ground_truth_output='actual_body_telemetry_and_evaluation_only',
                         actual_body_source_caller=body_caller(args.robot_id),
                         actual_body_telemetry_is_slam_input=False,
                         computes_pose_or_map=False)
            status.publish(String(data=json.dumps(value, allow_nan=False))); rate.sleep()
    finally:
        for subscription in subscriptions: subscription.unregister()


if __name__ == '__main__':
    import rospy
    run_ros(arguments(rospy.myargv()[1:]))
