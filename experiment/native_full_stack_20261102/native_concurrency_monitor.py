#!/usr/bin/env python3
"""Bounded read-only stream/transaction witness; no mesh or image deserialization.

Run one witness on each producer host. Large topics use rospy.AnyMsg and only
their small leading Header is decoded. No raw payload is copied to disk or sent
to the other host. Counts are subscriber observations, not loss-free delivery
certificates. Global acceptance is reported from the existing verifier, not
reimplemented by this monitor.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import signal
import struct
import threading
import time


def header(raw):
    if len(raw) < 16:
        return None
    sequence, sec, nsec, size = struct.unpack_from('<IIII', raw)
    if size > 1024 or len(raw) < 16 + size or nsec >= 10**9:
        return None
    return dict(sequence=sequence, stamp_ns=sec * 10**9 + nsec,
                frame=raw[16:16+size].decode('utf-8', 'replace'))


def main():
    import rospy
    import roslib.message
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=1500.)
    parser.add_argument('--global-status', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])
    args.output_directory.mkdir(parents=True, exist_ok=True)
    rospy.init_node('readonly_native_concurrency_r%d' % args.robot_id)
    cutoff_ns = round(args.cutoff * 1e9)
    started = time.monotonic()
    lock = threading.RLock()
    samples = (args.output_directory/'samples.jsonl').open('a', buffering=1)
    events = (args.output_directory/'events.jsonl').open('a', buffering=1)
    streams = {}
    statuses = {}
    previous_statuses = {}
    transaction_windows = []
    active_windows = {}
    status_parse_times = {}
    global_latest = {}
    global_counts = Counter()
    errors = []
    stopping = [False]
    subscriptions = []
    rounds = {}
    command_counts = Counter()
    metadata = dict(run_id=args.run_id, cutoff_ns=cutoff_ns, robot_id=args.robot_id,
        started_wall=time.time(), maximum_wall_duration_s=args.duration,
        ROS_publishers_created=0, ROS_service_calls=0, truth_topics_subscribed=0,
        readonly=True, large_payload_deserialization=False, raw_payload_archival=False,
        validity='Subscriber-observed rates and existing bridge/verifier reports; no SLAM accuracy or loss-free certificate')
    (args.output_directory/'meta.json').write_text(json.dumps(metadata, indent=2))

    def event(kind, **values):
        events.write(json.dumps(dict(event=kind, wall=time.time(), **values), allow_nan=False)+'\n')

    def summarize_bridge(value, now):
        accepted = value.get('accepted_joint_mesh_receipt') or {}
        approved = value.get('approved_path') or {}
        proof = approved.get('proof') or {}
        return dict(received_wall=now, source_wall=value.get('wall'),
            fatal_error=value.get('fatal_error'), native_local_gauge_verified=value.get('native_local_gauge_verified'),
            local_keyframes=value.get('local_keyframes'), graph_forwarded_through_key=value.get('graph_forwarded_through_key'),
            phase=value.get('mesh_transaction_phase'), counts=value.get('counts') or {},
            local_tsdf_continues_during_pgmo_transaction=value.get('local_tsdf_continues_during_pgmo_transaction'),
            native_original_priority_queue_length=value.get('native_original_priority_queue_length'),
            native_original_priority_queue_max_length=value.get('native_original_priority_queue_max_length'),
            native_original_max_queue_delay_s=value.get('native_original_max_queue_delay_s'),
            native_control_max_queue_delay_s=value.get('native_control_max_queue_delay_s'),
            actual_gyro_max_queue_delay_s=value.get('actual_gyro_max_queue_delay_s'),
            gyro_maximum_sample_gap_s=value.get('gyro_maximum_sample_gap_s'),
            gyro_maximum_used_gap_s=value.get('gyro_maximum_used_gap_s'),
            gyro_maximum_endpoint_residual_rad=value.get('gyro_maximum_endpoint_residual_rad'),
            native_vio_maximum_interpolated_gap_s=value.get('native_vio_maximum_interpolated_gap_s'),
            optimized_candidates_waiting=value.get('optimized_candidates_waiting'),
            replaceable_full_mesh_overwrites=value.get('replaceable_full_mesh_overwrites'),
            native_deformation_eligible=value.get('native_deformation_eligible'),
            accepted_round=accepted.get('completed_round_serial'), accepted_owner=accepted.get('optimizer_owner'),
            accepted_weight_sha256=accepted.get('final_native_weight_batch_sha256'),
            accepted_mesh_sha256=accepted.get('canonical_native_mesh_sha256'),
            accepted_mesh_vertices=accepted.get('vertex_count'), accepted_wall=accepted.get('accepted_wall'),
            approved_round=(proof.get('round') or {}).get('serial'),
            sensor_pose_interpolation=value.get('sensor_pose_interpolation'))

    def status_callback(robot, message):
        now = time.time()
        if now-status_parse_times.get(robot, 0.) < .19:
            return
        status_parse_times[robot] = now
        try:
            raw = message._buff
            size = struct.unpack_from('<I', raw)[0]
            value = json.loads(raw[4:4+size])
            if value.get('run_id') != args.run_id or value.get('cutoff_ns') != cutoff_ns or value.get('robot_id') != robot:
                raise ValueError('bridge epoch/ownership mismatch')
            with lock:
                item = summarize_bridge(value, now)
                previous = statuses.get(robot)
                statuses[robot] = item
                phase = item['phase']
                count = item['counts'].get('native_mesh_transactions_started', 0)
                active = active_windows.get(robot)
                if phase and (active is None or active['transaction_index'] != count):
                    if active:
                        active.update(ended_wall=now, incomplete_window=True)
                        transaction_windows.append(active)
                    active = dict(robot_id=robot, transaction_index=count, observed_start_wall=now,
                        start_sensor_clouds=item['counts'].get('sensor_clouds', 0), phases=[phase])
                    active_windows[robot] = active
                    event('pgmo_transaction_observed_start', **active)
                elif phase and active and phase != active['phases'][-1]:
                    active['phases'].append(phase)
                elif not phase and active:
                    active.update(ended_wall=now, elapsed_observed_s=now-active['observed_start_wall'],
                        sensor_clouds_delta=item['counts'].get('sensor_clouds', 0)-active['start_sensor_clouds'],
                        end_sensor_clouds_during_mesh_transaction=item['counts'].get('sensor_clouds_during_mesh_transaction', 0),
                        incomplete_window=False)
                    transaction_windows.append(active)
                    event('pgmo_transaction_observed_end', **active)
                    del active_windows[robot]
                if previous and (previous['accepted_round'], previous['accepted_mesh_sha256']) != (item['accepted_round'], item['accepted_mesh_sha256']):
                    event('accepted_mesh_changed', robot_id=robot, accepted_round=item['accepted_round'],
                        accepted_owner=item['accepted_owner'], accepted_weight_sha256=item['accepted_weight_sha256'],
                        accepted_mesh_vertices=item['accepted_mesh_vertices'])
                if item['fatal_error'] and (not previous or previous['fatal_error'] != item['fatal_error']):
                    event('bridge_fatal_error', robot_id=robot, error=item['fatal_error'])
        except Exception as error:
            with lock:
                errors.append(dict(wall=now, stream='bridge%d' % robot, error=str(error)))
                del errors[:-100]

    def stream_callback(name, has_header, message):
        now = time.time()
        with lock:
            raw = message._buff
            info = streams.setdefault(name, dict(messages=0, bytes=0, first_wall=now,
                last_wall=now, max_receive_gap_s=0., sequence_jumps=0,
                header_stamp_backwards=0, callbacks_before_cutoff=0))
            info['messages'] += 1
            info['bytes'] += len(raw)
            info['max_receive_gap_s'] = max(info['max_receive_gap_s'], now-info['last_wall'])
            info['last_wall'] = now
            info['ros_type'] = message._connection_header.get('type')
            info['callerid'] = message._connection_header.get('callerid')
            if has_header:
                value = header(raw)
                if value:
                    previous = info.get('header')
                    if previous and value['sequence'] > previous['sequence']+1:
                        info['sequence_jumps'] += value['sequence']-previous['sequence']-1
                    if previous and value['stamp_ns'] < previous['stamp_ns']:
                        info['header_stamp_backwards'] += 1
                    if value['stamp_ns'] < cutoff_ns:
                        info['callbacks_before_cutoff'] += 1
                    info['header'] = value

    def global_callback(message):
        now = time.time()
        if now-status_parse_times.get('global', 0.) < .19:
            return
        status_parse_times['global'] = now
        try:
            raw = message._buff
            size = struct.unpack_from('<I', raw)[0]
            value = json.loads(raw[4:4+size])
            if value.get('run_id') != args.run_id or value.get('cutoff_ns') != cutoff_ns:
                raise ValueError('global verifier epoch mismatch')
            with lock:
                prior = dict(global_latest)
                global_latest.clear()
                global_latest.update({key: value.get(key) for key in ('fusion_ready', 'reason', 'points',
                    'robot_point_counts', 'completed_round_serial', 'optimizer_owner',
                    'final_native_weight_batch_sha256', 'snapshot_identity', 'archive_count')})
                global_latest['received_wall'] = now
                global_counts['ready_observations' if value.get('fusion_ready') else 'not_ready_observations'] += 1
                if prior.get('fusion_ready') != value.get('fusion_ready') or prior.get('snapshot_identity') != value.get('snapshot_identity') or prior.get('reason') != value.get('reason'):
                    event('global_readiness_changed', **global_latest)
        except Exception as error:
            with lock:
                errors.append(dict(wall=now, stream='global', error=str(error)))
                del errors[:-100]

    command_type = roslib.message.get_message_class('dpgo_ros/Command')
    command_names = {}
    if command_type:
        command_names = {int(getattr(command_type, key)): key for key in ('REQUEST_POSE_GRAPH',
            'UPDATE_WEIGHT', 'TERMINATE', 'HARD_TERMINATE') if hasattr(command_type, key)}

    def command_callback(robot, message):
        now = time.time()
        name = command_names.get(int(message.command), str(message.command))
        with lock:
            command_counts['r%d_%s' % (robot, name)] += 1
            if name == 'REQUEST_POSE_GRAPH':
                rounds[robot] = dict(owner=robot, request_wall=now, update_weights=0, active_robots=list(message.active_robots))
            elif name == 'UPDATE_WEIGHT' and robot in rounds:
                rounds[robot]['update_weights'] += 1
            elif name in ('TERMINATE', 'HARD_TERMINATE') and robot in rounds:
                ended = rounds.pop(robot)
                ended.update(terminate_wall=now, termination=name)
                event('native_round_complete_observed', **ended)

    base = '/sim_%d/' % args.robot_id
    topics = {'raw_rgb': ('kimera/rgb/image_raw', True), 'raw_depth': ('kimera/depth/image_raw', True),
        'raw_imu': ('kimera/imu', True), 'raw_lidar': ('lidar/points', True),
        'native_odom': ('kimera_vio_ros/odometry', True),
        'local_tsdf_points': ('native_dense/rgbd_points', True),
        'raw_local_vertices': ('native_dense/raw_local_vertices', True),
        'tsdf_mesh': ('native_tsdf/mesh', False),
        'native_frontend_full_mesh': ('native_mesh_frontend/full_mesh', True)}
    for name, (topic, has_header) in topics.items():
        subscriptions.append(rospy.Subscriber(base+topic, rospy.AnyMsg,
            lambda m, n=name, h=has_header: stream_callback(n, h, m), queue_size=1,
            buff_size=2**24, tcp_nodelay=True))
    for robot in ((0, 1) if args.global_status else (args.robot_id,)):
        subscriptions.append(rospy.Subscriber('/sim_%d/native_dense/status' % robot, rospy.AnyMsg,
            lambda m, r=robot: status_callback(r, m), queue_size=1, buff_size=2**22, tcp_nodelay=True))
        if command_type:
            subscriptions.append(rospy.Subscriber('/sim_%d/dpgo_ros_node/command' % robot, command_type,
                lambda m, r=robot: command_callback(r, m), queue_size=100, tcp_nodelay=True))
    if args.global_status:
        subscriptions.append(rospy.Subscriber('/visualization/native_mesh/status', rospy.AnyMsg,
            global_callback, queue_size=1, buff_size=2**22, tcp_nodelay=True))
    stop_file = args.output_directory/'STOP'

    def stop(_signum, _frame):
        stopping[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    previous_streams = {}
    previous_wall = time.time()
    while not stopping[0] and not rospy.is_shutdown() and time.monotonic()-started < args.duration and not stop_file.exists():
        time.sleep(1.)
        now = time.time()
        with lock:
            observed = {}
            for name, value in streams.items():
                prior = previous_streams.get(name, dict(messages=0, bytes=0))
                observed[name] = dict(value, hz_observed=(value['messages']-prior['messages'])/(now-previous_wall),
                    bytes_per_second_observed=(value['bytes']-prior['bytes'])/(now-previous_wall),
                    age_s=now-value['last_wall'])
                previous_streams[name] = dict(messages=value['messages'], bytes=value['bytes'])
            bridges = {str(robot): dict(value, age_s=now-value['received_wall']) for robot, value in statuses.items()}
            pair = None
            if all(robot in statuses for robot in (0, 1)):
                keys = ('accepted_owner', 'accepted_round', 'accepted_weight_sha256')
                pair = dict(accepted_generation_matches=all(statuses[0][key] is not None and statuses[0][key] == statuses[1][key] for key in keys),
                    rounds=[statuses[robot]['accepted_round'] for robot in (0, 1)])
            record = dict(wall=now, elapsed_s=time.monotonic()-started, streams=observed,
                bridge_status=bridges, global_verifier=dict(global_latest), global_counts=dict(global_counts),
                accepted_generation_pair=pair, command_counts=dict(command_counts), active_transactions=dict(active_windows),
                finished_transaction_count=len(transaction_windows), errors=list(errors))
            samples.write(json.dumps(record, allow_nan=False)+'\n')
            (args.output_directory/'latest.json').write_text(json.dumps(record, indent=2, allow_nan=False))
            previous_wall = now
    with lock:
        final = dict(metadata, final=True, ended_wall=time.time(), elapsed_s=time.monotonic()-started,
            streams=streams, bridge_status=statuses, global_verifier=global_latest,
            global_counts=dict(global_counts), command_counts=dict(command_counts),
            transaction_windows=transaction_windows, unfinished_transactions=active_windows, errors=errors)
        (args.output_directory/'summary.json').write_text(json.dumps(final, indent=2, allow_nan=False))
    samples.close()
    events.close()
    print(json.dumps(dict(ended_wall=time.time(), output_directory=str(args.output_directory), errors=errors)))


if __name__ == '__main__':
    main()
