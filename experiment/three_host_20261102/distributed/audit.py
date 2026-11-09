#!/usr/bin/env python3
"""Read-only local topology and live progress witness for distributed runs."""
import argparse
from collections import Counter
import json
from pathlib import Path
import threading
import time


def topology(state, robot):
    pub, sub, services = map(dict, state)
    base = '/sim_%d' % robot
    expected = {base+'/kimera_vio_ros/odometry': base+'/kimera_vio_ros/kimera_vio_ros_node',
        '/drone_%d_visual_slam/odom' % robot: '/drone_%d_continuous_body' % robot,
        '/drone_%d_planning/pos_cmd' % robot: '/drone_%d_traj_server' % robot,
        '/drone_%d_planning/bspline' % robot: '/drone_%d_ego_planner_node' % robot,
        base+'/kimera/rgb/image_raw': '/sim_%d_multimodal_sensors' % robot,
        base+'/kimera/imu': '/sim_%d_multimodal_sensors' % robot,
        base+'/connected_peer_ids': '/sim_%d_peer_bridge' % robot}
    checks = {topic+'_unique_source': pub.get(topic) == [source] for topic, source in expected.items()}
    for topic in (base+'/kimera_vio_ros/vlc_frame_query', base+'/distributed_loop_closure/request_pose_graph'):
        checks[topic+'_local_service'] = bool(services.get(topic))
    checks['no_remote_raw_camera'] = not pub.get('/sim_%d/kimera/rgb/image_raw' % (1-robot))
    checks['native_goal_consumer'] = '/drone_%d_ego_planner_node' % robot in sub.get('/two_pc/drone_%d/goal' % robot, [])
    checks['body_consumes_native_command'] = '/drone_%d_continuous_body' % robot in sub.get('/drone_%d_planning/pos_cmd' % robot, [])
    return checks


def status_checks(statuses, received, now, run_id, robot, cutoff):
    checks = {}
    for key in ('dense', 'local_sensor'):
        value = statuses.get(key, {})
        checks[key+'_identity'] = (value.get('run_id'), value.get('robot_id'), value.get('cutoff_ns')) == (
            run_id, robot, round(cutoff*1e9))
        checks[key+'_no_fatal'] = 'fatal_error' in value and value['fatal_error'] is None
        checks[key+'_fresh'] = now-received.get(key, -1e9) < 3.
    return checks


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import String
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-id', type=int, choices=(0,1), required=True)
    p.add_argument('--run-id', required=True); p.add_argument('--cutoff', type=float, required=True)
    p.add_argument('--duration', type=float, default=20.)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(rospy.myargv()[1:]); rospy.init_node('distributed_readonly_audit', anonymous=True)
    lock = threading.Lock(); counts = Counter(); stamps = {}; received = {}; ages = {}; gaps = {}; statuses = {}
    def sample(m, key):
        now = time.monotonic()
        with lock:
            counts[key] += 1
            if key in received: gaps[key] = max(gaps.get(key, 0.), now-received[key])
            received[key] = now
            if hasattr(m, 'header') and m.header.stamp.to_nsec():
                stamp = m.header.stamp.to_nsec()
                if stamp < stamps.get(key, stamp): counts[key+'_regressions'] += 1
                stamps[key] = stamp; ages[key] = time.time()-stamp/1e9
            if isinstance(m, String):
                try: statuses[key] = json.loads(m.data)
                except ValueError: counts[key+'_invalid_json'] += 1
    base = '/sim_%d' % a.robot_id
    streams = {'actual': ('/drone_%d_visual_slam/odom' % a.robot_id, Odometry),
        'vio': (base+'/kimera_vio_ros/odometry', Odometry), 'imu': (base+'/kimera/imu', Imu),
        'rgb': (base+'/kimera/rgb/image_raw', Image),
        'peer': (base+'/peer_bridge/status', String), 'dense': (base+'/native_dense/status', String),
        'local_sensor': (base+'/native_dense/local_sensor_status', String)}
    subscriptions = [rospy.Subscriber(topic, kind, sample, callback_args=key, queue_size=20, tcp_nodelay=True)
                     for key, (topic, kind) in streams.items()]
    began = time.monotonic(); until = began+a.duration
    while not rospy.is_shutdown() and time.monotonic() < until: time.sleep(.1)
    code, message, state = rospy.get_master().getSystemState()
    if code != 1: raise RuntimeError(message)
    checks = topology(state, a.robot_id)
    with lock:
        checks.update({key+'_received': counts[key] > 0 for key in streams})
        checks['peer_session_ready'] = statuses.get('peer', {}).get('session_ready') is True
        checks['no_timestamp_regression'] = not any(v for k, v in counts.items() if k.endswith('_regressions'))
        checks.update(status_checks(statuses, received, time.monotonic(), a.run_id, a.robot_id, a.cutoff))
        elapsed = time.monotonic()-began
        result = {'run_id': a.run_id, 'robot_id': a.robot_id, 'elapsed_s': elapsed, 'ready': all(checks.values()),
            'checks': checks, 'counts': dict(counts), 'rates_hz': {k:v/elapsed for k,v in counts.items()},
            'latest_source_ages_s_including_clock_offset': ages, 'maximum_callback_gaps_s': gaps,
            'statuses': statuses, 'topology': state,
            'vio_parameters': rospy.get_param(base+'/dpgo_ros_node/agent', {})}
    a.output.parent.mkdir(parents=True, exist_ok=True); a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'ready': result['ready'], 'checks': checks, 'rates_hz': result['rates_hz']}))
    if not result['ready']: raise SystemExit(2)


if __name__ == '__main__': main()
