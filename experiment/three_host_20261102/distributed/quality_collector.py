#!/usr/bin/env python3
"""Collect compact observer poses for offline checks; never send truth to SLAM."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time


def main():
    import rospy
    from nav_msgs.msg import Odometry, Path as RosPath
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--run-id', required=True)
    p.add_argument('--output', type=Path, required=True); p.add_argument('--duration', type=float, default=1200.)
    a = p.parse_args(rospy.myargv()[1:]); rospy.init_node('distributed_quality_collector', disable_signals=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    lock = threading.Lock(); seen = set(); stopped = threading.Event(); counts = {}
    for s in (signal.SIGINT, signal.SIGTERM): signal.signal(s, lambda *_: stopped.set())
    with a.output.open('a') as stream:
        def put(robot, kind, stamp, pose):
            if not stamp.to_nsec(): return
            p, q = pose.position, pose.orientation
            key = (robot, kind, stamp.to_nsec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            with lock:
                if key in seen: return
                seen.add(key); counts[kind] = counts.get(kind, 0)+1
                stream.write(json.dumps({'run_id': a.run_id, 'robot_id': robot, 'kind': kind,
                    'stamp_ns': stamp.to_nsec(), 'received_wall': time.time(),
                    'p': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w]})+'\n')
        def odom(m, identity): put(*identity, m.header.stamp, m.pose.pose)
        def path(m, robot):
            for pose in m.poses: put(robot, 'body_path', pose.header.stamp, pose.pose)
        subscriptions = []
        for robot in (0, 1):
            for kind in ('actual', 'vio'):
                subscriptions.append(rospy.Subscriber('/observer/sim_%d/%s' % (robot, kind), Odometry, odom,
                                      callback_args=(robot, kind), queue_size=200))
            subscriptions.append(rospy.Subscriber('/observer/sim_%d/shared_body_path' % robot, RosPath, path,
                                  callback_args=robot, queue_size=4))
        until = time.monotonic()+a.duration
        while not stopped.is_set() and not rospy.is_shutdown() and time.monotonic() < until:
            with lock: stream.flush()
            time.sleep(.5)
    print(json.dumps({'run_id': a.run_id, 'counts': counts, 'output': str(a.output)}))


if __name__ == '__main__': main()
