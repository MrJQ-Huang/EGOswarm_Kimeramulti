#!/usr/bin/env python3
"""Execute preloaded goals locally; EGO alone plans paths and controls the body.

Peer stage barriers are independent of the observer. Only compact mission state
is persisted. Simulator state is for navigation supervision, never SLAM input.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time
import xmlrpc.client


class StageBarrier:
    """Monotonic local stage barrier tolerates the peer advancing first."""
    def __init__(self, stage_count, laps):
        if stage_count < 1 or laps < 1: raise ValueError('Empty mission')
        self.total = stage_count * laps
        self.stage_count = stage_count
        self.index = 0
        self.completed = -1
        self.settled_since = None
        self.joint_since = None
        self.arrived = False

    @property
    def done(self): return self.index >= self.total

    def update(self, now, error, peer, tolerance, settle, dwell):
        if self.done: return False
        if error < tolerance:
            if self.settled_since is None: self.settled_since = now
            self.arrived = now - self.settled_since >= settle
        else:
            self.settled_since = None; self.arrived = False; self.joint_since = None
        peer_arrived = bool(peer and (peer.get('completed_stage', -1) >= self.index or
                            (peer.get('stage_index') == self.index and peer.get('arrived') is True)))
        if self.arrived and peer_arrived:
            if self.joint_since is None: self.joint_since = now
            if now - self.joint_since >= dwell:
                self.completed = self.index; self.index += 1
                self.arrived = False; self.settled_since = None; self.joint_since = None
                return True
        else: self.joint_since = None
        return False


class TimedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        c = super().make_connection(host); c.timeout = 1.; return c


def shutdown_local_controls(rospy, robot, reason):
    """Stop the two local control nodes; the body holds actual state on timeout."""
    stopped = []
    for role in ('ego_planner_node', 'traj_server'):
        name = '/drone_%d_%s' % (robot, role)
        try:
            code, _, uri = rospy.get_master().lookupNode(name)
            if code != 1: continue
            with xmlrpc.client.ServerProxy(uri, transport=TimedTransport()) as api:
                api.shutdown(rospy.get_name(), 'Local mission safeguard: ' + reason)
            stopped.append(name)
        except (OSError, xmlrpc.client.Error): pass
    return stopped


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from traj_utils.msg import Bspline
    from quadrotor_msgs.msg import PositionCommand
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-id', type=int, choices=(0, 1), required=True)
    p.add_argument('--run-id', required=True); p.add_argument('--cutoff', type=float, required=True)
    p.add_argument('--route', type=Path, required=True); p.add_argument('--laps', type=int, default=2)
    p.add_argument('--start-at', type=float, default=0.)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--minimum-separation', type=float, default=1.)
    a = p.parse_args(rospy.myargv()[1:]); route = json.loads(a.route.read_text())
    if not route.get('stages') or a.laps < 1: p.error('Empty mission')
    robot = a.robot_id; peer_id = 1 - robot
    rospy.init_node('sim_%d_local_mission' % robot, disable_signals=True)
    lock = threading.RLock()
    state = {'position': None, 'received': 0., 'stamp_ns': 0, 'peer': None, 'peer_received': 0.,
             'sensors': False, 'vio_received': 0., 'splines': 0, 'commands': 0, 'bridge_ready': False,
             'failure': None, 'length': 0., 'first': None, 'minimum_separation': None}
    barrier = StageBarrier(len(route['stages']), a.laps)

    def odom(m):
        if (m._connection_header or {}).get('callerid') != '/drone_%d_continuous_body' % robot: return
        if m.header.stamp.to_sec() < a.cutoff: return
        q = m.pose.pose.position; xyz = [q.x, q.y, q.z]
        with lock:
            if state['position'] is not None: state['length'] += math.dist(state['position'], xyz)
            if state['first'] is None: state['first'] = xyz
            state.update(position=xyz, received=time.monotonic(), stamp_ns=m.header.stamp.to_nsec())

    def peer(m):
        if (m._connection_header or {}).get('callerid') != '/sim_%d_peer_bridge' % robot: return
        try:
            value = json.loads(m.data)
            if value.get('run_id') != a.run_id or value.get('robot_id') != peer_id: return
            with lock: state.update(peer=value, peer_received=time.monotonic())
        except (ValueError, TypeError): pass

    def ready(m):
        with lock: state['sensors'] = bool(m.data)

    def vio(m):
        if (m._connection_header or {}).get('callerid') != '/sim_%d/kimera_vio_ros/kimera_vio_ros_node' % robot: return
        if m.header.stamp.to_sec() < a.cutoff: return
        with lock: state['vio_received'] = time.monotonic()

    def count(m, key):
        expected = '/drone_%d_%s' % (robot, 'ego_planner_node' if key == 'splines' else 'traj_server')
        if (m._connection_header or {}).get('callerid') == expected:
            with lock: state[key] += 1

    def bridge(m):
        if (m._connection_header or {}).get('callerid') != '/sim_%d_peer_bridge' % robot: return
        try:
            v = json.loads(m.data)
            with lock: state['bridge_ready'] = bool(v.get('run_id') == a.run_id and v.get('session_ready'))
        except (ValueError, TypeError): pass

    subscribers = [rospy.Subscriber('/drone_%d_visual_slam/odom' % robot, Odometry, odom, queue_size=1, tcp_nodelay=True),
                   rospy.Subscriber('/sim_%d/mission/status' % peer_id, String, peer, queue_size=2, tcp_nodelay=True),
                   rospy.Subscriber('/sim_%d/sensors/ready' % robot, Bool, ready, queue_size=1),
                   rospy.Subscriber('/sim_%d/kimera_vio_ros/odometry' % robot, Odometry, vio, queue_size=1),
                   rospy.Subscriber('/drone_%d_planning/bspline' % robot, Bspline, count, 'splines', queue_size=4),
                   rospy.Subscriber('/drone_%d_planning/pos_cmd' % robot, PositionCommand, count, 'commands', queue_size=4),
                   rospy.Subscriber('/sim_%d/peer_bridge/status' % robot, String, bridge, queue_size=1)]
    goals = rospy.Publisher('/two_pc/drone_%d/goal' % robot, PoseStamped, queue_size=1)
    status = rospy.Publisher('/sim_%d/mission/status' % robot, String, queue_size=2, latch=True)
    def interrupt(*_):
        # A requested shutdown after all stages completed must preserve that
        # terminal result while still stopping the local control chain.
        state['stop_requested'] = True
        if not barrier.done and not state['failure']: state['failure'] = 'Mission interrupted'
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    began = time.monotonic(); motion_began = None; stage_began = None; goal_index = -1
    last_summary = 0.; phase = 'waiting'; stopped = []; history = []
    terminal_wall = None; terminal_monotonic = None
    a.output.parent.mkdir(parents=True, exist_ok=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        now = time.monotonic()
        with lock: current = dict(state)
        if current['failure']: phase = 'aborted'
        elif barrier.done: phase = 'completed'
        elif motion_began is None:
            can_start = (current['position'] is not None and now-current['received'] < .6 and current['sensors'] and
                         now-current['vio_received'] < 2. and current['bridge_ready'] and
                         current['peer'] is not None and now-current['peer_received'] < .6 and
                         current['peer'].get('local_ready') and time.time() >= a.start_at)
            if can_start: motion_began = now; phase = 'flying'; stage_began = now
            elif now - began > 120: state['failure'] = 'Local/peer startup readiness timeout'
        if phase == 'flying':
            peer_value = current['peer'] or {}
            if now-current['received'] > .6: state['failure'] = 'Local actual odometry stale'
            elif not current['bridge_ready'] or now-current['peer_received'] > .6: state['failure'] = 'Peer mission/link stale'
            elif peer_value.get('phase') == 'aborted': state['failure'] = 'Peer mission aborted'
            elif now-current['vio_received'] > 2.: state['failure'] = 'Local VIO stale'
            elif peer_value.get('position') and current['position']:
                sep = math.dist(current['position'], peer_value['position'])
                with lock:
                    prev = state['minimum_separation']; state['minimum_separation'] = sep if prev is None else min(prev, sep)
                    if sep < a.minimum_separation: state['failure'] = 'Actual separation below configured guard'
            if not state['failure']:
                stage = route['stages'][barrier.index % len(route['stages'])]
                if goal_index != barrier.index:
                    msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = route.get('frame_id', 'world')
                    xyz = stage['targets'][robot]
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz; msg.pose.orientation.w = 1.
                    goals.publish(msg); goal_index = barrier.index; stage_began = now
                    history.append({'stage_index': goal_index, 'target': xyz, 'released_wall': time.time()})
                error = math.dist(current['position'], stage['targets'][robot])
                if barrier.update(now, error, peer_value, stage.get('goal_tolerance_m', .35),
                                  stage.get('settle_seconds', 2.), stage.get('dwell_seconds', 0.)):
                    history[-1].update(completed_wall=time.time(), error_m=error)
                elif now-stage_began > stage.get('timeout_seconds', 120): state['failure'] = 'EGO goal timeout'
        if state['failure']:
            phase = 'aborted'
            if not stopped: stopped = shutdown_local_controls(rospy, robot, state['failure'])
        elif current.get('stop_requested') and phase == 'completed' and not stopped:
            stopped = shutdown_local_controls(rospy, robot, 'Completed mission shutdown')
        if phase in ('completed', 'aborted') and terminal_wall is None:
            terminal_wall = time.time(); terminal_monotonic = now
        value = {'run_id': a.run_id, 'robot_id': robot, 'wall': time.time(), 'phase': phase,
                 'stage_index': barrier.index, 'completed_stage': barrier.completed, 'arrived': barrier.arrived,
                 'position': current['position'], 'source_stamp_ns': current['stamp_ns'],
                 'local_ready': bool(current['position'] is not None and now-current['received'] < .6 and
                                     current['sensors'] and now-current['vio_received'] < 2.),
                 'failure': state['failure'], 'actual_path_length_m': current['length'],
                 'terminal_wall': terminal_wall,
                 'motion_duration_s': ((terminal_monotonic if terminal_monotonic is not None else now)-motion_began
                                       if motion_began is not None else None),
                 'minimum_separation_m': state['minimum_separation'], 'planner_splines': current['splines'],
                 'traj_server_commands': current['commands'], 'observer_required': False,
                 'goal_only_navigation': True, 'completed_stages': len([x for x in history if 'completed_wall' in x])}
        status.publish(String(data=json.dumps(value)))
        if now-last_summary > 1 or phase == 'aborted' or current.get('stop_requested'):
            value.update(stage_history=history, stopped_local_controls=stopped)
            tmp = a.output.with_suffix('.tmp'); tmp.write_text(json.dumps(value, indent=2) + '\n'); tmp.replace(a.output)
            last_summary = now
        if phase == 'aborted':
            time.sleep(.5); raise SystemExit(2)
        if current.get('stop_requested') and phase == 'completed': break
        rate.sleep()


if __name__ == '__main__': main()
