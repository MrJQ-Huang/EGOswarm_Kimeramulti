#!/usr/bin/env python3
"""Own edge simulation, EGO control and sensor uplink roles for two drones.

The hub runs VIO, all mapping, mission goal dispatch and raw-data recording.
Edge runs retain logs and process-ownership metadata, never raw-data bags or maps.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time


MASTER_URI = 'http://192.168.5.14:11431'
START_ROLES = ('supervisor', 'sensors', 'uplink', 'heartbeat')
ROOTS = {
    'amov': Path('/home/amov/Drone/two_pc_ego_20261030'),
    'uros': Path('/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030'),
}

STOP = r'''
import json,os,signal,sys,time
spec=json.loads(sys.argv[1]);expected=spec['identity']
def matches():
 result=[]
 for name in os.listdir('/proc'):
  if not name.isdigit() or int(name)==os.getpid():continue
  try:
   values=open('/proc/'+name+'/environ','rb').read().split(b'\0')
   actual={k.decode():v.decode() for item in values if b'=' in item for k,v in [item.split(b'=',1)]}
   raw=open('/proc/'+name+'/stat').read();state=raw[raw.rfind(')')+2:].split()[0]
  except (OSError,UnicodeError):continue
  if all(actual.get(k)==v for k,v in expected.items()) and state!='Z':result.append(int(name))
 return sorted(result)
before=matches();signalled=[]
if spec.get('signal'):
 for pid in before:
  try:os.kill(pid,signal.SIGINT);signalled.append(pid)
  except ProcessLookupError:pass
 deadline=time.monotonic()+spec.get('wait_seconds',8.)
 while time.monotonic()<deadline:
  if not matches():break
  time.sleep(.1)
remaining=matches()
print(json.dumps({'signalled':signalled,'before':before,'remaining':remaining,'identity':expected,
                  'closed':not remaining,'confirmation_scope':'Exact inherited run/role/master ownership; non-zombie operating-system processes'}))
'''


def query_owned(entry, run_id, signal_process=False):
    # Keep using the saved prefix, including old native-container prefixes, so
    # an upgrade can inspect/stop an earlier deployment without broad kills.
    spec = {'identity': {'ROS_MASTER_URI': MASTER_URI,
                         'THREE_HOST_RUN_ID': run_id,
                         'THREE_HOST_ROLE': entry['role']},
            'signal': signal_process, 'wait_seconds': 8. if signal_process else 0.}
    raw = subprocess.check_output(entry['prefix'] + ['python3', '-c', STOP, json.dumps(spec)], text=True)
    return json.loads(raw)


def close_owned(entry, run_id):
    receipt = query_owned(entry, run_id, signal_process=True)
    entry['stop_requested_wall'] = time.time()
    entry['stop_receipt'] = receipt
    if receipt['closed']:
        entry['stopped_wall'] = time.time()
    return receipt


def write_state(path, state):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(state, indent=2) + '\n')
    temporary.replace(path)


def argument_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=START_ROLES + ('status', 'stop'))
    p.add_argument('--host', required=True, choices=tuple(ROOTS))
    p.add_argument('--run-id', required=True)
    p.add_argument('--cutoff', type=float)
    p.add_argument('--camera-rate', type=float, default=5.)
    p.add_argument('--lidar-rate', type=float, default=1.)
    p.add_argument('--lidar-stride', type=int, default=4)
    p.add_argument('--telemetry-rate', type=float, default=30.)
    p.add_argument('--warmup-seconds', type=float, default=8.)
    p.add_argument('--max-frame-age', type=float, default=2.)
    p.add_argument('--max-pending', type=int, default=8)
    p.add_argument('--compression-level', type=int, choices=range(10), default=1)
    p.add_argument('--roles', nargs='+',
                   help='Stop saved roles, including roles from an older deployment')
    p.add_argument('--dry-run', action='store_true',
                   help='Print an edge start command without creating files or processes')
    return p


def validate_args(a, p):
    for label, value in (('camera rate', a.camera_rate), ('lidar rate', a.lidar_rate),
                         ('warmup duration', a.warmup_seconds), ('maximum frame age', a.max_frame_age),
                         ('telemetry rate', a.telemetry_rate)):
        if not math.isfinite(value) or value <= 0:
            p.error(label + ' must be finite and positive')
    if a.cutoff is not None and (not math.isfinite(a.cutoff) or a.cutoff <= 0):
        p.error('Cutoff must be finite and positive')
    if a.action == 'uplink' and a.cutoff is None:
        p.error('Sensor uplink requires an explicit epoch cutoff')
    if a.max_pending < 2:
        p.error('At least two pending sensor messages are required')
    if a.lidar_stride < 1 or not 1 <= a.telemetry_rate <= 100:
        p.error('Use positive lidar-stride and telemetry-rate in [1,100] Hz')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', a.run_id):
        p.error('Invalid run ID')
    if a.roles and a.action != 'stop':
        p.error('--roles is only valid for stop')
    if a.dry_run and a.action not in START_ROLES:
        p.error('--dry-run only applies to start roles')


def bind_epoch(a, host_out):
    path = host_out / 'epoch_config.json'
    expected = {'run_id': a.run_id, 'master_uri': MASTER_URI,
                'cutoff_ns': round(a.cutoff * 1e9) if a.cutoff is not None else None}
    if path.exists():
        prior = json.loads(path.read_text())
        if any(prior[k] != expected[k] for k in ('run_id', 'master_uri')):
            raise RuntimeError('Existing run identity differs')
        if prior.get('cutoff_ns') is not None:
            if expected['cutoff_ns'] is not None and prior['cutoff_ns'] != expected['cutoff_ns']:
                raise RuntimeError('Existing run cutoff differs; use a fresh run ID')
            return
    write_state(path, expected)


def sensor_topic_remaps(robot):
    # A single shared wire contract defines every simulator output. Import it
    # without importing rospy so dry-run and ownership queries remain offline.
    path = str(Path(__file__).resolve().parents[1] / 'harness' / 'multimodal')
    if path not in sys.path:
        sys.path.insert(0, path)
    from sensor_wire import sensor_remaps
    return sensor_remaps(robot)


def start_plan(a, root=None):
    """Return a reviewable edge command; this function has no side effects."""
    root = Path(root) if root is not None else ROOTS[a.host]
    inside = Path('/experiment') if a.host == 'uros' else root
    out = inside / 'three_host_20261102' / 'runs' / a.run_id / a.host
    host_out = root / 'three_host_20261102' / 'runs' / a.run_id / a.host
    robot = 0 if a.host == 'amov' else 1
    harness = inside / 'harness' / 'multimodal'
    prefix = (['docker', 'exec', 'ego-native-full-uros-20261102'] if a.host == 'uros' else []) + [
        'bash', str(inside / 'harness' / ('env_' + a.host + '.bash')),
        'env', 'ROS_MASTER_URI=' + MASTER_URI,
        'ROS_LOG_DIR=' + str(out / 'ros_edge'), 'OMP_NUM_THREADS=1', 'OPENBLAS_NUM_THREADS=1']
    if a.action == 'supervisor':
        command = ['python3', str(inside / 'harness' / 'scenario_supervisor.py'),
                   '--drone-id', str(robot), '--root', str(inside),
                   '--scene', str(inside / 'harness' / 'broad_loop_scene.json'), '--local-planning-map']
    elif a.action == 'sensors':
        command = ['python3', str(harness / 'sim_sensors.py'),
                   '--scene', str(inside / 'harness' / 'broad_loop_scene.json'),
                   '--drone-id', str(robot), '--camera-rate', str(a.camera_rate),
                   '--lidar-rate', str(a.lidar_rate), '--warmup-seconds', str(a.warmup_seconds),
                   '--lidar-stride', str(a.lidar_stride), '--no-truth-relay',
                   '--body-imu', '__name:=sim_%d_edge_sensors' % robot]
        command += list(sensor_topic_remaps(robot))
    elif a.action == 'uplink':
        command = ['python3', str(harness / 'sensor_uplink.py'), '--robot-id', str(robot),
                   '--run-id', a.run_id, '--cutoff', str(a.cutoff),
                   '--max-frame-age', str(a.max_frame_age), '--max-pending', str(a.max_pending),
                   '--compression-level', str(a.compression_level), '--telemetry-rate', str(a.telemetry_rate)]
    elif a.action == 'heartbeat':
        command = ['python3', str(harness / 'peer_heartbeat.py'), '--robot-id', str(robot)]
    else:
        raise ValueError('Unsupported edge start role: ' + a.action)
    owned_command = prefix + ['env', 'THREE_HOST_RUN_ID=' + a.run_id,
                              'THREE_HOST_ROLE=' + a.action,
                              'ROS_LOG_DIR=' + str(out / 'ros_edge')] + command
    return {'role': a.action, 'host': a.host, 'robot_id': robot, 'prefix': prefix,
            'command': owned_command, 'log': str(host_out / (a.action + '.log')),
            'output': str(host_out), 'master_uri': MASTER_URI,
            'edge_raw_recording_enabled': False, 'edge_slam_enabled': False}


def main():
    p = argument_parser()
    a = p.parse_args()
    validate_args(a, p)
    if a.dry_run:
        print(json.dumps(start_plan(a), indent=2))
        return
    root = ROOTS[a.host]
    stage = root / 'three_host_20261102'
    stage.mkdir(exist_ok=True)
    import fcntl
    with (stage / 'manager.lock').open('a') as manager_lock:
        fcntl.flock(manager_lock, fcntl.LOCK_EX)
        run_locked(a, p, root, stage)


def run_locked(a, p, root, stage):
    state_path = stage / 'managed_processes.json'
    state = json.loads(state_path.read_text()) if state_path.exists() else {'host': a.host, 'roles': {}}
    if a.action == 'status':
        result = {}
        for key, entry in state['roles'].items():
            if entry['run_id'] != a.run_id:
                continue
            value = dict(entry)
            try:
                value['owned_processes'] = query_owned(entry, a.run_id)
            except Exception as error:
                value['query_error'] = repr(error)
            result[key] = value
        print(json.dumps(result, indent=2))
        return
    if a.action == 'stop':
        result = []
        for key, entry in state['roles'].items():
            if entry['run_id'] != a.run_id or entry.get('stopped_wall'):
                continue
            if a.roles and entry['role'] not in a.roles:
                continue
            result.append({'role': entry['role'], 'response': close_owned(entry, a.run_id)})
        write_state(state_path, state)
        print(json.dumps(result))
        return
    key = a.run_id + ':' + a.action
    for entry in state['roles'].values():
        if entry['run_id'] != a.run_id and not entry.get('stopped_wall'):
            raise RuntimeError('Close the previous owned run before starting a new epoch: ' + entry['run_id'])
    if key in state['roles'] and not state['roles'][key].get('stopped_wall'):
        raise RuntimeError('Role already has an unclosed ownership record: ' + key)
    plan = start_plan(a, root)
    host_out = Path(plan['output'])
    host_out.mkdir(parents=True, exist_ok=True)
    bind_epoch(a, host_out)
    if a.action == 'supervisor':
        (root / 'logs').mkdir(exist_ok=True)
    with Path(plan['log']).open('a') as log:
        process = subprocess.Popen(plan['command'], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    state['roles'][key] = {'run_id': a.run_id, 'role': a.action,
                           'prefix': plan['prefix'], 'command': plan['command'],
                           'pid': process.pid, 'started_wall': time.time(), 'log': plan['log'],
                           'cutoff_ns': round(a.cutoff * 1e9) if a.cutoff is not None else None}
    write_state(state_path, state)
    print(json.dumps(state['roles'][key]))


if __name__ == '__main__':
    main()
