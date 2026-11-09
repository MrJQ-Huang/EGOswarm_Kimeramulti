#!/usr/bin/env python3
"""Own independent edge SLAM and observer roles without starting the central stack.

Dry runs are pure. Each real role is identified by run, role and local master;
stop never selects unrelated ROS nodes or another deployment profile.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

IPS = {'amov': '192.168.5.6', 'uros': '192.168.5.21', 'hub': '192.168.5.14'}
ROOTS = {'amov': '/home/amov/Drone/two_pc_ego_20261030',
         'uros': '/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030',
         'hub': '/home/mrjq311/swarm_kimera_runtime/experiment'}
EDGE_ROLES = ('master', 'supervisor', 'reset', 'sensors', 'vio', 'adapter',
              'local-frontend', 'backend', 'dense', 'peer', 'results', 'mission', 'witness', 'resource')
HUB_ROLES = ('master', 'observer', 'observer-telemetry', 'rviz', 'rviz-local0', 'rviz-local1', 'rviz-actual', 'quality', 'resource', 'gpu')
FLAGS = ('VioBackend.flags', 'VioFrontend.flags', 'VioParameters.flags', 'Mesher.flags')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=tuple(dict.fromkeys(EDGE_ROLES + HUB_ROLES)) + ('stop', 'status'))
    p.add_argument('--host', choices=tuple(IPS), required=True)
    p.add_argument('--run-id', required=True)
    p.add_argument('--cutoff', type=float, required=True)
    p.add_argument('--root')
    p.add_argument('--gauge-id', help='Session label only; actual shared alignment requires native evidence')
    p.add_argument('--camera-rate', type=float, default=5.)
    p.add_argument('--cloud-rate', type=float, default=2.)
    p.add_argument('--voxel-size', type=float, default=.15, help='Native local TSDF voxel size in metres; use the same value on both robots')
    p.add_argument('--mesh-period', type=float, default=2.)
    p.add_argument('--telemetry-rate', type=float, default=30.)
    p.add_argument('--map-rate', type=float, default=.5)
    p.add_argument('--delta', action='store_true', help='Negotiate lossless mesh byte-block deltas with full recovery')
    p.add_argument('--laps', type=int, default=2)
    p.add_argument('--start-at', type=float, default=0.)
    p.add_argument('--roles', nargs='+')
    p.add_argument('--opengv-guard', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    return p


def paths(a):
    root = Path(a.root or ROOTS[a.host])
    inside = PurePosixPath('/experiment') if a.host in ('uros', 'hub') else PurePosixPath(str(root))
    stage = root / 'three_host_20261102' / 'distributed'
    out = stage / 'runs' / a.run_id / a.host
    inner = inside / 'three_host_20261102' / 'distributed' / 'runs' / a.run_id / a.host
    return root, inside, stage, out, inner


def validate(a, p):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', a.run_id): p.error('Invalid run ID')
    for name in ('cutoff', 'camera_rate', 'cloud_rate', 'voxel_size', 'mesh_period', 'telemetry_rate', 'map_rate'):
        value = getattr(a, name)
        if not math.isfinite(value) or value <= 0: p.error(name + ' must be finite and positive')
    if not math.isfinite(a.start_at) or a.start_at < 0 or a.laps < 1: p.error('Invalid mission timing')
    if a.action not in ('stop', 'status') and a.action not in (HUB_ROLES if a.host == 'hub' else EDGE_ROLES):
        p.error('This host does not own that role')
    if a.roles and a.action != 'stop': p.error('--roles is only valid with stop')


def prefix(a, inside, inner):
    master = 'http://' + IPS[a.host] + ':11431'
    ego = a.action in ('supervisor', 'reset', 'sensors', 'mission')
    if a.host == 'hub':
        result = ['docker', 'exec', 'kimera-central-hub-gpu-20261103', 'env',
                  'NATIVE_ROS_MASTER_URI=' + master, 'THREE_HOST_ROS_LOG_DIR=' + str(inner / 'ros_logs'),
                  'bash', str(inside / 'three_host_20261102/env_hub.bash')]
    elif ego:
        result = (['docker', 'exec', 'ego-native-full-uros-20261102'] if a.host == 'uros' else [])
        result += ['bash', str(inside / ('harness/env_' + a.host + '.bash'))]
    else:
        result = (['docker', 'exec', 'kimera-native-full-uros-20261102'] if a.host == 'uros' else [])
        result += ['env', 'NATIVE_ROS_MASTER_URI=' + master]
        if a.opengv_guard: result += ['NATIVE_USE_OPENGV_GUARD=1']
        result += ['bash', str(inside / 'three_host_20261102/distributed/env_edge.bash'), a.host]
    return result + ['env', 'ROS_MASTER_URI=' + master, 'ROS_IP=' + IPS[a.host],
                     'ROS_LOG_DIR=' + str(inner / 'ros_logs'), 'THREE_HOST_RUN_ID=' + a.run_id,
                     'THREE_HOST_ROLE=distributed-' + a.action,
                     'DISTRIBUTED_KIMERA_PROFILE=1', 'PYTHONUNBUFFERED=1']


def plan(a):
    root, inside, stage, out, inner = paths(a)
    script = inside / 'three_host_20261102/distributed'
    harness = inside / 'harness/multimodal'
    robot = 0 if a.host == 'amov' else 1
    gauge = a.gauge_id or a.run_id + '.robot0-initial'
    identity = ['--robot-id', str(robot), '--run-id', a.run_id, '--cutoff', str(a.cutoff)]
    if a.action == 'master':
        command = ['roscore', '-p', '11431']
    elif a.action == 'supervisor':
        command = ['python3', str(inside / 'harness/scenario_supervisor.py'), '--drone-id', str(robot),
                   '--root', str(inside), '--scene', str(inside / 'harness/broad_loop_scene.json'), '--local-planning-map']
    elif a.action == 'reset':
        command = ['python3', str(script / 'runtime_control.py'), 'reset'] + identity
    elif a.action == 'sensors':
        command = ['python3', str(harness / 'sim_sensors.py'), '--scene', str(inside / 'harness/broad_loop_scene.json'),
                   '--drone-id', str(robot), '--camera-rate', str(a.camera_rate), '--lidar-rate', '1',
                   '--lidar-stride', '4', '--no-truth-relay', '--body-imu']
    elif a.action == 'vio':
        command = ['roslaunch', '--screen', str(inner / 'vio/vio.launch')]
    elif a.action == 'adapter':
        command = ['python3', str(harness / 'native_vio_graph_adapter.py'), '--robot-id', str(robot),
                   '--expected-caller', '/sim_%d/kimera_vio_ros/kimera_vio_ros_node' % robot]
    elif a.action == 'local-frontend':
        command = ['roslaunch', '--screen', str(harness / 'local_frontend.launch'), 'robot_id:=' + str(robot),
                   'robot_name:=sim_' + str(robot), 'run_id:=' + a.run_id, 'cutoff:=' + str(a.cutoff),
                   'harness_directory:=' + str(harness), 'output_directory:=' + str(inner / 'local-frontend'),
                   'cloud_rate:=' + str(a.cloud_rate), 'voxel_size:=' + str(a.voxel_size),
                   'mesh_period:=' + str(a.mesh_period), 'no_persist:=true']
    elif a.action == 'backend':
        command = ['roslaunch', '--screen', str(harness / 'backend.launch'), 'robot_id:=' + str(robot),
                   'robot_name:=sim_' + str(robot), 'runtime_dir:=' + str(inner / 'backend'),
                   'odom_suffix:=vio_odom', 'local_pose_graph_topic:=/sim_%d/kimera_camera_adapter/pose_graph_incremental' % robot,
                   'no_persist:=true']
    elif a.action == 'dense':
        command = ['roslaunch', '--screen', str(harness / 'hub_backend.launch'), 'robot_id:=' + str(robot),
                   'robot_name:=sim_' + str(robot), 'run_id:=' + a.run_id, 'cutoff:=' + str(a.cutoff),
                   'harness_directory:=' + str(harness), 'output_directory:=' + str(inner / 'dense'),
                   'execution_mode:=online', 'no_persist:=true', 'require_cross:=true',
                   'gauge_id:=' + gauge, 'remote_bridge_caller:=/sim_%d_peer_bridge' % robot]
    elif a.action == 'peer':
        command = ['python3', str(script / 'peer_bridge.py')] + identity + [
            '--listen-host', IPS[a.host], '--listen-port', '11640', '--peer-host', IPS['uros' if robot == 0 else 'amov'],
            '--peer-port', '11640', '--output-directory', str(inner / 'peer')]
    elif a.action == 'results':
        command = ['python3', str(script / 'result_stream.py')] + identity + [
            '--gauge-id', gauge, '--bind', IPS[a.host], '--telemetry-rate', str(a.telemetry_rate),
            '--map-rate', str(a.map_rate)]
        if a.delta: command += ['--delta']
    elif a.action == 'witness':
        command = ['python3', str(inside / 'three_host_20261102/native_tsdf_epoch_witness.py'),
                   '--run-id', a.run_id, '--cutoff', str(a.cutoff),
                   '--master-uri', 'http://' + IPS[a.host] + ':11431',
                   '--epoch-file', str(inner / 'epoch_config.json'), '--ros-log-root', str(inner / 'ros_logs'),
                   '--output-directory', str(inner / 'tsdf_witness'), '--duration', '1800']
    elif a.action == 'mission':
        command = ['python3', str(script / 'local_mission.py')] + identity + [
            '--route', str(inside / 'harness/broad_loop_route.json'), '--laps', str(a.laps),
            '--start-at', str(a.start_at), '--output', str(inner / 'mission_summary.json')]
    elif a.action in ('observer', 'observer-telemetry'):
        command = ['python3', str(script / 'observer.py'), '--peer', '0@' + IPS['amov'], '--peer', '1@' + IPS['uros'],
                   '--run-id', a.run_id, '--cutoff', str(a.cutoff), '--gauge-id', gauge,
                   '--output-directory', str(inner / 'observer'),
                   '--channels', 'telemetry' if a.action == 'observer-telemetry' else 'map']
    elif a.action == 'quality':
        command = ['python3', str(script / 'quality_collector.py'), '--run-id', a.run_id,
                   '--output', str(inner / 'quality_poses.jsonl'), '--duration', '1800']
    elif a.action == 'resource':
        command = ['python3', str(inside / 'three_host_20261102/sample_three_host_resources.py'),
                   '--run-id', a.run_id, '--host', 'mrjq311' if a.host == 'hub' else a.host,
                   '--ros-master-uri', 'http://' + IPS[a.host] + ':11431', '--include-rviz',
                   '--output-prefix', str(inner / 'resource/samples'), '--duration', '1800']
    elif a.action == 'gpu':
        command = ['nvidia-smi', '--query-gpu=timestamp,utilization.gpu,memory.used,power.draw',
                   '--format=csv,noheader,nounits', '--loop=1']
    elif a.action.startswith('rviz'):
        config = ('observer_shared.rviz' if a.action == 'rviz' else
                  'observer_actual.rviz' if a.action == 'rviz-actual' else
                  'observer_sim_%s.rviz' % a.action[-1])
        command = ['rviz', '-d', str(script / config)]
    else:
        raise ValueError('No start plan for ' + a.action)
    return {'host': a.host, 'role': a.action, 'run_id': a.run_id,
            'master_uri': 'http://' + IPS[a.host] + ':11431', 'prefix': prefix(a, inside, inner),
            'command': prefix(a, inside, inner) + command, 'output': str(out),
            'log': str(out / (a.action + '.log')), 'raw_recording': False,
            'slam_local': a.host != 'hub', 'gauge_id': gauge, 'gauge_label_is_alignment_proof': False}


def prepare(a, spec):
    root, inside, stage, out, inner = paths(a)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'ros_logs').mkdir(exist_ok=True)
    if a.action == 'supervisor': (root / 'logs').mkdir(exist_ok=True)
    epoch_path = out / 'epoch_config.json'
    epoch = {'run_id': a.run_id, 'cutoff_ns': round(a.cutoff * 1e9), 'master_uri': spec['master_uri'],
             'gauge_id': spec['gauge_id'], 'profile': 'distributed'}
    if epoch_path.exists() and json.loads(epoch_path.read_text()) != epoch:
        raise ValueError('Existing run configuration differs; use a new run ID')
    epoch_path.write_text(json.dumps(epoch, indent=2) + '\n')
    if a.action == 'backend':
        target = out / 'backend'; target.mkdir(exist_ok=True)
        (target / 'robot_names.yaml').write_bytes((root / 'configs' / a.host / 'multimodal/robot_names.yaml').read_bytes())
    if a.action == 'vio':
        source = root / 'configs' / a.host / 'multimodal'
        target = out / 'vio'; target.mkdir(exist_ok=True)
        params = target / 'params'; params.mkdir(exist_ok=True)
        hashes = {}
        for path in sorted((source / 'params').glob('*.yaml')):
            data = path.read_bytes(); hashes[path.name] = hashlib.sha256(data).hexdigest()
            dest = params / path.name
            if dest.exists() and dest.read_bytes() != data: raise ValueError('VIO parameters changed within run')
            dest.write_bytes(data)
        flag_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (source / 'flags').glob('*.flags')}
        if len(flag_hashes) < 4: raise ValueError('Missing per-host native VIO flags')
        package = subprocess.check_output(spec['prefix'] + ['rospack', 'find', 'kimera_vio_ros'], text=True).strip()
        inputs = {'params': hashes, 'flags': flag_hashes, 'package': package}
        inputs_path = target / 'native_inputs.json'
        if inputs_path.exists() and json.loads(inputs_path.read_text()) != inputs: raise ValueError('VIO inputs changed')
        inputs_path.write_text(json.dumps(inputs, indent=2) + '\n')
        doc = ET.parse(source / 'vio.launch')
        include = list(doc.iter('include')); assert len(include) == 1
        include[0].set('file', package + '/launch/kimera_vio_ros.launch')
        values = {'params_folder': str(inner / 'vio/params'), 'flags_folder': str(inside / 'configs' / a.host / 'multimodal/flags'),
                  'log_output': 'false', 'log_output_path': '', 'visualize': 'true', 'use_rviz': 'true'}
        args = {e.get('name'): e for e in include[0].findall('arg')}
        for key, value in values.items():
            if key not in args: args[key] = ET.SubElement(include[0], 'arg', name=key)
            args[key].set('value', value)
        doc.write(target / 'vio.launch', encoding='unicode')


OWNED = r'''
import os,json,signal,time,sys
s=json.loads(sys.argv[1]);found=[]
def scan():
 result=[]
 for n in os.listdir('/proc'):
  if not n.isdigit() or int(n)==os.getpid():continue
  try:
   e=dict(x.split(b'=',1) for x in open('/proc/'+n+'/environ','rb').read().split(b'\0') if b'=' in x)
   stat=open('/proc/'+n+'/stat').read();state=stat[stat.rfind(')')+2:].split()[0]
  except OSError:continue
  if state!='Z' and all(e.get(k.encode())==v.encode() for k,v in s['identity'].items()):result.append(int(n))
 return result
before=scan()
if s.get('stop'):
 for n in before:
  try:os.kill(n,signal.SIGINT)
  except ProcessLookupError:pass
 until=time.monotonic()+12
 while scan() and time.monotonic()<until:time.sleep(.1)
remaining=scan();print(json.dumps({'before':before,'remaining':remaining,'closed':not remaining}))
'''


def ownership(entry, stop=False):
    identity = {'THREE_HOST_RUN_ID': entry['run_id'], 'THREE_HOST_ROLE': 'distributed-' + entry['role'],
                'ROS_MASTER_URI': entry['master_uri'], 'DISTRIBUTED_KIMERA_PROFILE': '1'}
    # Saved docker exec prefix is still used after a source upgrade.
    return json.loads(subprocess.check_output(entry['prefix'] + ['python3', '-c', OWNED,
                      json.dumps({'identity': identity, 'stop': stop})], text=True))


def write_state(path, state):
    temporary = path.with_suffix('.tmp'); temporary.write_text(json.dumps(state, indent=2) + '\n'); temporary.replace(path)


class MasterTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host); connection.timeout = 2.
        return connection


def master_identity(uri):
    with xmlrpc.client.ServerProxy(uri, transport=MasterTransport()) as api:
        code, _, pid = api.getPid('/distributed_manager_readiness')
        status, _, uuid = api.getParam('/distributed_manager_readiness', '/run_id')
    if code != 1 or status != 1 or not isinstance(uuid, str) or not uuid:
        raise RuntimeError('ROS master has not established its run UUID')
    return pid, uuid


def wait_for_owned_master(entry, process, timeout=30.):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if process.poll() is not None: raise RuntimeError('Managed ROS master exited during startup')
        try:
            pid, uuid = master_identity(entry['master_uri'])
            # getPid is inside the relevant native/container PID namespace.
            # The ownership probe uses that same environment and namespace.
            if pid in ownership(entry)['remaining'] and process.poll() is None: return uuid
        except (OSError, xmlrpc.client.Error, RuntimeError): pass
        time.sleep(.1)
    raise RuntimeError('Owned ROS master readiness timeout; no dependent roles started')


def main():
    p = parser(); a = p.parse_args(); validate(a, p)
    if a.dry_run:
        if a.action in ('stop', 'status'): p.error('Dry run requires a start role')
        print(json.dumps(plan(a), indent=2)); return
    root, inside, stage, out, inner = paths(a)
    stage.mkdir(parents=True, exist_ok=True)
    import fcntl
    with (stage / 'manager.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        state_path = stage / 'managed_processes.json'
        state = json.loads(state_path.read_text()) if state_path.exists() else {'profile': 'distributed', 'host': a.host, 'roles': {}}
        if a.action in ('status', 'stop'):
            result = {}
            ordered = sorted(state['roles'].items(), key=lambda item: (
                item[1]['role'] == 'master', item[1]['role'] != 'mission'))
            for key, entry in ordered:
                if entry['run_id'] != a.run_id: continue
                if a.action == 'stop' and a.roles and entry['role'] not in a.roles: continue
                receipt = ownership(entry, a.action == 'stop'); result[key] = receipt
                if a.action == 'stop':
                    entry['stop_receipt'] = receipt
                    if receipt['closed']: entry['stopped_wall'] = time.time()
            if a.action == 'stop': write_state(state_path, state)
            print(json.dumps(result)); return
        key = a.run_id + ':' + a.action
        for entry in state['roles'].values():
            if entry.get('stopped_wall'): continue
            if entry['run_id'] != a.run_id or entry['role'] == a.action:
                if ownership(entry)['remaining']: raise RuntimeError('An owned role is still running')
        if a.action != 'master':
            master = state['roles'].get(a.run_id+':master', {})
            if not master.get('master_uuid'):
                raise RuntimeError('Start and await this run\'s managed master before dependent roles')
            _, current_uuid = master_identity(master['master_uri'])
            if current_uuid != master['master_uuid']:
                raise RuntimeError('ROS master UUID changed; close this run and use a new run')
        spec = plan(a); prepare(a, spec)
        # A reset is an explicit finite local control action, not a daemon.
        with Path(spec['log']).open('a') as log:
            process = subprocess.Popen(spec['command'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        entry = dict(spec, pid=process.pid, started_wall=time.time(), cutoff=a.cutoff)
        state['roles'][key] = entry; write_state(state_path, state)
        if a.action == 'master':
            entry['master_uuid'] = wait_for_owned_master(entry, process)
            entry['ready_wall'] = time.time(); write_state(state_path, state)
        print(json.dumps(entry))


if __name__ == '__main__': main()
