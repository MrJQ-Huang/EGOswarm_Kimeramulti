#!/usr/bin/env python3
"""Own centralized perception, mapping, recording and display for two drones.

Run this on the host; ROS processes execute in the imported native container.
The edge hosts retain EGO planning/control and sensor simulation. All VIO,
incremental TSDF, mesh and cooperative backends run here after one sensor
ingress per robot. Stop requires a saved run/role owner, exact argv selector
and matching ROS master.
The retained backend protocol observes completed optimization windows; this
manager does not enable the later atomic-round extensions.
"""
import argparse
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from urllib.parse import urlparse

DEFAULT_ROOT = '/home/mrjq311/swarm_kimera_runtime/experiment'
DEFAULT_CONTAINER = 'kimera-central-hub-gpu-20261103'
DEFAULT_IMAGE = 'kimera-three-host:uros-snapshot-20261102'
STAGE_NAME = 'three_host_20261102'
ROLES = ('master', 'ingress0', 'ingress1', 'vio0', 'vio1', 'adapter0', 'adapter1',
         'local-frontend0', 'local-frontend1', 'backend0', 'backend1', 'dense0', 'dense1',
         'mission',
         'global-view', 'trajectory', 'world', 'record', 'tsdf-witness',
         'rviz-local0', 'rviz-local1', 'rviz-global', 'rviz-trajectory')
RVIZ_CONFIGS = {
    'rviz-local0': 'rviz_native_amov_cloud.rviz',
    'rviz-local1': 'rviz_native_uros_cloud.rviz',
    'rviz-global': 'rviz_native_global_cloud.rviz',
    'rviz-trajectory': 'rviz_swarm_trajectories.rviz',
}
RVIZ_TILES = {'rviz-local0': (10, 32), 'rviz-local1': (970, 32),
              'rviz-global': (10, 560), 'rviz-trajectory': (970, 560)}
RECORD_PROFILES = ('map', 'replay', 'debug')
VIO_FLAG_FILES = ('Mesher.flags', 'VioBackend.flags',
                  'RegularVioBackend.flags', 'Visualizer3D.flags')
VIO_FLAGS_CODE = '''import hashlib,json,pathlib,sys
folder=pathlib.Path(sys.argv[1])
names=json.loads(sys.argv[2])
missing=[name for name in names if not (folder/name).is_file()]
if missing:raise RuntimeError('Missing native VIO flags in '+str(folder)+': '+', '.join(missing))
print(json.dumps(dict(flags_folder=str(folder),
    flag_sha256={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in names})))'''


def tile_rviz_window(config, role):
    """Touch only top-level Window Geometry scalars, never view/focal axes."""
    marker = '\nWindow Geometry:\n'
    if config.count(marker) != 1:
        raise ValueError('Expected one RViz Window Geometry section')
    prefix, geometry = config.split(marker, 1)
    x, y = RVIZ_TILES[role]
    values = {'Width': 930, 'Height': 470, 'X': x, 'Y': y}
    for name, value in values.items():
        geometry, count = re.subn(r'(?m)^  ' + name + r':[^\n]*$',
                                  '  ' + name + ': ' + str(value), geometry)
        if count != 1:
            raise ValueError('Expected one top-level RViz window ' + name)
    return prefix + marker + geometry


def debug_record_topics():
    """Explicit diagnostic archive; telemetry still crosses the LAN only once."""
    topics = ['/visualization/native_mesh/global_cloud',
              '/visualization/native_mesh/status', '/visualization/swarm/status',
              '/visualization/swarm/markers', '/visualization/swarm/vehicles',
              '/visualization/swarm/environment']
    suffixes = [
        'kimera/rgb/image_raw', 'kimera/depth/image_raw', 'kimera/depth/measured_mask',
        'kimera/rgb/camera_info', 'kimera/depth/camera_info', 'kimera/imu',
        'lidar/points', 'ground_truth/odometry', 'sensors/ready', 'sensors/ingress_status',
        'kimera_vio_ros/odometry', 'kimera_vio_ros/pose_graph_incremental',
        'kimera_vio_ros/bow_query', 'kimera_vio_ros/vlc_frames',
        'kimera_camera_adapter/pose_graph_incremental', 'kimera_camera_adapter/diagnostics',
        'native_dense/local_sensor_status', 'native_dense/rgbd_points',
        'native_tsdf/mesh', 'native_mesh_frontend/full_mesh',
        'native_mesh_frontend/mesh_graph_incremental', 'body/status',
        'kimera_distributed/pose_graph_incremental', 'kimera_distributed/optimized_nodes',
        'kimera_distributed/optimized_path',
        'kimera_distributed/bow_requests', 'kimera_distributed/vlc_requests',
        'kimera_distributed/vlc_responses', 'kimera_distributed/loop_closures',
        'kimera_distributed/loop_ack',
        'dpgo_ros_node/command', 'dpgo_ros_node/status', 'dpgo_ros_node/path',
        'dpgo_ros_node/trajectory', 'dpgo_ros_node/optimized_pose_graph',
        'dpgo_ros_node/measurement_weights',
        'dpgo_ros_node/public_poses', 'dpgo_ros_node/public_measurements',
        'dpgo_ros_node/lifting_matrix', 'dpgo_ros_node/anchor',
        'native_dense/pose_graph_incremental', 'native_dense/mesh_graph_incremental',
        'native_dense/full_mesh', 'native_dense/approved_body_path',
        'native_dense/native_optimized_mesh', 'native_dense/accepted_joint_mesh',
        'native_dense/accepted_joint_vertices', 'native_dense/raw_local_vertices',
        'native_dense/display_vertices', 'native_dense/status',
        'native_pgmo/optimized_mesh', 'native_pgmo/optimized_path',
        'native_pgmo/pose_graph', 'native_pgmo/tf_debug',
    ]
    for robot in (0, 1):
        topics.extend('/sim_%d/%s' % (robot, suffix) for suffix in suffixes)
        topics.extend('/visualization/swarm/%s_%d' % (kind, robot)
                      for kind in ('actual', 'vio', 'optimized'))
        topics.extend('/drone_%d_%s' % (robot, suffix)
                      for suffix in ('planning/pos_cmd', 'planning/bspline'))
        topics.append('/two_pc/drone_%d/goal' % robot)
    topics.extend(['/tf', '/tf_static', '/broadcast_bspline', '/two_pc/scene_state'])
    return list(dict.fromkeys(topics))


def hub_record_topics(profile='map'):
    """Keep map results by default; raw replay and intermediate dumps are opt-in.

    Native pair/mesh receipts are written by the mapping components outside the
    bag. export_final_global_map.py makes their final self-contained artifact.
    An input is still consumed online when it is absent from this record list.
    """
    if profile not in RECORD_PROFILES:
        raise ValueError('Unknown recording profile: ' + str(profile))
    if profile == 'debug':
        return debug_record_topics()
    topics = ['/visualization/native_mesh/global_cloud',
              '/visualization/native_mesh/status', '/two_pc/scene_state', '/tf_static']
    for robot in (0, 1):
        topics.extend('/sim_%d/%s' % (robot, suffix) for suffix in (
            'ground_truth/odometry', 'body/status', 'kimera_vio_ros/odometry',
            'sensors/ready', 'sensors/ingress_status'))
        topics.append('/two_pc/drone_%d/goal' % robot)
        if profile == 'replay':
            topics.extend('/sim_%d/%s' % (robot, suffix) for suffix in (
                'kimera/rgb/image_raw', 'kimera/depth/image_raw',
                'kimera/depth/measured_mask', 'kimera/rgb/camera_info',
                'kimera/depth/camera_info', 'kimera/imu'))
    if profile == 'replay':
        topics.append('/tf')
    return list(dict.fromkeys(topics))


def host_start_ticks(pid):
    try:
        raw = Path('/proc', str(pid), 'stat').read_text()
        tail = raw[raw.rfind(')') + 2:].split()
        return None if tail[0] == 'Z' else int(tail[19])
    except (FileNotFoundError, ProcessLookupError):
        return None


def host_process_alive(entry):
    actual = host_start_ticks(entry['pid'])
    return actual is not None and actual == entry.get('host_start_ticks')


def validate(args, parser):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', args.run_id):
        parser.error('run-id must be a simple unique name')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', args.container):
        parser.error('container must be a simple Docker container name')
    uri = urlparse(args.master_uri)
    if (uri.scheme != 'http' or uri.hostname not in ('192.168.5.14', '127.0.0.1')
            or uri.port not in (11431, 11561) or uri.username or uri.password
            or uri.query or uri.fragment or uri.path not in ('', '/')):
        parser.error('Use the dedicated hub master 11431 or isolated test master 11561')
    args.master_uri = args.master_uri.rstrip('/')
    if args.cutoff is not None and (not math.isfinite(args.cutoff) or args.cutoff <= 0):
        parser.error('Epoch cutoff must be finite and positive')
    if args.action in ('ingress0', 'ingress1', 'local-frontend0', 'local-frontend1',
                       'dense0', 'dense1', 'global-view', 'trajectory', 'tsdf-witness'):
        if args.cutoff is None:
            parser.error('This role requires an explicit positive epoch cutoff')
    if args.roles and args.action != 'stop':
        parser.error('--roles is only valid for stop')
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('record duration must be finite and positive')
    if not math.isfinite(args.mesh_phase_timeout) or args.mesh_phase_timeout <= 0:
        parser.error('Mesh timeout must be finite and positive')
    for label, value in (('cloud rate', args.cloud_rate), ('mesh period', args.mesh_period),
                         ('trajectory rate', args.trajectory_rate)):
        if not math.isfinite(value) or value <= 0:
            parser.error(label + ' must be finite and positive')
    if args.trajectory_rate > 60:
        parser.error('Trajectory publishing rate must not exceed 60 Hz')
    if not 1 <= args.rviz_fps <= 120:
        parser.error('RViz FPS must be between 1 and 120')
    if args.laps < 1:
        parser.error('Laps must be positive')


def command_for(args, inner_stage, harness, output):
    """Return exact ROS argv and its independent ownership selector."""
    action = args.action
    if action == 'master':
        port = str(urlparse(args.master_uri).port)
        return ['roscore', '-p', port], ['/opt/ros/noetic/bin/roscore', '-p', port]
    if action == 'record':
        target = str(output / 'hub_native.bag')
        command = ['rosbag', 'record', '--lz4', '--duration=' + str(args.duration),
                   '-O', target] + hub_record_topics(args.record_profile)
        return command, ['-O', target]
    if action == 'tsdf-witness':
        target = str(inner_stage / 'native_tsdf_epoch_witness.py')
        command = ['python3', target, '--run-id', args.run_id, '--cutoff', str(args.cutoff),
                   '--master-uri', args.master_uri, '--epoch-file', str(output / 'epoch_config.json'),
                   '--ros-log-root', str(output / 'ros_logs'),
                   '--output-directory', str(output / 'tsdf_witness'), '--duration', str(args.duration)]
        return command, [target, '--run-id', args.run_id]
    if action.startswith('ingress'):
        rid = int(action[-1])
        target = str(harness / 'sensor_ingress.py')
        command = ['python3', target, '--robot-id', str(rid), '--run-id', args.run_id,
                   '--cutoff', str(args.cutoff)]
        return command, [target, '--robot-id', str(rid), '--run-id', args.run_id]
    if action.startswith('vio'):
        target = str(output / action / 'vio.launch')
        return ['roslaunch', '--screen', target], [target]
    if action.startswith('adapter'):
        rid = int(action[-1])
        target = str(harness / 'native_vio_graph_adapter.py')
        command = ['python3', target, '--robot-id', str(rid), '--expected-caller',
                   '/sim_%d/kimera_vio_ros/kimera_vio_ros_node' % rid]
        return command, [target, '--robot-id', str(rid)]
    if action.startswith('local-frontend'):
        rid = int(action[-1])
        target = str(harness / 'local_frontend.launch')
        command = ['roslaunch', '--screen', target, 'robot_id:=' + str(rid),
                   'robot_name:=sim_' + str(rid), 'run_id:=' + args.run_id,
                   'cutoff:=' + str(args.cutoff), 'harness_directory:=' + str(harness),
                   'output_directory:=' + str(output / action),
                   'cloud_rate:=' + str(args.cloud_rate), 'mesh_period:=' + str(args.mesh_period)]
        return command, [target, 'run_id:=' + args.run_id, 'robot_id:=' + str(rid)]
    if action.startswith('backend'):
        rid = int(action[-1])
        target = str(harness / 'backend.launch')
        return (['roslaunch', '--screen', target, 'robot_id:=' + str(rid),
                 'robot_name:=sim_' + str(rid), 'runtime_dir:=' + str(output / action),
                 'odom_suffix:=vio_odom',
                 'local_pose_graph_topic:=/sim_%d/kimera_camera_adapter/pose_graph_incremental' % rid],
                [target, 'robot_id:=' + str(rid)])
    if action.startswith('dense'):
        rid = int(action[-1])
        target = str(harness / 'hub_backend.launch')
        command = ['roslaunch', '--screen', target, 'robot_id:=' + str(rid),
                   'robot_name:=sim_' + str(rid), 'run_id:=' + args.run_id,
                   'cutoff:=' + str(args.cutoff), 'harness_directory:=' + str(harness),
                   'output_directory:=' + str(output / action),
                   'require_cross:=true', 'mesh_phase_timeout:=' + str(args.mesh_phase_timeout)]
        return command, [target, 'run_id:=' + args.run_id, 'robot_id:=' + str(rid)]
    if action == 'global-view':
        target = str(harness / 'native_dense_global_view.py')
        command = ['python3', target, '--run-id', args.run_id, '--cutoff', str(args.cutoff),
                   '--output-directory', str(output / 'global_display')]
        return command, [target, '--run-id', args.run_id]
    if action == 'trajectory':
        target = str(harness / 'live_trajectory_view.py')
        command = ['python3', target, '--run-id', args.run_id, '--cutoff', str(args.cutoff),
                   '--scene', '/experiment/harness/broad_loop_scene.json',
                   '--output', str(output / 'trajectory_view.json'),
                   '--publish-rate', str(args.trajectory_rate), '--max-points', '4000',
                   '--central-telemetry']
        return command, [target, '--run-id', args.run_id]
    if action == 'mission':
        target = str(harness / 'closed_loop_missions.py')
        result = str(output / (args.run_id + '_mission.json'))
        command = ['python3', target, '--frontend', 'vio', '--route',
                   '/experiment/harness/broad_loop_route.json', '--laps', str(args.laps),
                   '--output', result, '--central-telemetry']
        return command, [target, '--output', result]
    if action == 'world':
        target = '/experiment/harness/complex_world_view.py'
        return (['python3', target, '--scene', '/experiment/harness/broad_loop_scene.json',
                 '--central-telemetry'], [target])
    if action in RVIZ_CONFIGS:
        config = str(output / 'rviz' / RVIZ_CONFIGS[action])
        command = ['rviz', '-d', config, '__name:=' + action.replace('-', '_') + '_hub']
        return command, ['-d', config]
    raise ValueError('Unsupported start action: ' + action)


def read_state(path):
    return json.loads(path.read_text()) if path.exists() else {'host': 'mrjq311', 'roles': {}}


def write_state(path, state):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def bind_epoch(args, output):
    path = output / 'epoch_config.json'
    expected = dict(run_id=args.run_id, master_uri=args.master_uri,
                    cutoff_ns=round(args.cutoff*1e9) if args.cutoff is not None else None)
    if path.exists():
        prior = json.loads(path.read_text())
        if any(prior.get(key) != expected[key] for key in ('run_id','master_uri')):
            raise RuntimeError('Existing run/master identity differs')
        if prior.get('cutoff_ns') is not None:
            if expected['cutoff_ns'] is not None and prior['cutoff_ns'] != expected['cutoff_ns']:
                raise RuntimeError('Existing run cutoff differs; use a fresh run ID')
            return
    write_state(path, expected)


# The Docker exec CLI PID is not the actual ROS process PID. Query inside the
# correct container and require the inherited owner plus exact argv tokens.
OWNED_PROCESS_CODE = r'''
import json,os,signal,sys,time
from pathlib import Path
spec=json.loads(sys.argv[1]); found=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  argv=[x.decode(errors='replace') for x in path.read_bytes().split(b'\0') if x]
  raw=path.with_name('environ').read_bytes().split(b'\0')
  env={x.split(b'=',1)[0].decode():x.split(b'=',1)[1].decode(errors='replace') for x in raw if b'=' in x}
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
 if not all(token in argv for token in spec['selector']):continue
 if env.get('THREE_HOST_RUN_ID') != spec['run_id']:continue
 if env.get('THREE_HOST_ROLE') != spec['role']:continue
 if env.get('ROS_MASTER_URI','').rstrip('/') != spec['master_uri']:continue
 pid=int(path.parent.name)
 if pid==os.getpid():continue
 item={'pid':pid,'argv':argv}
 if spec.get('signal'):
  try:os.kill(pid,signal.SIGINT);item['signalled']='SIGINT'
  except ProcessLookupError:item['already_exited']=True
 found.append(item)
print(json.dumps({'wall':time.time(),'matches':found}))
'''


def owned_query(entry, signal_process=False):
    spec = {key: entry[key] for key in ('run_id', 'role', 'selector', 'master_uri')}
    spec['signal'] = bool(signal_process)
    raw = subprocess.check_output(['docker', 'exec', entry['container'], 'python3', '-c',
                                  OWNED_PROCESS_CODE, json.dumps(spec)], text=True)
    return json.loads(raw)


def resolve_vio_package(args):
    """Resolve and check the installed VIO launch inside the selected container."""
    code = '''import json,pathlib,subprocess
package=subprocess.check_output(['rospack','find','kimera_vio_ros'],text=True).strip()
launch=pathlib.Path(package)/'launch/kimera_vio_ros.launch'
if not launch.is_file():raise RuntimeError('Missing native VIO launch: '+str(launch))
print(json.dumps(package))'''
    raw = subprocess.check_output([
        'docker', 'exec', args.container, 'bash',
        '/experiment/' + STAGE_NAME + '/env_hub.bash', 'python3', '-c', code], text=True)
    package = json.loads(raw)
    if not isinstance(package, str) or not PurePosixPath(package).is_absolute():
        raise RuntimeError('rospack returned an invalid native VIO package path')
    return package


def resolve_vio_flags(args, robot):
    """Check the exact legacy flags for this robot in the selected container."""
    host = 'amov' if robot == 0 else 'uros'
    folder = '/experiment/configs/' + host + '/multimodal/flags'
    raw = subprocess.check_output([
        'docker', 'exec', args.container, 'python3', '-c', VIO_FLAGS_CODE,
        folder, json.dumps(VIO_FLAG_FILES)], text=True)
    return json.loads(raw)


def prepare_vio_files(root, output, inner_output, robot, package, flags):
    """Keep host-specific calibration and rewrite all machine-specific VIO paths."""
    host = 'amov' if robot == 0 else 'uros'
    source = root / 'configs' / host / 'multimodal'
    runtime = output / ('vio' + str(robot))
    inner = inner_output / ('vio' + str(robot))
    runtime.mkdir(parents=True, exist_ok=True)
    params = runtime / 'params'
    if not params.exists():
        shutil.copytree(str(source / 'params'), str(params))
    expected = {str(p.relative_to(source / 'params')): p.read_bytes()
                for p in (source / 'params').rglob('*') if p.is_file()}
    observed = {str(p.relative_to(params)): p.read_bytes()
                for p in params.rglob('*') if p.is_file()}
    if observed != expected:
        raise RuntimeError('Native VIO parameters changed within this epoch; use a fresh run ID')
    inputs_path = runtime / 'native_inputs.json'
    native_inputs = dict(package=package, **flags)
    if inputs_path.exists() and json.loads(inputs_path.read_text()) != native_inputs:
        raise RuntimeError('Native VIO launch or flags changed within this epoch; use a fresh run ID')
    inputs_path.write_text(json.dumps(native_inputs, indent=2) + '\n')
    (runtime / 'vio_logs').mkdir(exist_ok=True)
    document = ET.parse(str(source / 'vio.launch'))
    includes = list(document.iter('include'))
    if len(includes) != 1:
        raise RuntimeError('Expected one native VIO include in the host configuration')
    includes[0].set('file', str(PurePosixPath(package) / 'launch/kimera_vio_ros.launch'))
    # The checked-in calibration contains YAML only. Preserve each device's
    # actual legacy flags separately instead of the native launch's implicit
    # params_folder/flags fallback or a different dataset's defaults.
    flag_args = [a for a in includes[0].findall('arg') if a.get('name') == 'flags_folder']
    if not flag_args:
        flag_args = [ET.SubElement(includes[0], 'arg', name='flags_folder')]
    for argument in flag_args:
        argument.set('value', flags['flags_folder'])
    values = {'robot_id': str(robot), 'params_folder': str(inner / 'params'),
              'log_output_path': str(inner / 'vio_logs'), 'log_output': 'true',
              'visualize': 'true', 'use_rviz': 'true', 'multirobot': 'true'}
    for argument in document.iter('arg'):
        if argument.get('name') in values:
            argument.set('value', values[argument.get('name')])
    document.write(str(runtime / 'vio.launch'), encoding='unicode')


def prepare_role_files(args, root, stage, output):
    if args.action == 'record':
        (output / 'hub_record_topics.json').write_text(json.dumps({
            'run_id': args.run_id, 'duration_s': args.duration, 'record_profile': args.record_profile,
            'topics': hub_record_topics(args.record_profile),
            'raw_lan_sensor_or_frontend_inputs_repeated': False,
            'local_edge_raw_recorders_enabled': False,
            'raw_sensor_archive_location': ('hub-local normalized sensor ingress topics'
                                            if args.record_profile != 'map' else None),
            'raw_inputs_consumed_online_in_all_profiles': True,
            'final_map_export_required': True,
            'final_map_export_tool': '/experiment/harness/multimodal/export_final_global_map.py',
            'native_final_map_evidence': 'Accepted native pair and original mesh archives, separate from bag',
            'map_profile_is_not_a_raw_sensor_replay_archive': args.record_profile == 'map',
            'calibration_ros_parameters': ['/sim_0/sensors/calibration', '/sim_1/sensors/calibration'],
            'calibration_note': 'ROS parameters require a separate snapshot after sensors start; they are not topics',
        }, indent=2) + '\n')
    if args.action.startswith('vio'):
        inner_output = PurePosixPath('/experiment') / STAGE_NAME / 'runs' / args.run_id / 'mrjq311'
        robot = int(args.action[-1])
        prepare_vio_files(root, output, inner_output, robot,
                          resolve_vio_package(args), resolve_vio_flags(args, robot))
    if args.action.startswith(('dense', 'local-frontend')):
        (output / args.action).mkdir(parents=True, exist_ok=True)
    if args.action.startswith('backend'):
        runtime = output / args.action
        (runtime / 'lcd').mkdir(parents=True, exist_ok=True)
        (runtime / 'dpgo').mkdir(parents=True, exist_ok=True)
        source = root / 'multimodal/robot_names.yaml'
        (runtime / 'robot_names.yaml').write_bytes(source.read_bytes())
    if args.action in RVIZ_CONFIGS:
        source = root / 'harness/multimodal' / RVIZ_CONFIGS[args.action]
        target = output / 'rviz' / source.name
        target.parent.mkdir(parents=True, exist_ok=True)
        config = source.read_text()
        # Control only rendering load; fixed frames and actual native topics
        # remain as committed in the audited source configuration.
        config = re.sub(r'(?m)^(\s*Frame Rate:)\s*.*$',
                        lambda match: match[1] + ' ' + str(args.rviz_fps), config)
        config = tile_rviz_window(config, args.action)
        target.write_text(config)


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=ROLES + ('status', 'stop'))
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--root', default=DEFAULT_ROOT)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--master-uri', default='http://192.168.5.14:11431')
    parser.add_argument('--cutoff', type=float)
    parser.add_argument('--opengv-guard', action='store_true')
    parser.add_argument('--mesh-phase-timeout', type=float, default=30.)
    parser.add_argument('--cloud-rate', type=float, default=5.,
                        help='Independent local TSDF point-cloud input rate in Hz')
    parser.add_argument('--mesh-period', type=float, default=1.,
                        help='Native mesh generation period in seconds')
    parser.add_argument('--rviz-fps', type=int, default=30,
                        help='RViz redraw ceiling; independent of mapping and sensor rates')
    parser.add_argument('--trajectory-rate', type=float, default=20.,
                        help='Vehicle/trajectory visualization publishing rate in Hz')
    parser.add_argument('--laps', type=int, default=2)
    parser.add_argument('--duration', type=float, default=1500.,
                        help='Finite rosbag record duration in seconds (record role)')
    parser.add_argument('--record-profile', choices=RECORD_PROFILES, default='map',
                        help='map: result/telemetry; replay: add RGB-D/IMU; debug: all intermediate topics')
    parser.add_argument('--roles', choices=ROLES, nargs='+')
    parser.add_argument('--stop-master', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main():
    parser = argument_parser()
    args = parser.parse_args()
    validate(args, parser)
    root = Path(args.root)
    stage = root / STAGE_NAME
    inner_stage = PurePosixPath('/experiment') / STAGE_NAME
    harness = PurePosixPath('/experiment/harness/multimodal')
    inner_output = inner_stage / 'runs' / args.run_id / 'mrjq311'
    output = stage / 'runs' / args.run_id / 'mrjq311'
    state_path = stage / 'managed_processes.json'
    if args.action not in ('status', 'stop'):
        command, selector = command_for(args, inner_stage, harness, inner_output)
        prefix = ['docker', 'exec', '-e', 'THREE_HOST_RUN_ID=' + args.run_id,
                  '-e', 'THREE_HOST_ROLE=' + args.action,
                  '-e', 'NATIVE_ROS_MASTER_URI=' + args.master_uri,
                  '-e', 'THREE_HOST_ROS_LOG_DIR=' + str(inner_output / 'ros_logs')]
        if args.opengv_guard:
            prefix += ['-e', 'NATIVE_USE_OPENGV_GUARD=1']
        prefix += [args.container, 'bash', str(inner_stage / 'env_hub.bash')]
        if args.action != 'master':
            command = ['nice', '-n', '5'] + command
        if args.dry_run:
            print(json.dumps({'role': args.action, 'command': prefix + command,
                              'selector': selector, 'output': str(output)}, indent=2))
            return
    stage.mkdir(parents=True, exist_ok=True)
    # Prevent concurrent state writers. The lock is automatically released if
    # an SSH session fails; no permanently stale .lock state is possible.
    import fcntl
    with (stage / 'manager.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        state = read_state(state_path)
        if args.action == 'status':
            result = {}
            for key, entry in state['roles'].items():
                if entry['run_id'] != args.run_id:
                    continue
                item = dict(entry, host_cli_alive=host_process_alive(entry))
                try:
                    item['container_processes'] = owned_query(entry)
                except subprocess.CalledProcessError as exc:
                    item['container_query_error'] = str(exc)
                result[key] = item
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return
        if args.action == 'stop':
            results = []
            # Stop consumers/GUI first and the master last when explicitly asked.
            entries = sorted(state['roles'].items(), key=lambda pair: pair[1]['role'] == 'master')
            for key, entry in entries:
                if entry['run_id'] != args.run_id or entry.get('stopped_wall'):
                    continue
                if args.roles and entry['role'] not in args.roles:
                    continue
                if entry['role'] == 'master' and not args.stop_master:
                    continue
                result = owned_query(entry, signal_process=True)
                # An empty match is also evidence: it never authorizes another
                # run or an unrelated process with a reused OS PID to be killed.
                entry['stop_requested_wall'] = time.time()
                entry['stop_receipt'] = result
                remaining = None
                for _ in range(30):
                    remaining = owned_query(entry)
                    if not remaining['matches']:
                        break
                    time.sleep(.2)
                result['remaining'] = remaining['matches']
                if not remaining['matches']:
                    entry['stopped_wall'] = time.time()
                results.append({'role': entry['role'], 'result': result})
            write_state(state_path, state)
            print(json.dumps({'run_id': args.run_id, 'stops': results,
                              'master_preserved': not args.stop_master}, ensure_ascii=False, indent=2))
            return
        # Check actual container processes as well as the outer Docker CLI.
        # A disconnected CLI cannot make a still-running ROS role ownerless.
        for key, entry in state['roles'].items():
            if entry.get('stopped_wall'):
                continue
            is_other_run = entry['run_id'] != args.run_id
            same_role = entry['run_id'] == args.run_id and entry['role'] == args.action
            if not (is_other_run or same_role):
                continue
            if owned_query(entry)['matches']:
                raise RuntimeError('An owned role is still active; stop it explicitly: ' + key)
        inspect = json.loads(subprocess.check_output(['docker', 'inspect', args.container], text=True))[0]
        if not inspect['State']['Running']:
            raise RuntimeError('The native hub container is not running')
        network = inspect['HostConfig']['NetworkMode']
        if network != 'host':
            raise RuntimeError('The ROS1 hub requires Docker host networking')
        output.mkdir(parents=True, exist_ok=True)
        bind_epoch(args, output)
        prepare_role_files(args, root, stage, output)
        logfile = output / (args.action + '.log')
        with logfile.open('a') as stream:
            process = subprocess.Popen(prefix + command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        entry = {'role': args.action, 'run_id': args.run_id, 'container': args.container,
                 'pid': process.pid, 'host_start_ticks': host_start_ticks(process.pid),
                 'started_wall': time.time(), 'command': prefix + command,
                 'selector': selector, 'master_uri': args.master_uri,
                 'opengv_guard': args.opengv_guard, 'log': str(logfile),
                 'cutoff_ns': round(args.cutoff*1e9) if args.cutoff is not None else None}
        state['roles'][args.run_id + ':' + args.action] = entry
        write_state(state_path, state)
        print(json.dumps(entry, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
