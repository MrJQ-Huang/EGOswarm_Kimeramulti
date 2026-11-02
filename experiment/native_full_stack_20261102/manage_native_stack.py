#!/usr/bin/env python3
"""Manage only the isolated native geometry experiment, with per-run ownership."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time
import shutil
import xml.etree.ElementTree as ET
from urllib.parse import urlparse


def running(pid):
    try:
        return Path('/proc', str(pid), 'stat').read_text().split()[2] != 'Z'
    except FileNotFoundError:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('master', 'vio', 'adapter', 'backend', 'heartbeat',
                                          'dense', 'global-view', 'record', 'supervisor', 'sensors',
                                          'world', 'mission', 'trajectory', 'status', 'stop'))
    parser.add_argument('--host', choices=('uros', 'amov'), required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', type=float)
    parser.add_argument('--mode', choices=('local', 'joint'), default='local')
    parser.add_argument('--require-cross', action='store_true')
    parser.add_argument('--recorded-native-bag')
    parser.add_argument('--recorded-native-bag-sha256')
    parser.add_argument('--duration', type=float, default=240.)
    parser.add_argument('--cloud-rate', type=float, default=1.)
    parser.add_argument('--container', default='kimera-native-full-uros-20261102')
    parser.add_argument('--ego-container', default='ego-native-full-uros-20261102')
    parser.add_argument('--laps', type=int, default=2)
    parser.add_argument('--opengv-guard', action='store_true',
                        help='Explicitly load the separately tested native reserve-exception guard')
    parser.add_argument('--master-uri', default='http://192.168.5.21:11431')
    parser.add_argument('--stop-master', action='store_true')
    parser.add_argument('--roles', nargs='+', help='Stop only these owned roles in this run')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', args.run_id):
        parser.error('run-id must be a simple unique name')
    uri = urlparse(args.master_uri)
    if uri.scheme != 'http' or uri.hostname not in ('127.0.0.1', '192.168.5.21') or uri.port not in (11431, 11561):
        parser.error('Use the dedicated shared master 11431 or isolated local test master 11561')
    args.master_uri = args.master_uri.rstrip('/')
    root = Path('/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030'
                if args.host == 'uros' else '/home/amov/Drone/two_pc_ego_20261030')
    stage = root/'native_full_stack_20261102'
    stage.mkdir(exist_ok=True)
    inside = Path('/experiment') if args.host == 'uros' else root
    inner_stage = inside/'native_full_stack_20261102'
    harness = inside/'harness/multimodal'
    rid = 1 if args.host == 'uros' else 0
    env = inner_stage/('env_'+args.host+'_full.bash')
    prefix = (['docker', 'exec', args.container] if args.host == 'uros' else [])
    prefix += ['env', 'NATIVE_ROS_MASTER_URI='+args.master_uri, 'bash', str(env)]
    if args.opengv_guard:
        prefix.insert(prefix.index('NATIVE_ROS_MASTER_URI='+args.master_uri), 'NATIVE_USE_OPENGV_GUARD=1')
    if args.action in ('supervisor', 'sensors', 'world', 'mission'):
        ego_env = inside/'harness'/('env_uros.bash' if args.host == 'uros' else 'env_amov.bash')
        prefix = (['docker','exec',args.ego_container] if args.host == 'uros' else [])
        prefix += ['bash',str(ego_env),'env','ROS_MASTER_URI='+args.master_uri,
                   'ROS_LOG_DIR='+str(inner_stage/'logs/ego_ros'),
                   'OMP_NUM_THREADS=1','OPENBLAS_NUM_THREADS=1']
    state_file = stage/'managed_processes.json'
    state = json.loads(state_file.read_text()) if state_file.exists() else {'host': args.host, 'roles': {}}
    if args.action == 'status':
        print(json.dumps({k: dict(v, alive=running(v['pid'])) for k,v in state['roles'].items()}, indent=2))
        return
    role_key = args.run_id+':'+args.action
    output = inner_stage/'runs'/args.run_id/args.host
    (stage/'runs'/args.run_id/args.host).mkdir(parents=True, exist_ok=True)
    if args.action == 'stop':
        results = []
        code = '''import json,os,signal,sys,time
from pathlib import Path
data=json.loads(sys.argv[1]);found=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  argv=[x.decode(errors='replace') for x in path.read_bytes().split(b'\\0') if x]
  env=path.with_name('environ').read_bytes().split(b'\\0')
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
 if not all(value in argv for value in data['selector']):continue
 actual=[v.split(b'=',1)[1].decode().rstrip('/') for v in env if v.startswith(b'ROS_MASTER_URI=')]
 if actual != [data['master_uri']]:continue
 pid=int(path.parent.name)
 if pid==os.getpid():continue
 os.kill(pid,signal.SIGINT);found.append({'pid':pid,'args':argv})
print(json.dumps({'wall':time.time(),'signalled':found}))'''
        for key, entry in state['roles'].items():
            if entry.get('run_id') != args.run_id or entry.get('stopped_wall'):
                continue
            if args.roles and entry['role'] not in args.roles:
                continue
            if entry['role'] == 'master' and not args.stop_master:
                continue  # Shared master has an independent lifecycle.
            if not entry.get('stop_selector'):
                raise RuntimeError('Missing process ownership selector')
            stop_prefix = entry.get('stop_prefix', prefix)
            response = subprocess.check_output(stop_prefix+['python3', '-c', code,
                        json.dumps({'selector': entry['stop_selector'], 'master_uri':entry['master_uri']})], text=True)
            results.append({'role':entry['role'], 'result':json.loads(response)})
            entry['stopped_wall'] = time.time()
        state_file.write_text(json.dumps(state, indent=2)+'\n')
        print(json.dumps({'run_id':args.run_id, 'stops':results, 'master_preserved':not args.stop_master}))
        return
    for key, entry in state['roles'].items():
        if running(entry['pid']) and entry['role'] != 'master' and entry['run_id'] != args.run_id:
            raise RuntimeError('Another managed native run is active; stop it before a new epoch: '+key)
    old = state['roles'].get(role_key)
    if old and running(old['pid']):
        raise RuntimeError('Role already running: '+role_key)
    selector = []
    if args.action == 'master':
        if args.host != 'uros': parser.error('Shared master belongs to UROS')
        command = ['roscore', '-p', str(uri.port)]; selector = ['/opt/ros/noetic/bin/roscore', '-p', str(uri.port)]
    elif args.action == 'vio':
        (stage/'runs'/args.run_id/args.host/'vio_logs').mkdir(parents=True,exist_ok=True)
        host_params = stage/'runs'/args.run_id/args.host/'params'
        if not host_params.exists(): shutil.copytree(str(root/'multimodal/params'),str(host_params))
        document = ET.parse(str(root/'multimodal/vio.launch'))
        for argument in document.iter('arg'):
            if argument.get('name') == 'log_output_path':
                argument.set('value',str(output/'vio_logs'))
            if argument.get('name') == 'params_folder':
                argument.set('value',str(output/'params'))
        target = str(output/'vio.launch')
        host_target = stage/'runs'/args.run_id/args.host/'vio.launch'
        document.write(str(host_target),encoding='unicode')
        command = ['roslaunch', '--screen', target]; selector = [target]
    elif args.action == 'adapter':
        target = str(harness/'native_vio_graph_adapter.py')
        command = ['python3', target, '--robot-id', str(rid), '--expected-caller',
                   '/sim_%d/kimera_vio_ros/kimera_vio_ros_node'%rid]
        selector = [target, '--robot-id', str(rid)]
    elif args.action == 'backend':
        target = str(harness/'backend.launch')
        host_backend = stage/'runs'/args.run_id/args.host/'backend'
        for name in ('lcd','dpgo'): (host_backend/name).mkdir(parents=True,exist_ok=True)
        shutil.copyfile(str(root/'multimodal/robot_names.yaml'),str(host_backend/'robot_names.yaml'))
        command = ['roslaunch', '--screen', target, 'robot_id:='+str(rid), 'robot_name:=sim_'+str(rid),
                   'runtime_dir:='+str(output/'backend'), 'odom_suffix:=vio_odom',
                   'local_pose_graph_topic:=/sim_%d/kimera_camera_adapter/pose_graph_incremental'%rid]
        selector = [target, 'robot_id:='+str(rid)]
    elif args.action == 'heartbeat':
        target = str(harness/'peer_heartbeat.py')
        command = ['python3', target, '--robot-id', str(rid)]; selector = [target, '--robot-id', str(rid)]
    elif args.action == 'dense':
        if args.cutoff is None or args.cutoff <= 0: parser.error('An explicit positive epoch cutoff is required')
        target = str(harness/'native_dense.launch')
        command = ['roslaunch', '--screen', target, 'robot_id:='+str(rid), 'robot_name:=sim_'+str(rid),
                   'run_id:='+args.run_id, 'cutoff:='+str(args.cutoff), 'mode:='+args.mode,
                   'harness_directory:='+str(harness), 'output_directory:='+str(output/'dense'),
                   'cloud_rate:='+str(args.cloud_rate),
                   'require_cross:='+str(args.require_cross).lower()]
        if args.recorded_native_bag:
            if not args.recorded_native_bag_sha256: parser.error('Recorded native fixture SHA256 required')
            if args.mode != 'local': parser.error('Recorded-input replay is a local interface test only')
            command += ['recorded_native_bag:='+args.recorded_native_bag,
                        'recorded_native_bag_sha256:='+args.recorded_native_bag_sha256]
        selector = [target, 'run_id:='+args.run_id]
    elif args.action == 'global-view':
        if args.host != 'uros' or args.cutoff is None: parser.error('Global view requires UROS and an epoch')
        target = str(harness/'native_dense_global_view.py')
        command = ['python3', target, '--run-id', args.run_id, '--cutoff', str(args.cutoff),
                   '--output-directory', str(output/'global_display')]
        selector = [target, '--run-id', args.run_id]
    elif args.action == 'record':
        bag = str(output/'native_inputs_and_meshes.bag')
        base = '/sim_%d/'%rid
        names = ['kimera/rgb/image_raw', 'kimera/depth/image_raw', 'kimera/rgb/camera_info', 'kimera/imu',
                 'kimera_vio_ros/odometry', 'kimera_vio_ros/pose_graph_incremental',
                 'kimera_camera_adapter/pose_graph_incremental', 'kimera_distributed/optimized_nodes',
                 'kimera_distributed/loop_closures', 'dpgo_ros_node/command', 'dpgo_ros_node/path',
                 'dpgo_ros_node/measurement_weights', 'native_tsdf/mesh', 'native_dense/status',
                 'native_dense/accepted_joint_mesh', 'native_pgmo/optimized_mesh', 'native_pgmo/pose_graph']
        names += ['kimera/depth/measured_mask', 'lidar/points', 'ground_truth/odometry',
                  'kimera_vio_ros/bow_query', 'kimera_vio_ros/vlc_frames',
                  'kimera_camera_adapter/diagnostics', 'dpgo_ros_node/status', 'body/status',
                  'native_mesh_frontend/full_mesh', 'native_dense/mesh_graph_incremental',
                  'native_pgmo/optimized_path']
        # Preserve the actual PGMO wire inputs, not only the upstream originals.
        # Their arrival order determines native pose/mesh association. The exact
        # submitted body path cannot be recovered byte-for-byte from camera
        # graph metadata after the live process has stopped.
        names += ['native_dense/pose_graph_incremental', 'native_dense/full_mesh',
                  'native_dense/approved_body_path', 'native_dense/rgbd_points',
                  'native_dense/raw_local_vertices',
                  'native_mesh_frontend/mesh_graph_incremental',
                  'kimera_distributed/pose_graph_incremental',
                  'dpgo_ros_node/optimized_pose_graph', 'dpgo_ros_node/public_measurements',
                  'dpgo_ros_node/public_poses', 'dpgo_ros_node/anchor',
                  'dpgo_ros_node/lifting_matrix', 'dpgo_ros_node/trajectory',
                  'dpgo_ros_node/loop_closures',
                  'connected_peer_ids']
        command = ['rosbag', 'record', '--lz4', '--duration='+str(args.duration), '-O', bag]
        command += [base+name for name in names]
        command += ['/drone_%d_planning/pos_cmd'%rid, '/drone_%d_visual_slam/odom'%rid,
                    '/two_pc/drone_%d/goal'%rid, '/broadcast_bspline', '/two_pc/scene_state']
        command += ['/tf', '/tf_static', '/visualization/native_mesh/global_cloud',
                    '/visualization/native_mesh/status', '/visualization/swarm/status']
        command += ['/visualization/swarm/'+kind+'_'+str(rid)
                    for kind in ('actual', 'vio', 'optimized')]
        selector = ['/opt/ros/noetic/lib/rosbag/record', bag]
    elif args.action == 'supervisor':
        target = str(inside/'harness/scenario_supervisor.py')
        command = ['python3',target,'--drone-id',str(rid),'--root',str(inside),
                   '--scene',str(inside/'harness/broad_loop_scene.json')]
        selector = [target,'--drone-id',str(rid)]
    elif args.action == 'sensors':
        target = str(harness/'sim_sensors.py')
        command = ['python3',target,'--scene',str(inside/'harness/broad_loop_scene.json'),
                   '--drone-id',str(rid),'--camera-rate','10','--lidar-rate','10',
                   '--warmup-seconds','8','--body-imu']
        selector = [target,'--drone-id',str(rid)]
    elif args.action == 'world':
        if args.host != 'uros': parser.error('World is a single simulator environment publisher on UROS')
        target = str(inside/'harness/complex_world_view.py')
        command = ['python3',target,'--scene',str(inside/'harness/broad_loop_scene.json')]
        selector = [target]
    elif args.action == 'mission':
        if args.host != 'uros' or args.laps < 1: parser.error('Mission requires UROS and positive laps')
        target = str(harness/'closed_loop_missions.py')
        command = ['python3',target,'--frontend','vio','--route',str(inside/'harness/broad_loop_route.json'),
                   '--laps',str(args.laps),'--output',str(output/'closed_loop_mission.json')]
        selector = [target,'--output',str(output/'closed_loop_mission.json')]
    elif args.action == 'trajectory':
        if args.host != 'uros' or args.cutoff is None: parser.error('Trajectory requires UROS and an epoch')
        target = str(harness/'live_trajectory_view.py')
        command = ['python3',target,'--run-id',args.run_id,'--cutoff',str(args.cutoff),
                   '--scene',str(inside/'harness/broad_loop_scene.json'),
                   '--output',str(output/'trajectory_view.json')]
        command += ['--publish-rate','0.5','--max-points','4000']
        selector = [target,'--run-id',args.run_id]
    logfile = stage/'runs'/args.run_id/args.host/(args.action+'.log')
    if args.action in ('dense','global-view','trajectory'):
        command = ['nice','-n','10']+command
    with logfile.open('a') as stream:
        process = subprocess.Popen(prefix+command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    state['roles'][role_key] = dict(role=args.action, run_id=args.run_id, pid=process.pid,
                                   started_wall=time.time(), command=prefix+command,
                                   master_uri=args.master_uri,
                                   opengv_reserve_guard_enabled=args.opengv_guard,
                                   stop_prefix=prefix,
                                   stop_selector=selector, log=str(logfile))
    state_file.write_text(json.dumps(state, indent=2)+'\n')
    print(json.dumps(state['roles'][role_key]))


if __name__ == '__main__':
    main()
