#!/usr/bin/env python3
"""Centralized deployment contracts without launching ROS or Docker processes."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


STAGE = Path(__file__).resolve().parents[1]
REPO_EXPERIMENT = STAGE.parent
SPEC = importlib.util.spec_from_file_location('central_manager_hub_under_test', STAGE / 'manager_hub.py')
hub = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hub)


def arguments(action, *extra):
    parser = hub.argument_parser()
    args = parser.parse_args([action, '--run-id', 'central_contract', '--cutoff', '1793700000', *extra])
    hub.validate(args, parser)
    return args


def command(action, *extra):
    args = arguments(action, *extra)
    stage = PurePosixPath('/experiment/three_host_20261102')
    output = stage / 'runs/central_contract/mrjq311'
    return hub.command_for(args, stage, PurePosixPath('/experiment/harness/multimodal'), output)[0]


def legacy_flags(robot):
    host = 'amov' if robot == 0 else 'uros'
    source = REPO_EXPERIMENT / 'configs' / host / 'multimodal/flags'
    return dict(flags_folder='/experiment/configs/' + host + '/multimodal/flags',
                flag_sha256={name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                             for name in hub.VIO_FLAG_FILES})


class HubPlacementContracts(unittest.TestCase):
    def test_every_role_dry_run_is_reviewable_and_creates_no_run_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'unused_experiment'
            for role in hub.ROLES:
                with self.subTest(role=role):
                    result = subprocess.run([
                        sys.executable, str(STAGE / 'manager_hub.py'), role,
                        '--run-id', 'central_contract', '--cutoff', '1793700000',
                        '--root', str(root), '--dry-run'], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    review = json.loads(result.stdout)
                    self.assertEqual(review['role'], role)
                    self.assertTrue(review['selector'])
                    self.assertEqual(review['command'][0:2], ['docker', 'exec'])
                    self.assertFalse(root.exists())

    def test_hub_owns_both_perception_chains_and_no_ego_execution_process(self):
        for robot in (0, 1):
            ingress = command('ingress' + str(robot))
            self.assertIn('/experiment/harness/multimodal/sensor_ingress.py', ingress)
            self.assertEqual(ingress[ingress.index('--robot-id') + 1], str(robot))
            self.assertIn('/experiment/three_host_20261102/runs/central_contract/mrjq311/vio%d/vio.launch' % robot,
                          command('vio' + str(robot)))
            self.assertIn('/experiment/harness/multimodal/native_vio_graph_adapter.py',
                          command('adapter' + str(robot)))
            frontend = command('local-frontend' + str(robot))
            self.assertIn('/experiment/harness/multimodal/local_frontend.launch', frontend)
            self.assertIn('robot_id:=' + str(robot), frontend)
            self.assertIn('cloud_rate:=5.0', frontend)
            self.assertIn('mesh_period:=1.0', frontend)
        mission = command('mission', '--laps', '3')
        self.assertIn('/experiment/harness/multimodal/closed_loop_missions.py', mission)
        self.assertIn('/experiment/harness/broad_loop_route.json', mission)
        self.assertTrue(all('--central-telemetry' in command(role)
                            for role in ('mission', 'trajectory', 'world')))
        self.assertEqual(mission[mission.index('--laps') + 1], '3')
        for role in hub.ROLES:
            argv = command(role)
            self.assertFalse(any(Path(token).name in ('ego_planner_node', 'traj_server', 'continuous_body.py')
                                 for token in argv))

    def test_backend_keeps_the_existing_completed_window_protocol(self):
        for robot in (0, 1):
            backend = command('backend' + str(robot))
            dense = command('dense' + str(robot))
            self.assertIn('/experiment/harness/multimodal/hub_backend.launch', dense)
            self.assertFalse(any('atomic' in value for value in backend + dense + command('global-view')))

    def test_recording_profiles_keep_results_and_opt_in_to_replay_inputs(self):
        for profile in hub.RECORD_PROFILES:
            topics = hub.hub_record_topics(profile)
            self.assertEqual(len(topics), len(set(topics)))
            self.assertFalse(any(topic.startswith('/edge/') or '/uplink/' in topic for topic in topics))
            self.assertFalse(any(topic.startswith('/drone_') and topic.endswith('visual_slam/odom')
                                 for topic in topics))
            for robot in (0, 1):
                self.assertIn('/sim_%d/ground_truth/odometry' % robot, topics)
        minimal = hub.hub_record_topics('map')
        replay = hub.hub_record_topics('replay')
        debug = hub.hub_record_topics('debug')
        self.assertIn('/visualization/native_mesh/global_cloud', minimal)
        for robot in (0, 1):
            for suffix in ('kimera/rgb/image_raw', 'kimera/depth/image_raw',
                           'kimera/depth/measured_mask', 'kimera/rgb/camera_info',
                           'kimera/depth/camera_info', 'kimera/imu'):
                topic = '/sim_%d/%s' % (robot, suffix)
                self.assertNotIn(topic, minimal)
                self.assertIn(topic, replay)
            for suffix in ('native_dense/rgbd_points', 'native_tsdf/mesh',
                           'native_mesh_frontend/full_mesh', 'native_pgmo/optimized_mesh'):
                topic = '/sim_%d/%s' % (robot, suffix)
                self.assertNotIn(topic, minimal + replay)
                self.assertIn(topic, debug)
        self.assertFalse(any(topic.startswith('/visualization/swarm/') for topic in minimal))
        record = command('record')
        self.assertEqual(record[record.index('-O') + 1],
                         '/experiment/three_host_20261102/runs/central_contract/mrjq311/hub_native.bag')
        self.assertNotIn('/sim_0/kimera/rgb/image_raw', record)
        self.assertIn('/sim_0/kimera/rgb/image_raw', command('record', '--record-profile', 'replay'))
        for role in ('vio0', 'local-frontend0', 'backend0', 'dense0'):
            self.assertEqual(command(role, '--record-profile', 'map'),
                             command(role, '--record-profile', 'replay'))

    def test_publishing_and_mapping_rates_are_separately_configurable(self):
        trajectory = command('trajectory')
        self.assertEqual(trajectory[trajectory.index('--publish-rate') + 1], '20.0')
        frontend = command('local-frontend0', '--cloud-rate', '8', '--mesh-period', '0.5')
        self.assertIn('cloud_rate:=8.0', frontend)
        self.assertIn('mesh_period:=0.5', frontend)
        invalid = (('--cloud-rate', '0'), ('--mesh-period', 'nan'),
                   ('--trajectory-rate', '61'), ('--rviz-fps', '0'))
        for option, value in invalid:
            with self.subTest(option=option), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    arguments('master', option, value)


class NativeConfigurationContracts(unittest.TestCase):
    def test_each_hub_vio_preserves_its_calibration_and_replaces_device_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'experiment'
            shutil.copytree(REPO_EXPERIMENT / 'configs', root / 'configs')
            output = root / 'three_host_20261102/runs/central_contract/mrjq311'
            inner = PurePosixPath('/experiment/three_host_20261102/runs/central_contract/mrjq311')
            for robot, host in ((0, 'amov'), (1, 'uros')):
                flags = legacy_flags(robot)
                hub.prepare_vio_files(root, output, inner, robot, '/opt/native/kimera_vio_ros', flags)
                doc = ET.parse(output / ('vio%d/vio.launch' % robot))
                include = next(doc.iter('include'))
                self.assertEqual(include.get('file'), '/opt/native/kimera_vio_ros/launch/kimera_vio_ros.launch')
                args = {a.get('name'): a.get('value') for a in include.iter('arg')}
                self.assertEqual(args['params_folder'], str(inner / ('vio%d/params' % robot)))
                self.assertEqual(args['flags_folder'], flags['flags_folder'])
                self.assertEqual(args['log_output_path'], str(inner / ('vio%d/vio_logs' % robot)))
                self.assertEqual(args['robot_id'], str(robot))
                self.assertEqual(args['left_cam_topic'], '/sim_%d/kimera/rgb/image_raw' % robot)
                self.assertEqual(args['odom_frame_id'], 'sim_%d/vio_odom' % robot)
                self.assertEqual(args['use_external_odom'], 'false')
                self.assertEqual(args['multirobot'], 'true')
                self.assertEqual(args['visualize'], 'true')
                self.assertEqual(args['use_rviz'], 'true')
                for source in (root / 'configs' / host / 'multimodal/params').iterdir():
                    self.assertEqual(source.read_bytes(),
                                     (output / ('vio%d/params' % robot) / source.name).read_bytes())
                before = (output / ('vio%d/vio.launch' % robot)).read_bytes()
                hub.prepare_vio_files(root, output, inner, robot, '/opt/native/kimera_vio_ros', flags)
                self.assertEqual((output / ('vio%d/vio.launch' % robot)).read_bytes(), before)
                self.assertEqual(json.loads((output / ('vio%d/native_inputs.json' % robot)).read_text()),
                                 dict(package='/opt/native/kimera_vio_ros', **flags))
            # A retained runtime directory must never silently reuse another host's parameters.
            stale = output / 'vio0/params/BackendParams.yaml'
            stale.write_text('stale UROS parameters')
            with self.assertRaisesRegex(RuntimeError, 'fresh run ID'):
                hub.prepare_vio_files(root, output, inner, 0, '/opt/native/kimera_vio_ros', legacy_flags(0))

    def test_native_flags_are_checked_before_launch_and_frozen_for_restarts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'experiment'
            shutil.copytree(REPO_EXPERIMENT / 'configs', root / 'configs')
            output = root / 'three_host_20261102/runs/central_contract/mrjq311'
            inner = PurePosixPath('/experiment/three_host_20261102/runs/central_contract/mrjq311')
            folder = root / 'configs/amov/multimodal/flags'
            argv = [sys.executable, '-c', hub.VIO_FLAGS_CODE, str(folder), json.dumps(hub.VIO_FLAG_FILES)]
            actual = json.loads(subprocess.check_output(argv, text=True))
            self.assertEqual(actual['flag_sha256'], legacy_flags(0)['flag_sha256'])
            hub.prepare_vio_files(root, output, inner, 0, '/opt/native/kimera_vio_ros', actual)
            (folder / 'Mesher.flags').write_text('--return_mesh_2d=false\n')
            changed = json.loads(subprocess.check_output(argv, text=True))
            with self.assertRaisesRegex(RuntimeError, 'flags changed.*fresh run ID'):
                hub.prepare_vio_files(root, output, inner, 0, '/opt/native/kimera_vio_ros', changed)
            (folder / 'Mesher.flags').unlink()
            missing = subprocess.run(argv, text=True, capture_output=True)
            self.assertNotEqual(missing.returncode, 0)
            self.assertIn('Missing native VIO flags', missing.stderr)
            self.assertIn('Mesher.flags', missing.stderr)

    def test_flags_resolution_uses_each_devices_legacy_flags_in_selected_container(self):
        args = arguments('vio0', '--container', 'selected-gpu-native')
        for robot in (0, 1):
            expected = legacy_flags(robot)
            with patch.object(hub.subprocess, 'check_output', return_value=json.dumps(expected)) as call:
                self.assertEqual(hub.resolve_vio_flags(args, robot), expected)
            argv = call.call_args[0][0]
            self.assertEqual(argv[:4], ['docker', 'exec', 'selected-gpu-native', 'python3'])
            self.assertEqual(argv[-2], expected['flags_folder'])
            self.assertEqual(json.loads(argv[-1]), list(hub.VIO_FLAG_FILES))

    def test_vio_package_is_resolved_in_the_selected_native_container(self):
        args = arguments('vio0', '--container', 'selected-gpu-native')
        with patch.object(hub.subprocess, 'check_output', return_value='"/opt/native/kimera_vio_ros"\n') as call:
            self.assertEqual(hub.resolve_vio_package(args), '/opt/native/kimera_vio_ros')
        argv = call.call_args[0][0]
        self.assertEqual(argv[:4], ['docker', 'exec', 'selected-gpu-native', 'bash'])
        self.assertIn("['rospack','find','kimera_vio_ros']", argv[-1])
        self.assertIn('launch.is_file()', argv[-1])

    def test_rviz_ceiling_changes_redraw_without_altering_native_topics_or_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'experiment'
            shutil.copytree(REPO_EXPERIMENT / 'harness/multimodal', root / 'harness/multimodal')
            output = root / 'three_host_20261102/runs/central_contract/mrjq311'
            output.mkdir(parents=True)
            for role, name in hub.RVIZ_CONFIGS.items():
                args = arguments(role, '--rviz-fps', '45')
                source = root / 'harness/multimodal' / name
                hub.prepare_role_files(args, root, root / hub.STAGE_NAME, output)
                target = output / 'rviz' / name
                old, new = source.read_text(), target.read_text()
                self.assertEqual(re.search(r'(?m)^\s*Frame Rate:\s*(\d+)', new)[1], '45')
                for key in ('Fixed Frame', 'Topic'):
                    pattern = r'(?m)^\s*' + key + r':[^\n]*$'
                    self.assertEqual(re.findall(pattern, old), re.findall(pattern, new))


if __name__ == '__main__':
    unittest.main()
