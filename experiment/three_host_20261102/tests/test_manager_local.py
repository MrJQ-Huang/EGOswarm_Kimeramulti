#!/usr/bin/env python3
"""Edge/hub boundary and unchanged EGO goal-to-execution contracts."""
import ast
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

STAGE = Path(__file__).resolve().parents[1]
EXPERIMENT = STAGE.parent
HARNESS = EXPERIMENT / 'harness'
MULTIMODAL = HARNESS / 'multimodal'
sys.path.insert(0, str(MULTIMODAL))
spec = importlib.util.spec_from_file_location('edge_manager_under_test', STAGE / 'manager_local.py')
manager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manager)
from sensor_wire import STREAMS, canonical_topic, edge_topic
from continuous_body import Command, ContinuousBody


def arguments(role, host='amov', *extra):
    p = manager.argument_parser()
    a = p.parse_args([role, '--host', host, '--run-id', 'edge_contract', '--cutoff', '123.5', *extra])
    manager.validate_args(a, p)
    return a


class EdgeRoles(unittest.TestCase):
    def test_edge_cli_cannot_start_a_recorder_or_slam_role(self):
        self.assertEqual(set(manager.START_ROLES), {'supervisor', 'sensors', 'uplink', 'heartbeat'})
        for role in ('record', 'vio', 'adapter', 'local-frontend', 'mission', 'dense'):
            with self.subTest(role=role), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                arguments(role)
        for host in ('amov', 'uros'):
            for role in manager.START_ROLES:
                command = manager.start_plan(arguments(role, host))['command']
                joined = ' '.join(command)
                self.assertNotIn('kimera-native-full', joined)
                self.assertNotIn('rosbag', joined)
                self.assertNotIn('kimera_vio', joined)
                self.assertNotIn('tsdf_server', joined)
                self.assertNotIn('--output-directory', command)

    def test_all_raw_sensor_outputs_are_remapped_without_remapping_local_control_inputs(self):
        for host, robot in (('amov', 0), ('uros', 1)):
            command = manager.start_plan(arguments('sensors', host))['command']
            mappings = dict(value.split(':=', 1) for value in command if ':=' in value)
            self.assertEqual(mappings.pop('__name'), 'sim_%d_edge_sensors' % robot)
            expected = {canonical_topic(robot, item['suffix']): edge_topic(robot, item['suffix'])
                        for item in STREAMS.values()}
            self.assertEqual(mappings, expected)
            self.assertEqual(len(expected), 9)
            self.assertNotIn('/sim_%d/body/imu' % robot, mappings)
            self.assertNotIn('/drone_%d_visual_slam/odom' % robot, mappings)
            self.assertNotIn('/drone_%d_planning/pos_cmd' % robot, mappings)
            self.assertIn('--body-imu', command)
            self.assertIn('--camera-rate', command)
            self.assertEqual(command[command.index('--camera-rate') + 1], '5.0')
            self.assertEqual(command[command.index('--lidar-rate') + 1], '1.0')
            self.assertEqual(command[command.index('--lidar-stride') + 1], '4')
            self.assertIn('--no-truth-relay', command)
            supervisor = manager.start_plan(arguments('supervisor', host))['command']
            self.assertIn('--local-planning-map', supervisor)

    def test_uplink_is_epoch_bound_and_has_bounded_transport_settings(self):
        p = manager.argument_parser()
        a = p.parse_args(['uplink', '--host', 'uros', '--run-id', 'missing_cutoff'])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            manager.validate_args(a, p)
        command = manager.start_plan(arguments('uplink', 'uros'))['command']
        for option, expected in (('--robot-id', '1'), ('--run-id', 'edge_contract'),
                                 ('--cutoff', '123.5'), ('--max-pending', '8'),
                                 ('--max-frame-age', '2.0'), ('--compression-level', '1'),
                                 ('--telemetry-rate', '30.0')):
            self.assertEqual(command[command.index(option) + 1], expected)
        self.assertIn('THREE_HOST_ROLE=uplink', command)
        self.assertIn('ROS_MASTER_URI=' + manager.MASTER_URI, command)

    def test_dry_run_creates_no_files_and_starts_no_processes(self):
        with tempfile.TemporaryDirectory() as folder:
            sentinel = Path(folder) / 'nonexistent_root'
            with patch.object(manager, 'ROOTS', {'amov': sentinel}), \
                    patch.object(sys, 'argv', ['manager_local.py', 'sensors', '--host', 'amov',
                                               '--run-id', 'dry_check', '--cutoff', '123.5', '--dry-run']), \
                    patch.object(manager.subprocess, 'Popen', side_effect=AssertionError('No launch allowed')), \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                manager.main()
            self.assertFalse(sentinel.exists())
            plan = json.loads(output.getvalue())
            self.assertFalse(plan['edge_raw_recording_enabled'])
            self.assertFalse(plan['edge_slam_enabled'])

    def test_old_owned_recorder_can_still_be_inspected_and_stopped(self):
        prefix = ['docker', 'exec', 'kimera-native-full-uros-20261102', 'bash', '/old/native_env.bash']
        entry = {'role': 'record', 'prefix': prefix}
        receipt = {'closed': True, 'remaining': [], 'signalled': []}
        with patch.object(manager.subprocess, 'check_output', return_value=json.dumps(receipt)) as execute:
            manager.close_owned(entry, 'old_run')
        command = execute.call_args[0][0]
        self.assertEqual(command[:len(prefix)], prefix)
        identity = json.loads(command[-1])
        self.assertEqual(identity['identity'], {'ROS_MASTER_URI': manager.MASTER_URI,
                                              'THREE_HOST_RUN_ID': 'old_run', 'THREE_HOST_ROLE': 'record'})
        self.assertTrue(identity['signal'])
        self.assertIn('stopped_wall', entry)
        pending = {'role': 'record', 'prefix': prefix}
        with patch.object(manager, 'query_owned', return_value={'closed': False, 'remaining': [999]}):
            manager.close_owned(pending, 'old_run')
        self.assertNotIn('stopped_wall', pending)


class EgoExecutionBoundary(unittest.TestCase):
    def test_route_dispatch_publishes_goal_pose_and_does_not_publish_execution_commands(self):
        tree = ast.parse((MULTIMODAL / 'closed_loop_missions.py').read_text())
        publishers = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                      and isinstance(node.func, ast.Attribute) and node.func.attr == 'Publisher']
        self.assertEqual({node.args[1].id for node in publishers}, {'PoseStamped', 'String'})
        self.assertTrue(any(isinstance(node.args[0], ast.BinOp)
                            and isinstance(node.args[0].left, ast.Constant)
                            and node.args[0].left.value == '/two_pc/drone_%d/goal' for node in publishers))
        launch = ET.parse(HARNESS / 'one_drone.launch').getroot()
        self.assertTrue(any(node.get('file') == '$(find ego_planner)/launch/run_in_sim.launch'
                            for node in launch.iter('include')))
        self.assertTrue(any(node.get('from') == '/move_base_simple/goal'
                            and node.get('to') == '/two_pc/drone_$(arg drone_id)/goal'
                            for node in launch.iter('remap')))
        body_tree = ast.parse((MULTIMODAL / 'continuous_body.py').read_text())
        command_inputs = [node for node in ast.walk(body_tree) if isinstance(node, ast.Call)
                          and isinstance(node.func, ast.Attribute) and node.func.attr == 'Subscriber'
                          and isinstance(node.args[1], ast.Name) and node.args[1].id == 'PositionCommand']
        self.assertEqual(len(command_inputs), 1)
        self.assertEqual(command_inputs[0].args[0].left.value, '/drone_%d_planning/pos_cmd')

    def test_simulated_body_integrates_a_reference_without_copying_goal_or_command_pose(self):
        body = ContinuousBody([0., 0., 1.])
        body.advance(1.)
        reference = np.array([10., 2., 1.])
        body.set_command(Command(1., 1., reference, np.zeros(3), np.zeros(3), 0., 0.))
        np.testing.assert_allclose(body.position, [0., 0., 1.])
        self.assertTrue(body.advance(1.01))
        self.assertGreater(float(np.linalg.norm(body.position - [0., 0., 1.])), 0.)
        self.assertLess(float(np.linalg.norm(body.position - [0., 0., 1.])), .001)
        self.assertFalse(np.allclose(body.position, reference))


if __name__ == '__main__':
    unittest.main()
