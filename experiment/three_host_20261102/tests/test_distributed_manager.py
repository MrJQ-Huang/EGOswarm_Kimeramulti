import importlib.util
import contextlib
import io
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1] / 'distributed'


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / (name + '.py'))
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


class DistributedOwnershipTest(unittest.TestCase):
    def test_masters_are_host_local_and_dry_plan_does_not_touch_files(self):
        m = module('manager')
        for host, ip in m.IPS.items():
            args = m.parser().parse_args(['master', '--host', host, '--run-id', 'unit', '--cutoff', '100'])
            p = m.plan(args)
            self.assertEqual(p['master_uri'], 'http://' + ip + ':11431')
            self.assertIn('ROS_MASTER_URI=' + p['master_uri'], p['command'])
            self.assertIn('DISTRIBUTED_KIMERA_PROFILE=1', p['command'])

    def test_observer_host_cannot_run_slam(self):
        m = module('manager')
        args = m.parser().parse_args(['vio', '--host', 'hub', '--run-id', 'unit', '--cutoff', '100'])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): m.validate(args, m.parser())

    def test_local_sensors_do_not_enter_lan_uplink(self):
        m = module('manager')
        for host in ('amov', 'uros'):
            args = m.parser().parse_args(['sensors', '--host', host, '--run-id', 'unit', '--cutoff', '100'])
            command = m.plan(args)['command']
            self.assertIn('--body-imu', command)
            self.assertFalse(any('/edge/' in x for x in command))
            self.assertNotIn('sensor_uplink.py', ' '.join(command))

    def test_edge_dense_uses_online_nonpersistent_mode(self):
        m = module('manager')
        args = m.parser().parse_args(['dense', '--host', 'amov', '--run-id', 'unit', '--cutoff', '100'])
        c = m.plan(args)['command']
        self.assertIn('execution_mode:=online', c); self.assertIn('no_persist:=true', c)
        self.assertIn('remote_bridge_caller:=/sim_0_peer_bridge', c)

    def test_map_and_telemetry_have_independently_owned_observer_roles(self):
        m = module('manager'); plans = []
        for role, channel in [('observer', 'map'), ('observer-telemetry', 'telemetry')]:
            args = m.parser().parse_args([role, '--host', 'hub', '--run-id', 'unit', '--cutoff', '100'])
            m.validate(args, m.parser()); plan = m.plan(args); plans.append(plan)
            self.assertEqual(plan['command'][plan['command'].index('--channels')+1], channel)
            self.assertIn('THREE_HOST_ROLE=distributed-'+role, plan['command'])
        self.assertNotEqual(plans[0]['log'], plans[1]['log'])

    def test_explicit_native_resolution_reaches_both_architectures(self):
        m = module('manager')
        for host in ('amov', 'uros'):
            args = m.parser().parse_args(['local-frontend', '--host', host, '--run-id', 'unit', '--cutoff', '100', '--voxel-size', '0.20'])
            m.validate(args, m.parser())
            self.assertIn('voxel_size:=0.2', m.plan(args)['command'])


class BarrierTest(unittest.TestCase):
    def test_peer_advanced_first_does_not_deadlock(self):
        b = module('local_mission').StageBarrier(2, 1)
        peer = {'stage_index': 1, 'completed_stage': 0, 'arrived': False}
        self.assertFalse(b.update(0, 0., peer, .35, 2, 3))
        self.assertFalse(b.update(2, 0., peer, .35, 2, 3))
        self.assertTrue(b.update(5, 0., peer, .35, 2, 3))
        self.assertEqual(b.index, 1)

    def test_peer_must_arrive_and_local_motion_restarts_settle(self):
        b = module('local_mission').StageBarrier(1, 1)
        peer = {'stage_index': 0, 'arrived': True}
        self.assertFalse(b.update(0, 0., None, .35, 2, 0))
        self.assertFalse(b.update(3, 1., peer, .35, 2, 0))
        self.assertFalse(b.update(4, 0., peer, .35, 2, 0))
        self.assertTrue(b.update(6, 0., peer, .35, 2, 0))
        self.assertTrue(b.done)


if __name__ == '__main__': unittest.main()
