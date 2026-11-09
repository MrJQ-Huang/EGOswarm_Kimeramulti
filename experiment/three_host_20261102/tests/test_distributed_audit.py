import importlib.util
from pathlib import Path
import unittest

p = Path(__file__).resolve().parents[1] / 'distributed/audit.py'
s = importlib.util.spec_from_file_location('distributed_audit', p)
m = importlib.util.module_from_spec(s); s.loader.exec_module(m)


class StatusWitnessTest(unittest.TestCase):
    def test_fatal_status_is_not_healthy_even_with_live_messages(self):
        value = {'run_id':'test', 'robot_id':0, 'cutoff_ns':1000000000, 'fatal_error':'Wrong native source'}
        checks = m.status_checks({'dense':value, 'local_sensor':dict(value, fatal_error=None)},
                                 {'dense':10.,'local_sensor':10.}, 10.1, 'test', 0, 1.)
        self.assertFalse(checks['dense_no_fatal']); self.assertTrue(checks['local_sensor_no_fatal'])

    def test_previous_run_and_stale_status_rejected(self):
        value = {'run_id':'old', 'robot_id':0, 'cutoff_ns':1000000000, 'fatal_error':None}
        checks = m.status_checks({'dense':value}, {'dense':1.}, 10., 'test', 0, 1.)
        self.assertFalse(checks['dense_identity']); self.assertFalse(checks['dense_fresh'])
        self.assertFalse(checks['local_sensor_no_fatal'])


if __name__ == '__main__': unittest.main()
