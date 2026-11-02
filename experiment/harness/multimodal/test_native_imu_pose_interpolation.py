#!/usr/bin/env python3
"""Physical-curve and ingress regressions for native VIO time interpolation."""
from collections import Counter, deque
import copy
import tempfile
from pathlib import Path
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import numpy as np

import native_dense_bridge as dense_module
from native_dense_bridge import NativeDenseBridge, PoseHistory, pose_matrix
from native_imu_pose_interpolation import (GyroCoverageError, GyroRotationHistory,
    quaternion_matrix, rotation_vector_quaternion)


def rotation(vector):
    return quaternion_matrix(rotation_vector_quaternion(np.asarray(vector, dtype=float)))


def pose(position, vector=(0., 0., 0.)):
    q = rotation_vector_quaternion(np.asarray(vector, dtype=float))
    return S(position=S(x=position[0], y=position[1], z=position[2]),
             orientation=S(x=q[0], y=q[1], z=q[2], w=q[3]))


class Stamp:
    def __init__(self, ns): self.ns = ns
    def to_nsec(self): return self.ns


class GyroInterpolationContracts(unittest.TestCase):
    def history(self, samples, **kwargs):
        result = GyroRotationHistory(100_000_001, **kwargs)
        for stamp, omega in samples:
            result.put(stamp, omega)
        return result

    def reversal(self):
        # Fast turn then return: identical keyframe orientations hide the turn
        # completely from SLERP, but the actual gyro still measures it.
        return self.history([(1_000_000_000+i*100_000_000,
            [0., 0., 2. if i < 5 else -2.]) for i in range(11)])

    def test_rapid_reversal_is_not_erased_by_equal_keyframe_endpoints(self):
        history = self.reversal()
        actual = history.rotation_at(np.eye(3), np.eye(3), 1_000_000_000,
                                     2_000_000_000, 1_500_000_000)
        np.testing.assert_allclose(actual, rotation([0., 0., 1.]), atol=1e-12)
        self.assertGreater(np.linalg.norm(actual-np.eye(3)), 1.)

    def test_left_endpoint_partial_intervals_use_measured_rate(self):
        history = self.reversal()
        # Between samples, +2 rad/s over .025 s adds .05 rad. No midpoint
        # averaging with the next sample's opposite angular rate is allowed.
        actual = history.rotation_at(np.eye(3), np.eye(3), 1_000_000_000,
                                     2_000_000_000, 1_475_000_000)
        np.testing.assert_allclose(actual, rotation([0., 0., .95]), atol=1e-12)

    def test_noncommuting_body_rotations_keep_correct_product_order(self):
        history = self.history([(1_000_000_000+i*100_000_000,
            [2., 0., 0.] if i < 5 else [0., 2., 0.]) for i in range(11)])
        endpoint = rotation([1., 0., 0.])@rotation([0., 1., 0.])
        actual = history.rotation_at(np.eye(3), endpoint, 1_000_000_000,
                                     2_000_000_000, 1_750_000_000)
        np.testing.assert_allclose(actual, rotation([1., 0., 0.])@rotation([0., .5, 0.]), atol=1e-12)

    def test_native_gauge_and_exact_endpoints_are_preserved_without_gyro(self):
        history = GyroRotationHistory(50_000_000)
        first, second = rotation([.2, -.3, .1]), rotation([-.1, .4, .6])
        np.testing.assert_array_equal(history.rotation_at(first, second, 10, 20, 10), first)
        np.testing.assert_array_equal(history.rotation_at(first, second, 10, 20, 20), second)
        with self.assertRaises(GyroCoverageError):
            history.rotation_at(first, second, 10, 20, 15)

    def test_native_endpoint_residual_is_distributed_over_curve(self):
        history = self.reversal()
        initial = rotation([.3, -.2, 0.])
        endpoint = initial@rotation([0., 0., .1])
        actual = history.rotation_at(initial, endpoint, 1_000_000_000,
                                     2_000_000_000, 1_500_000_000)
        np.testing.assert_allclose(actual, initial@rotation([0., 0., 1.05]), atol=1e-12)
        self.assertAlmostEqual(history.maximum_endpoint_residual_rad, .1)

    def test_real_dropout_is_rejected_and_later_interval_recovers(self):
        history = GyroRotationHistory(50_000_000)
        for ns in [100_000_000, 110_000_000, 210_000_000, 220_000_000, 230_000_000]:
            history.put(ns, [0., 0., 1.])
        with self.assertRaises(GyroCoverageError) as caught:
            history.rotation_at(np.eye(3), rotation([0., 0., .12]), 100_000_000, 220_000_000, 150_000_000)
        self.assertEqual(caught.exception.kind, 'gap')
        result = history.rotation_at(np.eye(3), rotation([0., 0., .02]), 210_000_000, 230_000_000, 225_000_000)
        np.testing.assert_allclose(result, rotation([0., 0., .015]), atol=1e-12)

    def test_future_and_past_coverage_are_distinguished(self):
        history = self.history([(100, [0., 0., 0.]), (200, [0., 0., 0.])])
        for start, end, query, kind in [(50, 200, 150, 'past'), (100, 300, 150, 'future')]:
            with self.assertRaises(GyroCoverageError) as caught:
                history.rotation_at(np.eye(3), np.eye(3), start, end, query)
            self.assertEqual(caught.exception.kind, kind)

    def test_pruning_has_bounded_memory_and_keeps_absolute_curve(self):
        history = GyroRotationHistory(2, maximum_samples=3)
        for ns in range(1, 1033): history.put(ns, [0., 0., 1.])
        self.assertLessEqual(len(history.stamps), 515)
        result = history.rotation_at(np.eye(3), rotation([0., 0., 2e-9]), 1030, 1032, 1031)
        np.testing.assert_allclose(result, rotation([0., 0., 1e-9]), atol=1e-14)
        with self.assertRaises(GyroCoverageError) as caught:
            history.rotation_at(np.eye(3), np.eye(3), 1, 3, 2)
        self.assertEqual(caught.exception.kind, 'past')

    def test_invalid_or_ambiguous_sensor_contract_fails(self):
        with self.assertRaises(ValueError): GyroRotationHistory(50, sample_convention='instantaneous')
        history = GyroRotationHistory(50)
        for stamp, omega in [(0, [0., 0., 0.]), (1, [np.nan, 0., 0.]), (1, [0., 0.])]:
            with self.assertRaises(ValueError): history.put(stamp, omega)
        history.put(10, [1., 0., 0.])
        history.put(10, [1., 0., 0.])
        for stamp, omega in [(10, [0., 0., 0.]), (9, [1., 0., 0.])]:
            with self.assertRaises(ValueError): history.put(stamp, omega)

    def test_pose_history_changes_only_interior_rotation(self):
        gyros = self.reversal()
        measured = PoseHistory(2_000_000_000, gyros=gyros)
        baseline = PoseHistory(2_000_000_000)
        first, second = pose([1., 2., 3.]), pose([5., -2., 9.])
        original = copy.deepcopy((vars(first.position), vars(first.orientation), vars(second.position), vars(second.orientation)))
        for history in [measured, baseline]:
            history.put(1_000_000_000, first)
            history.put(2_000_000_000, second)
        actual, previous = measured.at(1_500_000_000), baseline.at(1_500_000_000)
        np.testing.assert_array_equal(actual[:3, 3], previous[:3, 3])
        np.testing.assert_allclose(actual[:3, :3], rotation([0., 0., 1.]), atol=1e-12)
        np.testing.assert_array_equal(measured.at(1_000_000_000), pose_matrix(first))
        np.testing.assert_array_equal(measured.at(2_000_000_000), pose_matrix(second))
        self.assertEqual(original, (vars(first.position), vars(first.orientation), vars(second.position), vars(second.orientation)))

    def test_imu_ingress_authenticates_and_does_not_wait_for_mesh_lock(self):
        bridge = object.__new__(NativeDenseBridge)
        bridge.a = S(robot_name='sim_0'); bridge.cutoff_ns = 100
        bridge.lock = threading.RLock(); bridge.native_input_lock = threading.Lock()
        bridge.native_inputs = deque(); bridge.native_input_max_queue = 0
        bridge.native_imu_max_delay = 0.; bridge.subs = []; bridge.counts = Counter()
        bridge.gyros = GyroRotationHistory(50_000_000)
        failures, callbacks = [], []
        bridge.fail = lambda error: failures.append(str(error))
        def subscriber(topic, kind, callback, **kwargs):
            self.assertIsNone(kwargs['queue_size'])
            callbacks.append(callback)
            return S()
        def message(stamp=100, caller='/sim_0_multimodal_sensors', frame='sim_0/imu'):
            return S(_connection_header={'callerid': caller}, header=S(stamp=Stamp(stamp), frame_id=frame),
                     angular_velocity=S(x=0., y=0., z=1.))
        with patch.object(dense_module, 'rospy', S(Subscriber=subscriber), create=True):
            bridge.subscribe('/sim_0/kimera/imu', object, bridge.on_imu,
                '/sim_0_multimodal_sensors', None, priority='imu')
        done = threading.Event()
        def receive():
            for i in range(1000): callbacks[0](message(100+i*10_000_000))
            done.set()
        with bridge.lock:
            worker = threading.Thread(target=receive); worker.start()
            self.assertTrue(done.wait(2.), 'Actual gyro ingress must not wait for a busy mesh lock')
            self.assertEqual(len(bridge.native_inputs), 1000)
            bridge.drain_native_inputs()
        worker.join(2.)
        self.assertEqual(bridge.gyros.samples_seen, 1000)
        self.assertEqual(bridge.counts['priority_actual_imu_samples_processed'], 1000)
        self.assertEqual(failures, [])
        for invalid in [message(caller='/foreign'), message(stamp=99), message(frame='world')]: callbacks[0](invalid)
        self.assertEqual(len(failures), 3)
        self.assertEqual(len(bridge.native_inputs), 0)

    def test_closed_publisher_is_benign_only_during_actual_ros_shutdown(self):
        for shutdown, error, should_fail in [
            (True, 'publish() to a closed topic', False),
            (False, 'publish() to a closed topic', True),
            (True, 'Actual gyro timestamps must be monotonic', True)]:
            bridge = object.__new__(NativeDenseBridge)
            bridge.lock = threading.RLock(); bridge.fatal = None
            bridge.a = S(run_id='shutdown-contract')
            events, signals = [], []
            bridge.event = lambda *args, **kw: events.append((args, kw))
            with tempfile.TemporaryDirectory() as directory:
                bridge.evidence = Path(directory)
                with patch.object(dense_module, 'rospy', S(is_shutdown=lambda: shutdown,
                        logfatal=lambda *args: None, signal_shutdown=signals.append), create=True):
                    bridge.fail(RuntimeError(error))
                self.assertEqual(bridge.fatal is not None, should_fail)
                self.assertEqual((bridge.evidence/'fatal.json').exists(), should_fail)
                self.assertEqual(len(signals), int(should_fail))


if __name__ == '__main__':
    unittest.main()
