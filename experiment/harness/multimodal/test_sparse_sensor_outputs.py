#!/usr/bin/env python3
"""Sparse exported LiDAR retains measured coordinates and physical calibration."""
import contextlib
import io
import unittest

import numpy as np

from sim_sensors import arguments, calibration, sparse_lidar_hits


class SparseLidarOutputContracts(unittest.TestCase):
    def test_selection_preserves_measurement_values_order_and_dtype(self):
        hits = np.arange(33, dtype='<f4').reshape(11, 3)
        original = hits.copy()
        sparse = sparse_lidar_hits(hits, 4)
        np.testing.assert_array_equal(sparse, original[[0, 4, 8]])
        np.testing.assert_array_equal(hits, original)
        self.assertEqual(sparse.dtype, hits.dtype)
        self.assertTrue(sparse.flags.c_contiguous)
        self.assertEqual(sparse_lidar_hits(np.empty((0, 3), dtype='<f4'), 4).shape, (0, 3))

    def test_output_sparsity_does_not_change_sensor_intrinsics_or_ray_model(self):
        full = calibration(arguments(['--lidar-stride', '1']))
        sparse = calibration(arguments(['--lidar-stride', '4', '--no-truth-relay']))
        for key in ('image_size', 'K', 'T_BC', 'T_BI', 'T_BL', 'lidar', 'imu'):
            self.assertEqual(full[key], sparse[key])
        self.assertEqual(sparse['lidar_output']['measured_hit_stride'], 4)
        self.assertFalse(sparse['lidar_output']['local_ego_planning_cloud_thinned'])
        self.assertFalse(sparse['renderer_truth_relay_enabled'])

    def test_invalid_strides_are_rejected_and_ros_remaps_still_parse(self):
        for stride in (0, -1, True, 1.5):
            with self.subTest(stride=stride), self.assertRaises(ValueError):
                sparse_lidar_hits(np.zeros((3, 3)), stride)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            arguments(['--lidar-stride', '0'])
        parsed = arguments(['--lidar-stride', '4', '--no-truth-relay', '__name:=edge'])
        self.assertEqual(parsed.lidar_stride, 4)
        self.assertTrue(parsed.no_truth_relay)


if __name__ == '__main__':
    unittest.main()
