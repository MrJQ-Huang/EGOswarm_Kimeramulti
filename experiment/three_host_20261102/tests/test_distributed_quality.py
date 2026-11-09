import importlib.util
from pathlib import Path
import unittest
import numpy as np

p = Path(__file__).resolve().parents[1] / 'distributed/quality.py'
s = importlib.util.spec_from_file_location('quality', p); q = importlib.util.module_from_spec(s); s.loader.exec_module(q)


class JointQualityTest(unittest.TestCase):
    def test_single_gauge_can_be_removed_without_scale(self):
        points = np.array([[0,0,0], [1,0,0], [0,2,0], [0,0,3]], float)
        rotation = np.array([[0,-1,0], [1,0,0], [0,0,1]], float)
        truth = points @ rotation.T + [4,-3,2]
        fit = q.rigid_alignment(points, truth)
        self.assertTrue(np.allclose(points @ fit[:3,:3].T+fit[:3,3], truth))
        self.assertAlmostEqual(np.linalg.det(fit[:3,:3]), 1.)

    def test_joint_fit_does_not_hide_different_robot_offsets(self):
        points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], float)
        actual = np.vstack([points, points+[0,3,0]])
        estimated = np.vstack([points, points+[10,3,0]])
        fit = q.rigid_alignment(estimated, actual)
        corrected = estimated@fit[:3,:3].T+fit[:3,3]
        self.assertGreater(np.sqrt(np.mean(np.sum((corrected-actual)**2, axis=1))), 1.)

    def test_truth_gap_is_not_filled(self):
        rows = [{'stamp_ns': 1, 'p':[0,0,0], 'q':[0,0,0,1]},
                {'stamp_ns': 1000000001, 'p':[1,0,0], 'q':[0,0,0,1]}]
        self.assertIsNone(q.truth_at(500000001, rows))


START = 1000 * 10 ** 9


def source_row(kind, robot, milliseconds):
    seconds = milliseconds / 1000.
    return dict(kind=kind, robot_id=robot, stamp_ns=START + milliseconds * 10 ** 6,
                p=[seconds, robot * 3., .1 * np.sin(seconds)], q=[0., 0., 0., 1.])


def sources(truth_times, estimate_times, kind='body_path'):
    return ([source_row('actual', robot, t) for robot in (0, 1) for t in truth_times] +
            [source_row(kind, robot, t) for robot in (0, 1) for t in estimate_times])


class SourceTimeCoverageTests(unittest.TestCase):
    def test_lagging_path_exposes_missing_motion_prefix_and_suffix(self):
        rows = sources(range(0, 10001, 50), range(2000, 8001, 1000))
        report = q.evaluate(rows, START, START + 10 * 10 ** 9)
        coverage = report['results']['body_path']['robots']['0']['matched_time_coverage']
        self.assertEqual(coverage['unique_keys'], 7)
        self.assertEqual(coverage['span_s'], 6.)
        self.assertEqual(coverage['missing_prefix_s'], 2.)
        self.assertEqual(coverage['missing_suffix_s'], 2.)
        self.assertEqual(coverage['supported_fraction'], .6)
        self.assertEqual(coverage['unsupported_intervals_ns'],
                         [[START, START + 2 * 10 ** 9], [START + 8 * 10 ** 9, START + 10 * 10 ** 9]])

    def test_mid_motion_estimate_outage_is_not_hidden_by_full_first_to_last_span(self):
        rows = sources(range(0, 12001, 50), [0, 1000, 2000, 10000, 11000, 12000])
        report = q.evaluate(rows, START, START + 12 * 10 ** 9)
        result = report['results']['body_path']['robots']['1']
        coverage = result['source_time_coverage']
        self.assertEqual(coverage['sample_extent_fraction'], 1.)
        self.assertAlmostEqual(coverage['supported_fraction'], 1/3)
        self.assertEqual(coverage['maximum_gap_s'], 8.)
        self.assertEqual(coverage['unsupported_intervals_ns'], [[START + 2 * 10 ** 9, START + 10 * 10 ** 9]])
        self.assertLess(result['ate_translation_m']['rmse'], 1e-10)

    def test_twenty_two_second_truth_outage_reduces_matched_coverage_not_estimate_extent(self):
        truth_times = list(range(0, 3001, 50)) + list(range(25000, 30001, 50))
        rows = sources(truth_times, range(0, 30001, 500))
        report = q.evaluate(rows, START, START + 30 * 10 ** 9)
        robot = report['results']['body_path']['robots']['0']
        self.assertEqual(robot['source_time_coverage']['supported_fraction'], 1.)
        self.assertEqual(robot['matched_time_coverage']['maximum_gap_s'], 22.)
        self.assertEqual(robot['truth_pairing']['counts_by_reason']['truth_bracket_exceeds_maximum_gap'], 43)
        self.assertAlmostEqual(robot['matched_time_coverage']['supported_fraction'], 8/30)
        truth = report['truth_time_coverage_by_robot']['0']
        self.assertEqual(truth['maximum_gap_s'], 22.)
        self.assertEqual(truth['unsupported_intervals_ns'], [[START + 3 * 10 ** 9, START + 25 * 10 ** 9]])
        self.assertLess(robot['ate_translation_m']['rmse'], 1e-10)

    def test_long_stationary_prewarm_cannot_dilute_motion_alignment_or_coverage(self):
        moving = sources(range(0, 2001, 50), [0, 1000, 2000])
        for row in moving:
            if row['kind'] == 'body_path' and row['robot_id'] == 1:
                row['p'][0] += 10.
        prewarm = []
        for robot in (0, 1):
            for t in range(-200000, 0, 100):
                for kind in ('actual', 'body_path'):
                    row = source_row(kind, robot, t); row['p'] = [0., robot * 3., 0.]
                    prewarm.append(row)
        original = q.evaluate(moving, START, START + 2 * 10 ** 9)
        repeated = q.evaluate(prewarm + moving, START, START + 2 * 10 ** 9)
        self.assertEqual(repeated['results']['body_path']['paired_samples'], 6)
        self.assertEqual(original['results']['body_path']['team'], repeated['results']['body_path']['team'])
        np.testing.assert_allclose(original['results']['body_path']['shared_alignment'],
                                   repeated['results']['body_path']['shared_alignment'])
        robot = repeated['results']['body_path']['robots']['0']
        self.assertEqual(robot['source_time_coverage']['unique_keys'], 3)
        self.assertEqual(robot['excluded_outside_requested_window_rows'], 2000)
        self.assertEqual(repeated['truth_time_coverage_by_robot']['0']['outside_window_keys_retained_only_for_bracketing'], 2000)

    def test_truth_brackets_outside_window_remain_available_without_adding_error_samples(self):
        rows = sources([-50, 50], [0, 15, 30])
        report = q.evaluate(rows, START, START + 30 * 10 ** 6)
        robot = report['results']['body_path']['robots']['0']
        self.assertEqual(robot['paired_samples'], 3)
        self.assertEqual(robot['truth_pairing']['counts_by_reason']['interpolated'], 3)
        self.assertEqual(report['truth_time_coverage_by_robot']['0']['unique_keys'], 0)
        self.assertEqual(report['truth_time_coverage_by_robot']['0']['supported_fraction'], 1.)
        self.assertEqual(report['truth_time_coverage_by_robot']['0']['outside_window_keys_retained_only_for_bracketing'], 2)

    def test_missing_robot_is_explicit_even_if_other_robot_has_many_good_pairs(self):
        rows = sources(range(0, 2001, 50), [0, 1000, 2000])
        rows = [row for row in rows if row['kind'] != 'body_path' or row['robot_id'] == 0]
        report = q.evaluate(rows, START, START + 2 * 10 ** 9)
        body = report['results']['body_path']
        self.assertEqual(body['status'], 'missing_robot')
        self.assertEqual(body['missing_estimate_robots'], [1])
        self.assertNotIn('shared_alignment', body)
        missing = body['robots']['1']
        self.assertEqual(missing['status'], 'missing_estimates')
        self.assertEqual(missing['source_time_coverage']['supported_fraction'], 0.)
        self.assertEqual(missing['source_time_coverage']['unsupported_intervals_ns'], [[START, START + 2 * 10 ** 9]])

    def test_revised_body_pose_keys_are_unique_and_last_revision_wins(self):
        rows = sources(range(0, 2001, 50), [0, 1000, 2000])
        wrong = source_row('body_path', 0, 1000); wrong['p'][0] += 500.
        rows.insert(0, wrong)
        report = q.evaluate(rows, START, START + 2 * 10 ** 9)
        robot = report['results']['body_path']['robots']['0']
        self.assertEqual(robot['paired_samples'], 3)
        self.assertEqual(robot['repeated_or_revised_source_keys'], 1)
        self.assertLess(robot['ate_translation_m']['rmse'], 1e-10)

    def test_without_requested_end_no_completeness_percentage_is_invented(self):
        report = q.evaluate(sources([0, 1000, 2000], [0, 1000, 2000]), START)
        coverage = report['results']['body_path']['robots']['0']['source_time_coverage']
        self.assertFalse(report['explicit_complete_motion_window'])
        self.assertIsNone(coverage['supported_fraction'])
        self.assertIsNone(coverage['missing_suffix_s'])

    def test_vio_retains_independent_baseline_and_exposes_missing_robot(self):
        rows = sources(range(0, 2001, 50), [0, 1000, 2000], kind='vio')
        rows = [row for row in rows if row['kind'] != 'vio' or row['robot_id'] == 0]
        report = q.evaluate(rows, START, START + 2 * 10 ** 9)
        vio = report['results']['vio']
        self.assertTrue(vio['baseline_independent_local_gauges'])
        self.assertEqual(vio['status'], 'partially_evaluated')
        self.assertEqual(vio['robots']['0']['status'], 'evaluated')
        self.assertEqual(vio['robots']['1']['status'], 'missing_estimates')
        self.assertEqual(vio['missing_estimate_robots'], [1])


if __name__ == '__main__': unittest.main()
