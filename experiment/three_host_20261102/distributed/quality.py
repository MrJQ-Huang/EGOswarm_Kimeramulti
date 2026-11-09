#!/usr/bin/env python3
"""Offline trajectory checks. Truth never returns to the estimator or viewer.

Joint trajectories receive exactly one SE(3) alignment for the whole team.
Independent per-robot alignments are explicitly limited to the VIO baseline.
Errors describe only paired keys, so source-time completeness is reported
separately. An observer outage, missing truth bracket or lagging accepted path
must remain visible even when the surviving paired-key ATE is small.
"""
import argparse
import bisect
from collections import Counter
import json
from pathlib import Path
import numpy as np


def transform(position, quaternion):
    t = np.asarray(position, dtype=float); q = np.asarray(quaternion, dtype=float)
    if t.shape != (3,) or q.shape != (4,) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError('Invalid pose')
    norm = np.linalg.norm(q)
    if abs(norm-1.) > .01: raise ValueError('Quaternion is not unit length')
    x, y, z, w = q/norm
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    out = np.eye(4); out[:3, :3] = r; out[:3, 3] = t; return out


def rigid_alignment(estimated, actual):
    """Kabsch rotation/translation only: no scale, per-map ICP, or deformation."""
    x = np.asarray(estimated, float); y = np.asarray(actual, float)
    if x.shape != y.shape or x.ndim != 2 or x.shape[1] != 3 or len(x) < 3:
        raise ValueError('At least three paired positions required')
    if not np.isfinite(x).all() or not np.isfinite(y).all(): raise ValueError('Nonfinite samples')
    cx, cy = x.mean(0), y.mean(0)
    u, _, vt = np.linalg.svd((x-cx).T @ (y-cy))
    d = np.eye(3); d[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ d @ u.T
    out = np.eye(4); out[:3, :3] = r; out[:3, 3] = cy-r@cx; return out


def truth_match(stamp, rows, maximum_gap_ns=100000000, times=None):
    """Return the pose and pairing reason; never bridge a long truth outage."""
    times = [r['stamp_ns'] for r in rows] if times is None else times
    k = bisect.bisect_left(times, stamp)
    if k < len(rows) and times[k] == stamp:
        return transform(rows[k]['p'], rows[k]['q']), 'exact_stamp', 0
    if not rows: return None, 'no_truth_source', None
    if k == 0: return None, 'before_first_truth', None
    if k == len(rows): return None, 'after_last_truth', None
    lo, hi = rows[k-1], rows[k]
    gap = hi['stamp_ns']-lo['stamp_ns']
    if gap > maximum_gap_ns: return None, 'truth_bracket_exceeds_maximum_gap', gap
    a = (stamp-lo['stamp_ns'])/(hi['stamp_ns']-lo['stamp_ns'])
    q0, q1 = np.asarray(lo['q']), np.asarray(hi['q'])
    if q0 @ q1 < 0: q1 = -q1
    q = (1-a)*q0+a*q1; q /= np.linalg.norm(q)
    pos = (1-a)*np.asarray(lo['p'])+a*np.asarray(hi['p'])
    return transform(pos, q), 'interpolated', gap


def truth_at(stamp, rows, maximum_gap_ns=100000000):
    return truth_match(stamp, rows, maximum_gap_ns)[0]


def interval_distribution(values):
    values = np.asarray(values, dtype=np.int64)
    if not len(values):
        return dict(samples=0, minimum_ns=None, mean_ns=None, median_ns=None, p95_ns=None, maximum_ns=None)
    return dict(samples=len(values), minimum_ns=int(values.min()), mean_ns=float(values.mean()),
                median_ns=float(np.median(values)), p95_ns=float(np.percentile(values, 95)),
                maximum_ns=int(values.max()))


def temporal_coverage(stamps, start, end, maximum_gap_ns, support_stamps=None):
    """Observed key extent and gap-limited interval support are distinct.

    Truth may use one bracketing observation outside a requested boundary to
    support interpolation inside it. Such observations never inflate the key
    count, key span or error/alignment sample set for the requested motion.
    """
    original = sorted(set(stamps))
    selected = [t for t in original if (start is None or t >= start) and (end is None or t <= end)]
    first, last = (selected[0], selected[-1]) if selected else (None, None)
    gaps = [b-a for a, b in zip(selected, selected[1:])]
    support = sorted(set(original if support_stamps is None else support_stamps))
    support_intervals = []
    for a, b in zip(support, support[1:]):
        if b-a > maximum_gap_ns: continue
        lo, hi = max(a, start) if start is not None else a, min(b, end) if end is not None else b
        if hi <= lo: continue
        if support_intervals and lo <= support_intervals[-1][1]:
            support_intervals[-1][1] = max(hi, support_intervals[-1][1])
        else: support_intervals.append([lo, hi])
    covered = sum(b-a for a, b in support_intervals)
    complete_window = start is not None and end is not None
    duration = end-start if complete_window else None
    uncovered = None
    if complete_window:
        uncovered = []; cursor = start
        for a, b in support_intervals:
            if a > cursor: uncovered.append([cursor, a])
            cursor = max(cursor, b)
        if cursor < end: uncovered.append([cursor, end])
    span = last-first if selected else None
    extent = min(last, end)-max(first, start) if complete_window and selected else None
    leading = [start, first] if first is not None and start is not None and first > start else None
    trailing = [last, end] if last is not None and end is not None and last < end else None
    return dict(unique_keys=len(selected), first_stamp_ns=first, last_stamp_ns=last,
        span_ns=span, span_s=span/1e9 if span is not None else None,
        maximum_gap_ns=max(gaps) if gaps else None,
        maximum_gap_s=max(gaps)/1e9 if gaps else None,
        sampling_intervals=interval_distribution(gaps),
        mean_source_sample_rate_hz=(len(selected)-1)*1e9/span if span else None,
        requested_window_ns=[start, end], requested_duration_ns=duration,
        missing_prefix_interval_ns=leading, missing_suffix_interval_ns=trailing,
        missing_prefix_s=(first-start)/1e9 if leading else (0. if first is not None and start is not None else None),
        missing_suffix_s=(end-last)/1e9 if trailing else (0. if last is not None and end is not None else None),
        no_source_keys_in_window=not bool(selected),
        gaps_exceeding_threshold_ns=[[a, b] for a, b in zip(selected, selected[1:]) if b-a > maximum_gap_ns],
        continuous_support_maximum_gap_ns=maximum_gap_ns,
        supported_duration_ns=covered, supported_duration_s=covered/1e9,
        unsupported_intervals_ns=uncovered,
        unsupported_duration_ns=(duration-covered) if complete_window else None,
        supported_fraction=(covered/duration) if duration else None,
        sample_extent_fraction=(max(0, extent)/duration) if duration and extent is not None else None,
        support_definition='Union of adjacent source-key intervals no wider than the stated threshold, clipped to the requested window',
        source_time_is_not_receive_time=True)


def summarize(pairs, alignment, rpe_interval_ns=1000000000):
    errors = []; rpe_t = []; rpe_r = []; grouped = {}
    for robot, stamp, est, truth in pairs:
        corrected = alignment @ est
        errors.append(np.linalg.norm(corrected[:3, 3]-truth[:3, 3]))
        grouped.setdefault(robot, []).append((stamp, corrected, truth))
    for rows in grouped.values():
        rows.sort(key=lambda r: r[0]); stamps = [r[0] for r in rows]
        for i, (stamp, estimate, truth) in enumerate(rows):
            j = bisect.bisect_left(stamps, stamp+rpe_interval_ns)
            if j >= len(rows) or stamps[j]-stamp-rpe_interval_ns > 300000000: continue
            _, other_est, other_truth = rows[j]
            expected = np.linalg.inv(truth) @ other_truth
            measured = np.linalg.inv(estimate) @ other_est
            delta = np.linalg.inv(expected) @ measured
            rpe_t.append(np.linalg.norm(delta[:3, 3]))
            rpe_r.append(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1)))
    def distribution(values):
        x = np.asarray(values, float)
        return None if not len(x) else {'samples': len(x), 'rmse': float(np.sqrt(np.mean(x*x))),
                                       'median': float(np.median(x)), 'p95': float(np.percentile(x, 95)), 'max': float(x.max())}
    return {'ate_translation_m': distribution(errors), 'rpe_translation_m': distribution(rpe_t),
            'rpe_rotation_rad': distribution(rpe_r), 'rpe_target_interval_s': rpe_interval_ns/1e9,
            'rpe_maximum_interval_excess_s': .3}


def evaluate(rows, start_stamp_ns=None, end_stamp_ns=None,
             maximum_truth_gap_ns=100000000, maximum_estimate_gap_ns=1000000000):
    for value in (start_stamp_ns, end_stamp_ns):
        if value is not None and (type(value) is not int or value <= 0):
            raise ValueError('Requested source timestamps must be positive integer nanoseconds')
    if start_stamp_ns is not None and end_stamp_ns is not None and start_stamp_ns > end_stamp_ns:
        raise ValueError('Requested source time window is reversed')
    if any(type(value) is not int or value <= 0 for value in (maximum_truth_gap_ns, maximum_estimate_gap_ns)):
        raise ValueError('Gap thresholds must be positive integer nanoseconds')
    truth = {}; estimates = {'vio': {}, 'body_path': {}}
    excluded = {kind: Counter() for kind in estimates}; repeated = {kind: Counter() for kind in estimates}
    run_ids = set()
    for row in rows:
        if row.get('kind') not in ('actual', 'vio', 'body_path'): continue
        robot = row['robot_id']; key = (robot, row['stamp_ns'])
        if type(robot) is not int or robot not in (0, 1) or type(row['stamp_ns']) is not int or row['stamp_ns'] <= 0:
            raise ValueError('Invalid pose source robot/timestamp')
        if row.get('run_id') is not None: run_ids.add(row['run_id'])
        if row['kind'] == 'actual': truth[key] = row
        elif row['kind'] in estimates:
            if ((start_stamp_ns is not None and row['stamp_ns'] < start_stamp_ns) or
                    (end_stamp_ns is not None and row['stamp_ns'] > end_stamp_ns)):
                excluded[row['kind']][robot] += 1; continue
            if key in estimates[row['kind']]: repeated[row['kind']][robot] += 1
            estimates[row['kind']][key] = row
    if len(run_ids) > 1: raise ValueError('Pose sources mix experiment run IDs')
    truths = {r: sorted([v for (robot, _), v in truth.items() if robot == r], key=lambda x: x['stamp_ns']) for r in (0, 1)}
    truth_times = {r: [row['stamp_ns'] for row in truths[r]] for r in (0, 1)}
    truth_coverage = {}
    for robot in (0, 1):
        covered = temporal_coverage(truth_times[robot], start_stamp_ns, end_stamp_ns,
                                     maximum_truth_gap_ns, support_stamps=truth_times[robot])
        covered.update(retained_source_keys_including_outside_brackets=len(truth_times[robot]),
            outside_window_keys_retained_only_for_bracketing=len(truth_times[robot])-covered['unique_keys'])
        truth_coverage[str(robot)] = covered
    report = {'evaluation_only': True, 'truth_is_estimator_input': False, 'scale_estimated': False,
              'joint_alignment': 'One shared SE(3) for both robots; never independently align joint submaps',
              'source_time_window_ns': [start_stamp_ns, end_stamp_ns],
              'explicit_complete_motion_window': start_stamp_ns is not None and end_stamp_ns is not None,
              'source_run_ids': sorted(run_ids), 'truth_time_coverage_by_robot': truth_coverage,
              'pairing_maximum_truth_bracket_ns': maximum_truth_gap_ns,
              'estimate_continuity_gap_threshold_ns': maximum_estimate_gap_ns,
              'coverage_is_a_sampling_support_measure_not_a_slam_success_guarantee': True,
              'pose_key_selection': 'Last recorded pose per robot/kind/source timestamp; repeats never add alignment weight',
              'error_statistics_scope': 'Only unique matched estimated keys within the requested motion window; inspect completeness separately',
              'results': {}}
    for kind, samples in estimates.items():
        pairs = []
        reasons = {r: Counter() for r in (0, 1)}; brackets = {r: [] for r in (0, 1)}
        for (robot, stamp), row in samples.items():
            actual, reason, bracket = truth_match(stamp, truths[robot], maximum_truth_gap_ns, truth_times[robot])
            reasons[robot][reason] += 1
            if reason == 'interpolated': brackets[robot].append(bracket)
            if actual is not None: pairs.append((robot, stamp, transform(row['p'], row['q']), actual))
        selected_pairs = {r: [p for p in pairs if p[0] == r] for r in (0, 1)}
        robot_reports = {}
        for robot in (0, 1):
            source_stamps = [stamp for owner, stamp in samples if owner == robot]
            matched_stamps = [p[1] for p in selected_pairs[robot]]
            robot_reports[str(robot)] = dict(
                status=('missing_estimates' if not source_stamps else
                        'no_truth_pairs' if not matched_stamps else 'insufficient_paired_data'),
                paired_samples=len(matched_stamps), unpaired_samples=len(source_stamps)-len(matched_stamps),
                excluded_outside_requested_window_rows=excluded[kind][robot],
                repeated_or_revised_source_keys=repeated[kind][robot],
                source_time_coverage=temporal_coverage(source_stamps, start_stamp_ns, end_stamp_ns, maximum_estimate_gap_ns),
                matched_time_coverage=temporal_coverage(matched_stamps, start_stamp_ns, end_stamp_ns, maximum_estimate_gap_ns),
                truth_pairing=dict(counts_by_reason=dict(reasons[robot]),
                    paired_fraction=len(matched_stamps)/len(source_stamps) if source_stamps else None,
                    interpolation_bracket_intervals=interval_distribution(brackets[robot])))
        result = dict(status='insufficient_paired_data', paired_samples=len(pairs),
            unpaired_samples=len(samples)-len(pairs), robots=robot_reports,
            missing_estimate_robots=[r for r in (0, 1) if not any(owner == r for owner, _ in samples)],
            missing_truth_robots=[r for r in (0, 1) if not truths[r]],
            missing_paired_robots=[r for r in (0, 1) if not selected_pairs[r]])
        if kind == 'body_path' and result['missing_paired_robots']:
            result['status'] = 'missing_robot'
            report['results'][kind] = result; continue
        if len(pairs) < 3:
            report['results'][kind] = result; continue
        if kind == 'body_path':
            fit = rigid_alignment([x[2][:3, 3] for x in pairs], [x[3][:3, 3] for x in pairs])
            result.update(status='evaluated', shared_alignment=fit.tolist(), team=summarize(pairs, fit))
            for robot in (0, 1):
                robot_reports[str(robot)].update(summarize(selected_pairs[robot], fit), status='evaluated')
        else:
            result['baseline_independent_local_gauges'] = True
            for robot in (0, 1):
                selected = selected_pairs[robot]
                if len(selected) < 3: continue
                fit = rigid_alignment([x[2][:3, 3] for x in selected], [x[3][:3, 3] for x in selected])
                robot_reports[str(robot)].update(summarize(selected, fit), status='evaluated')
            evaluated = sum(value['status'] == 'evaluated' for value in robot_reports.values())
            result['status'] = 'evaluated' if evaluated == 2 else 'partially_evaluated' if evaluated else 'insufficient_paired_data'
        report['results'][kind] = result
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('input', type=Path); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-stamp-ns', type=int); p.add_argument('--end-stamp-ns', type=int)
    p.add_argument('--maximum-truth-gap-ms', type=float, default=100.)
    p.add_argument('--maximum-estimate-gap-s', type=float, default=1.)
    a = p.parse_args(); rows = [json.loads(line) for line in a.input.read_text().splitlines() if line.strip()]
    if not all(np.isfinite(v) and v > 0 for v in (a.maximum_truth_gap_ms, a.maximum_estimate_gap_s)):
        p.error('Coverage gap thresholds must be finite and positive')
    result = evaluate(rows, a.start_stamp_ns, a.end_stamp_ns,
        round(a.maximum_truth_gap_ms*1e6), round(a.maximum_estimate_gap_s*1e9))
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n'); print(json.dumps(result, allow_nan=False))


if __name__ == '__main__': main()
