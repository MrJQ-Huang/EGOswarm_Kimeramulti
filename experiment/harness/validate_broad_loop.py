#!/usr/bin/env python3
"""Offline static-design checks; no ROS, motion commands, or SLAM claims."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from scene_geometry import clearance, points
from validate_complex_run import normalize_obstacles, segment_intersection


def sample_segment(a, b, step=.05):
    count = max(2, math.ceil(math.dist(a, b) / step) + 1)
    return [[a[k] + (b[k] - a[k]) * j / (count - 1) for k in range(3)] for j in range(count)]


def validate(scene, route):
    obstacles, _ = normalize_obstacles(scene)
    targets, chords, lengths, pair_min = [], [], [0., 0.], math.inf
    witness_min, witness_clear = math.inf, True
    for name, values in [('initial', route['starts'])] + [(s['name'], s['targets']) for s in route['stages']]:
        for robot, position in enumerate(values):
            margin, nearest = clearance(scene, position, include_floor=False)
            all_margin, _ = clearance(scene, position)
            targets.append({'stage': name, 'robot_id': robot, 'position': position,
                            'obstacle_center_clearance_m': margin, 'closest_obstacle': nearest,
                            'minimum_including_floor_m': all_margin,
                            'sphere_radius_0p2_clearance_including_floor_m': all_margin - .2})
    previous = route['starts']
    for stage in route['stages']:
        for robot, (start, goal) in enumerate(zip(previous, stage['targets'])):
            length = math.dist(start, goal)
            lengths[robot] += length
            hits = [o['id'] for o in obstacles if segment_intersection(start, goal, o)]
            margin = min(clearance(scene, p, include_floor=False)[0] for p in sample_segment(start, goal))
            witness = stage['free_path_witnesses'][str(robot)]
            if witness[0] != start or witness[-1] != goal:
                raise ValueError('Free-path witness endpoints do not match stage')
            guide_min = math.inf
            for a, b in zip(witness, witness[1:]):
                guide_min = min(guide_min, min(clearance(scene, p, include_floor=False)[0]
                                             for p in sample_segment(a, b)))
                witness_clear &= not any(segment_intersection(a, b, o) for o in obstacles)
            witness_min = min(witness_min, guide_min)
            chords.append({'stage': stage['stage'], 'name': stage['name'], 'robot_id': robot,
                           'straight_goal_distance_m': length, 'sampled_min_obstacle_center_sdf_m': margin,
                           'analytic_segment_intersections': hits, 'requires_EGO_detour': bool(hits),
                           'sampled_free_path_witness_min_obstacle_clearance_m': guide_min})
        for j in range(501):
            fraction = j / 500
            pair = [[a[k] + fraction * (b[k] - a[k]) for k in range(3)]
                    for a, b in zip(previous, stage['targets'])]
            pair_min = min(pair_min, math.dist(*pair))
        previous = stage['targets']
    closed = previous == route['starts']
    target_min = min(t['obstacle_center_clearance_m'] for t in targets)
    target_floor_min = min(t['minimum_including_floor_m'] for t in targets)
    return {'method': 'scene_geometry exact point SDF; <=0.05m segment samples; analytic box/cylinder center-segment intersections',
            'target_checks': targets, 'all_targets_at_least_1m_from_obstacles': target_min >= 1.,
            'minimum_target_obstacle_center_clearance_m': target_min,
            'minimum_target_including_floor_center_clearance_m': target_floor_min,
            'nominal_chords': chords, 'nominal_chords_that_need_EGO_detours': sum(c['requires_EGO_detour'] for c in chords),
            'free_path_witnesses_all_center_collision_free': bool(witness_clear),
            'sampled_min_free_path_witness_obstacle_center_clearance_m': witness_min,
            'same_progress_nominal_chord_min_robot_center_separation_m': pair_min,
            'per_robot_nominal_goal_chord_length_m': lengths, 'each_position_loop_closed': closed,
            'static_design_checks_passed': bool(closed and target_min >= 1. and target_floor_min >= 1.
                                                and witness_clear and witness_min >= 1. and pair_min >= 2.),
            'claim_boundary': 'Static design checks do not validate actual EGO trajectories, camera tracking, LCD, cross-robot constraints or SLAM map accuracy.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=Path(__file__).with_name('broad_loop_scene.json'))
    parser.add_argument('--route', type=Path, default=Path(__file__).with_name('broad_loop_route.json'))
    parser.add_argument('--write-validation', action='store_true')
    parser.add_argument('--visibility', action='store_true', help='Optional small NumPy ray probes; no feature/LCD claims')
    args = parser.parse_args()
    scene, route = [json.loads(p.read_text(encoding='utf8')) for p in (args.scene, args.route)]
    report = validate(scene, route)
    report['scene_sha256'] = hashlib.sha256(args.scene.read_bytes()).hexdigest()
    old = json.loads(Path(__file__).with_name('complex_scene.json').read_text(encoding='utf8'))
    report['planning_cloud_point_counts'] = {'old_step_0p12': len(points(old, .12)),
                                            'new_step_0p12': len(points(scene, .12)),
                                            'new_step_0p18': len(points(scene, .18))}
    if args.visibility:
        import sys
        import numpy as np
        sys.path.insert(0, str(Path(__file__).with_name('multimodal')))
        from raycast_geometry import RayScene, camera_rays, R_BC
        renderer = RayScene(scene, use_native=False)
        rays, _ = camera_rays(80, 60, 40., 40., 39.5, 29.5)
        probes = []
        for stage in route['stages']:
            if not stage.get('shared_observation'): continue
            for position in stage['targets']:
                ranges, _, _, ids = renderer.cast(position, rays @ R_BC.T, maximum=8.)
                probes.append({'position': position, 'yaw_rad': 0., 'resolution': [80, 60],
                               'finite_hit_fraction_8m': float(np.isfinite(ranges).mean()),
                               'vertical_obstacle_hit_fraction_8m': float((ids > 0).mean())})
        report['shared_heading_visibility_probes'] = probes
    if args.write_validation:
        route.setdefault('validation', {}).update(report)
        route['per_robot_nominal_goal_chord_length_m'] = report['per_robot_nominal_goal_chord_length_m']
        route['per_lap_nominal_motion_plus_dwell_seconds'] = [
            length / route['recommended_speed_m_s'] + sum(s['dwell_seconds'] for s in route['stages'])
            for length in report['per_robot_nominal_goal_chord_length_m']]
        args.route.write_text(json.dumps(route, indent=2) + '\n', encoding='utf8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('target_checks', 'nominal_chords')}, indent=2))
    if not report['static_design_checks_passed']: raise SystemExit(2)


if __name__ == '__main__':
    main()
