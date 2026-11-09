#!/usr/bin/env python3
"""Offline geometry evaluation using one already-computed team SE(3).

Truth is used only for scoring. No ICP, scale fitting, independent submap
alignment, mesh deformation, topology edits or estimator/viewer feedback occurs.
Coverage is nearest native vertex distance, not exact point-to-triangle distance
and not a claim of reproducing every metric in the Kimera-Multi paper.
GT views cover the trajectory evaluation window, including late observations
when a mesh may be stale. Mesh header stamps are diagnostic only: repeated
observations can integrate or deform geometry without creating new topology.
Primary surface scores use every native vertex referenced by a native face.
Unreferenced buffer slots remain in the untouched source map and are counted
and scored separately as diagnostics; they cannot supply surface coverage.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

_HARNESS = Path(__file__).resolve().parents[2] / 'harness' / 'multimodal'
if str(_HARNESS) not in sys.path:
    sys.path.insert(0, str(_HARNESS))
from raycast_geometry import R_BC, RayScene, camera_rays, rotation_xyzw
from export_final_global_map import MESSAGE_MD5, decode_mesh


THRESHOLDS = (.15, .30, .50)


def points_array(points):
    value = np.asarray(points, dtype=float)
    if value.ndim != 2 or value.shape[1] != 3 or not np.isfinite(value).all():
        raise ValueError('Expected finite Nx3 positions')
    return value


def validate_alignment(value):
    transform = np.asarray(value, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-9, rtol=0)):
        raise ValueError('Shared alignment must be a finite homogeneous SE(3)')
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7, rtol=0) or
            abs(np.linalg.det(rotation) - 1.) > 1e-7):
        raise ValueError('Shared alignment cannot contain scale or reflection')
    return transform.copy()


def apply_alignment(points, alignment):
    positions = points_array(points)
    transform = validate_alignment(alignment)
    return positions @ transform[:3, :3].T + transform[:3, 3]


def rectangle_surface_distances(points, x_bounds, y_bounds, z=0.):
    positions = points_array(points)
    low = np.asarray([x_bounds[0], y_bounds[0]], float)
    high = np.asarray([x_bounds[1], y_bounds[1]], float)
    if (not np.isfinite(np.r_[low, high, z]).all() or np.any(high <= low)):
        raise ValueError('Invalid bounded floor')
    outside = np.maximum(np.maximum(low - positions[:, :2], positions[:, :2] - high), 0.)
    return np.sqrt(np.sum(outside * outside, axis=1) + (positions[:, 2] - z) ** 2)


def box_surface_distances(points, center, size):
    positions = points_array(points)
    center, size = np.asarray(center, float), np.asarray(size, float)
    if (center.shape != (3,) or size.shape != (3,) or not np.isfinite(center).all()
            or not np.isfinite(size).all() or np.any(size <= 0)):
        raise ValueError('Invalid finite axis-aligned box')
    offset = np.abs(positions - center) - size / 2.
    outside = np.linalg.norm(np.maximum(offset, 0.), axis=1)
    inside = np.minimum(np.max(offset, axis=1), 0.)
    return np.abs(outside + inside)


def cylinder_surface_distances(points, center, radius, height, base_z=0.):
    """Exact unsigned distance to a finite vertical cylinder, including caps."""
    positions = points_array(points)
    center = np.asarray(center, float)
    if (center.shape != (2,) or not np.isfinite(center).all() or
            not all(math.isfinite(float(x)) for x in (radius, height, base_z)) or
            radius <= 0 or height <= 0):
        raise ValueError('Invalid finite capped cylinder')
    radial = np.linalg.norm(positions[:, :2] - center, axis=1) - radius
    vertical = np.abs(positions[:, 2] - base_z - height / 2.) - height / 2.
    offset = np.column_stack((radial, vertical))
    return np.abs(np.linalg.norm(np.maximum(offset, 0.), axis=1) +
                  np.minimum(np.max(offset, axis=1), 0.))


def scene_surface_distances(points, description):
    """Minimum distance to the individual analytic primitive surfaces.

    Occluded and intersecting primitive faces remain in this accuracy reference;
    coverage below uses only nearest-ray visible surface samples instead.
    """
    positions = points_array(points)
    bounds = description['bounds']
    nearest = rectangle_surface_distances(positions, bounds['x'], bounds['y'],
                                           description.get('floor_z', 0.))
    for obstacle in description['obstacles']:
        if obstacle['type'] == 'box':
            distance = box_surface_distances(positions, obstacle['center'], obstacle['size'])
        elif obstacle['type'] == 'cylinder':
            distance = cylinder_surface_distances(positions, obstacle['center'],
                obstacle['radius'], obstacle['height'], obstacle.get('base_z', 0.))
        else:
            raise ValueError('Unsupported scene primitive: ' + str(obstacle['type']))
        nearest = np.minimum(nearest, distance)
    return nearest


def distance_distribution(values):
    values = np.asarray(values, float)
    if values.ndim != 1 or not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError('Expected finite nonnegative distances')
    if not len(values):
        return {'samples': 0, 'mean': None, 'rmse': None, 'median': None,
                'p90': None, 'p95': None, 'p99': None, 'max': None,
                'fractions_within_m': {format(t, '.2f'): None for t in THRESHOLDS}}
    return dict(samples=len(values), mean=float(values.mean()),
        rmse=float(np.sqrt(np.mean(values * values))), median=float(np.median(values)),
        p90=float(np.percentile(values, 90)), p95=float(np.percentile(values, 95)),
        p99=float(np.percentile(values, 99)), max=float(values.max()),
        fractions_within_m={format(t, '.2f'): float(np.mean(values <= t)) for t in THRESHOLDS})


def voxel_samples(points, voxel_size=.10):
    positions = points_array(points)
    if not math.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError('Voxel size must be positive and finite')
    keys = np.floor(positions / voxel_size).astype(np.int64)
    if not len(keys):
        return keys, positions.copy()
    unique, indices = np.unique(keys, axis=0, return_index=True)
    # Keep an actual ray hit rather than a voxel center or cross-face centroid.
    return unique, positions[indices].copy()


def select_actual_views(rows, robot, start_stamp_ns=None, end_stamp_ns=None,
                        max_views=150):
    """Nearest recorded true pose to each uniformly spaced time target."""
    if isinstance(max_views, bool) or not isinstance(max_views, int) or not 1 <= max_views <= 200:
        raise ValueError('Use between one and 200 sampled views per robot')
    candidates = {}
    for row in rows:
        if row.get('kind') != 'actual' or row.get('robot_id') != robot:
            continue
        stamp = row.get('stamp_ns')
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise ValueError('Invalid actual-pose timestamp')
        if start_stamp_ns is not None and stamp < start_stamp_ns:
            continue
        if end_stamp_ns is not None and stamp > end_stamp_ns:
            continue
        position, quaternion = np.asarray(row['p'], float), np.asarray(row['q'], float)
        if (position.shape != (3,) or quaternion.shape != (4,) or
                not np.isfinite(position).all() or not np.isfinite(quaternion).all() or
                abs(np.linalg.norm(quaternion) - 1.) > .01):
            raise ValueError('Invalid actual-pose position/quaternion')
        candidates[stamp] = row
    stamps = sorted(candidates)
    if len(stamps) <= max_views:
        return [candidates[t] for t in stamps]
    selected = set()
    # Relative times preserve nanosecond resolution better than linspace on a
    # large absolute UNIX epoch float.
    offsets = np.linspace(0, stamps[-1] - stamps[0], max_views)
    for offset in offsets:
        target = stamps[0] + int(round(float(offset)))
        i = bisect.bisect_left(stamps, target)
        choices = [j for j in (i-1, i) if 0 <= j < len(stamps)]
        selected.add(min(choices, key=lambda j: (abs(stamps[j] - target), j)))
    return [candidates[stamps[i]] for i in sorted(selected)]


def visible_truth(description, poses, *, width=80, height=60, maximum_range=8.,
                  voxel_size=.10):
    if width < 1 or height < 1 or not math.isfinite(maximum_range) or maximum_range <= .1:
        raise ValueError('Invalid visibility camera geometry')
    scene = RayScene(description, use_native=False)
    # Preserve the original 320x240 camera's fx=fy=160 field of view. A smaller
    # pixel represents the center of a bin in that original image plane.
    rays, _ = camera_rays(width, height, width * 160. / 320.,
                          height * 160. / 240., (width-1)/2., (height-1)/2.)
    hits = []
    for pose in poses:
        rotation = rotation_xyzw(pose['q'])
        directions = rays @ (rotation @ R_BC).T
        distance, points, _normals, _ids = scene.cast(pose['p'], directions,
                                                     minimum=.1, maximum=maximum_range)
        hits.append(points[np.isfinite(distance)])
    points = np.vstack(hits) if hits else np.empty((0, 3))
    keys, samples = voxel_samples(points, voxel_size)
    return {'keys': keys, 'points': samples, 'views': len(poses),
            'selected_pose_stamps_ns': [p['stamp_ns'] for p in poses],
            'raw_ray_hit_count': len(points), 'voxel_sample_count': len(samples)}


def nearest_vertex_distances(surface_samples, vertices):
    samples, vertices = points_array(surface_samples), points_array(vertices)
    if not len(vertices):
        raise ValueError('Native vertex set is empty')
    from scipy.spatial import cKDTree
    distances, _ = cKDTree(vertices).query(samples, k=1)
    return np.asarray(distances, float)


def referenced_vertex_indices(vertices, triangles):
    positions = points_array(vertices)
    faces = np.asarray(triangles)
    if (faces.ndim != 2 or faces.shape[1] != 3 or not len(faces) or
            faces.dtype.kind not in 'iu' or np.any(faces < 0) or np.any(faces >= len(positions))):
        raise ValueError('Native faces must contain valid integer vertex indices')
    return np.unique(faces.ravel()).astype(np.int64)


def coverage_report(surface_samples, vertices):
    distance = nearest_vertex_distances(surface_samples, vertices)
    return dict(method='visible GT sample to nearest supplied evaluation vertex',
                evaluated_vertex_count=len(vertices),
                distance_m=distance_distribution(distance),
                coverage_at_m={format(t, '.2f'): (float(np.mean(distance <= t))
                    if len(distance) else None) for t in THRESHOLDS})


def evaluate_meshes(meshes, description, quality, rows, start_stamp_ns=None,
                     max_views=150, voxel_size=.10, end_stamp_ns=None):
    if set(meshes) != {0, 1}:
        raise ValueError('Exactly one native mesh from each robot is required')
    result = quality.get('results', {}).get('body_path', {})
    if result.get('status') != 'evaluated' or 'shared_alignment' not in result:
        raise ValueError('Trajectory quality has no evaluated shared body-path alignment')
    alignment = validate_alignment(result['shared_alignment'])
    window = quality.get('source_time_window_ns', [None, None])
    if not isinstance(window, (list, tuple)) or len(window) != 2:
        raise ValueError('Trajectory quality time window must have two boundaries')
    start = start_stamp_ns if start_stamp_ns is not None else window[0]
    end = end_stamp_ns if end_stamp_ns is not None else window[1]
    if any(value is not None and (isinstance(value, bool) or not isinstance(value, int) or value <= 0)
           for value in (start, end)):
        raise ValueError('Evaluation timestamps must be positive integer nanoseconds')
    if start is not None and end is not None and start > end:
        raise ValueError('Evaluation time window is reversed')
    report = dict(schema_version=2, evaluation_only=True, truth_is_estimator_input=False,
        map_is_modified=False, icp_performed=False, scale_estimated=False,
        independent_submap_alignment=False, shared_alignment=alignment.tolist(),
        alignment_source='quality.results.body_path.shared_alignment',
        native_acceptance_validated_by_this_tool=False,
        native_input_selection='Caller supplies accepted native meshes; receipts are validated upstream',
        accuracy_weighting='all face-referenced native vertices, not uniform surface area',
        primary_surface_vertex_scope='Every unique native vertex index referenced by native triangles; no decimation',
        unreferenced_vertex_policy='Preserve original map; exclude unused buffer slots from primary accuracy and every coverage KDTree',
        accuracy_definition='Unsigned distance to nearest individual bounded analytic primitive surface',
        coverage_definition='Voxel-deduplicated nearest-ray visible GT points to nearest face-referenced native vertices',
        limitations=[
            'Point-to-face-referenced-native-vertex approximation, not exact point-to-triangle distance.',
            'Uniform-time finite camera-ray samples and voxel occupancy are not uniform surface-area sampling.',
            'Accuracy reference includes occluded/intersecting primitive faces; visible-GT coverage excludes occluded rays.',
            'No semantic evaluation and no claim of reproducing the complete original-paper evaluation.',
            'Caller selects accepted native mesh files; this tool does not revalidate native optimization receipts.'],
        visibility=dict(width=80, height=60, fx=40., fy=40., minimum_range_m=.1,
            maximum_range_m=8., body_to_camera_rotation=R_BC.tolist(),
            body_to_camera_translation_m=[0., 0., 0.], maximum_views_per_robot=max_views,
            voxel_size_m=voxel_size, representative='first measured hit per world-axis-aligned voxel',
            start_stamp_ns=start, end_stamp_ns=end,
            time_window_source='CLI overrides quality.source_time_window_ns; absent bounds include all actual poses',
            mesh_stamp_usage='diagnostic only; never narrows the visible-GT coverage denominator'), robots={})
    transformed, surfaces, visible, accuracy, all_slot_accuracy = {}, {}, {}, {}, {}
    for robot in (0, 1):
        record = meshes[robot]
        transformed[robot] = apply_alignment(record['vertices'], alignment)
        if not len(transformed[robot]):
            raise ValueError('Empty native mesh')
        references = referenced_vertex_indices(record['vertices'], record['triangles'])
        surfaces[robot] = transformed[robot][references]
        poses = select_actual_views(rows, robot, start, end, max_views)
        if not poses:
            raise ValueError('No actual poses in evaluation time window for robot %d' % robot)
        visible[robot] = visible_truth(description, poses, voxel_size=voxel_size)
        accuracy[robot] = scene_surface_distances(surfaces[robot], description)
        all_slot_accuracy[robot] = scene_surface_distances(transformed[robot], description)
        report['robots'][str(robot)] = dict(native_vertices=len(transformed[robot]),
            native_face_referenced_vertices=len(references),
            native_unreferenced_vertex_slots=len(transformed[robot])-len(references),
            native_triangles=len(record['triangles']), mesh_stamp_ns=record.get('stamp_ns'),
            transformed_vertices_to_analytic_surface_m=distance_distribution(accuracy[robot]),
            all_native_slots_to_analytic_surface_m_diagnostic=distance_distribution(all_slot_accuracy[robot]),
            visible_truth={k: v for k, v in visible[robot].items() if k not in ('keys', 'points')},
            visible_surface_coverage=coverage_report(visible[robot]['points'], surfaces[robot]))
    all_surface_vertices = np.vstack([surfaces[0], surfaces[1]])
    _, union_points = voxel_samples(np.vstack([visible[0]['points'], visible[1]['points']]), voxel_size)
    report['team'] = dict(native_vertices=sum(len(transformed[r]) for r in (0, 1)),
        native_face_referenced_vertices=len(all_surface_vertices),
        native_unreferenced_vertex_slots=sum(len(transformed[r])-len(surfaces[r]) for r in (0, 1)),
        native_triangles=sum(len(meshes[r]['triangles']) for r in (0, 1)),
        transformed_vertices_to_analytic_surface_m=distance_distribution(np.r_[accuracy[0], accuracy[1]]),
        all_native_slots_to_analytic_surface_m_diagnostic=distance_distribution(np.r_[all_slot_accuracy[0], all_slot_accuracy[1]]),
        visible_union_sample_count=len(union_points),
        visible_union_coverage=coverage_report(union_points, all_surface_vertices))
    indices0 = {tuple(k): i for i, k in enumerate(visible[0]['keys'])}
    keys1 = {tuple(k) for k in visible[1]['keys']}
    common = sorted(set(indices0).intersection(keys1))
    common_points = (visible[0]['points'][[indices0[k] for k in common]]
                     if common else np.empty((0, 3)))
    distance0 = nearest_vertex_distances(common_points, surfaces[0])
    distance1 = nearest_vertex_distances(common_points, surfaces[1])
    report['jointly_visible_truth'] = dict(
        definition='Intersection of the two robots visible GT voxel IDs; representative is robot 0 actual hit',
        sample_count=len(common_points),
        robot_0_coverage=coverage_report(common_points, surfaces[0]),
        robot_1_coverage=coverage_report(common_points, surfaces[1]),
        either_map_coverage_at_m={format(t, '.2f'): (float(np.mean(np.minimum(distance0, distance1) <= t))
            if len(common_points) else None) for t in THRESHOLDS},
        both_maps_coverage_at_m={format(t, '.2f'): (float(np.mean(np.maximum(distance0, distance1) <= t))
            if len(common_points) else None) for t in THRESHOLDS})
    return report


def load_native_mesh(path, robot):
    """Decode the locked full native ROS1 schema without a ROS runtime.

    The raw file has no TCPROS MD5 handshake. Its complete schema, frame,
    ownership, array layouts, finite data and triangle indices are validated by
    the same decoder used by the independently verified native-map exporter.
    Native acceptance/transport MD5 provenance is checked upstream.
    """
    raw = Path(path).read_bytes()
    mesh = decode_mesh(raw, 0, robot)
    return dict(vertices=mesh['xyz'], triangles=mesh['triangles'],
        stamp_ns=mesh['stamp_ns'], frame_id=mesh['frame'],
        robot_id=robot, source=str(Path(path).resolve()), bytes=len(raw),
        sha256=hashlib.sha256(raw).hexdigest(),
        canonical_ros_sha256=mesh['canonical_sha256'],
        message_type='kimera_pgmo_msgs/KimeraPgmoMesh', message_md5=MESSAGE_MD5,
        decoder='locked_complete_native_ros1_schema_without_ros_runtime',
        message_md5_is_decoder_schema_contract=True)


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', action='append', required=True, help='ROBOT=FILE; exactly robot 0 and 1')
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--quality', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--start-stamp-ns', type=int)
    parser.add_argument('--end-stamp-ns', type=int,
                        help='Full evaluation cutoff; defaults to quality.source_time_window_ns[1]')
    parser.add_argument('--max-views-per-robot', type=int, default=150)
    parser.add_argument('--gt-voxel-size', type=float, default=.10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = {}
    for item in args.mesh:
        try:
            owner, path = item.split('=', 1)
            robot = int(owner)
        except (ValueError, TypeError):
            parser.error('Each mesh must have the form 0=FILE or 1=FILE')
        if robot not in (0, 1) or robot in paths:
            parser.error('Provide exactly one mesh per robot')
        paths[robot] = Path(path)
    if set(paths) != {0, 1}:
        parser.error('Both robot mesh files are required')
    inputs = {'scene': args.scene, 'trajectory_quality': args.quality,
              'quality_poses': args.poses, 'mesh_0': paths[0], 'mesh_1': paths[1]}
    if args.output.resolve() in {p.resolve() for p in inputs.values()}:
        parser.error('Output must not overwrite any source input')
    hashes = {name: dict(path=str(path.resolve()), sha256=file_sha256(path))
              for name, path in inputs.items()}
    meshes = {r: load_native_mesh(paths[r], r) for r in (0, 1)}
    description, quality = json.loads(args.scene.read_text()), json.loads(args.quality.read_text())
    rows = [json.loads(line) for line in args.poses.read_text().splitlines() if line.strip()]
    run_ids = sorted({str(row['run_id']) for row in rows
        if row.get('kind') == 'actual' and 'run_id' in row})
    if len(run_ids) > 1:
        raise ValueError('Actual poses mix multiple experiment run IDs')
    result = evaluate_meshes(meshes, description, quality, rows, args.start_stamp_ns,
                             args.max_views_per_robot, args.gt_voxel_size,
                             end_stamp_ns=args.end_stamp_ns)
    result['sources'] = hashes
    result['native_mesh_inputs'] = {str(r): {k: v for k, v in meshes[r].items()
        if k not in ('vertices', 'triangles')} for r in (0, 1)}
    result['actual_pose_run_ids'] = run_ids
    for name, path in inputs.items():
        if file_sha256(path) != hashes[name]['sha256']:
            raise ValueError('An input changed during evaluation: ' + name)
    result['source_inputs_unchanged'] = True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + '.tmp')
    temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    temporary.replace(args.output)
    print(json.dumps(dict(output=str(args.output), vertices=result['team']['native_vertices'],
        evaluated_surface_vertices=result['team']['native_face_referenced_vertices'],
        accuracy_m=result['team']['transformed_vertices_to_analytic_surface_m'],
        coverage=result['team']['visible_union_coverage']['coverage_at_m']), allow_nan=False))


if __name__ == '__main__':
    main()
