import copy
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'distributed'))
from mesh_quality import (apply_alignment, box_surface_distances,
    coverage_report, cylinder_surface_distances, evaluate_meshes,
    load_native_mesh, rectangle_surface_distances, scene_surface_distances, select_actual_views,
    referenced_vertex_indices, validate_alignment, visible_truth, voxel_samples)


def floor_scene():
    return dict(bounds={'x': [-20., 20.], 'y': [-20., 20.], 'z': [0., 5.]},
                floor_z=0., obstacles=[])


def pose(robot=0, stamp=100):
    return dict(kind='actual', robot_id=robot, stamp_ns=stamp,
                p=[0., 0., 1.], q=[0., 0., 0., 1.])


def all_referenced_faces(vertex_count):
    return np.array([[0, i, i+1] for i in range(1, vertex_count-1)])


def native_fixture(frame='kimera_world', robot=0, faces=None):
    target = io.BytesIO()
    def string(value):
        raw = value.encode(); target.write(struct.pack('<I', len(raw)) + raw)
    target.write(struct.pack('<III', 3, 101, 0)); string(frame); string(str(robot))
    xyz = np.array([[.12345678901234, 0., 0.], [1., 0., 0.], [0., 1., 0.]], dtype='<f8')
    face = np.array([[0, 1, 2]] if faces is None else faces, dtype='<u4')
    for array in (face, xyz, np.array([[101, 0]] * 3, dtype='<u4'),
                  np.arange(3, dtype='<i4'), np.ones((3, 4), dtype='<f4')):
        target.write(struct.pack('<I', len(array))); target.write(array.tobytes())
    return target.getvalue(), xyz, face


class RosFreeNativeLoaderTests(unittest.TestCase):
    def test_full_locked_native_schema_loads_without_ros_and_preserves_arrays(self):
        raw, xyz, faces = native_fixture()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'native.rosbin'; path.write_bytes(raw)
            with mock.patch.dict(sys.modules, {'rospy': None, 'kimera_pgmo_msgs': None}):
                mesh = load_native_mesh(path, 0)
            np.testing.assert_array_equal(mesh['vertices'], xyz)
            np.testing.assert_array_equal(mesh['triangles'], faces)
            self.assertEqual(mesh['stamp_ns'], 101 * 10 ** 9)
            self.assertEqual(mesh['frame_id'], 'kimera_world')
            self.assertEqual(mesh['decoder'], 'locked_complete_native_ros1_schema_without_ros_runtime')
            self.assertTrue(mesh['message_md5_is_decoder_schema_contract'])
            self.assertEqual(path.read_bytes(), raw)

    def test_wrong_frame_robot_truncation_extra_bytes_and_invalid_faces_fail_closed(self):
        raw, _, _ = native_fixture()
        variants = [raw[:-1], raw + b'unknown schema suffix',
                    native_fixture(frame='world')[0], native_fixture(robot=1)[0],
                    native_fixture(faces=[[0, 1, 100]])[0]]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'native.rosbin'
            for payload in variants:
                path.write_bytes(payload)
                with self.subTest(bytes=len(payload)), self.assertRaises(ValueError):
                    load_native_mesh(path, 0)


class AnalyticSurfaceTests(unittest.TestCase):
    def test_floor_is_a_bounded_rectangle_not_an_infinite_plane(self):
        actual = rectangle_surface_distances([[0., 0., 2.], [2., 0., 0.], [2., 2., 1.]],
                                             [-1., 1.], [-1., 1.])
        np.testing.assert_allclose(actual, [2., 1., np.sqrt(3.)])

    def test_box_distance_is_to_its_surface_also_for_interior_points(self):
        actual = box_surface_distances([[0., 0., 0.], [1., 0., 0.],
            [2., 2., 2.], [.5, .5, .5]], [0., 0., 0.], [2., 2., 2.])
        np.testing.assert_allclose(actual, [1., 0., np.sqrt(3.), .5])

    def test_finite_cylinder_includes_caps_and_rim_distances(self):
        actual = cylinder_surface_distances([[0., 0., 1.], [.5, 0., 1.],
            [1., 0., 1.], [0., 0., 3.], [2., 0., 3.], [0., 0., -1.]],
            [0., 0.], 1., 2.)
        np.testing.assert_allclose(actual, [1., .5, 0., 1., np.sqrt(2.), 1.])

    def test_scene_takes_nearest_primitive_and_rejects_nonfinite_vertices(self):
        scene = floor_scene()
        scene['obstacles'] = [dict(type='box', center=[2., 0., 1.], size=[2., 2., 2.])]
        np.testing.assert_allclose(scene_surface_distances([[1., 0., 1.], [0., 0., .1]], scene), [0., .1])
        with self.assertRaises(ValueError):
            scene_surface_distances([[float('nan'), 0., 0.]], scene)

    def test_shared_transform_rejects_scale_and_reflection_without_mutation(self):
        source = np.array([[1., 2., 3.]])
        transform = np.eye(4); transform[:3, 3] = [2., 3., 4.]
        np.testing.assert_equal(apply_alignment(source, transform), [[3., 5., 7.]])
        np.testing.assert_equal(source, [[1., 2., 3.]])
        for diagonal in ([2., 2., 2., 1.], [-1., 1., 1., 1.]):
            with self.assertRaises(ValueError):
                validate_alignment(np.diag(diagonal))


class VisibilityCoverageTests(unittest.TestCase):
    def test_actual_views_are_uniform_in_time_and_ignore_vio(self):
        rows = [pose(stamp=t) for t in range(1, 102)]
        rows.append(dict(pose(stamp=1000), kind='vio'))
        selected = select_actual_views(rows, 0, 10, 90, max_views=3)
        self.assertEqual([p['stamp_ns'] for p in selected], [10, 50, 90])
        uneven = [pose(stamp=t) for t in [1, 2, 3, 4, 100]]
        self.assertEqual([p['stamp_ns'] for p in select_actual_views(uneven, 0, max_views=3)], [1, 4, 100])

    def test_visibility_uses_nearest_occluding_surface_and_eight_meter_range(self):
        scene = floor_scene()
        scene['obstacles'] = [dict(type='box', center=[3., 0., 1.], size=[1., 2., 2.]),
                              dict(type='box', center=[6., 0., 1.], size=[1., 2., 2.])]
        visible = visible_truth(scene, [pose()], width=1, height=1)
        np.testing.assert_allclose(visible['points'], [[2.5, 0., 1.]])
        scene['obstacles'] = [dict(type='box', center=[10., 0., 1.], size=[1., 2., 2.])]
        self.assertEqual(visible_truth(scene, [pose()], width=1, height=1)['voxel_sample_count'], 0)

    def test_voxels_keep_real_hits_and_coverage_is_point_to_vertex(self):
        points = np.array([[.01, .02, .03], [.02, .02, .03], [1., 0., 0.]])
        keys, samples = voxel_samples(points, .1)
        self.assertEqual(len(keys), 2)
        self.assertTrue(all(any(np.array_equal(p, q) for q in points) for p in samples))
        report = coverage_report([[0., 0., 0.], [.2, 0., 0.], [1., 0., 0.]], [[0., 0., 0.]])
        self.assertEqual(report['coverage_at_m'], {'0.15': 1/3, '0.30': 2/3, '0.50': 2/3})

    def test_both_native_meshes_use_the_same_existing_team_alignment(self):
        scene = floor_scene(); rows = [pose(0), pose(1)]
        truth = visible_truth(scene, [rows[0]])['points']
        alignment = np.eye(4); alignment[:3, 3] = [5., -2., .4]
        native = truth - alignment[:3, 3]
        meshes = {r: dict(vertices=native.copy(), triangles=all_referenced_faces(len(native)), stamp_ns=100)
                  for r in (0, 1)}
        before = copy.deepcopy(meshes)
        quality = {'results': {'body_path': {'status': 'evaluated', 'shared_alignment': alignment.tolist()}}}
        result = evaluate_meshes(meshes, scene, quality, rows, max_views=2)
        self.assertEqual(result['shared_alignment'], alignment.tolist())
        # The shared simulator ray caster produces float32 hit coordinates.
        self.assertLess(result['team']['transformed_vertices_to_analytic_surface_m']['max'], 1e-6)
        self.assertEqual(result['team']['visible_union_coverage']['coverage_at_m']['0.15'], 1.)
        self.assertEqual(result['jointly_visible_truth']['both_maps_coverage_at_m']['0.15'], 1.)
        self.assertTrue(result['evaluation_only'])
        self.assertFalse(result['independent_submap_alignment'])
        for robot in (0, 1):
            np.testing.assert_equal(meshes[robot]['vertices'], before[robot]['vertices'])
            np.testing.assert_equal(meshes[robot]['triangles'], before[robot]['triangles'])

    def test_missing_shared_alignment_never_falls_back_to_independent_fits(self):
        with self.assertRaises(ValueError):
            evaluate_meshes({0: {}, 1: {}}, floor_scene(), {'results': {'vio': {}}}, [])

    def test_late_truth_views_are_not_hidden_by_an_old_mesh_header(self):
        scene = floor_scene()
        initial_surface = visible_truth(scene, [pose()])['points']
        meshes = {r: dict(vertices=initial_surface.copy(), triangles=all_referenced_faces(len(initial_surface)), stamp_ns=100)
                  for r in (0, 1)}
        rows = [pose(r, 100) for r in (0, 1)]
        rows += [dict(pose(r, 300), p=[10., 0., 1.]) for r in (0, 1)]
        quality = {'source_time_window_ns': [100, 300], 'results': {'body_path': {
            'status': 'evaluated', 'shared_alignment': np.eye(4).tolist()}}}
        report = evaluate_meshes(meshes, scene, quality, rows, max_views=150)
        for robot in ('0', '1'):
            self.assertEqual(report['robots'][robot]['mesh_stamp_ns'], 100)
            self.assertEqual(report['robots'][robot]['visible_truth']['selected_pose_stamps_ns'], [100, 300])
        self.assertLess(report['team']['visible_union_coverage']['coverage_at_m']['0.50'], .75)
        # A deliberate operator-specified evaluation cutoff remains explicit.
        earlier = evaluate_meshes(meshes, scene, quality, rows, max_views=150, end_stamp_ns=100)
        self.assertEqual(earlier['robots']['0']['visible_truth']['selected_pose_stamps_ns'], [100])
        self.assertEqual(earlier['visibility']['end_stamp_ns'], 100)

    def test_isolated_ghost_slots_cannot_supply_surface_coverage(self):
        scene = floor_scene(); rows = [pose(0), pose(1)]
        truth = visible_truth(scene, [rows[0]])['points']
        # Only these distant floor points form an actual triangle. The native
        # buffer also carries orphan slots exactly on all visible GT samples.
        active = np.array([[-15., -15., 0.], [-14., -15., 0.], [-15., -14., 0.]])
        vertices = np.vstack([active, truth, [[1000., 1000., 1000.]]])
        meshes = {r: dict(vertices=vertices.copy(), triangles=np.array([[0, 1, 2]]), stamp_ns=100)
                  for r in (0, 1)}
        before = copy.deepcopy(meshes)
        self.assertEqual(coverage_report(truth, vertices)['coverage_at_m']['0.15'], 1.)
        quality = {'results': {'body_path': {'status': 'evaluated', 'shared_alignment': np.eye(4).tolist()}}}
        report = evaluate_meshes(meshes, scene, quality, rows)
        self.assertEqual(report['schema_version'], 2)
        self.assertEqual(report['team']['native_face_referenced_vertices'], 6)
        self.assertEqual(report['team']['native_unreferenced_vertex_slots'], 2*(len(truth)+1))
        self.assertEqual(report['team']['transformed_vertices_to_analytic_surface_m']['max'], 0.)
        self.assertGreater(report['team']['all_native_slots_to_analytic_surface_m_diagnostic']['max'], 1000.)
        for coverage in (report['team']['visible_union_coverage'],
                         report['robots']['0']['visible_surface_coverage'],
                         report['robots']['1']['visible_surface_coverage']):
            self.assertEqual(coverage['coverage_at_m']['0.50'], 0.)
        self.assertEqual(report['jointly_visible_truth']['either_map_coverage_at_m']['0.50'], 0.)
        for robot in (0, 1):
            np.testing.assert_array_equal(meshes[robot]['vertices'], before[robot]['vertices'])
            np.testing.assert_array_equal(meshes[robot]['triangles'], before[robot]['triangles'])

    def test_surface_reference_indices_reject_invalid_or_fractional_faces(self):
        vertices = np.eye(3)
        for triangles in ([[0, 1, 9]], [[0, -1, 2]], [[0., 1.5, 2.]], []):
            with self.subTest(triangles=triangles), self.assertRaises(ValueError):
                referenced_vertex_indices(vertices, triangles)


if __name__ == '__main__':
    unittest.main()
