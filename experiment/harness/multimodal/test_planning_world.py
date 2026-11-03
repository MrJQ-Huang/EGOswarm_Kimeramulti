"""The edge-local planning baseline must match the previous hub oracle cloud."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

HARNESS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HARNESS))
from planning_world import planning_occupancy_points, solid_axis


class LocalPlanningGeometryTests(unittest.TestCase):
    def test_full_scene_matches_pre_migration_float32_planning_cloud(self):
        scene = json.loads((HARNESS / 'broad_loop_scene.json').read_text())
        points = np.asarray(planning_occupancy_points(scene, scene.get('planning_cloud_step_m', .12)), dtype='<f4')
        # Measured from the deployed legacy hub function before migration.
        self.assertEqual(points.shape, (209643, 3))
        self.assertEqual(hashlib.sha256(points.tobytes()).hexdigest(),
                         '2262e5aae235e663c7eb6b88cd0210cbb5f0b954c0e4b7e6db1ff49c9fd034f5')

    def test_obstacle_interiors_remain_occupied(self):
        scene = dict(bounds=dict(x=[-2., 6.], y=[-2., 2.], z=[0., 3.]), floor_z=0.,
                     obstacles=[dict(type='box', center=[0., 0., 1.], size=[2., 2., 2.]),
                                dict(type='cylinder', center=[4., 0.], radius=1., height=2.)])
        occupied = set(planning_occupancy_points(scene, .5))
        self.assertIn((0., 0., 1.), occupied)
        self.assertIn((4., 0., 1.), occupied)
        self.assertNotIn((5., 1., 1.), occupied)

    def test_lattice_has_endpoints_center_and_bounded_spacing(self):
        axis = solid_axis(-.23, .77, .12)
        self.assertEqual((axis[0], axis[-1]), (-.23, .77))
        self.assertIn(.27, axis)
        self.assertLessEqual(max(np.diff(axis)), .12 + 1e-9)
        for step in (0., -1., float('nan')):
            with self.assertRaises(ValueError):
                planning_occupancy_points(dict(obstacles=[]), step)


if __name__ == '__main__': unittest.main()
