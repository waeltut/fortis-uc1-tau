import math
from types import SimpleNamespace
import unittest
import numpy as np
from mur_base_selector.core import (Grid, angle, hull, padded, transform_points,
    swept_polygons, diverse_order, path_length, score)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.poly = np.array([[-.4, -.2], [.4, -.2], [.4, .2], [-.4, .2]])

    def grid(self, cells=None, origin=(0, 0, 0)):
        d = np.zeros((100, 100), dtype=np.uint8)
        for x, y, value in cells or []:
            d[y, x] = value
        return Grid(d, 100, 100, .1, origin)

    def test_obstacle_inside_footprint_not_only_outline(self):
        g = self.grid([(50, 50, 254)])
        p = transform_points(self.poly, (5, 5, 0))
        self.assertFalse(g.check(p).valid)

    def test_centre_clear_corner_hits(self):
        g = self.grid([(53, 51, 254)])
        self.assertFalse(g.check(transform_points(self.poly, (5, 5, 0))).valid)

    def test_cell_corner_contact_is_collision(self):
        g = self.grid([(54, 52, 254)])
        self.assertFalse(g.check(transform_points(self.poly, (5, 5, 0))).valid)

    def test_inflation_is_soft(self):
        g = self.grid([(50, 50, 253)])
        c = g.check(transform_points(self.poly, (5, 5, 0)))
        self.assertTrue(c.valid)
        self.assertGreater(c.cost, 0)

    def test_unknown_policy(self):
        g = self.grid([(50, 50, 255)])
        p = transform_points(self.poly, (5, 5, 0))
        self.assertFalse(g.check(p).valid)
        self.assertTrue(g.check(p, allow_unknown=True).valid)

    def test_global_and_local_coverage(self):
        g = self.grid()
        p = transform_points(self.poly, (-5, -5, 0))
        self.assertFalse(g.check(p).valid)
        self.assertTrue(g.check(p, partial=True).valid)
        self.assertFalse(g.check(p, partial=True).covered)

    def test_partial_local_overlap_still_detects_obstacle(self):
        g = self.grid([(0, 50, 254)])
        self.assertFalse(g.check(transform_points(self.poly, (-.2, 5, 0)), partial=True).valid)

    def test_rotated_map_origin(self):
        origin = (10, -4, math.pi/2)
        p = transform_points(transform_points(self.poly, (5, 5, 0)), origin)
        self.assertFalse(self.grid([(50, 50, 254)], origin).check(p).valid)
        self.assertTrue(self.grid(origin=origin).check(p).valid)

    def test_padding_closes_small_gap(self):
        g = self.grid([(55, 50, 254)])
        self.assertTrue(g.check(transform_points(self.poly, (5, 5, 0))).valid)
        self.assertFalse(g.check(transform_points(padded(self.poly, .11), (5, 5, 0))).valid)

    def test_sweep_catches_obstacle_between_waypoints(self):
        g = self.grid([(50, 50, 254)])
        poses = [(4, 5, 0), (6, 5, 0)]
        self.assertTrue(all(g.check(transform_points(self.poly, p)).valid for p in poses))
        self.assertTrue(any(not g.check(p).valid for p in swept_polygons(self.poly, poses, .05, .1)))

    def test_turn_sweep_catches_corner_collision(self):
        poly = np.array([[-1., -.1], [1., -.1], [1., .1], [-1., .1]])
        g = self.grid([(56, 56, 254)])
        poses = [(5, 5, 0), (5, 5, math.pi/2)]
        self.assertTrue(all(g.check(transform_points(poly, p)).valid for p in poses))
        self.assertTrue(any(not g.check(p).valid for p in swept_polygons(poly, poses, .05, .15)))

    def test_shortest_angle_across_wrap(self):
        self.assertAlmostEqual(angle(-math.pi+.01-(math.pi-.01)), .02)
        sweeps = list(swept_polygons(self.poly, [(0, 0, math.pi-.01), (0, 0, -math.pi+.01)], .05, .1))
        self.assertEqual(len(sweeps), 2)

    def test_invalid_grid_data_rejected(self):
        for args in [([0], 2, 2, .1), ([-1], 1, 1, .1), ([256], 1, 1, .1), ([0], 1, 1, 0)]:
            with self.assertRaises(ValueError):
                Grid(*args)

    def test_degenerate_or_nan_footprint_rejected(self):
        for p in [[(0, 0), (1, 0), (2, 0)], [(0, 0), (1, 0), (0, math.nan)]]:
            with self.assertRaises(ValueError):
                hull(p)

    def test_diversity_keeps_fallback_candidates(self):
        cs = [SimpleNamespace(id=i, pose=p, preliminary=float(i)) for i, p in
              enumerate([(0, 0, 0), (.01, 0, 0), (1, 0, 0), (0, 0, 1)])]
        result = diverse_order(cs)
        self.assertEqual([c.id for c in result], [0, 2, 3, 1])

    def test_actual_path_length_and_score(self):
        self.assertAlmostEqual(path_length([(0, 0, 0), (3, 4, 0), (3, 5, 0)]), 6.)
        self.assertLess(score(1, 0, 0, 0, [1, 2, .2, .2], 5),
                        score(1, 1, 0, 0, [1, 2, .2, .2], 5))


if __name__ == '__main__':
    unittest.main()
