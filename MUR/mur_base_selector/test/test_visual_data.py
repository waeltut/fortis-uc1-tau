import unittest
from types import SimpleNamespace
import numpy as np
from mur_base_selector.visual_data import voxel_layers,candidate_snapshot

class VisualDataTests(unittest.TestCase):
    def volume(self):
        return SimpleNamespace(points=np.array([[1,0,.5],[1,0,1.],[1,0,1.5]]),coverage=np.array([[.95,.5,1.],[.8,.95,1.]]),done=np.array([[True,True,False],[True,True,True]]))
    def test_threshold_and_completed(self):
        layers=voxel_layers(self.volume(),.9,-10,10,'both',100)
        self.assertEqual([v[3] for v in layers],[1,2])
    def test_height_slice(self):
        layers=voxel_layers(self.volume(),.1,.9,1.1,'both',100)
        self.assertEqual([v[3] for v in layers],[1,1])
        np.testing.assert_array_equal(layers[0][1],[[1,0,1.]])
    def test_intersection_requires_both_completed_and_coverage(self):
        self.assertEqual(voxel_layers(self.volume(),.9,-10,10,'intersection',100)[0][3],0)
        self.assertEqual(voxel_layers(self.volume(),.75,-10,10,'intersection',100)[0][3],1)
    def test_cap_preserves_exact_count(self):
        layer=voxel_layers(self.volume(),.1,-10,10,'right',1)[0]
        self.assertEqual(len(layer[1]),1);self.assertEqual(layer[3],3)
    def test_empty_candidates(self):
        d=candidate_snapshot([],10);self.assertEqual(d['counts']['total'],0);self.assertFalse(d['sampled'])
    def test_forward_sector_survives_sampling(self):
        rows=[((i,0,0),'left',1.57) for i in range(100)]+[((0,0,0),'right',0.)]
        d=candidate_snapshot(rows,10)
        self.assertEqual(d['counts']['total'],101);self.assertEqual(d['counts']['forward_30deg'],1)
        self.assertTrue(any(p[4]==0 for p in d['poses']));self.assertTrue(d['sampled']);self.assertLessEqual(d['displayed'],10)
    def test_invalid_view_limits(self):
        for args in ((1.1,-10,10,'both',10),(.9,1,0,'both',10),(.9,-10,10,'bad',10)):
            with self.assertRaises(ValueError):voxel_layers(self.volume(),*args)
