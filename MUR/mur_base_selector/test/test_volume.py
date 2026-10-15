import math
from pathlib import Path
import tempfile
import time
import unittest
import sys
sys.path.insert(0,str(Path(__file__).parents[1]))
import numpy as np
from mur_base_selector.volume import Volume,save_volume,orientations,workspace_points,allowed_orientations,TravelField,sector_penalty,qmul
from mur_base_selector.core import Grid


class VolumeTests(unittest.TestCase):
    def volume(self,mask=None,done=True):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        cfg={'tool_axis':[1,0,0],'min_inward_angle_deg':0,'workspace_fov_deg':250,'workspace_front_offset_deg':0,'inward_reference':'horizontal'}
        points=np.array([[.6,0,.8],[.6,.1,.8]])
        quats=np.array([[0,0,0,1],[0,0,1,0.]])
        masks=np.ones((2,2,2),bool) if mask is None else mask
        path=Path(tmp.name)/'cache.npz'
        save_volume(path,{'schema':2,'reachability_policy':'self_body_ignore_other_arm_v1','generation':cfg},points,quats,masks,np.full((2,2),done))
        return path

    def query(self,v,**kwargs):
        args=dict(target=[2.5,0,.8,0,0,0,1],base_z=0,yaw_bins=4,min_coverage=.9,max_height_error=.05,max_angle_deg=1,fov_deg=250,front_offset_deg=0,mode='left')
        args.update(kwargs);return v.query(**args)

    def test_inverse_projection_and_yaw(self):
        rows=self.query(Volume(self.volume()))
        self.assertEqual(len(rows),4)
        self.assertAlmostEqual(rows[0][0][0],1.9)
        back=next(r for r in rows if abs(r[0][2]-math.pi)<1e-9)
        self.assertAlmostEqual(back[0][0],3.1)

    def test_missing_orientation_never_passes_broad_coverage(self):
        masks=np.ones((2,2,2),bool);masks[0,:,0]=False
        rows=self.query(Volume(self.volume(masks)),min_coverage=.5)
        self.assertTrue(rows);self.assertTrue(all(abs(r[0][2]-math.pi)<1e-9 for r in rows))

    def test_both_requires_same_voxel_orientation(self):
        masks=np.zeros((2,2,2),bool);masks[0,0]=True;masks[1,1]=True
        self.assertEqual(self.query(Volume(self.volume(masks)),mode='both'),[])

    def test_coverage_threshold(self):
        masks=np.ones((2,2,2),bool);masks[:,:,1]=False
        self.assertEqual(self.query(Volume(self.volume(masks))),[])

    def test_height_not_extrapolated(self):
        self.assertEqual(self.query(Volume(self.volume()),target=[2,0,1,0,0,0,1]),[])

    def test_quaternion_sign_equivalence(self):
        v=Volume(self.volume());self.assertEqual(self.query(v),self.query(v,target=[2.5,0,.8,0,0,0,-1]))

    def test_larger_fov_rejected(self):
        with self.assertRaises(ValueError):self.query(Volume(self.volume()),fov_deg=300)

    def test_smaller_fov_applied(self):
        rows=self.query(Volume(self.volume()),fov_deg=10)
        self.assertEqual(len(rows),2)

    def test_incomplete_cache_rejected(self):
        path=self.volume(done=False)
        with self.assertRaises(ValueError):Volume(path)
        self.assertFalse(Volume(path,False).done.all())

    def test_orientation_set_includes_roll_and_is_unique(self):
        q=orientations(90)
        self.assertTrue(np.allclose(np.linalg.norm(q,axis=1),1))
        dots=np.abs(q@q.T);np.fill_diagonal(dots,0)
        self.assertLess(dots.max(),1-1e-9)
        self.assertTrue(any(abs(np.dot(x,[math.sqrt(.5),0,0,math.sqrt(.5)]))>1-1e-9 for x in q))

    def test_tool_direction_rejects_inward_accepts_downward(self):
        qs=np.array([[0,0,0,1],[0,0,1,0],[0,math.sqrt(.5),0,math.sqrt(.5)]])
        self.assertEqual(allowed_orientations([.6,0,.8],qs,[1,0,0],70).tolist(),[True,False,True])

    def test_generation_fov_filters_rear(self):
        cfg={'bounds_min':[-1,-1,0],'bounds_max':[1,1,1],'voxel_size':.25,'workspace_front_offset_deg':0,'workspace_fov_deg':180}
        self.assertTrue((workspace_points(cfg)[:,0]>=0).all())

    def test_facing_sectors(self):
        self.assertEqual(sector_penalty(math.radians(30),0,60),0)
        self.assertGreater(sector_penalty(math.radians(-30),0,60),0)

    def test_field_detours_around_wall(self):
        data=np.zeros((7,7),dtype=np.uint8);data[:6,3]=254
        field=TravelField(Grid(data,7,7,1),(.5,.5))
        self.assertGreater(field.lookup([[6.5,.5]])[0],10)

    def test_diagonal_corner_cut_blocked(self):
        data=np.array([[0,254],[254,0]],dtype=np.uint8)
        field=TravelField(Grid(data,2,2,1),(.5,.5))
        self.assertTrue(math.isinf(field.lookup([[1.5,1.5]])[0]))

    def test_rotated_grid_origin(self):
        field=TravelField(Grid([0]*9,3,3,1,(10,20,math.pi/2)),(9.5,20.5))
        self.assertAlmostEqual(field.lookup([[9.5,22.5]])[0],2.)

    def test_truncated_field_does_not_publish_unsettled_costs(self):
        field=TravelField(Grid([0]*9,3,3,1),(.5,.5),deadline=time.monotonic()-1)
        self.assertTrue(field.truncated);self.assertTrue(np.isinf(field.lookup([[.5,.5],[2.5,2.5]])).all())

    def test_unknown_policy(self):
        grid=Grid([0,255,0],3,1,1)
        self.assertTrue(math.isinf(TravelField(grid,(.5,.5)).lookup([[2.5,.5]])[0]))
        self.assertTrue(math.isfinite(TravelField(grid,(.5,.5),allow_unknown=True).lookup([[2.5,.5]])[0]))


if __name__=='__main__':unittest.main()
