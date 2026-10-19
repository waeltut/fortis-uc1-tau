"""Derived profile acceptance must use only existing, successful sampled bits."""
import math
import tempfile
from pathlib import Path
import unittest
import numpy as np
from mur_base_selector.volume import Volume,save_volume
from mur_base_selector.visual_data import voxel_layers


def qyaw(degrees):
    a=math.radians(degrees)/2
    return [0,0,math.sin(a),math.cos(a)]


class ProfileTests(unittest.TestCase):
    def volume(self,left=(1,1,0,0),right=None):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        self.path=Path(directory.name)/'cache.npz'
        cfg={'tool_axis':[1,0,0],'min_inward_angle_deg':70.,'workspace_fov_deg':250.,'workspace_front_offset_deg':0.,'inward_reference':'horizontal'}
        masks=np.array([[left],[left if right is None else right]],dtype=bool)
        save_volume(self.path,{'schema':2,'reachability_policy':'self_body_ignore_other_arm_v1','generation':cfg},np.array([[.6,0,.8]]),np.array([qyaw(a) for a in (0,60,90,110)]),masks,np.ones((2,1),bool))
        return Volume(self.path)
    def query(self,v,yaw=0,minimum=.9,mode='left'):
        return v.query([2.5,0,.8,*qyaw(yaw)],0,1,minimum,.05,5,250,0,mode,include_profiles=True)
    def test_120_adds_voxel_without_modifying_file(self):
        v=self.volume();before=self.path.read_bytes();self.assertEqual(self.query(v),[])
        v.configure_profiles([70,90,120]);rows=self.query(v)
        self.assertEqual(len(rows),1);self.assertEqual(rows[0][6]['left']['min_inward_angle_deg'],120)
        self.assertEqual(rows[0][6]['left']['allowed_samples'],2)
        self.assertEqual(before,self.path.read_bytes());self.assertAlmostEqual(v.coverage[0,0],.5)
    def test_90_profile_includes_sideways_and_wins_before_120(self):
        v=self.volume((1,1,1,0));v.configure_profiles([70,90,120])
        self.assertEqual(self.query(v,90)[0][6]['left']['min_inward_angle_deg'],90)
    def test_actual_target_must_fit_profile_not_just_nearest_sample(self):
        v=self.volume();v.configure_profiles([70,90,120])
        self.assertEqual(self.query(v,62),[]) # nearest successful sample at60; actual inward angle118
    def test_successful_target_bit_still_required(self):
        v=self.volume((0,1,1,1));v.configure_profiles([70,90,120])
        self.assertEqual(self.query(v,0,minimum=.5),[])
    def test_union_keeps_base_when_narrow_profile_coverage_is_worse(self):
        v=self.volume((1,0,1,1));v.configure_profiles([70,90,120])
        self.assertEqual(self.query(v,0,minimum=.7)[0][6]['left']['min_inward_angle_deg'],70)
    def test_dual_arm_can_use_different_eligible_profiles(self):
        v=self.volume(right=(1,1,1,1));v.configure_profiles([70,90,120])
        evidence=self.query(v,mode='both')[0][6]
        self.assertEqual(evidence['left']['min_inward_angle_deg'],120)
        self.assertEqual(evidence['right']['min_inward_angle_deg'],70)
    def test_unsupported_wider_or_invalid_profile_rejected(self):
        v=self.volume()
        for angle in (60,-1,180,float('nan')):
            with self.assertRaises(ValueError):v.configure_profiles([angle])
    def test_base_only_is_identical(self):
        v=self.volume((1,1,1,1));before=self.query(v);v.configure_profiles([70]);self.assertEqual(before,self.query(v))
    def test_viewer_uses_selected_profile(self):
        v=self.volume()
        self.assertEqual(voxel_layers(v,.9,-1,2,'left',100,70)[0][3],0)
        self.assertEqual(voxel_layers(v,.9,-1,2,'left',100,120)[0][3],1)
    def test_configuring_profiles_retains_base_profile(self):
        v=self.volume();v.configure_profiles([120,90,120]);self.assertEqual(v.profile_angles,[70,90,120])
