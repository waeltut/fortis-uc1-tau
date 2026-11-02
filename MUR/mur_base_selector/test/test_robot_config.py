import tempfile
from pathlib import Path
import unittest
import numpy as np
from MUR.mur_base_selector.mur_base_selector.robot_config import arm_names,monitored_joints
from MUR.mur_base_selector.mur_base_selector.volume import Volume,save_volume
from MUR.mur_base_selector.mur_base_selector.volume_workers import generate_parallel
from MUR.mur_base_selector.mur_base_selector.visual_data import voxel_layers,candidate_snapshot


class RobotConfigTests(unittest.TestCase):
    def cache(self,names):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        path=Path(temp.name)/'volume.npz'
        meta={'schema':2,'reachability_policy':'self_body_ignore_other_arm_v1',
              'context':{'arm_names':names},'generation':{'tool_axis':[1,0,0],
              'min_inward_angle_deg':0,'workspace_fov_deg':360,'workspace_front_offset_deg':0}}
        save_volume(path,meta,np.array([[.6,0,.8]]),np.array([[0,0,0,1.]]),np.ones((len(names),1,1),bool),np.ones((len(names),1),bool))
        return Volume(path)

    def query(self,v,mode):
        return v.query([1,0,.8,0,0,0,1],0,1,.9,.01,1,360,0,mode,include_profiles=True)

    def test_single_arm_inverse_projection(self):
        v=self.cache(['manipulator']);rows=self.query(v,'either')
        self.assertEqual(len(rows),1);self.assertEqual(rows[0][1],'manipulator')
        self.assertAlmostEqual(rows[0][0][0],.4)
        self.assertEqual(list(rows[0][6]),['manipulator'])

    def test_three_arms_and_intersection(self):
        v=self.cache(['front','rear','upper'])
        self.assertEqual([r[1] for r in self.query(v,'either')],v.arm_names)
        self.assertEqual(self.query(v,'rear')[0][1],'rear')
        self.assertEqual(set(self.query(v,'both')[0][6]),set(v.arm_names))
        v.masks[2]=False
        self.assertEqual(self.query(v,'both'),[])

    def test_unknown_mode_rejected(self):
        with self.assertRaisesRegex(ValueError,'Unknown arm'):self.query(self.cache(['arm']),'left')

    def test_name_validation_and_legacy_order(self):
        self.assertEqual(arm_names({}),['left','right'])
        for names in ([],['arm','arm'],['both'],['Arm'],['tool/arm']):
            with self.assertRaises(ValueError):arm_names({'context':{'arm_names':names}})

    def test_joints_follow_configured_group_order(self):
        m={'context':{'arm_names':['tool','aux'],'arm_joints':{'tool':['joint2','joint1'],'aux':['joint3']}}}
        self.assertEqual(monitored_joints(m),['joint2','joint1','joint3'])

    def test_custom_arm_visualization(self):
        v=self.cache(['manipulator'])
        self.assertEqual(voxel_layers(v,.9,0,2,'both',10)[0][0],'manipulator')
        self.assertEqual(candidate_snapshot(self.query(v,'either'),10)['counts']['manipulator'],1)

    def test_one_arm_offline_worker(self):
        class Engine:
            def evaluate_offline(self,arm,poses,*args):
                self.arm=arm;return [True]*len(poses)
        engine=Engine();masks=np.zeros((1,1,1),bool);done=np.zeros((1,1),bool)
        generate_parallel([engine],np.array([[.6,0,.8]]),np.array([[0,0,0,1]]),np.ones((1,1),bool),
                          [0,0,0,0,0,0,1],dict(ik_timeout=.01,attempts=1,save_every_voxels=1),masks,done,
                          lambda:None,lambda *a:None,workers=1)
        self.assertTrue(masks.all());self.assertTrue(done.all());self.assertEqual(engine.arm,0)
