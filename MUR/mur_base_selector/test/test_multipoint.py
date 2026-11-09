"""Multi-target intersections must share XY AND yaw and preserve arm requirements."""
import asyncio
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from test_volume import VolumeTests
from mur_base_selector.volume import Volume
import test_service as service
import test_volume_workflow as workflow
import test_workflow as transport
import unittest

class CommonPoseTests(VolumeTests):
    def test_off_grid_target_retains_exact_candidate_pose(self):
        v=Volume(self.volume());v.meta['generation']['voxel_size']=.1
        base=(1.9,0.,0.)
        rows=self.query(v,target=[2.52,.02,.8,0,0,0,1],base_poses=[base])
        self.assertEqual([r[0] for r in rows],[base])
        self.assertAlmostEqual(rows[0][2],np.arctan2(.02,.62))

    def test_outside_voxel_not_accepted(self):
        v=Volume(self.volume());v.meta['generation']['voxel_size']=.1
        self.assertEqual(self.query(v,target=[2.6,0,.8,0,0,0,1],base_poses=[(1.9,0.,0.)]),[])

    def test_required_arm_cannot_borrow_other_arm_sample(self):
        masks=np.ones((2,2,2),bool);masks[1]=False
        v=Volume(self.volume(masks));v.meta['generation']['voxel_size']=.1
        for mode in ('right','both'):
            self.assertEqual(self.query(v,mode=mode,base_poses=[(1.9,0.,0.)]),[])
        self.assertEqual(self.query(v,mode='either',base_poses=[(1.9,0.,0.)])[0][1],'left')

    def test_yaw_is_not_projected_away(self):
        v=Volume(self.volume());v.meta['generation']['voxel_size']=.1
        self.assertEqual(self.query(v,base_poses=[(1.9,0.,np.pi/2)]),[])

    def test_actual_fov_checked_for_offset_target(self):
        v=Volume(self.volume());v.meta['generation']['voxel_size']=.1
        self.assertEqual(self.query(v,target=[2.5,.04,.8,0,0,0,1],fov_deg=1,base_poses=[(1.9,0.,0.)]),[])

class MultiWorkflowTests(unittest.TestCase):
    def test_intersection_uses_same_poses_and_returns_each_arm(self):
        n=workflow.node(); calls=[]
        def query(*args,**kwargs):
            calls.append((args[8],kwargs.get('base_poses')))
            if len(calls)==1:
                return [((1.,0.,0.),'left',0.,1.,0.,0.),((2.,0.,0.),'left',0.,1.,0.,0.)]
            return [((1.,0.,0.),'right',0.,1.,0.,0.)]
        n.volume.query=query; done=service.Completion()
        n.run([transport.PoseStamped(),transport.PoseStamped()],['left','right'],done)
        self.assertTrue(done.value['success'],done.value['message'])
        r=done.value['result'];self.assertEqual(r['point_arms'],['left','right'])
        self.assertEqual(r['arm'],'mixed');self.assertEqual(r['pose'],[1.,0.,0.])
        self.assertEqual(calls[1],('right',[(1.,0.,0.),(2.,0.,0.)]))

    def test_empty_intersection_never_plans(self):
        n=workflow.node();calls=[]
        def query(*a,**k):
            calls.append(1)
            return [((1.,0.,0.),'left',0.,1.,0.,0.)] if len(calls)==1 else []
        n.volume.query=query;n.compute_path=lambda *a:self.fail('Must not plan')
        done=service.Completion();n.run([transport.PoseStamped()]*2,['left','both'],done)
        self.assertFalse(done.value['success']);self.assertFalse(n.pose_pub.messages)

    def test_service_defaults_and_count_validation(self):
        async def check():
            n=workflow.node();seen=[]
            n.submit=lambda *a:seen.append(a) or (False,'test')
            with patch.dict(workflow.m.__dict__,Future=asyncio.Future):
                req=NS(targets=[transport.PoseStamped()]*2,arms=[])
                await n.select_service(req,NS())
                self.assertEqual(seen[0][1],['any','any'])
                req.arms=['left'];response=await n.select_service(req,NS())
                self.assertFalse(response.success);self.assertEqual(len(seen),1)
        asyncio.run(check())

    def test_submit_validates_every_point_and_normalizes_any(self):
        n=workflow.node();n.busy=False;n.volume.arm_names=['left','right'];seen=[]
        n.run=lambda *a:seen.append(a)
        bad=transport.PoseStamped();bad.header.frame_id=''
        self.assertFalse(n.submit([transport.PoseStamped(),bad],['any','left'])[0])
        self.assertTrue(n.submit([transport.PoseStamped()]*2,['','right'])[0])
        n.worker.join(1);self.assertEqual(seen[0][1],['either','right'])

    def test_reselect_keeps_entire_workspace(self):
        n=workflow.node();targets=[transport.PoseStamped()]*2
        n.last_workspace=(targets,['left','right']);seen=[]
        n.submit=lambda *args:seen.append(args) or (True,'started')
        response=n.reselect(NS(),NS())
        self.assertTrue(response.success);self.assertEqual(len(seen[0][0]),2)
        self.assertEqual(seen[0][1],['left','right'])

    def test_mixed_service_reports_union_groups_and_per_point_arms(self):
        async def check():
            n=workflow.node();n.volume.arm_names=['left','right']
            n.volume.meta['context']={'groups':{'left':'L','right':'R'},'tips':{'left':'lt','right':'rt'}}
            def submit(target,mode,completion):
                completion.set_result(dict(success=True,message='ok',request_id=3,
                    pose=transport.PoseStamped(),result=dict(arm='mixed',point_arms=['right','both'],score=1.,valid_for_seconds=15.)))
                return True,'ok'
            n.submit=submit
            with patch.dict(workflow.m.__dict__,Future=asyncio.Future):
                result=await n.select_service(NS(targets=[transport.PoseStamped()]*2,arms=['right','both']),NS())
            self.assertEqual(result.point_arms,['right','both'])
            self.assertEqual(result.planning_groups,['R','L'])
            self.assertEqual(result.tcp_frames,['rt','lt'])
        asyncio.run(check())
