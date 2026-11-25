"""Exercise the actual volume selection workflow with ROS transport doubles."""
import ast
import json
from pathlib import Path
import sys
import types
from types import SimpleNamespace as NS
import unittest
import numpy as np
import test_workflow as transport
from mur_base_selector.core import Grid
sys.modules['mur_base_selector.navigation']=transport.m
path=Path(__file__).parents[1]/'mur_base_selector'/'volume_node.py'
tree=ast.parse(path.read_text())
blocked=('rclpy','rcl_interfaces','std_msgs','nav_msgs','visualization_msgs','mur_base_selector_interfaces')
tree.body=[n for n in tree.body if not (isinstance(n,ast.ImportFrom) and n.module and n.module.split('.')[0] in blocked) and not (isinstance(n,ast.Import) and any(a.name in blocked for a in n.names))]
m=types.ModuleType('volume_workflow');m.__package__='mur_base_selector';sys.modules[m.__name__]=m
m.__dict__.update(rclpy=NS(ok=lambda:True),String=transport.Message,Bool=transport.Message,NavPath=transport.Message,SetParametersResult=transport.Message)
exec(compile(tree,str(path),'exec'),m.__dict__)


def node(rows=None):
    n=transport.node();n.__class__=m.VolumeSelector;n.cfg.update(m.DEFAULTS);n.cfg.update(left_preferred_distance=.65,right_preferred_distance=.65,left_facing_min_deg=0.,left_facing_max_deg=60.,right_facing_min_deg=-60.,right_facing_max_deg=0.);n.cfg['target_feasible_paths']=3;n.cfg['max_path_checks']=8
    n.ik_client=NS(service_is_ready=lambda:False)
    n.cfg['check_nav2_paths']=True
    n.publish_target=lambda:None;n.check_model=lambda *a:None;n.cache_context=lambda:None
    n.lookup=lambda *a:NS(transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    grid=Grid(np.zeros((50,50)),50,50,.2,(-5,-5,0))
    n.costmaps=lambda *a:[(grid,(0,0,0))]
    if rows is None:rows=[((1.+i*.2,0.,0.),'left',0.,1.,0.,0.) for i in range(4)]
    n.volume=NS(meta={'cache_id':'test'},query=lambda *a:rows)
    # No Engine exists. An accidental refresh/verify call fails this workflow.
    return n


def result(n):return json.loads(n.result_pub.messages[-1].data)


class VolumeWorkflowTests(unittest.TestCase):
    def test_mount_check_needs_no_parked_or_live_joint_data(self):
        n=node();n.volume.meta={'context':{'reference_frame':'mur'},'mount':[0,0,0,0,0,0,1]}
        n.joints={}
        m.VolumeSelector.cache_context(n)
        n.joints={'arbitrary_joint':(2.5,0)}
        m.VolumeSelector.cache_context(n)

    def test_actual_mount_change_is_rejected(self):
        n=node();n.volume.meta={'context':{'reference_frame':'mur'},'mount':[.1,0,0,0,0,0,1]}
        with self.assertRaisesRegex(Exception,'mount differs'):m.VolumeSelector.cache_context(n)

    def test_visual_stages_publish_raw_travel_and_shortlist_counts(self):
        n=node();n.visualization_pub=transport.Publisher();n.visualization_snapshot={}
        n.run(transport.PoseStamped())
        messages=[json.loads(m.data) for m in n.visualization_pub.messages]
        self.assertEqual([d['stage'] for d in messages],['checking','cache','travel','shortlist'])
        self.assertEqual(messages[-1]['cache']['counts']['total'],4)
        self.assertEqual(messages[-1]['shortlist']['counts']['forward_30deg'],4)
        self.assertTrue(result(n)['valid'])

    def test_no_path_mode_skips_planner_and_rechecks_destination(self):
        n=node()
        n.cfg['check_nav2_paths']=False
        n.planner=None
        n.compute_path=n.check_path=lambda *a:self.fail('Unexpected planner call')
        checks=[]
        n.polygon_check=lambda *a:(checks.append(True) or True,'',.2)
        n.run(transport.PoseStamped())
        r=result(n)
        self.assertTrue(r['requires_navigation'])
        self.assertFalse(r['path_checked'])
        self.assertIsNone(r['path_length_m'])
        self.assertGreater(len(checks),1)

    def test_live_ik_assignment_requires_every_requested_arm(self):
        from unittest.mock import patch
        from time import monotonic
        n=node()
        n.volume.arm_names=['left','right']
        n.volume.meta['context']={'reference_frame':'mur',
                                  'groups':{'left':'lg','right':'rg'},
                                  'tips':{'left':'lt','right':'rt'}}
        calls=[]
        client=NS(service_is_ready=lambda:True)
        def call(req):
            ik=req.ik_request
            calls.append(ik.group_name)
            self.assertTrue(ik.avoid_collisions)
            self.assertEqual(ik.robot_state.joint_state.name,['joint'])
            return ik.group_name
        client.call_async=call
        n.ik_client=client
        n.verify_measured_state=lambda *_:False
        def request():
            return NS(ik_request=NS(robot_state=NS(joint_state=NS())))
        srv=NS(GetPositionIK=NS(Request=request))
        duration=NS(Duration=lambda seconds:NS(to_msg=lambda:seconds))
        with patch.dict(sys.modules,{'moveit_msgs.srv':srv,'rclpy.duration':duration}):
            for modes, answers, expected in [
                (['either','right'],[False,True,True],['right','right']),
                (['both'],[True,False],None),
                (['both'],[True,True],['both']),
                (['left','right'],[True,False],None)]:
                codes=iter(answers)
                n.wait=lambda *_:NS(error_code=NS(val=1 if next(codes) else -31))
                stats={'online_ik_checks':0}
                got=n.current_pose_arms([transport.PoseStamped()]*len(modes),modes,
                                        {'joint':.3},monotonic()+1.,stats)
                self.assertEqual(got,expected)
                self.assertEqual(stats['online_ik_checks'],len(answers))

    def test_fk_witness_requires_matching_pose_and_valid_state(self):
        from unittest.mock import patch
        from time import monotonic
        from copy import deepcopy
        n = node()
        target = transport.PoseStamped()
        actual = deepcopy(target)
        ik = NS(pose_stamped=target, robot_state=object(), ik_link_name='tip')
        n.fk_client = NS(service_is_ready=lambda:True, call_async=lambda req:NS(
            error_code=NS(val=1), pose_stamped=[actual], fk_link_names=req.fk_link_names))
        valid = NS(valid=False)
        n.validity_client = NS(service_is_ready=lambda:True, call_async=lambda req:valid)
        n.wait = lambda response, remaining:response
        services = NS(GetPositionFK=NS(Request=NS), GetStateValidity=NS(Request=NS))
        with patch.dict(sys.modules,{'moveit_msgs.srv':services}):
            self.assertFalse(n.verify_measured_state(ik, monotonic()+1, {}))
            valid.valid = True
            self.assertTrue(n.verify_measured_state(ik, monotonic()+1, {}))
            actual.pose.position.x += .002
            self.assertFalse(n.verify_measured_state(ik, monotonic()+1, {}))

    def test_overlap_reports_padding_and_rotated_costmap_frame(self):
        n = node()
        grid = Grid(np.array([0,254,254,0]),4,1,1.,(0.,0.,0.))
        # tf rotates fixed-frame +X to costmap +Y and translates to (1,0).
        source = np.array([[.1,-.8],[.7,-.8],[.7,-.1],[.1,-.1]])
        checked = source.copy()
        checked[[0,1],1] = -1.2
        result = n.footprint_evidence(source,checked,(0.,0.,0.),[(grid,(1.,0.,np.pi/2))])
        layer = result['global']
        self.assertEqual(layer['lethal_touching_published'],1)
        self.assertEqual(layer['lethal_only_in_padding'],1)
        self.assertEqual(layer['published_cell_centres_xy'],[[.5,-.5]])
        self.assertEqual(layer['padding_cell_centres_xy'],[[.5,-1.5]])

    def test_stationary_global_margin_requires_local_clearance(self):
        from test_service import Completion
        source = np.array([[.051,.051],[.09,.051],[.09,.09],[.051,.09]])
        checked = np.array([[.031,.031],[.11,.031],[.11,.11],[.031,.11]])
        for global_hit, local_hit, local_width, should_stay in [
                ((2,1),None,4,True), ((1,1),None,4,False),
                ((2,1),(2,1),4,False), ((2,1),None,2,False)]:
            n=node([])
            n.robot_pose=lambda:(0.,0.,0.)
            n.current_pose_arms=lambda *args:['left']
            n.footprint_snapshot=lambda with_source=False:(source,checked) if with_source else checked
            maps=[]
            for cell,width in [(global_hit,4),(local_hit,local_width)]:
                data=np.zeros((4,width),dtype=np.uint8)
                if cell:data[cell[1],cell[0]]=254
                maps.append((Grid(data,width,4,.05), (0.,0.,0.)))
            n.costmaps=lambda *args:maps
            n.polygon_check=lambda poly,m:transport.m.BaseSelector.polygon_check(n,poly,m)
            done=Completion()
            n.run(transport.PoseStamped(),'left',done)
            self.assertEqual(done.value['success'],should_stay,done.value['message'])
            if should_stay:
                self.assertFalse(done.value['result']['requires_navigation'])
                self.assertTrue(done.value['result']['current_pose_check']['global_margin_only'])

    def test_footprint_rejection_is_preserved_in_failed_response(self):
        from test_service import Completion
        n=node([])
        n.current_pose_arms=lambda *args:['left']
        n.polygon_check=lambda *args:(False,'global_lethal_obstacle',0.)
        done=Completion()
        n.run(transport.PoseStamped(),'left',done)
        trace=done.value['diagnostics']['current_pose_check']
        self.assertEqual(trace['footprint'],'global_lethal_obstacle')
        self.assertEqual(trace['outcome'],'footprint_rejected')
        self.assertFalse(done.value['success'])

    def test_current_stance_preempts_volume_and_path_search(self):
        n=node()
        n.robot_pose=lambda:(.6, .4, .3)
        n.current_pose_arms=lambda *args:['left','both']
        n.volume.query=lambda *a:self.fail('Unexpected volume search')
        n.planner=None
        n.run([transport.PoseStamped()]*2,['either','both'])
        r=result(n)
        self.assertFalse(r['requires_navigation'])
        self.assertEqual(r['point_arms'],['left','both'])
        self.assertEqual(r['pose'],[.6,.4,.3])
        self.assertTrue(r['exact_target_ik_verified'])

    def test_zero_ik_and_multiple_paths(self):
        n=node();n.run(transport.PoseStamped());r=result(n)
        self.assertTrue(r['valid']);self.assertEqual(r['online_ik_checks'],0);self.assertEqual(r['path_checks'],3)
        self.assertEqual(r['joint_solutions'],[]);self.assertFalse(r['exact_target_ik_verified'])

    def test_later_better_path_wins(self):
        n=node();n.cfg['weight_working_distance']=0.;n.cfg['diversity_distance']=.01;costs=iter([.9,.5,0.,0.]);n.check_path=lambda *a:(True,'',next(costs))
        n.run(transport.PoseStamped());self.assertAlmostEqual(result(n)['pose'][0],1.4)

    def test_blocked_destination_never_plans(self):
        n=node();n.polygon_check=lambda *a:(False,'obstacle',0.)
        n.compute_path=lambda *a: self.fail('Blocked destination reached planner')
        n.run(transport.PoseStamped());self.assertFalse(n.pose_pub.messages)

    def test_cache_mount_failure_stops_before_lookup(self):
        n=node();n.cache_context=lambda:(_ for _ in ()).throw(ValueError('mount mismatch'))
        n.volume.query=lambda *a:self.fail('Lookup must not occur')
        n.run(transport.PoseStamped());self.assertFalse(n.pose_pub.messages)

    def test_fresh_map_failure_invalidates(self):
        n=node();n.cfg['target_feasible_paths']=1;calls=[]
        def polygon(*args):
            calls.append(1);return (True,'',0.) if len(calls)==1 else (False,'obstacle',0.)
        n.polygon_check=polygon;n.run(transport.PoseStamped());self.assertFalse(n.pose_pub.messages)

    def test_same_base_arm_preferences_avoid_duplicate_nav_plans(self):
        rows=[((1,0,0),'left',-.5,1.,0.,0.),((1,0,0),'right',-.5,1.,0.,0.)]
        n=node(rows);n.run(transport.PoseStamped());self.assertEqual(result(n)['arm'],'right');self.assertEqual(result(n)['path_checks'],1)

    def test_no_candidates_stops(self):
        n=node([]);n.run(transport.PoseStamped());self.assertFalse(n.pose_pub.messages)

    def test_parameter_guard_does_not_reject_initial_declarations(self):
        n=node();n.initializing_volume=True
        self.assertTrue(n.parameters_changed([NS(name='volume_file')]).successful)
        n.initializing_volume=False
        self.assertFalse(n.parameters_changed([NS(name='volume_file')]).successful)


if __name__=='__main__':unittest.main()

class StartupMainTests(unittest.TestCase):
    def test_missing_cache_returns_zero_and_shuts_down(self):
        from unittest.mock import patch
        events=[]
        ros=NS(init=lambda **kw:events.append('init'),ok=lambda:True,shutdown=lambda:events.append('shutdown'))
        executor=NS(shutdown=lambda **kw:events.append('executor_shutdown'))
        logger=NS(warning=lambda msg:events.append(msg),error=lambda msg:events.append(msg))
        def constructor():raise m.CacheStartupError('cache missing')
        with patch.dict(m.__dict__,rclpy=ros,SingleThreadedExecutor=lambda:executor,VolumeSelector=constructor,get_logger=lambda name:logger):
            self.assertEqual(m.main(),0)
        self.assertEqual(events,['init','cache missing','shutdown','executor_shutdown'])

    def test_invalid_cache_returns_one_without_traceback(self):
        from unittest.mock import patch
        messages=[]
        ros=NS(init=lambda **kw:None,ok=lambda:True,shutdown=lambda:None)
        executor=NS(shutdown=lambda **kw:None)
        logger=NS(warning=messages.append,error=messages.append)
        def constructor():raise m.CacheStartupError('bad cache',exit_code=1)
        with patch.dict(m.__dict__,rclpy=ros,SingleThreadedExecutor=lambda:executor,VolumeSelector=constructor,get_logger=lambda name:logger):
            self.assertEqual(m.main(),1)
        self.assertEqual(messages,['bad cache'])
