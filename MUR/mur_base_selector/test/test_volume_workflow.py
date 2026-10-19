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
blocked=('rclpy','rcl_interfaces','std_msgs','nav_msgs','visualization_msgs')
tree.body=[n for n in tree.body if not (isinstance(n,ast.ImportFrom) and n.module and n.module.split('.')[0] in blocked) and not (isinstance(n,ast.Import) and any(a.name in blocked for a in n.names))]
m=types.ModuleType('volume_workflow');m.__package__='mur_base_selector';sys.modules[m.__name__]=m
m.__dict__.update(rclpy=NS(ok=lambda:True),String=transport.Message,Bool=transport.Message,NavPath=transport.Message,SetParametersResult=transport.Message)
exec(compile(tree,str(path),'exec'),m.__dict__)


def node(rows=None):
    n=transport.node();n.__class__=m.VolumeSelector;n.cfg.update(m.DEFAULTS);n.cfg['target_feasible_paths']=3;n.cfg['max_path_checks']=8
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
