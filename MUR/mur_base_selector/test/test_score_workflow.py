"""Diagnostic workflow with transport doubles; no live ROS/Nav2 validation."""
import ast
import copy
import json
from pathlib import Path
import sys
import types
from types import SimpleNamespace as NS
import unittest
import test_workflow as transport
import test_volume_workflow as selector

path=Path(__file__).parents[1]/'mur_base_selector/score_visualizer.py'
tree=ast.parse(path.read_text())
blocked=('rclpy','rcl_interfaces','geometry_msgs','sensor_msgs','std_msgs','std_srvs',
         'nav_msgs','nav2_msgs','lifecycle_msgs','visualization_msgs')
tree.body=[n for n in tree.body if not (
    isinstance(n,ast.ImportFrom) and n.module and (n.module.split('.')[0] in blocked or n.module=='volume_node'))
    and not (isinstance(n,ast.Import) and any(a.name in blocked for a in n.names))]
m=types.ModuleType('score_workflow');m.__package__='mur_base_selector';sys.modules[m.__name__]=m
m.__dict__.update(Node=object,VolumeSelector=selector.m.VolumeSelector,VOLUME_DEFAULTS=selector.m.DEFAULTS,
                 rclpy=NS(ok=lambda:True),String=transport.Message)
exec(compile(tree,str(path),'exec'),m.__dict__)


def node(rows=None):
    n=selector.node(rows);n.__class__=m.ScoreVisualizer
    n.diag=copy.deepcopy(m.DIAGNOSTICS);n.volume.arm_names=['left','right']
    n.metadata_pub=transport.Publisher();n.clears=[];n.rasters=[]
    n.clear_outputs=lambda stamp:n.clears.append(stamp)
    def publish(mode,raster,records,stamp):
        layers,winners,ranges=raster.render(records,n.diag['score_max'])
        n.rasters.append((mode,copy.deepcopy(records),layers,stamp))
        return dict(scored_cells=len(winners),ranges=ranges)
    n.publish_raster=publish
    return n


class DiagnosticWorkflowTests(unittest.TestCase):
    def test_fast_uses_all_candidates_and_never_plans_or_selects(self):
        n=node()
        n.compute_path=lambda *a:self.fail('fast diagnostic must not request Nav2 paths')
        n.calculate(transport.PoseStamped())
        metadata=json.loads(n.metadata_pub.messages[-1].data)
        self.assertEqual(metadata['state'],'complete')
        self.assertEqual(metadata['candidate_count'],4)
        self.assertFalse(metadata['valid_for_navigation'])
        self.assertEqual(len(n.rasters[0][1]),4)
        self.assertFalse(n.pose_pub.messages);self.assertFalse(n.result_pub.messages)

    def test_detailed_budget_leaves_unchecked_candidates_unknown(self):
        n=node();n.diag.update(mode='detailed',max_detailed_candidates=1)
        n.calculate(transport.PoseStamped())
        metadata=json.loads(n.metadata_pub.messages[-1].data)
        self.assertTrue(metadata['detailed_partial']);self.assertEqual(metadata['path_requests'],1)
        detailed=n.rasters[-1][1]
        self.assertEqual(sum(r['state']=='unchecked' for r in detailed),3)
        self.assertEqual(sum(r['state']=='path_checked' for r in detailed),1)
        self.assertFalse(n.pose_pub.messages)

    def test_destination_failure_does_not_request_path(self):
        n=node();n.diag['mode']='detailed'
        n.polygon_check=lambda *a:(False,'obstacle',0.)
        n.compute_path=lambda *a:self.fail('blocked destination must not plan')
        n.calculate(transport.PoseStamped())
        self.assertTrue(all(r['scores'] is None for r in n.rasters[-1][1]))
        self.assertTrue(all(r['state']=='destination_obstacle' for r in n.rasters[-1][1]))

    def test_planner_timeout_retains_unknown_and_marks_partial(self):
        n=node();n.diag['mode']='detailed'
        def timeout(*a):raise transport.m.SelectionError('Request timed out')
        n.compute_path=timeout
        n.calculate(transport.PoseStamped())
        metadata=json.loads(n.metadata_pub.messages[-1].data)
        self.assertTrue(metadata['detailed_partial'])
        self.assertTrue(all(r['state']=='unchecked' for r in n.rasters[-1][1]))

    def test_cancel_clears_outputs_and_releases_busy(self):
        n=node();n.stop.set();n.calculate(transport.PoseStamped())
        self.assertEqual(json.loads(n.metadata_pub.messages[-1].data)['state'],'failed')
        self.assertFalse(n.busy);self.assertEqual(len(n.clears),2)

    def test_empty_query_replaces_old_map(self):
        n=node([]);n.calculate(transport.PoseStamped())
        metadata=json.loads(n.metadata_pub.messages[-1].data)
        self.assertEqual(metadata['state'],'complete');self.assertEqual(metadata['candidate_count'],0)
        self.assertEqual(n.rasters[-1][2]['total'].tolist(),[-1])

    def test_target_only_stored_without_computation(self):
        n=node();n.last_target=None
        n.calculate=lambda *a:self.fail('target callback must never calculate')
        target=transport.PoseStamped();target.pose.position.x=3.
        n.on_target(target);target.pose.position.x=4.
        self.assertEqual(n.last_target.pose.position.x,3.)

    def test_constructor_does_not_call_selector_constructors(self):
        method=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ScoreVisualizer')
        init=next(n for n in method.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        source=ast.unparse(init)
        self.assertIn('Node.__init__',source)
        self.assertNotIn('super()',source)
        for name in ('selected_pose','selected_valid','SelectBase','/base_selection/result'):
            self.assertNotIn(name,source)
