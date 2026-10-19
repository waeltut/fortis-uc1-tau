"""Exercise actual marker construction with message/transport doubles, not ROS."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
from mur_base_selector.visual_data import voxel_layers

class Marker:
    DELETEALL=3;ADD=0;CUBE_LIST=6;LINE_LIST=5;TEXT_VIEW_FACING=9
    def __init__(self,**kw):
        self.header=NS(frame_id='');self.pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(w=0.))
        self.scale=NS(x=0.,y=0.,z=0.);self.color=NS(a=0.);self.points=[];self.colors=[]
        self.__dict__.update(kw)

class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)

path=Path(__file__).parents[1]/'mur_base_selector/volume_visualizer.py'
tree=ast.parse(path.read_text());tree.body=[n for n in tree.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
ns=dict(Node=object,Marker=Marker,MarkerArray=NS,Point=NS,ColorRGBA=NS,np=np,math=math,json=json,voxel_layers=voxel_layers)
exec(compile(tree,str(path),'exec'),ns)

class VisualizerTests(unittest.TestCase):
    def node(self):
        n=object.__new__(ns['VolumeVisualizer']);n.frame='base_link';n.last_settings=None
        n.volume=NS(meta={'generation':{'voxel_size':.1,'min_inward_angle_deg':70.}},points=np.array([[.5,0,.8]]),coverage=np.array([[.95],[1.]]),done=np.ones((2,1),bool))
        n.volume.profile=lambda angle:{'coverage':n.volume.coverage}
        n.settings=dict(minimum_coverage=.9,z_min=-10.,z_max=10.,arm='both',max_voxels_per_arm=10,alpha=.45,voxel_scale=.85,show_voxels=True,profile_angle_deg=-1.0)
        n.get_parameter=lambda k:NS(value=n.settings[k]);n.voxel_pubs={k:Publisher() for k in ns['COLOURS']}
        n.label_pub=Publisher();n.diag_pub=Publisher();n.candidate_pubs={k:Publisher() for k in ('cache','travel','shortlist')}
        return n
    def test_cubes_follow_base_and_arm_toggle_clears_old_markers(self):
        n=self.node();n.voxels();m=n.voxel_pubs['left'].messages[-1].markers[1]
        self.assertTrue(m.frame_locked);self.assertEqual(m.header.frame_id,'base_link');self.assertEqual(len(m.points),1)
        self.assertAlmostEqual(m.scale.x,.085)
        n.settings['arm']='right';n.voxels();self.assertEqual(len(n.voxel_pubs['left'].messages[-1].markers),1)
    def test_snapshot_colours_and_new_request_clear_stages(self):
        n=self.node();d={'frame':'map','target':[1,2,.8],'request_id':1,'stage':'cache','cache':{'counts':{'total':1,'forward_30deg':1},'displayed':1,'poses':[[1.,2.,0.,'left',0.]]}}
        n.candidates(NS(data=json.dumps(d)));m=n.candidate_pubs['cache'].messages[-1].markers[1]
        self.assertEqual(len(m.points),6);self.assertEqual(len(m.colors),6);self.assertEqual(m.colors[0].g,1.)
        d.pop('cache');d['stage']='checking';d['request_id']=2;n.candidates(NS(data=json.dumps(d)))
        self.assertEqual(len(n.candidate_pubs['cache'].messages[-1].markers),1)
