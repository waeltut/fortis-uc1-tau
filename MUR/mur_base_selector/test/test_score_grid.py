import math
import unittest
from mur_base_selector.score_grid import ScoreRaster, components, heading_matches
from mur_base_selector.volume import working_score


def row(x,y=0,yaw=0,arm='left'):
    return ((x,y,yaw),arm,0.,1.,0.,0.)


def record(pose,total,arm='left',state='estimated',travel=None):
    scores=None if total is None else dict(travel=total if travel is None else travel,
                                          destination=0.,distance=0.,facing=0.,total=total)
    return dict(pose=pose,arm=arm,state=state,scores=scores)


class ScoreGridTests(unittest.TestCase):
    def test_same_total_winner_for_all_components(self):
        r=ScoreRaster([row(.01),row(.02)],.1,100)
        a=record((.01,0,0),5.,travel=0.)
        b=record((.02,0,1),4.,arm='right',travel=3.)
        layers,winners,_=r.render([a,b],10.)
        self.assertEqual(winners[0]['arm'],'right')
        self.assertEqual(layers['total'][0],40)
        self.assertEqual(layers['travel'][0],30)

    def test_row_major_negative_origin_and_unknown_holes(self):
        r=ScoreRaster([row(-.2,-.1),row(0,.1)],.1,100)
        layers,winners,_=r.render([record((-.2,-.1,0),1.),record((0,.1,0),2.)],10.)
        self.assertEqual((r.width,r.height),(3,3))
        self.assertEqual(list(winners),[0,8])
        self.assertEqual(layers['total'].tolist(),[10,-1,-1,-1,-1,-1,-1,-1,20])

    def test_exact_decimal_boundary_does_not_overflow(self):
        for x in (.3,-.3,0.6,-.6):
            r=ScoreRaster([row(x,x)],.1,10)
            self.assertEqual(r.index((x,x,0)),0)

    def test_zero_score_valid_clipping_reported(self):
        r=ScoreRaster([row(0),row(1)],.1,100)
        layers,_,ranges=r.render([record((0,0,0),0.),record((1,0,0),12.)],10.)
        self.assertEqual(layers['total'][0],0);self.assertEqual(layers['total'][-1],100)
        self.assertEqual(ranges['total']['clipped_cells'],1)

    def test_detailed_unknown_is_not_rejection(self):
        r=ScoreRaster([row(0),row(1),row(2)],1.,10)
        records=[record((0,0,0),None,state='destination_obstacle'),
                 record((1,0,0),None,state='path_obstacle'),record((1,0,1),None,state='unchecked'),
                 record((2,0,0),None,state='unchecked'),record((2,0,1),2.,state='path_checked')]
        layers,_,_=r.render(records,10.)
        self.assertEqual(layers['state'].tolist(),[100,50,0])
        self.assertEqual(layers['total'].tolist(),[-1,-1,20])

    def test_no_estimate_never_means_unreachable(self):
        r=ScoreRaster([row(0)],.1,100)
        layers,_,_=r.render([record((0,0,0),None,state='no_estimate')],10.)
        self.assertEqual(layers['state'][0],50)

    def test_fixed_scale_comparison(self):
        r=ScoreRaster([row(0)],.1,100)
        first,_,_=r.render([record((0,0,0),2.)],10.)
        second,_,_=r.render([record((0,0,0),4.)],10.)
        self.assertEqual(second['total'][0],2*first['total'][0])

    def test_heading_wrap_and_filter_off(self):
        self.assertTrue(heading_matches(math.radians(359),True,-1.,.01))
        self.assertFalse(heading_matches(math.pi,True,0.,7.5))
        self.assertTrue(heading_matches(math.pi,False,0.,7.5))

    def test_memory_cap_and_empty_snapshot(self):
        with self.assertRaises(ValueError):ScoreRaster([row(0),row(100)],.01,100)
        r=ScoreRaster([],.1,100)
        self.assertEqual(r.render([],10.)[0]['total'].tolist(),[-1])

    def test_components_equal_selector_score(self):
        cfg=dict(weight_navigation=.2,length_scale=5.,weight_destination_cost=2.,
                 weight_working_distance=.8,distance_scale=.25,weight_facing=.3,
                 left_preferred_distance=.75,left_facing_min_deg=0.,left_facing_max_deg=60.)
        pose=(1.,2.,0.);target=(2.,2.,1.);bearing=-.5
        values=components(pose,'left',bearing,target,3.,.2,cfg)
        expected=.2*3./5.+2.*.2+working_score(1.,bearing,'left',cfg)
        self.assertAlmostEqual(values['total'],expected)
