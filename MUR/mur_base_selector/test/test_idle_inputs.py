"""Activity gating, fresh-input warm-up and stale callback rejection without ROS."""
import ast
import copy
from pathlib import Path
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import test_workflow as transport

m=transport.m

class Buffer:
    def __init__(self): self.dynamic={}; self.static={}; self.clears=0
    def clear(self): self.dynamic.clear(); self.static.clear(); self.clears+=1
    def set_transform(self,t,authority): self.dynamic[t.child_frame_id]=t
    def set_transform_static(self,t,authority): self.static[t.child_frame_id]=t

class Subscriptions:
    def __init__(self): self.live=[]; self.removed=[]
    def create_subscription(self,kind,topic,callback,qos):
        sub=NS(topic=topic,callback=callback,qos=qos)
        self.live.append(sub);return sub
    def destroy_subscription(self,sub):self.live.remove(sub);self.removed.append(sub)

class IdleTests(unittest.TestCase):
    def setUp(self):
        self.env=patch.dict(m.__dict__,PolygonStamped=object,JointState=object,LaserScan=object,TFMessage=object,
                           QoSProfile=lambda **kw:NS(**kw),ReliabilityPolicy=NS(BEST_EFFORT=1,RELIABLE=2))
        self.env.start();self.addCleanup(self.env.stop)

    def receiver(self):
        r=m.TfReceiver.__new__(m.TfReceiver)
        r.node=Subscriptions();r.buffer=Buffer();r.lock=threading.RLock()
        r.static={};r.epoch=0;r.dynamic_subscription=None
        return r

    def node(self):
        n=transport.node()
        for name in ('activate_inputs','release_idle_inputs','wait_for_inputs','check_scan'):
            delattr(n,name)
        n.busy=False;n.tf_receiver=self.receiver();n.footprint=None;n.joints={};n.scan_seen=None
        n.initialize_inputs();n.subs=Subscriptions()
        n.create_subscription=n.subs.create_subscription;n.destroy_subscription=n.subs.destroy_subscription
        return n

    def test_idle_subscriptions_are_absent_and_activation_is_idempotent(self):
        n=self.node();self.assertEqual(n.subs.live,[])
        self.assertIsNone(n.tf_receiver.dynamic_subscription)
        n.activate_inputs();n.activate_inputs()
        self.assertEqual(len(n.subs.live),3)
        self.assertEqual([s.topic for s in n.tf_receiver.node.live],['/tf'])
        n.release_idle_inputs()
        self.assertEqual(n.subs.live,[]);self.assertIsNone(n.tf_receiver.dynamic_subscription)
        self.assertFalse(n.inputs_active)

    def test_busy_and_valid_result_retain_inputs(self):
        n=self.node();n.activate_inputs();n.busy=True;n.release_idle_inputs()
        self.assertTrue(n.inputs_active)
        n.busy=False;n.selected_context=((),{});n.release_idle_inputs()
        self.assertTrue(n.inputs_active)
        n.selected_context=None;n.release_idle_inputs();self.assertFalse(n.inputs_active)

    def test_callbacks_queued_before_pause_cannot_seed_new_request(self):
        n=self.node();n.activate_inputs()
        old_joint=next(s.callback for s in n.subs.live if s.topic==n.cfg['joint_states_topic'])
        msg=NS(name=['left_elbow_joint','unrelated'],position=[1.,2.])
        old_joint(msg);self.assertEqual(set(n.joints),{'left_elbow_joint'})
        n.pause_inputs();n.activate_inputs();old_joint(msg)
        self.assertEqual(n.joints,{})
        new_joint=next(s.callback for s in n.subs.live if s.topic==n.cfg['joint_states_topic'])
        new_joint(msg);self.assertEqual(n.joints['left_elbow_joint'][0],1.)

    def test_tf_pause_retains_static_only_and_rejects_old_epoch(self):
        r=self.receiver();static=NS(child_frame_id='mount');dynamic=NS(child_frame_id='base')
        r.on_static(NS(transforms=[static]));r.activate()
        old=r.dynamic_subscription.callback;old(NS(transforms=[dynamic]))
        self.assertIn('base',r.buffer.dynamic)
        r.pause();self.assertEqual(r.buffer.dynamic,{})
        self.assertIn('mount',r.buffer.static)
        r.activate();old(NS(transforms=[dynamic]));self.assertEqual(r.buffer.dynamic,{})
        r.dynamic_subscription.callback(NS(transforms=[dynamic]));self.assertIn('base',r.buffer.dynamic)

    def test_new_request_waits_until_all_inputs_are_available(self):
        n=self.node();n.cfg['input_wait_timeout']=.2;checks=[]
        def scan():
            checks.append('scan')
            if checks.count('scan')==1:raise m.SelectionError('waiting')
        n.check_scan=scan
        n.joint_snapshot=lambda:checks.append('joints')
        n.robot_pose=lambda:checks.append('tf')
        n.footprint_snapshot=lambda:checks.append('footprint')
        n.wait_for_inputs(time.monotonic()+1.)
        self.assertEqual(checks,['scan','scan','joints','tf','footprint'])

    def test_missing_input_times_out_and_cancel_propagates(self):
        n=self.node();n.cfg['input_wait_timeout']=.025
        with self.assertRaisesRegex(m.SelectionError,'warm-up.*scan'):
            n.wait_for_inputs(time.monotonic()+1.)
        n.stop.set()
        with self.assertRaises(m.Cancelled):n.wait_for_inputs(time.monotonic()+1.)

    def test_valid_result_stale_sensor_invalidates_and_pauses(self):
        n=self.node();n.activate_inputs();n.selected_context=((0,0,0),{})
        n.selection_deadline=time.monotonic()+10
        n.expire()  # scan is missing
        self.assertIsNone(n.selected_context);self.assertFalse(n.inputs_active)
        self.assertFalse(n.valid_pub.messages[-1].data)

    def test_expiry_and_cancel_release_subscriptions(self):
        for cancel in (False,True):
            n=self.node();n.activate_inputs();n.selected_context=((0,0,0),{})
            n.selection_deadline=time.monotonic()-1
            if cancel:n.cancel(NS(),NS())
            n.expire();self.assertFalse(n.inputs_active)

    def test_subscription_failure_rolls_back(self):
        n=self.node();count=[];original=n.create_subscription
        def create(*a):
            count.append(1)
            if len(count)==2:raise RuntimeError('DDS failure')
            return original(*a)
        n.create_subscription=create
        with self.assertRaisesRegex(RuntimeError,'DDS'):n.activate_inputs()
        self.assertEqual(n.subs.live,[]);self.assertFalse(n.inputs_active)

    def test_no_legacy_selector_topic_or_reselect_service(self):
        root=Path(__file__).parents[1]/'mur_base_selector'
        text=(root/'navigation.py').read_text()+(root/'volume_node.py').read_text()
        self.assertNotIn("'/base_selection/target'",text)
        self.assertNotIn("'/base_selection/selected_pose'",text)
        self.assertNotIn("'/base_selection/reselect'",text)
        self.assertNotIn('self.pose_pub.publish',text)
        self.assertNotIn('create_timer(1.0,self.publish_target)',text)

    def test_warmup_failure_never_calls_lookup_or_planner(self):
        import test_volume_workflow as workflow
        from test_service import Completion
        n=workflow.node()
        def missing(deadline):raise m.SelectionError('Fresh-input warm-up failed: no footprint')
        n.wait_for_inputs=missing
        n.volume.query=lambda *a:self.fail('Must not query reachability')
        n.compute_path=lambda *a:self.fail('Must not plan')
        done=Completion();n.run(transport.PoseStamped(),'left',done)
        self.assertFalse(done.value['success']);self.assertIn('warm-up',done.value['message'])
        self.assertFalse(n.busy);self.assertIsNone(n.selected_context)

    def test_valid_result_checks_motion_and_all_live_streams(self):
        n=self.node();n.activate_inputs();n.selected_context=((0,0,0),{})
        n.selection_deadline=time.monotonic()+10;seen=[]
        n.guard_motion=lambda *args:seen.append('motion')
        n.check_scan=lambda:seen.append('scan')
        n.footprint_snapshot=lambda:seen.append('footprint')
        n.expire()
        self.assertEqual(seen,['motion','scan','footprint']);self.assertTrue(n.inputs_active)
