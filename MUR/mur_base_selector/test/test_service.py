"""Service/workflow contracts with transport doubles; DDS tested separately."""
import asyncio
import json
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import test_workflow as transport
import test_volume_workflow as workflow


class Completion:
    def __init__(self): self.value=None; self.calls=0
    def set_result(self,value): self.value=value; self.calls+=1


class ServiceTests(unittest.TestCase):
    def test_success_completion_matches_topic_pose_and_result(self):
        n=workflow.node();done=Completion()
        n.run(transport.PoseStamped(),'left',done)
        self.assertEqual(done.calls,1)
        self.assertTrue(done.value['success'])
        self.assertEqual(done.value['result'],workflow.result(n))
        self.assertEqual(done.value['pose'].pose.position.x,done.value['result']['pose'][0])
        self.assertFalse(n.pose_pub.messages)  # No legacy selected_pose publication
        self.assertFalse(n.busy)

    def test_failed_query_never_returns_previous_success(self):
        n=workflow.node();first=Completion();second=Completion()
        n.run(transport.PoseStamped(),'left',first)
        n.volume.query=lambda *a:[]
        n.run(transport.PoseStamped(),'left',second)
        self.assertTrue(first.value['success']);self.assertFalse(second.value['success'])
        self.assertNotIn('pose',second.value)
        self.assertIn('No volume candidates',second.value['message'])

    def test_cancelled_query_completes_with_failure(self):
        n=workflow.node();done=Completion();n.stop.set()
        n.run(transport.PoseStamped(),'left',done)
        self.assertFalse(done.value['success']);self.assertIn('cancelled',done.value['message'])
        self.assertFalse(n.busy);self.assertFalse(n.pose_pub.messages)

    def test_request_arm_is_local_to_request(self):
        n=workflow.node();seen=[]
        n.volume.query=lambda *args:seen.append(args[8]) or []
        n.run(transport.PoseStamped(),'tool_arm',Completion())
        self.assertEqual(seen,['tool_arm']);self.assertEqual(n.cfg['arm_mode'],'either')

    def test_submit_unknown_arm_and_busy_do_not_start_worker(self):
        n=workflow.node();n.volume.arm_names=['tool_arm']
        self.assertIn('busy',n.submit(transport.PoseStamped(),'tool_arm')[1])
        n.busy=False
        self.assertIn('Unknown arm',n.submit(transport.PoseStamped(),'left')[1])
        target=transport.PoseStamped();target.pose.orientation.w=0.
        self.assertIn('quaternion',n.submit(target,'tool_arm')[1])
        self.assertFalse(n.busy)

    def test_submit_threads_capture_separate_mode_and_completion(self):
        n=workflow.node();n.busy=False;n.volume.arm_names=['tool_arm'];received=[]
        done=Completion()
        n.run=lambda *args: received.append(args)
        ok,_=n.submit(transport.PoseStamped(),'tool_arm',done)
        self.assertTrue(ok);n.worker.join(1)
        self.assertEqual(received[0][1],'tool_arm');self.assertIs(received[0][2],done)

    def test_callback_returns_group_and_tcp(self):
        async def scenario():
            n=workflow.node()
            n.volume.arm_names=['tool_arm'];n.volume.meta['context']={'groups':{'tool_arm':'moveit_arm'},'tips':{'tool_arm':'tcp'}}
            def submit(target,mode,completion):
                completion.set_result(dict(success=True,message='ok',request_id=42,pose=target,
                    result=dict(arm='tool_arm',score=1.2,valid_for_seconds=15.)))
                return True,'started'
            n.submit=submit
            with patch.dict(workflow.m.__dict__,Future=asyncio.Future):
                response=await n.select_service(NS(target=transport.PoseStamped(),arm='tool_arm'),NS())
            self.assertTrue(response.success);self.assertEqual(response.request_id,42)
            self.assertEqual(response.planning_groups,['moveit_arm']);self.assertEqual(response.tcp_frames,['tcp'])
        asyncio.run(scenario())

    def test_rejected_callback_returns_without_waiting(self):
        async def scenario():
            n=workflow.node();n.submit=lambda *args:(False,'Selector busy')
            with patch.dict(workflow.m.__dict__,Future=asyncio.Future):
                response=await n.select_service(NS(target=transport.PoseStamped(),arm=''),NS())
            self.assertFalse(response.success);self.assertEqual(response.message,'Selector busy')
        asyncio.run(scenario())
