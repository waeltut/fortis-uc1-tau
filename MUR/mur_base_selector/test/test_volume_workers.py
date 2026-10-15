import threading
import time
import unittest
import numpy as np
from mur_base_selector.volume_workers import generate_parallel


class Engine:
    def __init__(self,barrier=None,fail=False):self.barrier=barrier;self.fail=fail;self.calls=[];self.active=False
    def evaluate_offline(self,arm,poses,*args):
        self.active=True
        try:
            if self.barrier and not self.calls:self.barrier.wait(timeout=2)
            self.calls.append((arm,poses))
            if self.fail:raise RuntimeError('solver error')
            return [arm==0]*len(poses)
        finally:self.active=False


class WorkerTests(unittest.TestCase):
    def run_build(self,engines,workers=2,done=None,save=None,stop=None):
        points=np.array([[.1*i,0.,.8] for i in range(12)])
        quats=np.array([[0.,0.,0.,1.],[0.,0.,1.,0.]])
        allowed=np.ones((12,2),bool);masks=np.zeros((2,12,2),bool)
        completed=np.zeros((2,12),bool) if done is None else done
        saved=[]
        def checkpoint():
            saved.append((masks.copy(),completed.copy(),threading.get_ident()))
            if save:save()
        generate_parallel(engines,points,quats,allowed,[0,0,0,0,0,0,1],
            {'ik_timeout':.01,'attempts':3,'save_every_voxels':3},masks,completed,checkpoint,
            lambda *a:None,workers,stop)
        return masks,completed,saved

    def test_both_arm_threads_overlap_and_only_main_saves(self):
        barrier=threading.Barrier(2);engines=[Engine(barrier),Engine(barrier)]
        masks,done,saved=self.run_build(engines)
        self.assertTrue(done.all());self.assertTrue(masks[0].all());self.assertFalse(masks[1].any())
        self.assertEqual([len(e.calls) for e in engines],[12,12])
        self.assertTrue(all(t==threading.get_ident() for _,_,t in saved))
        for mask,completed,_ in saved:
            self.assertTrue(np.all(mask[0,completed[0]]))
            self.assertFalse(mask[0,~completed[0]].any())

    def test_resume_skips_completed_voxels(self):
        engines=[Engine(),Engine()];done=np.zeros((2,12),bool);done[0,:7]=True;done[1,:3]=True
        self.run_build(engines,done=done)
        self.assertEqual([len(e.calls) for e in engines],[5,9])

    def test_one_worker_can_use_shared_engine(self):
        engine=Engine();masks,done,_=self.run_build([engine,engine],workers=1)
        self.assertTrue(done.all());self.assertEqual(len(engine.calls),24)
        self.assertTrue(masks[0].all());self.assertFalse(masks[1].any())

    def test_parallel_shared_engine_rejected(self):
        engine=Engine()
        with self.assertRaises(ValueError):self.run_build([engine,engine])

    def test_solver_failure_saves_and_joins_other_worker(self):
        engines=[Engine(fail=True),Engine()];saved=[]
        with self.assertRaisesRegex(RuntimeError,'solver error'):
            self.run_build(engines,save=lambda:saved.append(True))
        self.assertTrue(saved);self.assertFalse(any(e.active for e in engines))

    def test_interrupt_saves_and_joins(self):
        engines=[Engine(),Engine()];calls=[]
        def save():
            calls.append(True)
            if len(calls)==1:raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):self.run_build(engines,save=save)
        self.assertGreaterEqual(len(calls),2);self.assertFalse(any(e.active for e in engines))

    def test_external_stop_preserves_partial_checkpoint(self):
        stop=threading.Event();stop.set();saved=[]
        with self.assertRaises(InterruptedError):self.run_build([Engine(),Engine()],stop=stop,save=lambda:saved.append(True))
        self.assertTrue(saved)

    def test_wrong_native_result_length_not_committed(self):
        bad=Engine();bad.evaluate_offline=lambda *a:[]
        with self.assertRaisesRegex(ValueError,'orientation count'):self.run_build([bad,Engine()])


if __name__=='__main__':unittest.main()
