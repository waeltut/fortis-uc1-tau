"""Per-arm offline workers. Only the caller commits results and saves checkpoints."""
from concurrent.futures import ThreadPoolExecutor
from queue import Queue, Empty, Full
import threading
import signal
import time
import numpy as np


def generate_parallel(engines, points, quats, allowed, mount, cfg, masks, completed,
                      save, report, workers=2, stop=None):
    """One task per arm, with bounded result buffering and atomic voxel commits.

    engines must be independent native model/scene owners for workers=2. Native
    evaluate_offline releases the GIL. No worker mutates masks/completed or files.
    On interrupt/error, uncommitted voxels stay incomplete and can be recomputed.
    """
    if workers not in (1,2):raise ValueError('workers must be 1 or 2')
    if len(engines)!=2 or (workers==2 and engines[0] is engines[1]):
        raise ValueError('Two-worker mode requires independent arm engines')
    stop=stop if stop is not None else threading.Event()
    pending=[np.flatnonzero(~completed[a]).tolist() for a in range(2)]
    results=Queue(maxsize=4)

    def run_arm(arm):
        for i in pending[arm]:
            if stop.is_set():return
            ids=np.flatnonzero(allowed[i])
            values=[]
            if len(ids):
                poses=np.column_stack((np.repeat(points[i][None,:],len(ids),axis=0),quats[ids]))
                values=engines[arm].evaluate_offline(arm,poses.tolist(),mount,cfg['ik_timeout'],cfg['attempts'])
                if len(values)!=len(ids):raise ValueError('Native worker returned wrong orientation count')
            item=(arm,i,ids,np.asarray(values,dtype=bool))
            while not stop.is_set():
                try:results.put(item,timeout=.1);break
                except Full:continue

    pool=ThreadPoolExecutor(max_workers=workers,thread_name_prefix='mur_volume_arm')
    futures=[];processed=0;started=time.monotonic()
    try:
        futures=[pool.submit(run_arm,a) for a in range(2)]
        while True:
            if stop.is_set():raise InterruptedError('Volume generation interrupted')
            for f in futures:
                if f.done():f.result()  # propagate failures; never call failed voxels complete
            try:item=results.get(timeout=.1)
            except Empty:
                if all(f.done() for f in futures):break
                continue
            arm,i,ids,values=item
            # The checkpoint writer only sees fully committed voxels.
            masks[arm,i,ids]=values
            completed[arm,i]=True;processed+=1
            if processed%cfg['save_every_voxels']==0:
                save();report(completed,time.monotonic()-started)
    finally:
        stop.set()
        # Finish bounded native batches before engine teardown. Ignore repeated
        # Ctrl+C while joining/saving so Thread.join cannot be interrupted.
        handler=None
        if threading.current_thread() is threading.main_thread():
            handler=signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            pool.shutdown(wait=True,cancel_futures=True)
            save()
        finally:
            if handler is not None:signal.signal(signal.SIGINT,handler)
    report(completed,time.monotonic()-started)
