"""Offline MoveIt checks for a sampled dexterous workspace (resumable)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import yaml
from .volume import orientations,workspace_points,allowed_orientations,save_volume,Volume
from .volume_workers import generate_parallel


def validate(cfg):
    if cfg['inward_reference'] not in ('horizontal','spatial'):raise ValueError('inward_reference must be horizontal or spatial')
    for key in ('workspace_fov_deg','orientation_step_deg','voxel_size','ik_timeout','attempts','save_every_voxels'):
        if not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('Invalid '+key)
    if cfg['workspace_fov_deg']>360 or not 0<=cfg['min_inward_angle_deg']<=180:raise ValueError('Invalid angular constraint')
    if not np.isfinite(cfg['tool_axis']).all() or np.asarray(cfg['tool_axis']).shape!=(3,) or not np.isclose(np.linalg.norm(cfg['tool_axis']),1):raise ValueError('tool_axis must be a unit 3-vector')
    if not math.isfinite(cfg['workspace_front_offset_deg']):raise ValueError('Invalid front offset')
    for key in ('attempts','save_every_voxels'):
        if not isinstance(cfg[key],int):raise ValueError(key+' must be integer')
    if cfg['attempts']>100:raise ValueError('attempts must be <=100')
    if cfg['orientation_step_deg']<15:raise ValueError('Use orientation_step_deg >=15 for this foundation implementation')


def main(args=None):
    from ament_index_python.packages import get_package_share_directory
    package=Path(get_package_share_directory('mur_base_selector'))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(package/'config/volume_generation.yaml'))
    parser.add_argument('--engine-config',default=str(Path(get_package_share_directory('mur_reachability'))/'config/volume_engine.yaml'))
    parser.add_argument('--output',help='Override volume_file from generation config')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--workers',type=int,choices=(1,2),default=2,help='Offline arm workers (default: 2, one per arm); does not change cache identity')
    parser.add_argument('--use-sim-time',action='store_true')
    parser.add_argument('--estimate-only',action='store_true',help='Print sample count without ROS or MoveIt')
    opts=parser.parse_args(args)
    cfg=yaml.safe_load(Path(opts.config).expanduser().read_text());validate(cfg)
    output=Path(opts.output or cfg.pop('volume_file')).expanduser()
    cfg.pop('volume_file',None)
    points,quats=workspace_points(cfg),orientations(cfg['orientation_step_deg'])
    if len(points)>200000:raise ValueError('Too many voxels; narrow bounds or increase voxel_size')
    allowed=np.array([allowed_orientations(p,quats,cfg['tool_axis'],cfg['min_inward_angle_deg'],cfg['inward_reference']) for p in points])
    print(f'{len(points)} FOV-filtered voxels; {len(quats)} full orientations; {2*int(allowed.sum())} arm/pose checks',flush=True)
    print('IK occurs ONLY here, offline. A failed bounded solve is recorded as unknown/unavailable.',flush=True)
    if opts.estimate_only:return
    if output.exists() and not opts.resume:raise RuntimeError('Output already exists. Use --resume or choose a new output path.')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from .navigation import TfReceiver
    from mur_reachability_engine import Engine
    rclpy.init(args=[])
    node=Node('mur_volume_builder',parameter_overrides=[Parameter('use_sim_time',value=opts.use_sim_time)])
    tf=TfReceiver(node);engine=None;right_engine=None
    try:
        engine=Engine(str(Path(opts.engine_config).expanduser()),opts.use_sim_time)
        if not hasattr(engine,'model_info'):raise RuntimeError('Rebuild and source mur_reachability: self/body collision engine required')
        context=yaml.safe_load(engine.model_info())
        if context.get('reachability_policy')!='self_body_ignore_other_arm_v1':raise RuntimeError('Wrong native reachability policy')
        deadline=time.monotonic()+10
        while True:
            try:t=tf.buffer.lookup_transform(context['base_frame'],context['reference_frame'],Time()).transform;break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.05)
        mount=[t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
        meta={'schema':2,'reachability_policy':'self_body_ignore_other_arm_v1','generation':cfg,'context':context,'mount':mount,'evidence':'sampled endpoint IK, joint limits and SRDF self/body collision checks; opposite arm excluded'}
        signature=json.dumps(meta,sort_keys=True)
        meta['cache_id']=hashlib.sha256(signature.encode()).hexdigest()[:20]
        masks=np.zeros((2,len(points),len(quats)),dtype=bool);completed=np.zeros((2,len(points)),dtype=bool)
        if opts.resume:
            old=Volume(output,require_complete=False)
            # Resume depends on geometry and settings, never live arm positions.
            if old.meta['generation']!=cfg or old.meta['context']['model_fingerprint']!=context['model_fingerprint'] or not np.allclose(old.meta['mount'],mount,atol=1e-6,rtol=0):
                raise RuntimeError('Checkpoint model/configuration/mount mismatch')
            if not np.array_equal(old.points,points) or not np.allclose(old.quats,quats):raise RuntimeError('Checkpoint geometry mismatch')
            masks,completed=old.masks,old.done;meta=old.meta
        engines=[engine,engine]
        if opts.workers==2 and not completed.all():
            right_engine=Engine(str(Path(opts.engine_config).expanduser()),opts.use_sim_time,'mur_kinematics_right')
            if yaml.safe_load(right_engine.model_info())!=context:raise RuntimeError('Worker robot models differ')
            engines[1]=right_engine
        def save():save_volume(output,meta,points,quats,masks,completed)
        def report(done,elapsed):
            left,right=map(int,done.sum(axis=1))
            print(f'{left+right}/{done.size} arm/voxels complete; left={left}/{len(points)}, right={right}/{len(points)}; {elapsed:.0f} s this run',flush=True)
        print(f'Using {opts.workers} offline worker(s); independent arm solvers; self/body collision checks enabled; opposite arm excluded.',flush=True)
        if not completed.all():
            generate_parallel(engines,points,quats,allowed,mount,cfg,masks,completed,save,report,opts.workers)
        volume=Volume(output)
        for a,name in enumerate(('left','right')):
            print(f'{name}: {int((volume.coverage[a]>=.9).sum())} voxels with >=90% sampled allowed-orientation coverage',flush=True)
        print(f'Saved {output}; cache_id={meta["cache_id"]}',flush=True)
    finally:
        if right_engine:right_engine.close()
        if engine:engine.close()
        tf.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
