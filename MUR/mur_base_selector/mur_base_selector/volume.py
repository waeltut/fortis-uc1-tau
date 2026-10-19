"""ROS-free sampled orientation volumes and shared 2-D travel-cost field.

A positive entry is evidence at a sampled pose, not a continuous IK certificate.
Orientation evidence is loaded from the offline cache; queries perform no IK.
"""
import heapq
import json
import math
import os
import time
from pathlib import Path
import numpy as np
from .core import angle, transform_points


def qmul(a, b):
    a, b = np.asarray(a), np.asarray(b)
    av, bv = a[..., :3], b[..., :3]
    return np.concatenate((a[..., 3, None]*bv+b[..., 3, None]*av+np.cross(av,bv),
                           (a[..., 3]*b[..., 3]-np.sum(av*bv,axis=-1))[...,None]),axis=-1)


def qrotate(q, v):
    q, v = np.asarray(q), np.asarray(v)
    t = 2*np.cross(q[..., :3], v)
    return v+q[..., 3,None]*t+np.cross(q[..., :3],t)


def orientations(step_deg):
    if not math.isfinite(step_deg) or not 5 <= step_deg <= 90 or abs(180/step_deg-round(180/step_deg))>1e-8:
        raise ValueError('orientation_step_deg must divide 180 and be in [5,90]')
    result=[]
    for yaw in np.arange(-180,180,step_deg):
        for pitch in np.arange(-90,90+step_deg/2,step_deg):
            for roll in np.arange(-180,180,step_deg):
                r,p,y=np.radians([roll,pitch,yaw])/2
                q=qmul(qmul([0,0,math.sin(y),math.cos(y)],[0,math.sin(p),0,math.cos(p)]),[math.sin(r),0,0,math.cos(r)])
                if not result or np.max(np.abs(np.asarray(result)@q))<1-1e-9:result.append(q)
    return np.asarray(result)


def allowed_orientations(point, quats, tool_axis, min_inward_deg, inward_reference="horizontal"):
    # 70 degrees from the inward ray allows outward and sideways/downward tools.
    p=np.asarray(point,dtype=float).copy()
    if inward_reference=="horizontal":p[2]=0.
    norm=np.linalg.norm(p)
    if norm<1e-9:return np.zeros(len(quats),dtype=bool)
    directions=qrotate(quats,np.asarray(tool_axis,dtype=float))
    return directions@(-p/norm)<=math.cos(math.radians(min_inward_deg))+1e-12


def workspace_points(cfg):
    lo,hi=np.asarray(cfg['bounds_min'],float),np.asarray(cfg['bounds_max'],float)
    res=float(cfg['voxel_size'])
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or (hi<=lo).any() or res<=0 or not math.isfinite(res):
        raise ValueError('Invalid volume bounds/resolution')
    axes=[np.arange(a+res/2,b,res) for a,b in zip(lo,hi)]
    p=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    bearing=np.arctan2(p[:,1],p[:,0])-math.radians(cfg['workspace_front_offset_deg'])
    bearing=np.arctan2(np.sin(bearing),np.cos(bearing))
    return p[np.abs(bearing)<=math.radians(cfg['workspace_fov_deg']/2)+1e-12]


def save_volume(path, metadata, points, quats, masks, completed):
    path=Path(path).expanduser();path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.tmp')
    with tmp.open('wb') as f:
        np.savez_compressed(f,metadata=json.dumps(metadata,sort_keys=True),points=points,quaternions=quats,
                            masks=np.packbits(masks,axis=-1,bitorder='little'),completed=completed)
        f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)


class IncompleteVolumeError(ValueError):
    """The cache is a valid checkpoint but generation has not finished."""


class Volume:
    def __init__(self,path,require_complete=True):
        with np.load(Path(path).expanduser(),allow_pickle=False) as f:
            self.meta=json.loads(str(f['metadata']));self.points=f['points'];self.quats=f['quaternions']
            self.done=f['completed'].astype(bool)
            self.masks=np.unpackbits(f['masks'],axis=-1,count=len(self.quats),bitorder='little').astype(bool)
        n,k=len(self.points),len(self.quats)
        if self.meta.get('schema')!=2 or self.meta.get('reachability_policy')!='self_body_ignore_other_arm_v1':
            raise ValueError('Incompatible cache policy: regenerate a fresh self/body collision cache; previous-policy checkpoints cannot be resumed')
        if self.points.shape!=(n,3) or self.quats.shape!=(k,4) or self.masks.shape!=(2,n,k) or self.done.shape!=(2,n):
            raise ValueError('Invalid volume schema/shapes')
        if not n or not k or not np.isfinite(self.points).all() or not np.isfinite(self.quats).all() or not np.allclose(np.linalg.norm(self.quats,axis=1),1):
            raise ValueError('Invalid volume geometry')
        if require_complete and not self.done.all():raise IncompleteVolumeError('Volume incomplete: resume generation first')
        cfg=self.meta['generation']
        self.allowed=np.array([allowed_orientations(p,self.quats,cfg['tool_axis'],cfg['min_inward_angle_deg'],cfg.get('inward_reference','horizontal')) for p in self.points])
        if (self.masks & ~self.allowed[None,:,:]).any():raise ValueError('Cache has disallowed orientation entries')
        self.coverage=self.masks.sum(axis=-1)/np.maximum(1,self.allowed.sum(axis=-1))[None,:]
        base=float(cfg['min_inward_angle_deg'])
        self.profile_cache={base:{'angle':base,'allowed':self.allowed,'coverage':self.coverage,'counts':self.allowed.sum(axis=-1)}}
        self.profile_angles=[base]

    def profile(self,angle_deg):
        cfg=self.meta['generation'];base=float(cfg['min_inward_angle_deg']);value=float(angle_deg)
        if not math.isfinite(value) or not base<=value<180:
            raise ValueError(f'Profile angle must be >= cached {base:g} and <180 degrees')
        if value not in self.profile_cache:
            allowed=np.array([allowed_orientations(p,self.quats,cfg['tool_axis'],value,cfg.get('inward_reference','horizontal')) for p in self.points])
            # Never infer untested orientations, even with floating-point boundary effects.
            allowed &= self.allowed
            counts=allowed.sum(axis=-1)
            coverage=(self.masks & allowed[None,:,:]).sum(axis=-1)/np.maximum(1,counts)[None,:]
            self.profile_cache[value]={'angle':value,'allowed':allowed,'coverage':coverage,'counts':counts}
        return self.profile_cache[value]

    def configure_profiles(self,angles):
        if not angles:raise ValueError('At least one orientation profile is required')
        values=sorted(set([float(self.meta['generation']['min_inward_angle_deg'])]+[float(a) for a in angles]))
        for a in values:self.profile(a)
        self.profile_angles=values


    def query(self,target,base_z,yaw_bins,min_coverage,max_height_error,max_angle_deg,fov_deg,front_offset_deg,mode,checkpoint=lambda:None,include_profiles=False):
        cfg=self.meta['generation']
        # A larger online FOV cannot recover voxels omitted at generation.
        if fov_deg>cfg['workspace_fov_deg']+1e-9 or abs(angle(math.radians(front_offset_deg-cfg['workspace_front_offset_deg'])))>1e-9:
            raise ValueError('Online FOV must fit cached FOV and use the same front offset; regenerate to expand/rotate')
        p=np.asarray(target[:3]);q=np.asarray(target[3:]);q=q/np.linalg.norm(q)
        height=np.flatnonzero(np.abs(self.points[:,2]-(p[2]-base_z))<=max_height_error+1e-12)
        if not len(height):return []
        # Use only the nearest sampled height layer, without merging optimistic layers.
        dz=np.abs(self.points[height,2]-(p[2]-base_z));nearest_z=self.points[height[int(np.argmin(dz))],2]
        height=height[np.abs(self.points[height,2]-nearest_z)<1e-9]
        rows=[]
        for b in range(yaw_bins):
            checkpoint();yaw=2*math.pi*b/yaw_bins
            relative=qmul([0,0,-math.sin(yaw/2),math.cos(yaw/2)],q)
            dots=np.abs(self.quats@relative);oi=int(np.argmax(dots));error=2*math.acos(min(1.,dots[oi]))
            if error>math.radians(max_angle_deg)+1e-12:continue
            points=self.points[height];bearings=np.arctan2(points[:,1],points[:,0])-math.radians(front_offset_deg)
            bearings=np.arctan2(np.sin(bearings),np.cos(bearings))
            good=np.abs(bearings)<=math.radians(fov_deg/2)+1e-12
            # Actual requested axis must satisfy the constraint too, not only its bin centre.
            dirs=qrotate(relative,np.asarray(cfg['tool_axis']))
            inward=points.copy()
            if cfg.get('inward_reference','horizontal')=='horizontal':inward[:,2]=0.
            norms=np.linalg.norm(inward,axis=1)
            good &= (norms>1e-9)
            inward_dot=np.sum((-inward/np.maximum(norms[:,None],1e-12))*dirs,axis=1)
            valid=np.zeros((2,len(height)),dtype=bool)
            selected_angles=np.zeros((2,len(height)),dtype=float)
            selected_coverage=np.zeros((2,len(height)),dtype=float)
            selected_counts=np.zeros((2,len(height)),dtype=int)
            # Broadest passing profile wins, but acceptance is the union of all profiles.
            for profile_angle in self.profile_angles:
                profile=self.profile_cache[profile_angle]
                direction_ok=inward_dot<=math.cos(math.radians(profile_angle))+1e-12
                sampled_ok=profile['allowed'][height,oi] & (profile['counts'][height]>0)
                eligible=(self.masks[:,height,oi] & self.done[:,height] &
                          (profile['coverage'][:,height]>=min_coverage) &
                          (good & direction_ok & sampled_ok)[None,:])
                take=eligible & ~valid
                selected_angles[take]=profile_angle
                selected_coverage[take]=profile['coverage'][:,height][take]
                selected_counts[take]=np.broadcast_to(profile['counts'][height],take.shape)[take]
                valid |= eligible
            cs,sn=math.cos(yaw),math.sin(yaw)
            bx=p[0]-(cs*points[:,0]-sn*points[:,1]);by=p[1]-(sn*points[:,0]+cs*points[:,1])
            for j,idx in enumerate(height):
                arms=['both'] if mode=='both' and valid[:,j].all() else []
                if mode!='both':arms=[name for a,name in enumerate(('left','right')) if valid[a,j] and mode in ('either',name)]
                for arm in arms:
                    arm_ids=(0,1) if arm=='both' else (int(arm=='right'),)
                    coverage=float(min(selected_coverage[a,j] for a in arm_ids))
                    row=((float(bx[j]),float(by[j]),yaw),arm,float(bearings[j]),coverage,float(abs(points[j,2]-(p[2]-base_z))),error)
                    if include_profiles:
                        evidence={('left','right')[a]:{'min_inward_angle_deg':float(selected_angles[a,j]),'coverage':float(selected_coverage[a,j]),'allowed_samples':int(selected_counts[a,j])} for a in arm_ids}
                        row+=(evidence,)
                    rows.append(row)
        return rows


def sector_penalty(bearing,low_deg,high_deg):
    low,high=math.radians(low_deg),math.radians(high_deg)
    if low<=bearing<=high:return 0.
    return (min(abs(angle(bearing-low)),abs(angle(bearing-high)))/(math.pi/2))**2


def working_score(distance,bearing,arm,cfg):
    pref=cfg[arm+'_preferred_distance'];scale=cfg['distance_scale']
    return cfg['weight_working_distance']*((distance-pref)/scale)**2+cfg['weight_facing']*sector_penalty(bearing,cfg[arm+'_facing_min_deg'],cfg[arm+'_facing_max_deg'])


class TravelField:
    """One Dijkstra expansion for all destinations; point-robot ranking only.

    Only settled distances are exposed when the computation budget expires.
    Final footprint/path validation is required for every selected destination.
    """
    def __init__(self,grid,start_xy,cost_weight=3.,allow_unknown=False,deadline=math.inf,checkpoint=lambda:None):
        self.grid=grid;self.dist=np.full((grid.height,grid.width),np.inf);self.settled=np.zeros_like(self.dist,dtype=bool)
        sx,sy=self.cells(np.asarray([start_xy]))[0];self.truncated=False
        if not (0<=sx<grid.width and 0<=sy<grid.height):raise ValueError('Robot outside global costmap')
        blocked=(grid.data==254)|((grid.data==255)&(not allow_unknown))
        if blocked[sy,sx]:raise ValueError('Robot centre in blocked global costmap cell')
        self.dist[sy,sx]=0.;heap=[(0.,int(sx),int(sy))];count=0
        while heap:
            if count%256==0:
                checkpoint()
                if time.monotonic()>=deadline:self.truncated=True;break
            d,x,y=heapq.heappop(heap);count+=1
            if self.settled[y,x]:continue
            self.settled[y,x]=True
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nx,ny=x+dx,y+dy
                if not(0<=nx<grid.width and 0<=ny<grid.height) or blocked[ny,nx]:continue
                if dx and dy and (blocked[y,nx] or blocked[ny,x]):continue
                c=(min(int(grid.data[y,x]),253)+min(int(grid.data[ny,nx]),253))/(2*253)
                nd=d+grid.resolution*math.hypot(dx,dy)*(1+cost_weight*c)
                if nd<self.dist[ny,nx]:self.dist[ny,nx]=nd;heapq.heappush(heap,(nd,nx,ny))
        self.dist[~self.settled]=np.inf

    def cells(self,xy):
        g=self.grid;x,y,yaw=g.origin
        return np.floor(transform_points(np.asarray(xy)-[x,y],(0,0,-yaw))/g.resolution).astype(int)

    def lookup(self,xy):
        cells=self.cells(xy);out=np.full(len(cells),np.inf)
        good=(cells[:,0]>=0)&(cells[:,1]>=0)&(cells[:,0]<self.grid.width)&(cells[:,1]<self.grid.height)
        out[good]=self.dist[cells[good,1],cells[good,0]]
        return out
