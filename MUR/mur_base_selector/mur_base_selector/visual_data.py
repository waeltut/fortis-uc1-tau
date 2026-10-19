"""ROS-free display filtering; never changes selection or cache contents."""
import math
import numpy as np


def bounded_indices(size, limit):
    if size <= limit:return np.arange(size,dtype=int)
    return np.linspace(0,size-1,limit,dtype=int)


def voxel_layers(volume, minimum, z_min, z_max, arm, limit, profile_angle=None):
    if not 0<=minimum<=1 or z_min>z_max or limit<1:raise ValueError('Invalid voxel display limits')
    if arm not in ('both','left','right','intersection'):raise ValueError('arm must be both, left, right or intersection')
    height=(volume.points[:,2]>=z_min)&(volume.points[:,2]<=z_max)
    names=('intersection',) if arm=='intersection' else (('left','right') if arm=='both' else (arm,))
    coverage_array=volume.coverage if profile_angle is None else volume.profile(profile_angle)['coverage']
    layers=[]
    for name in names:
        a=0 if name=='left' else 1
        coverage=coverage_array.min(axis=0) if name=='intersection' else coverage_array[a]
        done=volume.done.all(axis=0) if name=='intersection' else volume.done[a]
        ids=np.flatnonzero(height&done&(coverage>=minimum)&(coverage>0))
        count=len(ids);ids=ids[bounded_indices(count,limit)]
        layers.append((name,volume.points[ids],coverage[ids],count))
    return layers


def candidate_snapshot(rows, limit):
    """Counts are exact; samples distributed across arm and target-bearing sectors."""
    groups={};counts={'total':len(rows),'forward_30deg':0,'left':0,'right':0,'both':0}
    for row in rows:
        pose,arm,bearing,*_=row
        counts[arm]+=1
        if abs(bearing)<=math.pi/6+1e-9:counts['forward_30deg']+=1
        sector=int((bearing+math.pi)/(math.pi/6))%12
        groups.setdefault((arm,sector),[]).append([*map(float,pose),arm,float(bearing)])
    # Round robin preserves a sparse forward sector even if sideways poses dominate.
    chosen=[];keys=sorted(groups);positions={k:0 for k in keys}
    quota=max(1,math.ceil(limit/max(1,len(keys))))
    buckets={k:[groups[k][i] for i in bounded_indices(len(groups[k]),quota)] for k in keys}
    while len(chosen)<limit:
        progressed=False
        for k in keys:
            if positions[k]<len(buckets[k]):
                chosen.append(buckets[k][positions[k]]);positions[k]+=1;progressed=True
                if len(chosen)>=limit:break
        if not progressed:break
    profile_counts={}
    for row in rows:
        if len(row)>6:
            for arm,evidence in row[6].items():
                key=f"{arm}:{evidence['min_inward_angle_deg']:g}"
                profile_counts[key]=profile_counts.get(key,0)+1
    return {'profile_counts':profile_counts,'counts':counts,'displayed':len(chosen),'sampled':len(chosen)<len(rows),'poses':chosen}
