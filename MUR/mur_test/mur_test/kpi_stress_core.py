"""Pure Python benchmark construction/analysis; safe to import without ROS 2.

All distances are in metres; all rotations in radians internally. No robot commands.
"""
from __future__ import annotations

import csv
import hashlib
import os
import html
import json
import math
import random
import statistics
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

VERSION = '1.1.0'
PROFILES = {
    'pilot': {'repeat': 2, 'offsets': (.03, .10), 'directions': 4,
              'angles': (10,), 'axes': ('z',), 'multi_repetitions': 1,
              'multi_offsets': (.04, .10), 'coupled': 1, 'negative': 2},
    'standard': {'repeat': 3, 'offsets': (.025, .05, .10, .15, .25), 'directions': 8,
                 'angles': (5, 15, 30), 'axes': ('y', 'z'), 'multi_repetitions': 3,
                 'multi_offsets': (.03, .08, .15), 'coupled': 3, 'negative': 4},
    'extensive': {'repeat': 5, 'offsets': (.02, .05, .10, .15, .20, .30), 'directions': 16,
                  'angles': (5, 10, 20, 30, 45), 'axes': ('x', 'y', 'z'), 'multi_repetitions': 6,
                  'multi_offsets': (.025, .05, .10, .20), 'coupled': 6, 'negative': 6},
}


def unit_quaternion(q):
    q = tuple(float(x) for x in q)
    norm = math.sqrt(sum(x*x for x in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid quaternion')
    return tuple(x/norm for x in q)


def qmul(a, b):
    x,y,z,w = a
    X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def qinv(a):
    x,y,z,w = unit_quaternion(a)
    return (-x,-y,-z,w)


def qrotate(q, v):
    x,y,z,w = unit_quaternion(q)
    # Euler-Rodrigues vector formula; faster than three quaternion multiplies.
    vx,vy,vz = v
    tx = 2*(y*vz-z*vy); ty = 2*(z*vx-x*vz); tz = 2*(x*vy-y*vx)
    return (vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz),
            vz+w*tz+(x*ty-y*tx))


def axis_angle(axis, angle):
    vec = {'x': (1.,0.,0.), 'y': (0.,1.,0.), 'z': (0.,0.,1.)}[axis]
    return (*[v*math.sin(angle/2.) for v in vec], math.cos(angle/2.))


def compose(a, b):
    # (translation (3), quaternion x,y,z,w (4)) maps child -> parent.
    t,qa = a; s,qb = b
    r = qrotate(qa,s)
    return ((t[0]+r[0], t[1]+r[1], t[2]+r[2]), unit_quaternion(qmul(qa,qb)))


def invert(pose):
    t,q = pose
    iq = qinv(q)
    v = qrotate(iq,(-t[0],-t[1],-t[2]))
    return (v,iq)


def pose_to_tuple(pose):
    obj = pose['pose'] if 'pose' in pose else pose
    p,q=obj['position'],obj['orientation']
    return ((float(p['x']),float(p['y']),float(p['z'])),
            unit_quaternion([q['x'],q['y'],q['z'],q['w']]))


def tuple_to_pose(tq, frame='map'):
    p,q=tq
    return {'header': {'frame_id': frame}, 'pose': {
        'position': {'x':p[0], 'y':p[1], 'z':p[2]},
        'orientation': {'x':q[0], 'y':q[1], 'z':q[2], 'w':q[3]}}}


def yaw_of(q):
    x,y,z,w = unit_quaternion(q)
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def angle_difference(a,b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def _point(pose, arm):
    p,q=pose_to_tuple(pose)
    return dict(x=p[0],y=p[1],z=p[2],qx=q[0],qy=q[1],qz=q[2],qw=q[3],arm=arm,frame='map')


def _request(points):
    targets=[]
    for p in points:
        targets.append(tuple_to_pose(((p['x'],p['y'],p['z']),
                                     (p['qx'],p['qy'],p['qz'],p['qw'])),p['frame']))
    if len(points) == 1:
        return {'target':targets[0], 'arm':points[0]['arm']}
    return {'targets':targets, 'arms':[p['arm'] for p in points]}


def _moved(pose, delta=(0,0,0), axis=None, angle_deg=0):
    p,q=pose_to_tuple(pose)
    nq=unit_quaternion(qmul(q,axis_angle(axis,math.radians(angle_deg)))) if axis else q
    return tuple_to_pose(((p[0]+delta[0],p[1]+delta[1],p[2]+delta[2]),nq))


def build_cases(reference, profile='standard', seed=2026, center=None, orientation=None, max_yaw_deg=5., offset_scale=1.):
    """Build a deterministic, stratified benchmark anchored to observed TCP TF poses.

    Reference poses are positive controls. Perturbations are *unknown* feasibility,
    not ground-truth positive samples. Negative controls are high targets only.
    """
    if profile not in PROFILES:
        raise ValueError('Unknown profile: %s' % profile)
    for arm in ('left','right'):
        pose_to_tuple(reference[arm])  # Validate before generating any request.
    c=PROFILES[profile]
    rng=random.Random(seed)
    cases=[]
    def add(name, family, poses, constraints, offset=0., rotation=0.,
            control='unknown', repeat_group='', extra=None):
        assert len(poses)==len(constraints) and poses
        pts=[_point(p,a) for p,a in zip(poses,constraints)]
        case=dict(id=len(cases)+1, scenario=name, family=family,
                  control=control, expected_feasible=True if control=='positive' else
                  False if control=='negative' else None,
                  repeat_group=repeat_group, npoints=len(pts),
                  arm_constraint=','.join(constraints), offset_m=float(offset),
                  rotation_deg=float(rotation), targets=pts, request=_request(pts))
        if extra: case.update(extra)
        cases.append(case)

    # Repeated identical positive controls: results can drift as costmaps update.
    for n in range(c['repeat']):
        add('reference_left','reference', [reference['left']],['left'],
            control='positive', repeat_group='ref_left')
        add('reference_right','reference',[reference['right']],['right'],
            control='positive', repeat_group='ref_right')
        add('reference_left_any','reference',[reference['left']],['any'],
            control='positive', repeat_group='ref_left_any')
        add('reference_right_any','reference',[reference['right']],['any'],
            control='positive', repeat_group='ref_right_any')
        add('reference_both_points','reference',
            [reference['left'],reference['right']],['left','right'],
            control='positive', repeat_group='ref_both')

    if not math.isfinite(max_yaw_deg) or not 0 <= max_yaw_deg <= 180:
        raise ValueError('max_yaw_deg must be between 0 and 180')
    if not math.isfinite(offset_scale) or offset_scale <= 0:
        raise ValueError('offset_scale must be finite and positive')
    if center is not None and (len(center) != 3 or not all(map(math.isfinite, center))):
        raise ValueError('center must contain three finite map coordinates')
    anchors = {}
    for arm in ('left', 'right'):
        xyz, q = pose_to_tuple(reference[arm])
        anchors[arm] = tuple_to_pose((xyz if center is None else center,
                                      q if orientation is None else unit_quaternion(orientation)))
    reference = anchors
    c = dict(c, offsets=[r * offset_scale for r in c['offsets']],
             multi_offsets=[r * offset_scale for r in c['multi_offsets']])

    def rotated(pose, delta, axis, degrees):
        # Z means room heading (map Z). For tilted TCPs, local X/Y can also
        # change heading. Shrink those rotations if they exceed the yaw limit.
        position, q = pose_to_tuple(_moved(pose, delta))
        if axis == 'z':
            degrees = math.copysign(min(abs(degrees), max_yaw_deg), degrees)
            out = qmul(axis_angle('z', math.radians(degrees)), q)
        else:
            while True:
                out = qmul(q, axis_angle(axis, math.radians(degrees)))
                if math.degrees(angle_difference(yaw_of(out), yaw_of(q))) <= max_yaw_deg + 1e-9:
                    break
                degrees = degrees / 2 if abs(degrees) > 1e-6 else 0.
        return tuple_to_pose((position, unit_quaternion(out))), abs(degrees)

    # XYZ workspace slices. Directions lie in 3D with varied Z, not a fixed height.
    for arm in ('left','right'):
        ref=reference[arm]
        for radius in c['offsets']:
            for i in range(c['directions']):
                theta=2*math.pi*(i+.5*(arm=='right'))/c['directions']
                z_fraction=(-.55,0.,.55)[i%3]
                xy=math.sqrt(1-z_fraction*z_fraction)
                disp=(radius*xy*math.cos(theta),radius*xy*math.sin(theta),radius*z_fraction)
                target=_moved(ref,disp)
                add('xyz_%s_%03dmm' % (arm,round(radius*1000)),
                    'position', [target],[arm],offset=radius)
                if i%2==0 and profile!='pilot':
                    add('xyz_any_%03dmm' % round(radius*1000),
                        'position_any',[target],['any'],offset=radius,
                        extra={'anchor_arm':arm})

    # Orientation changes applied in TCP-local coordinates around a known joint pose.
    for arm in ('left','right'):
        for angle in c['angles']:
            for axis in c['axes']:
                for sign in (-1,1):
                    degrees = (angle / max(c['angles']) * min(max(c['angles']), max_yaw_deg)
                               if axis == 'z' else angle)
                    target, actual = rotated(reference[arm], (0, 0, 0), axis, sign * degrees)
                    add('rotation_%s_%s_%gdeg'%(arm,axis,actual),
                        'orientation',[target],
                        [arm],rotation=actual,extra={'rotation_axis':axis,
                                                         'rotation_sign':sign})

    # Single-arm waypoint clusters and constrained dual-arm clusters.
    multi_sizes=[2,3] if profile=='pilot' else [2,3,4] if profile=='standard' else [2,3,4,5]
    for arm in ('left','right'):
        for spread in c['multi_offsets']:
            for rep in range(c['multi_repetitions']):
                count=multi_sizes[rep%len(multi_sizes)]
                poses=[reference[arm]]
                for j in range(1,count):
                    theta=2*math.pi*j/max(2,count-1)+rng.uniform(-.15,.15)
                    r=spread*(.65+.35*rng.random())
                    poses.append(_moved(reference[arm],(r*math.cos(theta),
                                                       r*math.sin(theta),r*rng.uniform(-.3,.3))))
                add('multi_%s_%dpt_%03dmm'%(arm,count,round(spread*1000)),
                    'multi_same_arm',poses,[arm]*count,offset=spread)

    for spread in c['multi_offsets']:
        for rep in range(c['multi_repetitions']):
            count=[2,4,6][rep%3]
            poses=[]; constraints=[]
            for j in range(count):
                arm='left' if j%2==0 else 'right'
                if j<2: pose=reference[arm]  # Anchored pair initially reachable.
                else:
                    theta=2*math.pi*(j+rep)/count
                    pose=_moved(reference[arm],(spread*math.cos(theta),
                                               spread*math.sin(theta),
                                               spread*.20*((j%3)-1)))
                poses.append(pose); constraints.append(arm)
            add('multi_mixed_%dpt_%03dmm'%(count,round(spread*1000)),
                'multi_mixed',poses,constraints,offset=spread)

    # Coupled (translation + TCP rotation) stresses the dexterous reachability map.
    for arm in ('left','right'):
        for rep in range(c['coupled']):
            radius=c['offsets'][(rep*3+2)%len(c['offsets'])]
            theta=rng.uniform(-math.pi,math.pi)
            angle=[10.,20.,30.][rep%3]
            target, angle=rotated(reference[arm],
                          (radius*math.cos(theta),radius*math.sin(theta),
                           radius*rng.uniform(-.4,.4)),
                          axis=('x','y','z')[rep%3],degrees=angle*((-1)**rep))
            add('coupled_%s'%arm,'coupled',[target],[arm],radius,angle)

    # Intentionally extreme Z negative controls (do not mix with coverage %).
    for rep in range(c['negative']):
        arm=('left','right')[rep%2]
        pos,q=pose_to_tuple(reference[arm])
        pose=tuple_to_pose(((pos[0],pos[1],4.5+.25*(rep%3)),q))
        add('high_z_negative','negative',[pose],[arm],
            control='negative',extra={'height_m':4.5+.25*(rep%3)})

    rng.shuffle(cases)
    # Case ID is stable across resumes; ordering is recorded once in cases.json.
    return cases


def finite_float(value):
    try:
        x=float(value)
        return x if math.isfinite(x) else None
    except (ValueError,TypeError,OverflowError):
        return None


def percentile(values,pct):
    xs=sorted(x for x in (finite_float(v) for v in values) if x is not None)
    if not xs: return None
    at=(len(xs)-1)*pct/100.
    low=math.floor(at); high=math.ceil(at)
    return xs[low] if high==low else xs[low]*(high-at)+xs[high]*(at-low)


def distribution(values):
    xs=[x for x in (finite_float(v) for v in values) if x is not None]
    if not xs:return {'n':0,'min':None,'mean':None,'median':None,'std':None,
                      'p90':None,'p95':None,'p99':None,'max':None}
    return dict(n=len(xs),min=min(xs),mean=statistics.mean(xs),
                median=statistics.median(xs),std=statistics.pstdev(xs),
                p90=percentile(xs,90),p95=percentile(xs,95),
                p99=percentile(xs,99),max=max(xs))


def wilson(success,total,z=1.9599639845):
    if total<=0: return [None,None]
    p=success/total
    den=1+z*z/total
    cen=(p+z*z/(2*total))/den
    radius=z*math.sqrt((p*(1-p)+z*z/(4*total))/total)/den
    return [max(0,100*(cen-radius)),min(100,100*(cen+radius))]


def rate(rows,flag):
    n=len(rows); hits=sum(bool(flag(r)) for r in rows)
    return {'passed':hits,'total':n,'percent':round(100*hits/n,3) if n else None,
            'wilson_95_percent':wilson(hits,n)}


def extract_failure_hint(message,diagnostics):
    s=(str(message or '')+' '+str(diagnostics or '')).lower()
    for key,terms in [
        ('time_budget',('time budget','time_budget','max_path_checks','timeout','max_verified')),
        ('navigation',('no path','path planning','path_failed','planner','start_position_mismatch')),
        ('collision_or_costmap',('collision','lethal','costmap','occupied','footprint')),
        ('reachability_or_ik',('ik fail','no ik','unreachable','reachability','joint limits')),
        ('tf_or_map',('no transform','tf error','transform failure','missing tf')),
        ('no_common_base',('intersection','no common','common base','no candidates')),
    ]:
        if any(term in s for term in terms):return key
    return 'unspecified'


def repeatability(rows):
    grouped=defaultdict(list)
    for r in rows:
        if r.get('repeat_group'):grouped[r['repeat_group']].append(r)
    result={}
    for key, rs in sorted(grouped.items()):
        if len(rs)<2: continue
        successes=[r for r in rs if r.get('status')=='success']
        xy=[]; dy=[]; score=[]
        for r in successes:
            x=finite_float(r.get('selected_x')); y=finite_float(r.get('selected_y'))
            yaw=finite_float(r.get('selected_yaw_deg'))
            if x is not None and y is not None: xy.append((x,y))
            if yaw is not None:dy.append(math.radians(yaw))
            if finite_float(r.get('score')) is not None:score.append(float(r['score']))
        worst_xy=max((math.hypot(a[0]-b[0],a[1]-b[1]) for a in xy for b in xy),default=None)
        worst_yaw=max((math.degrees(angle_difference(a,b)) for a in dy for b in dy),default=None)
        result[key]={'count':len(rs),'success_rate':rate(rs,lambda r:r.get('status')=='success'),
                     'arms':dict(Counter(r.get('selected_arm') for r in successes)),
                     'max_selected_xy_delta_m':worst_xy,
                     'max_selected_yaw_delta_deg':worst_yaw,
                     'score':distribution(score),'latency_s':distribution(r.get('time_s') for r in rs)}
    return result


def summarize(rows, suite_total=0):
    """Rates have explicit denominators and never treat untested path checks as failures."""
    completed=len(rows)
    real=[r for r in rows if r.get('control')!='negative']
    perturb=[r for r in rows if r.get('control')=='unknown']
    positives=[r for r in rows if r.get('control')=='positive']
    negative=[r for r in rows if r.get('control')=='negative']
    selected=[r for r in rows if r.get('status')=='success']
    nav=[r for r in selected if r.get('nav_status') not in ('',None,'not_checked','not_required')]
    check_ik=[r for r in selected if r.get('ik_status') not in ('',None,'not_checked')]
    def group_summary(rs):
        s=[r for r in rs if r.get('status')=='success']
        return {'total':len(rs),'selected':len(s),
                'selection':rate(rs,lambda r:r.get('status')=='success'),
                'selector_latency_s':distribution(r.get('time_s') for r in rs),
                'successful_latency_s':distribution(r.get('time_s') for r in s),
                'selected_score':distribution(r.get('score') for r in s),
                'failure_status':dict(Counter(r.get('status') for r in rs if r.get('status')!='success')),
                'failure_hint':dict(Counter(r.get('failure_hint') or 'unspecified' for r in rs if r.get('status')=='no_solution'))}
    families={k:group_summary(v) for k,v in sorted(_groups(rows,'family').items())}
    by_offset={k:group_summary(v) for k,v in sorted(_groups([r for r in perturb if r.get('family') in ('position','position_any')],'offset_m').items(), key=lambda kv:float(kv[0]))}
    by_rotation={k:group_summary(v) for k,v in sorted(_groups([r for r in perturb if r.get('family')=='orientation'],'rotation_deg').items(), key=lambda kv:float(kv[0]))}
    by_npoints={k:group_summary(v) for k,v in sorted(_groups(real,'npoints').items(),key=lambda kv:int(kv[0]))}
    per_scenario={k:group_summary(v) for k,v in sorted(_groups(rows,'scenario').items())}
    return {
        'schema_version':VERSION,'generated_utc':datetime.now(timezone.utc).isoformat(),
        'completed':completed,'total_cases':suite_total,'remaining':max(0,suite_total-completed),
        'all_requests':group_summary(rows),'non_negative':group_summary(real),
        'perturbation_only':group_summary(perturb),
        'positive_controls':group_summary(positives),
        'negative_control_rejection':rate(negative,lambda r:r.get('status')=='no_solution'),
        'nav2_conditional_on_selection':rate(nav,lambda r:r.get('nav_status')=='success'),
        'nav2_attempted_count':len(nav),
        'current_pose_count':sum(r.get('nav_status')=='not_required' for r in selected),
        'current_pose_latency_s':distribution(r.get('time_s') for r in selected if r.get('nav_status')=='not_required'),
        'reposition_selection_latency_s':distribution(r.get('time_s') for r in selected if r.get('nav_status')!='not_required'),
        'joint_selection_and_nav2_success':rate(real,lambda r:r.get('status')=='success' and r.get('nav_status')=='success') if nav else None,
        'nav2_path_length_m':distribution(r.get('path_length_m') for r in nav if r.get('nav_status')=='success'),
        'nav2_path_efficiency':distribution(r.get('path_efficiency') for r in nav if r.get('nav_status')=='success'),
        'nav2_planner_elapsed_s':distribution(r.get('nav_time_s') for r in nav),
        'virtual_ik_requested_cases':len(check_ik),
        'virtual_ik_evaluated_cases':sum((finite_float(r.get('ik_trials')) or 0)>0 for r in check_ik),
        'virtual_ik_unavailable_cases':sum(r.get('ik_status')=='unavailable' for r in check_ik),
        'virtual_ik_pass_fraction':distribution(r.get('ik_pass_fraction') for r in check_ik),
        'selected_base_distance_m':distribution(r.get('base_displacement_m') for r in selected),
        'selected_base_heading_change_deg':distribution(r.get('base_heading_change_deg') for r in selected),
        'target_facing_error_deg':distribution(r.get('target_facing_error_deg') for r in selected),
        'selected_arms':dict(Counter(r.get('selected_arm') for r in selected)),
        'per_point_arm_assignments':dict(Counter(a for r in selected for a in json.loads(r.get('point_arms') or '[]'))),
        'by_family':families,'by_offset_m':by_offset,'by_rotation_deg':by_rotation,'by_number_of_points':by_npoints,
        'by_scenario':per_scenario,'positive_control_repeatability':repeatability(rows),
        'resource_cpu_one_core_percent':distribution(r.get('selector_cpu_one_core_percent') for r in rows),
        'resource_rss_mb':distribution(r.get('selector_rss_mb') for r in rows),
        'notes':[
          'No robot movement commanded; selector feasibility and Nav2 path calculation are planning-only.',
          'Positive controls use observed TCP poses; other perturbations are not certified reachable.',
          'Success rates are sample-conditioned; no global workspace or physical success guarantee.',
          'Nav2 availability is calculated only over successful selections with attempted path checks.',
          'Nonnegative joint success with Nav2 is only meaningful if --nav2 was used for all requests.',
          'IK robustness uses virtual base perturbations and independent kinematic IK calls; it is not a collision-free multi-arm plan.',
          'Collision, costmap, sensor and planning results may change during a long benchmark.',
          '95% Wilson intervals reflect finite sample uncertainty only, not the sampling design or dependence of repeated cases.'
        ]}


def _groups(rows,field):
    out=defaultdict(list)
    for row in rows:
        v=row.get(field)
        if v is not None:out[str(v)].append(row)
    return out


def write_csv(path,rows):
    if not rows:return
    cols=sorted(set(k for row in rows for k in row))
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    with open(tmp,'w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader();w.writerows(rows)
        f.flush();os.fsync(f.fileno())
    tmp.replace(path)


def read_csv(path):
    with open(path,newline='',encoding='utf-8') as f:return list(csv.DictReader(f))


def atomic_json(path,obj):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    tmp.replace(path)


def hash_case_schedule(cases):
    return hashlib.sha256(json.dumps(cases,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def markdown_report(summary,manifest):
    s=summary
    def pct(stats):
        p=stats.get('percent') if stats else None
        return 'n/a' if p is None else f'{p:.1f}% ({stats["passed"]}/{stats["total"]})'
    def fmt(v,precision=2):return 'n/a' if v is None else f'{v:.{precision}f}'
    lines=['# MUR base selector — offline KPI report','',
           f'Profile: **{manifest.get("profile")}**, seed: **{manifest.get("seed")}**',
           f'Completed: **{s["completed"]}/{s["total_cases"]}** requests','',
           '## KPI overview','',
           '| KPI | Result |','|---|---:|',
           f'| Reference positive controls | {pct(s["positive_controls"]["selection"])} |',
           f'| Perturbed target selection | {pct(s["perturbation_only"]["selection"])} |',
           f'| Negative controls correctly rejected | {pct(s["negative_control_rejection"])} |',
           f'| Nav2 paths found / path checks attempted | {pct(s["nav2_conditional_on_selection"])} |',
           f'| Selections needing no navigation | {s["current_pose_count"]} |',
           f'| Current-stance median latency | {fmt(s["current_pose_latency_s"]["median"])} s |',
           f'| Reposition selection median latency | {fmt(s["reposition_selection_latency_s"]["median"])} s |',
           f'| Selector median latency | {fmt(s["all_requests"]["selector_latency_s"]["median"])} s |',
           f'| Selector P95 latency | {fmt(s["all_requests"]["selector_latency_s"]["p95"])} s |',
           f'| Nav2 mean path efficiency | {fmt(s["nav2_path_efficiency"]["mean"],3)} |',
           '', '## Success by task family','',
           '| Family | Success | Median latency (s) |','|---|---:|---:|']
    for name,stats in s['by_family'].items():
        lines.append(f'| {name} | {pct(stats["selection"])} | {fmt(stats["selector_latency_s"]["median"])} |')
    lines+=['','## Success by displacement','',
            '| Offset (m) | Success |','|---:|---:|']
    for dist,stats in s['by_offset_m'].items():lines.append(f'| {dist} | {pct(stats["selection"])} |')
    lines+=['','## Notes','']+['- '+x for x in s['notes']]
    return '\n'.join(lines)+'\n'


def html_report(summary, manifest):
    def esc(v):return html.escape(str(v))
    def p(v):return '—' if v is None else f'{v:.1f}%'
    def f(v,dec=2):return '—' if v is None else f'{v:.{dec}f}'
    def mk_table(stats):
        rows=[]
        for key,entry in stats.items():
            metric=entry['selection'];lat=entry['selector_latency_s']
            rate_text=p(metric['percent'])
            lo,hi=metric['wilson_95_percent']
            interval=f'{f(lo,1)}–{f(hi,1)}%' if lo is not None else '—'
            value=metric['percent'] or 0
            rows.append(f'<tr><td>{esc(key)}</td><td>{esc(metric["passed"])}/{esc(metric["total"])}</td>'
                        f'<td><div class="bar"><i style="width:{value:.3f}%"></i></div>{rate_text}</td>'
                        f'<td>{interval}</td><td>{f(lat["median"])} s</td></tr>')
        return '<table><thead><tr><th>Scenario</th><th>Selected</th><th>Success</th><th>Wilson 95% CI</th><th>Median time</th></tr></thead><tbody>'+''.join(rows)+'</tbody></table>'
    nav=summary['nav2_conditional_on_selection'];pert=summary['perturbation_only']['selection']
    ref=summary['positive_controls']['selection'];lat=summary['all_requests']['selector_latency_s']
    cards=[('Reference controls',p(ref['percent'])),('Perturbed selection',p(pert['percent'])),
           ('Nav2 paths / checks',p(nav['percent'])),('Stayed at current pose',str(summary['current_pose_count'])),('Median response',f(lat['median'])+' s'),
           ('P95 response',f(lat['p95'])+' s'),('Cases finished',f'{summary["completed"]} / {summary["total_cases"]}')]
    card_html=''.join('<div class="card"><div class="muted">%s</div><strong>%s</strong></div>'%(esc(k),esc(v)) for k,v in cards)
    notes=''.join('<li>'+esc(n)+'</li>' for n in summary['notes'])
    return '''<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>MUR selection KPI report</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{font:15px/1.5 system-ui,sans-serif;margin:0;background:#0c1521;color:#e6eef8}
main{max-width:1080px;margin:auto;padding:32px 20px}h1{font-size:29px}h2{margin-top:34px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
.card{background:#1a2a3d;border:1px solid #304155;border-radius:12px;padding:16px}
.card strong{font-size:25px}.muted{color:#9cacc3;font-size:13px}
table{border-collapse:collapse;width:100%;font-size:14px}td,th{text-align:left;border-bottom:1px solid #324153;padding:10px 7px}
.bar{width:110px;height:8px;border-radius:6px;background:#39495b;overflow:hidden;display:inline-block;margin-right:9px}
.bar i{display:block;height:100%;background:#61c4a4}p,li{color:#c0cfe0}
</style></head><body><main><h1>MUR base selector KPI report</h1>
<p>Profile: <b>'''+esc(manifest.get('profile'))+'''</b> · Seed: <b>'''+esc(manifest.get('seed'))+'''</b> · Planning only</p>
<div class="grid">'''+card_html+'''</div><h2>Task families</h2>'''+mk_table(summary['by_family'])+'''
<h2>Position/orientation/multi-point breakdown</h2><h3>Position offsets (m)</h3>'''+mk_table(summary['by_offset_m'])+'''
<h3>TCP rotation changes (degrees)</h3>'''+mk_table(summary['by_rotation_deg'])+'''
<h3>Number of target poses</h3>'''+mk_table(summary['by_number_of_points'])+'''
<h3>Individual scenarios</h3>'''+mk_table(summary['by_scenario'])+'''
<h2>Interpretation limits</h2><ul>'''+notes+'''</ul></main></body></html>'''
