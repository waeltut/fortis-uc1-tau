#!/usr/bin/env python3
"""Extensive, resumable, planning-only KPI workload for mur_test.

Never publishes velocity/goal topics or calls execution actions. Existing selector
service is assumed planning-only. Optional Nav2 action is ComputePathToPose.
Optional MoveIt service calls GetPositionIK (IK only, never trajectory execution).
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
import os
import random
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from . import kpi_stress_core as C

SERVICE='/base_selection/select'
EXPECTED_TYPE='mur_base_selector_interfaces/srv/SelectBase'


def selector_cpu_snapshot(pid):
    """Optional /proc snapshot. CPU % later refers to ONE logical core."""
    if not pid:return None
    try:
        value=Path(f'/proc/{pid}/stat').read_text()
        fields=value[value.rindex(')')+2:].split()
        ticks=int(fields[11])+int(fields[12])  # utime/stime, fields 14 and 15.
        status=Path(f'/proc/{pid}/status').read_text()
        rss=None
        for line in status.splitlines():
            if line.startswith('VmRSS:'):
                rss=int(line.split()[1])/1024.
                break
        return (time.monotonic(),ticks,rss)
    except (OSError,ValueError,IndexError):
        return None


def cpu_delta(before,after):
    if before is None or after is None:return None,None
    elapsed=after[0]-before[0]
    if elapsed<=0:return None,after[2]
    clock_ticks=os.sysconf(os.sysconf_names['SC_CLK_TCK'])
    return max(0.,100*(after[1]-before[1])/clock_ticks/elapsed),after[2]


def read_snapshot_ros(ros,frame='map',base_frame='base_link',mur_frame='mur',left='left_tcp',right='right_tcp'):
    # The TF frame names must match the selector URDF/MoveIt installation.
    return {'created_utc':datetime.now(timezone.utc).isoformat(), 'frame':frame,
            'base_frame':base_frame,'mur_frame':mur_frame,
            'base':ros.get_tf_pose(frame,base_frame),
            'mur':ros.get_tf_pose(frame,mur_frame),
            'left':ros.get_tf_pose(frame,left),
            'right':ros.get_tf_pose(frame,right),
            'tcp_frames':{'left':left,'right':right}}


def validate_snapshot(s):
    if s.get('frame')!='map':
        raise ValueError('Only map-frame reference poses are supported')
    for name in ('base','mur','left','right'):
        if name not in s:
            raise ValueError('Snapshot missing %s pose' % name)
        C.pose_to_tuple(s[name])
        if s[name].get('header',{}).get('frame_id')!='map':
            raise ValueError('Snapshot %s must be in map frame' % name)


def orient_and_distance(case,pose,snapshot):
    base,_=C.pose_to_tuple(snapshot['base'])
    goal,q=C.pose_to_tuple(pose)
    sx,sy=goal[:2]
    distance=math.hypot(base[0]-sx,base[1]-sy)
    base_yaw=C.yaw_of(C.pose_to_tuple(snapshot['base'])[1])
    yaw=C.yaw_of(q)
    bearing=math.atan2(case['targets'][0]['y']-sy,case['targets'][0]['x']-sx)
    return distance,math.degrees(C.angle_difference(yaw,base_yaw)),math.degrees(C.angle_difference(yaw,bearing))


def case_row(case):
    return dict(id=case['id'],scenario=case['scenario'],family=case['family'],
                control=case['control'],expected_feasible=case['expected_feasible'],
                repeat_group=case.get('repeat_group',''),
                npoints=case['npoints'],arm_constraint=case['arm_constraint'],
                offset_m=case['offset_m'],rotation_deg=case['rotation_deg'],
                first_target_x=case['targets'][0]['x'],
                first_target_y=case['targets'][0]['y'],
                first_target_z=case['targets'][0]['z'],
                status='not_run',time_s=None,selected_arm='',
                selected_x=None,selected_y=None,selected_yaw_deg=None,
                selected_z=None,base_displacement_m=None,
                base_heading_change_deg=None,target_facing_error_deg=None,
                score=None,request_id=None,valid_for_seconds=None,
                point_arms='[]',planning_groups='[]',tcp_frames='[]',
                nav_status='not_checked',nav_time_s=None,
                path_length_m=None,path_efficiency=None,
                ik_status='not_checked',ik_trials=None,ik_passed=None,
                ik_pass_fraction=None,ik_error_counts='{}',
                selector_cpu_one_core_percent=None,selector_rss_mb=None,
                failure_hint='',message='',diagnostics_json='')


def get_assignment(response,case,index):
    n=len(case['targets'])
    groups=response.get('planning_groups') or []
    frames=response.get('tcp_frames') or []
    assigned=response.get('point_arms') or []
    if len(assigned)==n:
        arm=str(assigned[index]).strip().lower()
    else:
        arm=str(response.get('arm') or '').strip().lower()
        if arm not in ('left','right'):
            arm=case['targets'][index]['arm'].strip().lower()
    if len(groups)==n and groups[index] in ('left_ur_manipulator','right_ur_manipulator'):
        group=groups[index]
        arm='left' if group.startswith('left') else 'right'
    elif arm in ('left','right'):
        group=arm+'_ur_manipulator'
    else:
        return None,None,'unknown_arm_assignment'
    expected='left' if group.startswith('left') else 'right'
    tip=(frames[index] if len(frames)==n and frames[index] else expected+'_tcp')
    return group,tip,''


class VirtualIK:
    """Checks kinematic reachability at a *hypothetical* base pose.

    Never updates actual TF, robot state, or planning scene. External obstacle
    collision validity is NOT established by this test. Separate IK per point
    does not establish that all points have jointly collision-free trajectories.
    """
    def __init__(self,ros,snapshot,service='/compute_ik',timeout_s=.5):
        from moveit_msgs.srv import GetPositionIK
        self.ros=ros;self.service=service;self.timeout=timeout_s
        self.msg=GetPositionIK
        self.client=ros.node.create_client(GetPositionIK,service)
        self.available=self.client.wait_for_service(timeout_sec=2.)
        self.base_to_mur=C.compose(C.invert(C.pose_to_tuple(snapshot['base'])),
                                   C.pose_to_tuple(snapshot['mur']))

    def check_one(self,pose_in_mur,group,tip):
        if not self.available:return 'unavailable'
        req=self.msg.Request()
        ik=req.ik_request
        ik.group_name=group
        ik.ik_link_name=tip
        ik.pose_stamped.header.frame_id='mur'
        p,q=pose_in_mur
        ik.pose_stamped.pose.position.x,ik.pose_stamped.pose.position.y,ik.pose_stamped.pose.position.z=p
        ik.pose_stamped.pose.orientation.x,ik.pose_stamped.pose.orientation.y,\
            ik.pose_stamped.pose.orientation.z,ik.pose_stamped.pose.orientation.w=q
        ik.avoid_collisions=False  # Kinematic IK, not environment collision validation.
        ns=int(max(0.01,self.timeout)*1e9)
        ik.timeout.sec=ns//1_000_000_000
        ik.timeout.nanosec=ns%1_000_000_000
        future=self.client.call_async(req)
        if not self.ros.await_future(future,max(2.,self.timeout*4)):
            return 'timeout'
        try:
            response=future.result()
        except Exception as exc:
            return 'exception_'+type(exc).__name__
        return 'success' if response.error_code.val==1 else 'ik_code_'+str(response.error_code.val)

    def evaluate(self,case,selected_pose,response,trial_count,radius_m,yaw_deg):
        if not self.available:return {'ik_status':'unavailable'}
        base_p,base_q=C.pose_to_tuple(selected_pose)
        rng=random.Random(42_100+int(case['id']))
        trials=0;passed=0;errors={}
        for trial in range(trial_count):
            angle=rng.uniform(-math.pi,math.pi)
            rad=radius_m*math.sqrt(rng.random())
            dx=rad*math.cos(angle);dy=rad*math.sin(angle)
            dyaw=math.radians(rng.uniform(-yaw_deg,yaw_deg))
            # World-frame XY perturbation + world Z rotation about candidate base.
            q=C.unit_quaternion(C.qmul(C.axis_angle('z',dyaw),base_q))
            hypothetical_base=((base_p[0]+dx,base_p[1]+dy,base_p[2]),q)
            map_to_mur=C.compose(hypothetical_base,self.base_to_mur)
            mur_to_map=C.invert(map_to_mur)
            valid=True
            for point_index,p in enumerate(case['targets']):
                group,tip,reason=get_assignment(response,case,point_index)
                if reason:
                    errors[reason]=errors.get(reason,0)+1
                    valid=False;break
                tcp=C.pose_to_tuple(C.tuple_to_pose(((p['x'],p['y'],p['z']),
                                                      (p['qx'],p['qy'],p['qz'],p['qw']))))
                tcp_in_mur=C.compose(mur_to_map,tcp)
                state=self.check_one(tcp_in_mur,group,tip)
                if state!='success':
                    valid=False
                    errors[state]=errors.get(state,0)+1
                    break
            trials+=1
            if valid:passed+=1
            if 'timeout' in errors or 'unavailable' in errors:
                break  # Do not queue more calls to a possibly busy server.
        return {'ik_status':'partial_timeout' if 'timeout' in errors else 'complete',
                'ik_trials':trials,'ik_passed':passed,
                'ik_pass_fraction':passed/trials if trials else None,
                'ik_error_counts':json.dumps(errors,sort_keys=True)}


def finish_reports(out,rows,cases,manifest):
    summary=C.summarize(rows,len(cases))
    C.atomic_json(out/'summary.json',summary)
    (out/'report.md').write_text(C.markdown_report(summary,manifest),encoding='utf-8')
    (out/'report.html').write_text(C.html_report(summary,manifest),encoding='utf-8')
    return summary


def run(args):
    # Only the ROS-facing module imports rclpy. Importing core or viewing reports is safe offline.
    from .kpi_benchmark import RosInterface,response_info,pose_geometry,compact_diagnostic
    out=Path(args.out).expanduser().resolve()
    manifest_path=out/'manifest.json'; cases_path=out/'cases.json'
    if args.resume:
        if not manifest_path.is_file() or not cases_path.is_file():
            raise ValueError('--resume requires existing manifest.json and cases.json')
        manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
        cases=json.loads(cases_path.read_text(encoding='utf-8'))
        if C.hash_case_schedule(cases)!=manifest.get('case_sha256'):
            raise ValueError('cases.json checksum differs from manifest; refusing resume')
        opts=manifest.get('options',{})
        for flag,expected in [('nav2',args.nav2),('ik_robustness',args.ik_robustness)]:
            if opts.get(flag)!=expected:
                raise ValueError('--resume requires same --%s option as original run' % flag.replace('_','-'))
        if args.ik_robustness:
            for name in ('ik_trials','ik_every','robust_radius','robust_yaw'):
                if opts.get(name)!=getattr(args,name):
                    raise ValueError('--resume requires unchanged %s' % name)
        results=C.read_csv(out/'results.csv') if (out/'results.csv').is_file() else []
        processed={int(r['id']) for r in results}
        snapshot=manifest['snapshot']
        print(f'Resuming {len(results)}/{len(cases)} completed cases from {out}.',flush=True)
    else:
        if out.exists() and any((out/p).exists() for p in ('manifest.json','results.csv','cases.json')):
            raise ValueError('Output directory already contains benchmark data; use --resume or a new --out')
        out.mkdir(parents=True,exist_ok=True)
        snapshot=json.loads(Path(args.snapshot).read_text(encoding='utf-8')) if args.snapshot else None
        if snapshot is not None:validate_snapshot(snapshot)
        cases=[];results=[];processed=set();manifest={}

    ros=None
    try:
        ros=RosInterface(SERVICE,None)
        if ros.service_type_name!=EXPECTED_TYPE:
            raise ValueError('Unexpected selector service type: '+ros.service_type_name)
        if not args.resume:
            if snapshot is None:
                snapshot=read_snapshot_ros(ros,left=args.left_tcp,right=args.right_tcp,
                                           base_frame=args.base_frame,mur_frame=args.mur_frame)
            validate_snapshot(snapshot)
            cases=C.build_cases(snapshot,args.profile,args.seed,args.center,args.orientation,args.max_yaw_deg,args.offset_scale)
            manifest={'tool':'mur_kpi_stress','schema_version':C.VERSION,
                      'generated_utc':datetime.now(timezone.utc).isoformat(),
                      'profile':args.profile,'seed':args.seed,
                      'service':SERVICE,'service_type':EXPECTED_TYPE,
                      'snapshot':snapshot, 'center_map_xyz':args.center, 'orientation_xyzw':args.orientation,
                      'max_yaw_deg':args.max_yaw_deg, 'offset_scale':args.offset_scale,
                      'options':{'nav2':args.nav2,'ik_robustness':args.ik_robustness,
                                 'ik_trials':args.ik_trials,'ik_every':args.ik_every,
                                 'robust_radius':args.robust_radius,'robust_yaw':args.robust_yaw},
                      'case_sha256':C.hash_case_schedule(cases)}
            C.atomic_json(out/'manifest.json',manifest)
            C.atomic_json(out/'cases.json',cases)
        print(f'Prepared {len(cases)} cases ({manifest["profile"]} profile). '
              f'Existing completed: {len(results)}. Nav2={args.nav2}, '
              f'virtual IK={args.ik_robustness}.',flush=True)
        monitor_pid=args.selector_pid
        if monitor_pid and selector_cpu_snapshot(monitor_pid) is None:
            print('Warning: selector PID is not readable in /proc; CPU metrics unavailable.',flush=True)
        ik=None
        if args.ik_robustness:
            ik=VirtualIK(ros,snapshot,timeout_s=args.ik_timeout)
            if not ik.available:
                print('Warning: /compute_ik is not available; virtual IK results will be unavailable.',flush=True)
        if args.warmup and not args.resume:
            warmup=next(c for c in cases if c['scenario']=='reference_left')
            for j in range(args.warmup):
                _,t,err=ros.call(warmup['request'],args.timeout)
                print('Warmup %d/%d: %.3f s %s'%(j+1,args.warmup,t,err or 'response received'),flush=True)
                if err=='timeout':
                    print('Warmup timed out; stop to avoid overlapping selection requests.',flush=True)
                    return 3
        raw_file=open(out/'raw_responses.jsonl','a',encoding='utf-8') if args.save_raw else None
        aborted=False
        try:
            for i,case in enumerate(cases,1):
                if i>args.limit>0:break
                if case['id'] in processed:continue
                row=case_row(case)
                before=selector_cpu_snapshot(monitor_pid)
                response=None;service_timed_out=False
                try:
                    response,elapsed,error=ros.call(case['request'],args.timeout)
                    after=selector_cpu_snapshot(monitor_pid)
                    row['time_s']=round(elapsed,5)
                    cpu,rss=cpu_delta(before,after)
                    row['selector_cpu_one_core_percent']=cpu
                    row['selector_rss_mb']=rss
                    if error:
                        row['status']=error
                        service_timed_out=(error=='timeout')
                    else:
                        status,pose,arm,message=response_info(response,{
                            'response_success_path':'success','response_pose_path':'navigation_goal',
                            'response_arm_path':'arm','response_message_path':'message','frame':'map'})
                        row.update(status=status,selected_arm=arm,message=message,
                                   score=response.get('score') if status=='success' else None,
                                   request_id=response.get('request_id'),
                                   valid_for_seconds=response.get('valid_for_seconds'))
                        for key in ('point_arms','planning_groups','tcp_frames'):
                            row[key]=json.dumps(response.get(key) or [])
                        raw_diag=response.get('diagnostics_json') or ''
                        if raw_diag:
                            try:row['diagnostics_json']=json.dumps(json.loads(raw_diag),separators=(',',':'))
                            except (ValueError,TypeError):row['diagnostics_json']=str(raw_diag)
                        if status!='success':
                            row['failure_hint']=C.extract_failure_hint(message,row['diagnostics_json'])
                        else:
                            if not pose:
                                row['status']='malformed_success_no_pose'
                            elif pose.get('header',{}).get('frame_id')!='map':
                                row['status']='success_nonmap_goal'
                            else:
                                x,y,yaw=pose_geometry(pose)
                                row['selected_x']=x;row['selected_y']=y
                                row['selected_yaw_deg']=yaw
                                row['selected_z']=pose['pose']['position'].get('z')
                                dist,change,facing=orient_and_distance(case,pose,snapshot)
                                row.update(base_displacement_m=dist,base_heading_change_deg=change,
                                           target_facing_error_deg=facing)
                                if response.get('requires_navigation') is False:
                                    row['nav_status'] = 'not_required'
                                elif args.nav2:
                                    start=time.monotonic()
                                    # Humble set_message_fields may mutate nested input dictionaries.
                                    # Protect the response pose: virtual IK still needs it afterward.
                                    nstatus,plen,eff=ros.compute_nav_path(deepcopy(pose),args.nav_timeout)
                                    row.update(nav_status=nstatus,path_length_m=plen,
                                               path_efficiency=eff,
                                               nav_time_s=round(time.monotonic()-start,5))
                                if ik and (i%args.ik_every==0 or case['control']=='positive'):
                                    row.update(ik.evaluate(case,pose,response,args.ik_trials,
                                                           args.robust_radius,args.robust_yaw))
                    if raw_file and response is not None:
                        raw_file.write(json.dumps({'case_id':case['id'],'response':response},
                                                  ensure_ascii=False)+'\n')
                        raw_file.flush()
                except KeyboardInterrupt:
                    print('\nInterrupted: completed cases have been checkpointed.',flush=True)
                    aborted=True
                    break
                except Exception as exc:
                    row['status']='exception'
                    row['message']=type(exc).__name__+': '+str(exc)
                results.append(row)
                processed.add(case['id'])
                # Preserve results, including failures, after every request.
                C.write_csv(out/'results.csv',results)
                if i%args.report_every==0 or service_timed_out:
                    finish_reports(out,results,cases,manifest)
                print('[%d/%d] %-28s %-22s %7s s | Nav2=%s | IK=%s'%(
                    i,len(cases),case['scenario'][:28],row['status'],
                    f'{row["time_s"]:.3f}' if row['time_s'] is not None else 'n/a',
                    row['nav_status'],row['ik_status']),flush=True)
                if row['status']!='success' and row['status']!='not_run':
                    if row['message']:
                        print('    message:',row['message'][:500],flush=True)
                    if row['diagnostics_json']:
                        print('    diagnostics:',compact_diagnostic(row['diagnostics_json'],700),flush=True)
                if service_timed_out:
                    print('Stopping after selector timeout; original server call may still be running.',flush=True)
                    aborted=True
                    break
                if args.pause>0:time.sleep(args.pause)
        finally:
            if raw_file:raw_file.close()
        summary=finish_reports(out,results,cases,manifest)
        control=summary['positive_controls']['selection']
        perturb=summary['perturbation_only']['selection']
        print('\nReference controls: %s/%s; perturbations: %s/%s'%(control['passed'],control['total'],
              perturb['passed'],perturb['total']),flush=True)
        print('Selection P95: %s s'%summary['all_requests']['selector_latency_s']['p95'],flush=True)
        print('Reports:',out/'report.html',out/'summary.json',out/'results.csv',flush=True)
        print('Cases remaining:',summary['remaining'],flush=True)
        return 3 if aborted else 0
    finally:
        if ros:ros.shutdown()


def preview(args):
    snapshot=json.loads(Path(args.snapshot).read_text(encoding='utf-8')) if args.snapshot else None
    if snapshot is None:
        from .kpi_benchmark import RosInterface
        ros=RosInterface(SERVICE,None)
        try:snapshot=read_snapshot_ros(ros,left=args.left_tcp,right=args.right_tcp,
                                       base_frame=args.base_frame,mur_frame=args.mur_frame)
        finally:ros.shutdown()
    validate_snapshot(snapshot)
    cases=C.build_cases(snapshot,args.profile,args.seed,args.center,args.orientation,args.max_yaw_deg,args.offset_scale)
    print(f'{len(cases)} generated cases. First {min(args.limit or 10,len(cases))}:')
    for case in cases[:args.limit or 10]:
        print('%4d %-26s %2d points %-20s position %.3f m rotation %.1f deg'%(
            case['id'],case['scenario'],case['npoints'],case['arm_constraint'],
            case['offset_m'],case['rotation_deg']))
    if args.out:
        out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
        C.atomic_json(out/'preview_cases.json',cases)
        print('Preview written to',out/'preview_cases.json')
    return 0


def report(args):
    out=Path(args.out)
    manifest=json.loads((out/'manifest.json').read_text(encoding='utf-8'))
    cases=json.loads((out/'cases.json').read_text(encoding='utf-8'))
    if C.hash_case_schedule(cases)!=manifest.get('case_sha256'):
        raise ValueError('Case schedule checksum mismatch')
    rows=C.read_csv(out/'results.csv')
    summary=finish_reports(out,rows,cases,manifest)
    print('Regenerated reports for %d/%d completed cases.'%(len(rows),len(cases)))
    print(out/'report.html')
    return 0


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('mode',choices=('run','preview','report'))
    ap.add_argument('--profile',choices=tuple(C.PROFILES),default='standard')
    ap.add_argument('--center', nargs=3, type=float, metavar=('X','Y','Z'), help='Map-frame TCP centre in metres for perturbed cases')
    ap.add_argument('--orientation', nargs=4, type=float, metavar=('QX','QY','QZ','QW'), help='Map-frame TCP orientation; default uses each observed TCP quaternion')
    ap.add_argument('--max-yaw-deg', type=float, default=5., help='Maximum absolute change in map heading of a perturbed TCP')
    ap.add_argument('--offset-scale', type=float, default=1., help='Multiply profile translation offsets about each centre')
    ap.add_argument('--seed',type=int,default=2026)
    ap.add_argument('--out',default='/tmp/mur_kpi_stress',help='Output directory')
    ap.add_argument('--snapshot',help='Optional prior snapshot JSON; otherwise read live TCP poses from TF')
    ap.add_argument('--base-frame',default='base_link')
    ap.add_argument('--mur-frame',default='mur')
    ap.add_argument('--left-tcp',default='left_tcp')
    ap.add_argument('--right-tcp',default='right_tcp')
    ap.add_argument('--limit',type=int,default=0,help='Restrict number of scheduled cases executed (0 means all)')
    ap.add_argument('--resume',action='store_true',help='Continue exactly the existing saved case schedule')
    ap.add_argument('--nav2',action='store_true',help='Call planning-only ComputePathToPose after valid selections')
    ap.add_argument('--ik-robustness',action='store_true',
                    help='Virtual base XY/yaw errors followed by MoveIt kinematic IK, never execution')
    ap.add_argument('--ik-trials',type=int,default=12)
    ap.add_argument('--ik-every',type=int,default=5,help='IK checks every Nth successful scheduled case + all positive controls')
    ap.add_argument('--robust-radius',type=float,default=.05,help='Maximum XY shift in metres')
    ap.add_argument('--robust-yaw',type=float,default=5.,help='Maximum yaw shift in degrees')
    ap.add_argument('--ik-timeout',type=float,default=.5,help='IK algorithm timeout per target, seconds')
    ap.add_argument('--timeout',type=float,default=60.,help='Selector call timeout; halt suite on timeout')
    ap.add_argument('--nav-timeout',type=float,default=30.)
    ap.add_argument('--pause',type=float,default=0.,help='Delay between selector requests in seconds')
    ap.add_argument('--warmup',type=int,default=0,help='Exclude this many reference calls from statistics')
    ap.add_argument('--report-every',type=int,default=10,help='Regenerate reports every N cases + at finish')
    ap.add_argument('--save-raw',action='store_true',help='Append full service responses in JSONL')
    ap.add_argument('--selector-pid',type=int,help='Optional Linux PID for CPU and RSS monitoring')
    args=ap.parse_args(argv)
    if args.limit<0 or args.ik_every<1 or args.ik_trials<1 or args.report_every<1 or args.warmup<0:
        ap.error('Invalid nonpositive limit / trial / interval argument')
    if min(args.timeout,args.nav_timeout,args.ik_timeout)<=0 or args.pause<0 or args.robust_radius<0 or args.robust_yaw<0:
        ap.error('Invalid timeout, pause, or perturbation argument')
    try:
        return {'run':run,'preview':preview,'report':report}[args.mode](args)
    except (OSError,ValueError,RuntimeError,ImportError) as exc:
        print('ERROR:',exc,file=sys.stderr)
        return 2


if __name__=='__main__':
    sys.exit(main())
