"""Volume-only reachability selection. Never imports/constructs an IK engine."""
import copy
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult,ParameterType
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String,Bool
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker,MarkerArray
from .navigation import BaseSelector,Candidate,SelectionError,Cancelled,CheckingBudgetExhausted,pose_msg,quaternion
from .core import transform_points,diverse_order,path_length,angle
from .volume import TravelField,working_score
from .cache_startup import DEFAULT_VOLUME_FILE,CacheStartupError,load_selector_volume
from rclpy.logging import get_logger

DEFAULTS={
 'volume_file':DEFAULT_VOLUME_FILE,
 'selection_budget':12.0,'final_validation_reserve':2.0,'travel_field_budget':2.0,
 'ranking_pool_size':160,'max_destination_checks':80,'target_feasible_paths':3,
 'yaw_bins':24,'minimum_orientation_coverage':0.90,
 'max_height_error':0.05,'max_orientation_error_deg':30.0,
 'front_fov_deg':250.0,'front_yaw_offset_deg':0.0,
 'travel_inflation_weight':3.0,
 'weight_navigation':1.0,'weight_destination_cost':2.0,
 'weight_working_distance':1.0,'weight_facing':0.5,
 'distance_scale':0.25,
 'left_preferred_distance':0.65,'right_preferred_distance':0.65,'both_preferred_distance':0.65,
 'left_facing_min_deg':0.0,'left_facing_max_deg':60.0,
 'right_facing_min_deg':-60.0,'right_facing_max_deg':0.0,
 'both_facing_min_deg':-10.0,'both_facing_max_deg':10.0,
}


class VolumeSelector(BaseSelector):
    def __init__(self):
        self.initializing_volume=True
        super().__init__(defaults={'max_path_checks':8,'path_timeout':1.0})
        try:
            for key,value in DEFAULTS.items():
                self.declare_parameter(key,value);self.cfg[key]=self.get_parameter(key).value
            self.initializing_volume=False
            self.validate_volume()
            self.volume=load_selector_volume(self.cfg['volume_file'])
            if self.volume.meta['context']['base_frame']!=self.cfg['base_frame']:raise ValueError('Cache base_frame mismatch')
            generated=self.volume.meta['generation']
            if self.cfg['front_fov_deg']>generated['workspace_fov_deg'] or abs(self.cfg['front_yaw_offset_deg']-generated['workspace_front_offset_deg'])>1e-9:
                raise ValueError('Requested FOV exceeds cache or front offset differs; regenerate cache')
            self.model_client=self.create_client(GetParameters,self.volume.meta["context"]["source_node"].rstrip("/")+"/get_parameters",callback_group=self.group)
            self.visual_target=None
            qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.target_markers_pub=self.create_publisher(MarkerArray,'/base_selection/target_markers',qos)
            self.create_timer(1.0,self.publish_target)
            self.status('idle','Volume selector ready; zero online IK. Selection only; no motion commands.',cache_id=self.volume.meta['cache_id'])
        except Exception:
            self.tf_receiver.close();self.destroy_node();raise

    def publish_target(self):
        # Separate topic: candidate invalidation never deletes the requested target.
        with self.lock:
            target = copy.deepcopy(self.visual_target)
        if target is None:
            return
        markers = []
        for ident, kind in enumerate((Marker.SPHERE, Marker.ARROW, Marker.TEXT_VIEW_FACING)):
            marker = Marker()
            marker.header = copy.deepcopy(target.header)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = 'requested_target', ident, kind, Marker.ADD
            marker.pose = copy.deepcopy(target.pose)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.2, 0.8, 1.0
            if kind == Marker.SPHERE:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.07
            elif kind == Marker.ARROW:
                marker.scale.x, marker.scale.y, marker.scale.z = 0.25, 0.018, 0.035
            else:
                marker.pose.position.z += 0.13
                marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.06
                marker.text = 'Target (arrow: TCP +X)'
            markers.append(marker)
        self.target_markers_pub.publish(MarkerArray(markers=markers))


    def parameters_changed(self,params):
        if getattr(self,'initializing_volume',False):return SetParametersResult(successful=True)
        if any(p.name in DEFAULTS for p in params):return SetParametersResult(successful=False,reason='Restart volume selector to change configuration')
        return super().parameters_changed(params)

    def validate_volume(self):
        c=self.cfg
        for k in DEFAULTS:
            v=c[k]
            if isinstance(v,(float,int)) and not math.isfinite(v):raise ValueError('Nonfinite '+k)
        positive=('selection_budget','final_validation_reserve','travel_field_budget','ranking_pool_size','max_destination_checks','target_feasible_paths','yaw_bins','distance_scale')
        if any(c[k]<=0 for k in positive):raise ValueError('Counts/scales/budgets must be positive')
        for k in ('ranking_pool_size','max_destination_checks','target_feasible_paths','yaw_bins'):
            if not isinstance(c[k],int):raise ValueError(k+' must be integer')
        if c['yaw_bins']>360 or c['ranking_pool_size']<c['max_destination_checks'] or c['target_feasible_paths']>c['max_path_checks']:raise ValueError('Invalid candidate limits')
        if c['final_validation_reserve']>=c['selection_budget']:raise ValueError('Final reserve must be below total budget')
        if not 0<=c['minimum_orientation_coverage']<=1 or not 0<c['front_fov_deg']<=360:raise ValueError('Invalid coverage/FOV')
        if c['max_height_error']<0 or not 0<=c['max_orientation_error_deg']<=90:raise ValueError('Invalid approximation tolerance')
        for k in ('weight_navigation','weight_destination_cost','weight_working_distance','weight_facing','travel_inflation_weight'):
            if c[k]<0:raise ValueError('Negative '+k)
        for arm in ('left','right','both'):
            if c[arm+'_preferred_distance']<=0 or not -180<=c[arm+'_facing_min_deg']<=c[arm+'_facing_max_deg']<=180:raise ValueError('Invalid arm preference')

    def remaining(self,deadline,limit):
        self.checkpoint();left=min(limit,deadline-time.monotonic())
        if left<=0:raise CheckingBudgetExhausted('Selection budget exhausted')
        return left

    def check_model(self,deadline):
        timeout=self.remaining(deadline,self.cfg['service_timeout'])
        if not self.model_client.wait_for_service(timeout_sec=timeout):raise SelectionError('MoveIt model parameters unavailable')
        names=['robot_description','robot_description_semantic']
        reply=self.wait(self.model_client.call_async(GetParameters.Request(names=names)),self.remaining(deadline,self.cfg['service_timeout']))
        if len(reply.values)!=2 or any(v.type!=ParameterType.PARAMETER_STRING or v.string_value!=self.volume.meta['context'][n] for n,v in zip(names,reply.values)):
            raise SelectionError('Robot URDF/SRDF differs from cache; regenerate volume')

    def cache_context(self):
        context=self.volume.meta['context']
        mount=self.lookup(self.cfg['base_frame'],context['reference_frame']).transform
        current=np.array([mount.translation.x,mount.translation.y,mount.translation.z,*quaternion(mount.rotation)])
        expected=np.asarray(self.volume.meta['mount'])
        if np.linalg.norm(current[:3]-expected[:3])>1e-4 or abs(np.dot(current[3:],expected[3:]))<1-1e-8:raise SelectionError('Robot mount differs from volume; regenerate cache')

    def guard_motion(self,start,joints):
        super().guard_motion(start,joints)
        self.cache_context()  # rigid mount identity only; no cached arm posture

    def score_candidate(self,c,travel,destination_cost,target):
        distance=math.hypot(c.pose[0]-target[0],c.pose[1]-target[1])
        return (self.cfg['weight_navigation']*travel/self.cfg['length_scale']+
                self.cfg['weight_destination_cost']*destination_cost+
                working_score(distance,c.bearing,c.arm,self.cfg))

    def run(self,target):
        started=time.monotonic();deadline=started+self.cfg['selection_budget'];search_end=deadline-self.cfg['final_validation_reserve']
        timings={};candidates=[];feasible=[];paths=destinations=0
        try:
            self.status('checking','Checking robot state and Nav2; volume reachability uses no IK')
            frozen=self.transform_target(target,self.cfg['fixed_frame'])
            with self.lock:self.last_target=copy.deepcopy(frozen);self.visual_target=copy.deepcopy(frozen)
            self.publish_target();self.check_model(search_end);start,joints=self.robot_pose(),self.joint_snapshot();self.cache_context()
            footprint=self.footprint_snapshot()
            if not self.planner.wait_for_server(timeout_sec=self.remaining(search_end,self.cfg['service_timeout'])):raise SelectionError('Nav2 planner unavailable')
            maps=self.costmaps(search_end);base=self.lookup(self.cfg['fixed_frame'],self.cfg['base_frame']).transform
            timings['setup_seconds']=time.monotonic()-started
            p=frozen.pose.position;goal=[p.x,p.y,p.z,*quaternion(frozen.pose.orientation)]
            phase=time.monotonic()
            def checkpoint():self.remaining(search_end,self.cfg['selection_budget'])
            rows=self.volume.query(goal,base.translation.z,self.cfg['yaw_bins'],self.cfg['minimum_orientation_coverage'],self.cfg['max_height_error'],self.cfg['max_orientation_error_deg'],self.cfg['front_fov_deg'],self.cfg['front_yaw_offset_deg'],self.cfg['arm_mode'],checkpoint)
            timings['lookup_seconds']=time.monotonic()-phase
            if not rows:raise SelectionError('No volume candidates under current coverage/orientation/height/FOV limits; not proof of unreachability')
            phase=time.monotonic();grid,tf=maps[0]
            field=TravelField(grid,transform_points([start[:2]],tf)[0],self.cfg['travel_inflation_weight'],self.cfg['allow_unknown'],min(search_end,time.monotonic()+self.cfg['travel_field_budget']),self.checkpoint)
            travel=field.lookup(transform_points([r[0][:2] for r in rows],tf))
            timings['travel_field_seconds']=time.monotonic()-phase
            for i,(row,cost) in enumerate(zip(rows,travel)):
                if not math.isfinite(cost):continue
                pose,arm,bearing,coverage,height_error,rotation_error=row
                c=Candidate(i,pose,arm);c.bearing=bearing;c.coverage=coverage;c.height_error=height_error;c.rotation_error=rotation_error;c.travel=float(cost)
                cell=field.cells(transform_points([pose[:2]],tf))[0]
                centre_cost=min(int(grid.data[cell[1],cell[0]]),253)/253
                c.preliminary=self.score_candidate(c,c.travel,centre_cost,goal);candidates.append(c)
            # At an identical base pose, choose the best arm preference before planning.
            unique={}
            for c in sorted(candidates,key=lambda c:(c.preliminary,c.id)):unique.setdefault(c.pose,c)
            candidates=diverse_order(list(unique.values())[:self.cfg['ranking_pool_size']],self.cfg['diversity_distance'],self.cfg['diversity_angle'])
            self.publish_markers(candidates);timings['nav2_seconds']=timings['path_check_seconds']=timings['destination_seconds']=0.
            for c in candidates:
                self.checkpoint()
                if time.monotonic()>=search_end or paths>=self.cfg['max_path_checks'] or destinations>=self.cfg['max_destination_checks'] or len(feasible)>=self.cfg['target_feasible_paths']:break
                self.guard_motion(start,joints)
                phase=time.monotonic();ok,reason,cost=self.polygon_check(transform_points(footprint,c.pose),maps);destinations+=1
                timings['destination_seconds']+=time.monotonic()-phase
                if not ok:c.state=reason;continue
                c.destination_cost=cost;c.state='destination_clear'
                self.status('planning',f'Checking Nav2 path {paths+1}/{self.cfg["max_path_checks"]}; no online IK')
                phase=time.monotonic()
                try:poses,reason=self.compute_path(c,start,search_end)
                except Cancelled:raise
                except Exception:
                    if self.plan_pending or not feasible:raise
                    break
                timings['nav2_seconds']+=time.monotonic()-phase;paths+=1
                if poses is None:c.state=reason;continue
                phase=time.monotonic()
                try:ok,reason,path_cost=self.check_path(poses,footprint,maps,search_end)
                except CheckingBudgetExhausted:break
                timings['path_check_seconds']+=time.monotonic()-phase
                if not ok:c.state='path_'+reason;continue
                c.path=poses;c.length=path_length(poses);c.cost=path_cost
                c.score=self.score_candidate(c,c.length*(1+self.cfg['travel_inflation_weight']*path_cost),c.destination_cost,goal)
                c.state='path_checked';feasible.append(c);self.publish_markers(candidates)
            if not feasible:raise SelectionError(f'No validated path within limits; volume_candidates={len(rows)}, destination_checks={destinations}, paths={paths}, travel_field_truncated={field.truncated}')
            phase=time.monotonic();self.guard_motion(start,joints);self.cache_context()
            maps,footprint=self.costmaps(deadline),self.footprint_snapshot();fresh=time.monotonic();chosen=None
            for c in sorted(feasible,key=lambda c:c.score):
                self.remaining(deadline,1.)
                ok,reason,cost=self.polygon_check(transform_points(footprint,c.pose),maps)
                if not ok:continue
                ok,reason,path_cost=self.check_path(c.path,footprint,maps,deadline)
                if ok:
                    c.destination_cost=cost;c.cost=path_cost;c.score=self.score_candidate(c,c.length*(1+self.cfg['travel_inflation_weight']*path_cost),cost,goal);chosen=c;break
            if chosen is None:raise SelectionError('All shortlisted paths invalidated by fresh costmaps')
            self.remaining(deadline,1.);self.guard_motion(start,joints)
            if time.monotonic()-fresh>self.cfg['max_data_age']:raise SelectionError('Final costmaps aged out')
            chosen.state='selected';timings['final_validation_seconds']=time.monotonic()-phase
            pose=pose_msg(chosen.pose,self.cfg['fixed_frame']);pose.header.stamp=self.get_clock().now().to_msg()
            path=NavPath();path.header=copy.deepcopy(pose.header);path.poses=[pose_msg(p,self.cfg['fixed_frame']) for p in chosen.path]
            result={'request_id':self.sequence,'valid':True,'frame_id':self.cfg['fixed_frame'],'arm':chosen.arm,'pose':list(chosen.pose),'score':chosen.score,
                    'path_length_m':chosen.length,'path_cost':chosen.cost,'destination_cost':chosen.destination_cost,
                    'cache_id':self.volume.meta['cache_id'],'reachability_evidence':'sampled_volume_lookup','online_ik_checks':0,
                    'orientation_coverage':chosen.coverage,'height_error_m':chosen.height_error,'orientation_error_deg':math.degrees(chosen.rotation_error),
                    'volume_candidates':len(rows),'ranking_pool':len(candidates),'destination_checks':destinations,'path_checks':paths,'feasible_paths':len(feasible),
                    'travel_field_truncated':field.truncated,'timings':timings,'elapsed_seconds':time.monotonic()-started,'valid_for_seconds':self.cfg['selection_ttl'],
                    'reachability_policy':'self_body_ignore_other_arm_v1','cached_self_body_collision_checked':True,'opposite_arm_collision_checked':False,'optimality_guaranteed':False,'exact_target_ik_verified':False,'environment_arm_collision_checked':False,'combined_arm_collision_checked':False,'joint_solutions':[]}
            with self.lock:
                self.checkpoint();self.selection_deadline=time.monotonic()+self.cfg['selection_ttl'];self.selected_context=(start,joints)
                self.result_pub.publish(String(data=json.dumps(result,allow_nan=False)));self.path_pub.publish(path);self.pose_pub.publish(pose);self.valid_pub.publish(Bool(data=True));self.publish_markers(candidates)
                self.status('selected',f'{chosen.arm}: {chosen.length:.2f} m path; cached volume, zero online IK',**result)
        except Exception as e:
            if rclpy.ok():
                self.invalidate();self.publish_markers(candidates);self.status('cancelled' if isinstance(e,Cancelled) else 'failed',str(e),timings=timings,elapsed_seconds=time.monotonic()-started)
        finally:
            with self.lock:self.busy=False


def main(args=None):
    rclpy.init(args=args);node=None;executor=SingleThreadedExecutor()
    try:
        node=VolumeSelector();executor.add_node(node);executor.spin()
    except CacheStartupError as error:
        logger=get_logger('mur_base_selector')
        (logger.warning if error.exit_code==0 else logger.error)(str(error))
        return error.exit_code
    except KeyboardInterrupt:pass
    finally:
        if node:node.stop.set()
        if rclpy.ok():rclpy.shutdown()
        if node:
            if node.worker:node.worker.join()
            node.tf_receiver.close();node.destroy_node()
        executor.shutdown(timeout_sec=2.)
