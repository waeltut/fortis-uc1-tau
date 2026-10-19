"""Read-only workspace and selection diagnostics for Lichtblick/RViz."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA,String
from visualization_msgs.msg import Marker,MarkerArray
from .cache_startup import DEFAULT_VOLUME_FILE,load_selector_volume,CacheStartupError
from .visual_data import voxel_layers

COLOURS={'left':(.1,.6,1.),'right':(1.,.4,.05),'intersection':(.7,.2,1.)}


def marker(frame,namespace,kind,ident=0):
    m=Marker();m.header.frame_id=frame;m.ns=namespace;m.id=ident;m.type=kind;m.action=Marker.ADD
    m.pose.orientation.w=1.
    return m


def clear():return Marker(action=Marker.DELETEALL)


class VolumeVisualizer(Node):
    def __init__(self):
        super().__init__('mur_volume_visualizer')
        defaults={'volume_file':DEFAULT_VOLUME_FILE,'minimum_coverage':.9,'arm':'both','profile_angle_deg':-1.0,
                  'z_min':-10.,'z_max':10.,'alpha':.45,'voxel_scale':.85,
                  'max_voxels_per_arm':20000,'show_voxels':True}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.volume=load_selector_volume(self.get_parameter('volume_file').value)
        self.frame=self.volume.meta['context']['base_frame']
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.voxel_pubs={k:self.create_publisher(MarkerArray,'/base_selection/workspace/'+k,qos) for k in COLOURS}
        self.label_pub=self.create_publisher(MarkerArray,'/base_selection/workspace/labels',qos)
        self.candidate_pubs={k:self.create_publisher(MarkerArray,'/base_selection/'+k+'_candidates',qos) for k in ('cache','travel','shortlist')}
        self.diag_pub=self.create_publisher(MarkerArray,'/base_selection/diagnostic_labels',qos)
        self.subscription=self.create_subscription(String,'/base_selection/visualization_data',self.candidates,qos)
        self.last_settings=None;self.timer=self.create_timer(2.,self.voxels)
        self.voxels()
        self.get_logger().info('Workspace loaded; visualisation only. Blue=left, orange=right, purple=intersection. Display parameters can change live.')

    def voxels(self):
        settings=tuple(self.get_parameter(k).value for k in ('minimum_coverage','z_min','z_max','arm','max_voxels_per_arm','alpha','voxel_scale','show_voxels','profile_angle_deg'))
        if settings==self.last_settings:return
        minimum,zmin,zmax,arm,limit,alpha,scale,show,profile_angle=settings
        if profile_angle==-1.:profile_angle=float(self.volume.meta['generation']['min_inward_angle_deg'])
        try:
            if not all(math.isfinite(v) for v in (minimum,zmin,zmax,alpha,scale)) or not 0<alpha<=1 or not 0<scale<=1:raise ValueError('Invalid alpha/scale or nonfinite value')
            layers=voxel_layers(self.volume,minimum,zmin,zmax,arm,limit,profile_angle) if show else []
        except ValueError as e:
            self.get_logger().error(str(e));self.last_settings=settings;return
        self.last_settings=settings
        messages={k:[clear()] for k in COLOURS};lines=[]
        for name,points,coverage,total in layers:
            m=marker(self.frame,name,Marker.CUBE_LIST);m.frame_locked=True
            m.scale.x=m.scale.y=m.scale.z=float(self.volume.meta['generation']['voxel_size'])*scale
            m.color.a=float(alpha)
            for p,c in zip(points,coverage):
                m.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
                rgb=COLOURS[name];brightness=.3+.7*float(c)
                m.colors.append(ColorRGBA(r=rgb[0]*brightness,g=rgb[1]*brightness,b=rgb[2]*brightness,a=float(alpha)))
            messages[name].append(m);lines.append(f'{name}: {len(points)}/{total} voxels displayed')
        for name,pub in self.voxel_pubs.items():pub.publish(MarkerArray(markers=messages[name]))
        text=marker(self.frame,'workspace_info',Marker.TEXT_VIEW_FACING);text.frame_locked=True;text.pose.position.z=2.
        text.scale.z=.07;text.color=ColorRGBA(r=1.,g=1.,b=1.,a=1.)
        text.text=f'Cached workspace: {profile_angle:g} deg profile, coverage >= {minimum:.0%}\n'+ '\n'.join(lines)+'\nBrightness = coverage; no live arm collision check'
        self.label_pub.publish(MarkerArray(markers=[clear(),text]))

    def candidates(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        frame=data['frame'];lines=[f"Request {data['request_id']} | {data['stage']}"]
        for stage,pub in self.candidate_pubs.items():
            snapshot=data.get(stage);markers=[clear()]
            if snapshot:
                counts=snapshot['counts'];lines.append(f"{stage}: {counts['total']} poses; {counts['forward_30deg']} within +/-30 deg; displayed {snapshot['displayed']}")
                m=marker(frame,stage,Marker.LINE_LIST);m.scale.x=.012;m.color.a=1.
                z={'cache':.08,'travel':.11,'shortlist':.14}[stage]
                for x,y,yaw,arm,bearing in snapshot['poses']:
                    # Heading segment: green forward, yellow oblique, magenta sideways/back.
                    rgb=(.1,1.,.1) if abs(bearing)<=math.pi/6+1e-9 else ((1.,.8,0.) if abs(bearing)<=math.pi/3 else (1.,.1,.7))
                    colour=ColorRGBA(r=rgb[0],g=rgb[1],b=rgb[2],a=.8)
                    end=(x+.18*math.cos(yaw),y+.18*math.sin(yaw))
                    for a,b in (((x,y),end),(end,(end[0]-.06*math.cos(yaw-.5),end[1]-.06*math.sin(yaw-.5))),(end,(end[0]-.06*math.cos(yaw+.5),end[1]-.06*math.sin(yaw+.5)))):
                        m.points.extend([Point(x=float(a[0]),y=float(a[1]),z=z),Point(x=float(b[0]),y=float(b[1]),z=z)]);m.colors.extend([colour,colour])
                markers.append(m)
            pub.publish(MarkerArray(markers=markers))
        if 'cache' in data:lines.append('Passing profiles (arm:angle): '+str(data['cache'].get('profile_counts',{})))
        if 'travel_field_truncated' in data:lines.append('Travel field truncated: '+str(data['travel_field_truncated']))
        lines.append('Green: forward | yellow: oblique | magenta: sideways/back\nSnapshots for diagnosis, not navigation clearance')
        label=marker(frame,'selection_info',Marker.TEXT_VIEW_FACING);label.scale.z=.07;label.color=ColorRGBA(r=1.,g=1.,b=1.,a=1.)
        label.pose.position.x=float(data['target'][0]);label.pose.position.y=float(data['target'][1]);label.pose.position.z=float(data['target'][2])+.5
        label.text='\n'.join(lines);self.diag_pub.publish(MarkerArray(markers=[clear(),label]))


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=VolumeVisualizer();rclpy.spin(node)
    except CacheStartupError as e:
        rclpy.logging.get_logger('mur_volume_visualizer').error(str(e));return e.exit_code
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
    return 0
