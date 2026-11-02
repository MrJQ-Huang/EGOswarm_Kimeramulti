#!/usr/bin/env python3
"""Read-only world-frame views of actual body, native VIO and joint DPGO.

All publishers are restricted to /visualization/. No TF, control, odometry,
SLAM graph, goal or map input is published. Native VIO is aligned once per
robot using its first observed pose and the declared initialization. Joint
optimized camera poses are converted to body poses and share one immutable
world gauge from robot 0's first optimized key zero. No per-robot optimized
alignment, ICP or trajectory fitting is performed.
"""
import argparse
from collections import Counter
import copy
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
from native_vio_graph_adapter import SIM_T_BODY_CAMERA, pose_matrix, set_pose, validate_transform


PREFIX='/visualization/swarm'
COLORS=((52/255.,216/255.,237/255.),(1.,160/255.,75/255.))


def visualization_topic(suffix):
    topic=PREFIX+'/'+suffix
    if not topic.startswith('/visualization/') or '..' in topic:raise ValueError('Visualization-only outputs required')
    return topic


def initial_transform(position,yaw):
    c,s=math.cos(yaw),math.sin(yaw);result=np.eye(4)
    result[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];result[:3,3]=position
    return result


def sampled_full_extent(rows,maximum):
    if len(rows)<=maximum:return rows
    indices=np.unique(np.rint(np.linspace(0,len(rows)-1,maximum)).astype(int))
    return [rows[index] for index in indices]


def dashed_segments(positions,on=.18,off=.14):
    """Metric dashes keep the same appearance with uneven ROS sample rates."""
    points=[];progress=0.;period=on+off
    for left,right in zip(positions,positions[1:]):
        delta=right-left;length=float(np.linalg.norm(delta))
        if length<1e-10:continue
        along=0.
        while along<length-1e-10:
            phase=progress%period
            visible=phase<on
            remaining=(on-phase) if visible else (period-phase)
            step=min(length-along,max(remaining,1e-9))
            if visible:points.extend([left+delta*(along/length),left+delta*((along+step)/length)])
            along+=step;progress+=step
    return points


class TrajectoryState:
    def __init__(self,run_id,cutoff,starts,yaws=(0.,0.),max_points=30000,
                 sample_period=.08,minimum_distance=.025,body_camera=SIM_T_BODY_CAMERA):
        starts=np.asarray(starts,float)
        if starts.shape!=(2,3) or not np.isfinite(starts).all():raise ValueError('Exactly two finite start positions required')
        if len(yaws)!=2 or not np.isfinite(yaws).all():raise ValueError('Two finite initial yaws required')
        if not math.isfinite(cutoff) or cutoff<=0 or max_points<100:raise ValueError('Invalid epoch or path capacity')
        self.run_id,self.cutoff=run_id,float(cutoff);self.cutoff_ns=int(cutoff*1e9)
        self.initial=[initial_transform(starts[i],float(yaws[i])) for i in (0,1)]
        self.body_camera=validate_transform(body_camera);self.camera_body=np.linalg.inv(self.body_camera)
        self.maximum=int(max_points);self.period_ns=int(sample_period*1e9);self.distance=float(minimum_distance)
        self.paths={(kind,i):[] for kind in ('actual','vio','optimized') for i in (0,1)}
        self.latest_body={};self.raw_gauges={};self.raw_anchors={};self.optimized_gauge=None;self.optimized_anchor=None
        self.pending_optimized={};self.last_input_stamps={};self.rejected=Counter();self.accepted=Counter();self.decimations=Counter();self.optimized_replacements=Counter()
        self.started_wall=time.time();self.lock=threading.RLock()

    def reject(self,reason):self.rejected[reason]+=1;return False

    def ingest_odometry(self,robot,kind,stamp,transform,caller,frame):
        expected=('/drone_%d_continuous_body' if kind=='actual' else '/sim_%d/kimera_vio_ros/kimera_vio_ros_node')%robot
        with self.lock:
            if caller!=expected:return self.reject(kind+'_wrong_caller')
            if int(stamp)<self.cutoff_ns:return self.reject(kind+'_pre_epoch_or_zero_stamp')
            if kind=='actual' and frame.lstrip('/')!='world':return self.reject('actual_wrong_frame')
            if kind not in ('actual','vio'):raise ValueError('Invalid odometry role')
            transform=validate_transform(transform)
            old=self.last_input_stamps.get((kind,robot))
            if old is not None and int(stamp)<=old:return self.reject(kind+'_nonincreasing_stamp')
            self.last_input_stamps[kind,robot]=int(stamp)
            self.accepted[kind+'_'+str(robot)]+=1
            if kind=='actual':
                self.latest_body[robot]=(int(stamp),transform.copy())
                shown=transform
            else:
                if robot not in self.raw_gauges:
                    self.raw_gauges[robot]=self.initial[robot]@np.linalg.inv(transform)
                    self.raw_anchors[robot]={'stamp_ns':int(stamp),'first_native_pose':transform.tolist()}
                shown=self.raw_gauges[robot]@transform
            rows=self.paths[kind,robot]
            should_sample=(not rows or (stamp-rows[-1][0]>=self.period_ns and
                (np.linalg.norm(shown[:3,3]-rows[-1][1][:3,3])>=self.distance or stamp-rows[-1][0]>=int(2e9))))
            if should_sample:
                if len(rows)>=self.maximum:
                    rows=sampled_full_extent(rows,max(2,self.maximum//2));self.paths[kind,robot]=rows;self.decimations[kind+'_'+str(robot)]+=1
                rows.append((int(stamp),shown.copy()))
            return True

    def ingest_optimized(self,robot,nodes,caller):
        expected='/sim_%d/distributed_loop_closure/distributed_loop_closure_node'%robot
        with self.lock:
            if caller!=expected:return self.reject('optimized_wrong_caller')
            poses=[]
            for key,stamp,camera_pose in nodes:
                if int(stamp)<self.cutoff_ns:
                    self.reject('optimized_pre_epoch_or_zero_stamp');continue
                body=validate_transform(camera_pose)@self.camera_body
                poses.append((int(key),int(stamp),body))
            poses.sort(key=lambda row:row[0])
            if not poses:return False
            if len({row[0] for row in poses})!=len(poses):return self.reject('optimized_duplicate_key')
            self.pending_optimized[robot]=poses
            if self.optimized_gauge is None and robot==0:
                anchor=next((row for row in poses if row[0]==0),None)
                if anchor is None:return self.reject('optimized_waiting_for_actual_sim0_key_zero')
                self.optimized_gauge=self.initial[0]@np.linalg.inv(anchor[2])
                self.optimized_anchor={'robot_id':0,'key':0,'stamp_ns':anchor[1],'native_camera_pose':(anchor[2]@self.body_camera).tolist(),'canonical_body_pose':anchor[2].tolist()}
            if self.optimized_gauge is not None:
                for owner,values in self.pending_optimized.items():
                    rows=[(stamp,self.optimized_gauge@body) for _,stamp,body in values]
                    self.paths['optimized',owner]=sampled_full_extent(rows,self.maximum)
                    self.optimized_replacements[owner]+=1
            self.accepted['optimized_'+str(robot)]+=1
            return True

    def snapshot(self):
        with self.lock:
            return {'paths':{key:[(stamp,matrix.copy()) for stamp,matrix in rows] for key,rows in self.paths.items()},
                    'bodies':{i:(stamp,matrix.copy()) for i,(stamp,matrix) in self.latest_body.items()}}

    def status(self):
        with self.lock:
            return {'run_id':self.run_id,'epoch_cutoff_wall':self.cutoff,'started_wall':self.started_wall,'wall_time':time.time(),
                'role':'read_only_trajectory_display','frame':'world','estimator_inputs_published':False,'TF_published':False,
                'implementation_label':'Read-only actual-body and native VIO/LCD/DPGO trajectory display; map production is a separate component',
                'actual_source':'Actual simulated body odometry; not planned commands',
                'raw_vio_alignment':'Once per robot: declared world initialization times inverse(first observed native VIO body pose)',
                'optimized_alignment':'One shared frozen gauge from sim0 first optimized camera key0, converted to body; never align sim1 independently',
                'alignment_only_affects_visualization':True,'initial_world_body_poses':[x.tolist() for x in self.initial],
                'body_T_camera':self.body_camera.tolist(),'raw_vio_display_gauges':{str(i):x.tolist() for i,x in self.raw_gauges.items()},
                'raw_vio_anchor_evidence':self.raw_anchors,'shared_optimized_world_gauge':None if self.optimized_gauge is None else self.optimized_gauge.tolist(),
                'shared_optimized_anchor':self.optimized_anchor,'point_capacity_per_path':self.maximum,
                'bounded_memory_policy':'At capacity, uniformly thin stored samples while retaining first/last and complete elapsed extent; never keep only a sliding tail',
                'path_points':{kind+'_'+str(i):len(rows) for (kind,i),rows in self.paths.items()},
                'accepted_messages':dict(self.accepted),'rejected_messages':dict(self.rejected),'memory_decimations':dict(self.decimations),
                'optimized_path_replacements':dict(self.optimized_replacements),
                'styles':{'actual':'thick solid line','vio':'thin metric dashed line','optimized':'points'},
                'colors':{'AMOV':'0x34d8ed','UROS':'0xffa04b'},
                'last_input_header_ns':{kind+'_'+str(i):stamp for (kind,i),stamp in self.last_input_stamps.items()}}


def load_initialization(args):
    scene=json.loads(args.scene.read_text()) if args.scene else {}
    if args.starts is not None:starts=json.loads(args.starts)
    elif 'starts' in scene:starts=scene['starts']
    else:
        modes=scene.get('scenarios',{});name=args.scenario or ('broad_loop' if 'broad_loop' in modes else next(iter(modes)) if len(modes)==1 else None)
        if name not in modes:raise ValueError('Scene needs an unambiguous scenario with starts, or explicit --starts')
        starts=modes[name]['starts']
    yaws=json.loads(args.initial_yaws) if args.initial_yaws else scene.get('initial_yaw_rad',[0.,0.])
    return starts,yaws


def main():
    import rospy
    from geometry_msgs.msg import Point,PoseStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from pose_graph_tools_msgs.msg import PoseGraph
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker,MarkerArray
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True);parser.add_argument('--cutoff',type=float,required=True)
    parser.add_argument('--scene',type=Path);parser.add_argument('--scenario');parser.add_argument('--starts',help='JSON two XYZ initial positions')
    parser.add_argument('--initial-yaws',help='JSON two initial yaw angles in radians')
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--max-points',type=int,default=30000)
    parser.add_argument('--publish-rate',type=float,default=2.);parser.add_argument('--calibration-json')
    args=parser.parse_args(rospy.myargv()[1:]);starts,yaws=load_initialization(args)
    if not 0<args.publish_rate<=10:parser.error('Visualization publish rate must be in (0,10] Hz')
    calibration=SIM_T_BODY_CAMERA if args.calibration_json is None else json.loads(args.calibration_json)
    state=TrajectoryState(args.run_id,args.cutoff,starts,yaws,args.max_points,body_camera=calibration)
    rospy.init_node('live_swarm_trajectory_view')
    def publisher(suffix,kind):return rospy.Publisher(visualization_topic(suffix),kind,queue_size=1,latch=True)
    paths={(kind,i):publisher(kind+'_'+str(i),RosPath) for kind in ('actual','vio','optimized') for i in (0,1)}
    marker_pubs={kind:publisher(kind+'_markers',MarkerArray) for kind in ('actual','vio','optimized')}
    aggregate_pub=publisher('markers',MarkerArray);vehicle_pub=publisher('vehicles',MarkerArray);environment_pub=publisher('environment',MarkerArray);status_pub=publisher('status',String)
    environment=[];environment_lock=threading.RLock();errors=Counter();subscriptions=[]

    def on_odometry(robot,kind,message):
        try:state.ingest_odometry(robot,kind,message.header.stamp.to_nsec(),pose_matrix(message.pose.pose),message._connection_header.get('callerid'),message.header.frame_id)
        except Exception as error:errors[kind+'_callback']+=1;rospy.logwarn_throttle(5.,'Trajectory display rejected %s: %s',kind,error)

    def on_optimized(robot,message):
        try:
            nodes=[(int(node.key),node.header.stamp.to_nsec(),pose_matrix(node.pose)) for node in message.nodes if int(node.robot_id)==robot]
            state.ingest_optimized(robot,nodes,message._connection_header.get('callerid'))
        except Exception as error:errors['optimized_callback']+=1;rospy.logwarn_throttle(5.,'Trajectory display rejected optimized graph: %s',error)

    def on_environment(message):
        if message._connection_header.get('callerid')!='/two_pc_complex_world_and_view':errors['environment_wrong_caller']+=1;return
        transformed=[]
        for item in message.markers:
            if item.action!=Marker.ADD or item.type==Marker.TEXT_VIEW_FACING:continue
            if item.header.frame_id.lstrip('/')!='world':errors['environment_wrong_frame']+=1;continue
            marker=copy.deepcopy(item);marker.ns='environment/'+marker.ns
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=(.40,.44,.50,.23 if marker.ns.endswith('floor') else .32)
            transformed.append(marker)
        with environment_lock:environment[:]=transformed

    for i in (0,1):
        subscriptions.append(rospy.Subscriber('/drone_%d_visual_slam/odom'%i,Odometry,lambda m,i=i:on_odometry(i,'actual',m),queue_size=100,tcp_nodelay=True))
        subscriptions.append(rospy.Subscriber('/sim_%d/kimera_vio_ros/odometry'%i,Odometry,lambda m,i=i:on_odometry(i,'vio',m),queue_size=100,tcp_nodelay=True))
        subscriptions.append(rospy.Subscriber('/sim_%d/kimera_distributed/optimized_nodes'%i,PoseGraph,lambda m,i=i:on_optimized(i,m),queue_size=2,buff_size=2**24,tcp_nodelay=True))
    subscriptions.append(rospy.Subscriber('/two_pc/complex_environment',MarkerArray,on_environment,queue_size=1))

    def marker_base(namespace,identity,kind,stamp,color):
        result=Marker();result.header.frame_id='world';result.header.stamp=stamp;result.ns=namespace;result.id=identity;result.type=kind;result.action=Marker.ADD;result.pose.orientation.w=1.
        result.color.r,result.color.g,result.color.b=color;result.color.a=1.;return result

    def point(value):return Point(x=float(value[0]),y=float(value[1]),z=float(value[2]))

    def publish(_):
        try:
            snapshot=state.snapshot();now=rospy.Time.now();all_markers=[]
            for kind in ('actual','vio','optimized'):
                array=MarkerArray()
                for i in (0,1):
                    rows=snapshot['paths'][kind,i];path=RosPath();path.header.frame_id='world';path.header.stamp=rospy.Time(rows[-1][0]//1000000000,rows[-1][0]%1000000000) if rows else now
                    for stamp,transform in rows:
                        pose=PoseStamped();pose.header.frame_id='world';pose.header.stamp=rospy.Time(stamp//1000000000,stamp%1000000000);set_pose(pose.pose,transform);path.poses.append(pose)
                    paths[kind,i].publish(path)
                    style=Marker.LINE_STRIP if kind=='actual' else Marker.LINE_LIST if kind=='vio' else Marker.POINTS
                    marker=marker_base(kind+'_'+str(i),i,style,now,COLORS[i]);coordinates=[matrix[:3,3] for _,matrix in rows]
                    if kind=='vio':coordinates=dashed_segments(coordinates)
                    marker.points=[point(value) for value in coordinates]
                    marker.scale.x=.055 if kind=='actual' else .018 if kind=='vio' else .085;marker.scale.y=.085
                    marker.color.a=.65 if kind=='vio' else .95
                    if not marker.points:marker.action=Marker.DELETE
                    array.markers.append(marker)
                marker_pubs[kind].publish(array);all_markers.extend(array.markers)
            vehicles=MarkerArray()
            for i in (0,1):
                body=snapshot['bodies'].get(i)
                for offset,kind in enumerate((Marker.CUBE,Marker.ARROW,Marker.TEXT_VIEW_FACING)):
                    marker=marker_base('actual_body_'+str(i),i*3+offset,kind,now,COLORS[i])
                    if body is None:marker.action=Marker.DELETE
                    else:
                        set_pose(marker.pose,body[1])
                        if kind==Marker.CUBE:marker.scale.x,marker.scale.y,marker.scale.z=(.38,.28,.13)
                        elif kind==Marker.ARROW:marker.scale.x,marker.scale.y,marker.scale.z=(.85,.10,.10)
                        else:
                            marker.pose.position.z+=.55;marker.pose.orientation.x=marker.pose.orientation.y=marker.pose.orientation.z=0.;marker.pose.orientation.w=1.;marker.scale.z=.23;marker.text=('AMOV' if i==0 else 'UROS')+' actual body'
                    vehicles.markers.append(marker)
            vehicle_pub.publish(vehicles);all_markers.extend(vehicles.markers)
            with environment_lock:obstacles=copy.deepcopy(environment)
            environment_pub.publish(MarkerArray(markers=obstacles));aggregate_pub.publish(MarkerArray(markers=obstacles+all_markers))
            report=state.status();report['callback_errors']=dict(errors);report['published_paths']={kind+'_'+str(i):visualization_topic(kind+'_'+str(i)) for kind in ('actual','vio','optimized') for i in (0,1)};report['publisher_node']=rospy.get_name();report['environment_markers']=len(obstacles)
            status_pub.publish(String(data=json.dumps(report,allow_nan=False)))
            args.output.parent.mkdir(parents=True,exist_ok=True);temporary=args.output.with_suffix(args.output.suffix+'.tmp');temporary.write_text(json.dumps(report,indent=2,allow_nan=False));os.replace(temporary,args.output)
        except Exception as error:errors['publish_callback']+=1;rospy.logerr_throttle(5.,'Trajectory display error: %s',error)

    timer=rospy.Timer(rospy.Duration(1./args.publish_rate),publish)
    rospy.loginfo('Read-only trajectories ready: %s; all outputs under /visualization/',args.run_id)
    rospy.spin();timer.shutdown()


if __name__=='__main__':main()
