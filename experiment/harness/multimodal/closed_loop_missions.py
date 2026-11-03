#!/usr/bin/env python3
"""Send route goals to both EGO planners and measure actual continuous flights."""
import argparse
import copy
import csv
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import xmlrpc.client

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline
from telemetry_contract import (actual_odometry_topic, actual_odometry_caller,
                                actual_odometry_description)

p=argparse.ArgumentParser();p.add_argument('--route',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--frontend',choices=('lidar','vio'),default='lidar')
p.add_argument('--laps',type=int);p.add_argument('--navigation-only',action='store_true')
p.add_argument('--central-telemetry',action='store_true',
               help='Consume the source-authenticated hub ingress copy of actual body state')
# Harness-only simulation guard: 2 * (0.2m body sphere + 0.2m assumed tracking
# envelope) + 0.2m extra = 1.0m. The former 1.8m default had no stated envelope
# basis and aborted a non-colliding native EGO traversal. No EGO parameter changes.
p.add_argument('--minimum-separation',type=float,default=1.0)
p.add_argument('--motion-contract-seconds',type=float,default=10.)
a=p.parse_args(rospy.myargv()[1:])
if any(not math.isfinite(v) or v<=0 for v in (a.minimum_separation,a.motion_contract_seconds)):p.error('Safety limits must be positive and finite')
route=json.loads(a.route.read_text());laps=a.laps or route.get('recommended_laps',2)
if laps<1 or not route.get('stages'):p.error('Positive laps and nonempty stages required')
rospy.init_node('two_host_closed_loop_missions',disable_signals=True)
a.output.parent.mkdir(parents=True,exist_ok=True)
trace_path=a.output.with_suffix('.trajectory.csv');stream=trace_path.open('w',newline='');writer=csv.writer(stream)
writer.writerow(['wall_time','stamp_ns','robot_id','x','y','z','yaw_rad'])
lock=threading.RLock();trace_lock=threading.Lock();report_lock=threading.Lock()
poses=[None,None];yaws=[None,None];received=[0.,0.];estimate_received=[0.,0.];ready=[False,False]
first=[None,None];length=[0.,0.];trace_last=[0.,0.];stage_history=[];lap_history=[]
minimum_separation=math.inf;phase='initializing';lap=0;stage_name=None
velocities=[None,None];command_received=[0.,0.];commands=[deque(maxlen=300),deque(maxlen=300)]
body_statuses=[None,None];body_status_received=[0.,0.]
command_diagnostics=[dict(maximum_remote_receipt_gap_s=0.,remote_gap_over_0p4_checks=0,
    body_confirmed_stale_checks=0,unconfirmed_over_2s_checks=0,
    ignored_body_status_source_age_messages=0,body_no_command_first_source_time=None,
    last_health=None) for _ in (0,1)]
local_splines=[deque(maxlen=64),deque(maxlen=64)];broadcast_splines=[deque(maxlen=64),deque(maxlen=64)]
failure=None;stage_contract=None;last_chain_check={};stop_result={}
abort_event=threading.Event();finished=threading.Event();stop_done=threading.Event();stop_lock=threading.Lock()
CONTROL_NODES=['/drone_%d_%s'%(i,role) for i in (0,1) for role in ('traj_server','ego_planner_node')]
GUARD_BASIS=dict(body_sphere_radius_m_per_robot=.2,assumed_tracking_envelope_m_per_robot=.2,
    pair_body_plus_tracking_budget_m=.8,additional_margin_m=.2,default_center_distance_guard_m=1.,
    revision='Harness default revised from 1.8m, which had no stated vehicle-envelope basis; nav1753 guard abort is not classified as an EGO collision.',
    ego_swarm_clearance_parameters_changed=False,prediction_horizon_seconds=.75,
    scope='Simulation envelope assumption and execution safeguard; not a physical-flight safety certification')
started=time.time();run_id=a.output.stem
pubs=[rospy.Publisher('/two_pc/drone_%d/goal'%i,PoseStamped,queue_size=1) for i in (0,1)]
scene=rospy.Publisher('/two_pc/scene_state',String,queue_size=1,latch=True)

def fail(reason):
    global failure
    with lock:
        if failure is None:failure=dict(reason=str(reason),wall_time=time.time(),positions=[list(v) if v is not None else None for v in poses])
        abort_event.set()

for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda kind,*_:fail('Mission interrupted by signal '+str(kind)))

class TimedTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host);connection.timeout=.5;return connection

def rpc(uri,method,*args):
    with xmlrpc.client.ServerProxy(uri,transport=TimedTransport()) as api:
        code,message,value=getattr(api,method)(rospy.get_name(),*args)
    if code!=1:raise RuntimeError(str(message))
    return value

def master(method,*args):return rpc(os.environ['ROS_MASTER_URI'],method,*args)

def inspect_planner_chain():
    """Registration + actual planner XMLRPC subscription, never probe count alone."""
    published,subscribed,_=master('getSystemState');pubmap=dict(published);submap=dict(subscribed);checks={};nodes={}
    for i in (0,1):
        planner='/drone_%d_ego_planner_node'%i;server='/drone_%d_traj_server'%i;body='/drone_%d_continuous_body'%i
        goal='/two_pc/drone_%d/goal'%i;local='/drone_%d_planning/bspline'%i;cmd='/drone_%d_planning/pos_cmd'%i;odom='/drone_%d_visual_slam/odom'%i
        checks['robot%d_goal_master_subscriber'%i]=planner in submap.get(goal,[])
        checks['robot%d_local_chain'%i]=pubmap.get(local)==[planner] and server in submap.get(local,[])
        checks['robot%d_command_chain'%i]=pubmap.get(cmd)==[server] and body in submap.get(cmd,[])
        checks['robot%d_actual_body_unique'%i]=pubmap.get(odom)==[body] and planner in submap.get(odom,[])
        if a.central_telemetry:
            telemetry=actual_odometry_topic(i,True)
            checks['robot%d_single_hub_telemetry_source'%i]=pubmap.get(telemetry)==[actual_odometry_caller(i,True)]
            checks['robot%d_mission_uses_local_telemetry'%i]=(rospy.get_name() in submap.get(telemetry,[]) and
                                                            rospy.get_name() not in submap.get(odom,[]))
        checks['robot%d_broadcast_registered'%i]=planner in pubmap.get('/broadcast_bspline',[]) and planner in submap.get('/broadcast_bspline',[])
        uri=master('lookupNode',planner);subscriptions=dict(rpc(uri,'getSubscriptions'))
        checks['robot%d_goal_actual_planner_subscription'%i]=subscriptions.get(goal)=='geometry_msgs/PoseStamped'
        nodes[planner]=dict(uri=uri,pid=int(rpc(uri,'getPid')))
        uri=master('lookupNode',server);nodes[server]=dict(uri=uri,pid=int(rpc(uri,'getPid')))
    return dict(wall_time=time.time(),passed=all(checks.values()),checks=checks,nodes=nodes)

def stop_one_node(name):
    result={'node':name,'shutdown_requested':False,'confirmed_unreachable':False,
            'confirmation_scope':'ROS XMLRPC endpoint unreachable; not an OS PID-exit confirmation'}
    try:
        uri=master('lookupNode',name);result.update(uri=uri,pid=int(rpc(uri,'getPid')))
        try:rpc(uri,'shutdown','Closed-loop safety abort: '+str(failure));result['shutdown_requested']=True
        except Exception as error:result['shutdown_error']=repr(error)
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            try:rpc(uri,'getPid')
            except Exception:result['confirmed_unreachable']=True;break
            time.sleep(.05)
    except Exception as error:result['lookup_or_identity_error']=repr(error)
    return result

def stop_controls():
    """Only the four fixed launch nodes. Body's verified .5s stale-command hold remains alive."""
    if not stop_lock.acquire(blocking=False):stop_done.wait(8.);return
    try:
        if stop_done.is_set():return
        stop_result.update(started_wall=time.time(),policy='Shutdown both real traj_server and planner nodes; continuous_body command_timeout=.5s then actual-state PD hold. No synthetic command/trajectory is published.')
        with ThreadPoolExecutor(max_workers=4) as pool:stop_result['nodes']=list(pool.map(stop_one_node,CONTROL_NODES))
        until=time.monotonic()+4.;stable_since=None
        while time.monotonic()<until:
            with lock:
                now=time.monotonic();quiet=all(not t or now-t>.65 for t in command_received)
                stationary=all(v is not None and math.sqrt(sum(x*x for x in v))<.1 and now-received[i]<.6 for i,v in enumerate(velocities))
            if quiet and stationary:
                if stable_since is None:stable_since=now
                if now-stable_since>=.3:break
            else:stable_since=None
            time.sleep(.05)
        held=bool(stable_since is not None and time.monotonic()-stable_since>=.3)
        nodes={row['node']:row for row in stop_result['nodes']}
        servers_confirmed=all(nodes.get('/drone_%d_traj_server'%i,{}).get('confirmed_unreachable') is True for i in (0,1))
        stop_result.update(finished_wall=time.time(),commands_quiet=quiet,actual_bodies_stationary=stationary,
            stale_command_hold_observed=held,execution_cutoff_and_hold_confirmed=bool(servers_confirmed and quiet and stationary and held),
            all_control_nodes_shutdown_confirmed=all(nodes.get(name,{}).get('confirmed_unreachable') is True for name in CONTROL_NODES),
            shutdown_confirmation_scope='ROS XMLRPC shutdown/unreachable only; remote operating-system PID exit is not verified')
    finally:stop_done.set();stop_lock.release()

def checked():
    if abort_event.is_set():raise RuntimeError(failure['reason'])
    if rospy.is_shutdown():fail('ROS shutdown during mission');raise RuntimeError('ROS shutdown during mission')

def motion_certificates(now):
    if stage_contract is None:return [False,False]
    result=[]
    for i in (0,1):
        if not stage_contract['released'][i]:result.append(False);continue
        if stage_contract['certified'][i]:result.append(True);continue
        if not stage_contract['needs_motion'][i]:result.append(True);continue
        began=stage_contract['began'][i];minimum_start=stage_contract['ros_ns'][i]-50000000
        locals_={(ident,stamp) for ident,stamp,rx in local_splines[i] if rx>=began and stamp>=minimum_start}
        broadcasts={(ident,stamp) for ident,stamp,rx in broadcast_splines[i] if rx>=began and stamp>=minimum_start}
        matched=locals_&broadcasts
        execution=any(rx>=began and any(ident==key[0] for key in matched) for ident,rx in commands[i])
        moved=poses[i] is not None and math.dist(poses[i],stage_contract['origins'][i])>=.15
        stage_contract['certified'][i]|=bool(matched and execution and moved)
        if stage_contract['certified'][i]:stage_contract['certified_source_time'][i]=rospy.Time.now().to_sec()
        result.append(stage_contract['certified'][i])
    return result

def leader_gate_ready(leader,follower,goals):
    """Observe native execution and forward actual motion, never synthesize a path."""
    certificates=motion_certificates(time.monotonic())
    origin=stage_contract['origins'][leader];direction=[goals[leader][j]-origin[j] for j in range(3)]
    norm=math.sqrt(sum(v*v for v in direction))
    if norm<1e-6:return False,{'reason':'Leader goal has no forward direction'}
    direction=[v/norm for v in direction]
    progress=sum((poses[leader][j]-origin[j])*direction[j] for j in range(3))
    headway=sum((poses[leader][j]-poses[follower][j])*direction[j] for j in range(3))
    initial_headway=sum((stage_contract['gate_origins'][leader][j]-stage_contract['gate_origins'][follower][j])*direction[j] for j in range(3))
    separation=math.dist(poses[leader],poses[follower])
    initial_separation=math.dist(*stage_contract['gate_origins'])
    checks=dict(native_execution_certified=certificates[leader],forward_actual_progress_at_least_0p3m=progress>=.3,
                headway_gained_at_least_0p2m=headway>=initial_headway+.2,
                center_separation_gained_at_least_0p2m=separation>=initial_separation+.2)
    return all(checks.values()),dict(checks=checks,forward_progress_m=progress,headway_m=headway,
        initial_headway_m=initial_headway,center_separation_m=separation,initial_center_separation_m=initial_separation)

def release_goal(i,goals,tolerance):
    """Per-robot contract begins only when that robot's real goal is issued."""
    checked()
    m=PoseStamped();m.header.stamp=rospy.Time.now();m.header.frame_id=route.get('frame_id','world');m.pose.orientation.w=1
    m.pose.position.x,m.pose.position.y,m.pose.position.z=goals[i]
    with lock:
        stage_contract['began'][i]=time.monotonic();stage_contract['ros_ns'][i]=m.header.stamp.to_nsec()
        stage_contract['origins'][i]=list(poses[i]);stage_contract['needs_motion'][i]=math.dist(poses[i],goals[i])>tolerance
        stage_contract['released'][i]=True
        stage_contract['release_events'].append(dict(robot_id=i,wall_time=time.time(),stamp_ns=m.header.stamp.to_nsec(),goal=list(goals[i])))
    pubs[i].publish(m)

def command_health(now,remote_received,status,status_received,ros_now,
                   minimum_status_source_time=None,no_command_first_source_time=None):
    """Remote receipt gaps diagnose observation; local body telemetry confirms timeout."""
    gap=max(0.,now-remote_received) if remote_received else None
    observer_age=max(0.,now-status_received) if status_received else None
    source_age=ros_now-status['time'] if status is not None else None
    fresh=bool(status is not None and observer_age is not None and observer_age<=1.5 and
               source_age is not None and -.25<=source_age<=1.5)
    active_sample=bool(fresh and (minimum_status_source_time is None or status['time']>=minimum_status_source_time))
    age=status.get('command_received_age_s') if fresh else None
    no_command=bool(active_sample and status.get('has_command') is False)
    no_command_since=(status['time'] if no_command_first_source_time is None else no_command_first_source_time) if no_command else None
    confirmed_no_command=bool(no_command and status['time']-no_command_since>=2.)
    confirmed=bool(active_sample and status.get('stale_command_hold') is True and
                   ((status.get('has_command') is True and age is not None and age>.5) or confirmed_no_command))
    unavailable=bool((gap is None or gap>2.) and
                     (observer_age is None or observer_age>2. or (source_age is not None and source_age>2.)))
    reason='body_confirmed_native_timeout' if confirmed else ('native_state_unconfirmed_for_2s' if unavailable else None)
    return dict(remote_receipt_gap_s=gap,body_status_receipt_age_s=observer_age,
        body_status_source_age_s=source_age,body_status_fresh=fresh,
        body_report_after_stage_certification=active_sample,
        body_no_command_first_source_time=no_command_since,
        body_local_command_age_s=age,body_confirmed_stale=confirmed,reason=reason,
        scope='Remote .4s receipt gap is diagnostic only; native body .5s hold unchanged')

def watchdog():
    global minimum_separation
    while not finished.wait(.05):
        with lock:
            now=time.monotonic()
            if all(v is not None for v in poses):
                distance=math.dist(*poses);minimum_separation=min(minimum_separation,distance)
                if distance<a.minimum_separation:fail('Minimum body-center separation violated: %.3fm'%distance)
                if all(v is not None for v in velocities) and all(now-t<.6 for t in received):
                    r=[poses[0][j]-poses[1][j] for j in range(3)];dv=[velocities[0][j]-velocities[1][j] for j in range(3)]
                    vv=sum(v*v for v in dv);t=max(0.,min(.75,-sum(x*v for x,v in zip(r,dv))/vv)) if vv>1e-9 else 0.
                    closest=math.sqrt(sum((x+t*v)**2 for x,v in zip(r,dv)))
                    if closest<a.minimum_separation:fail('Predicted .75s center approach below guard: %.3fm'%closest)
            if stage_contract is not None:
                if any(now-t>.6 for t in received):fail('Actual body odometry stale during stage')
                certificates=motion_certificates(now)
                for i in (0,1):
                    if stage_contract['released'][i] and now-stage_contract['began'][i]>a.motion_contract_seconds and not certificates[i]:
                        fail('Fresh native local/broadcast/poscmd plus actual motion contract missing for robot '+str(i))
                    if stage_contract['certified'][i]:
                        health=command_health(now,command_received[i],body_statuses[i],body_status_received[i],rospy.Time.now().to_sec(),
                            stage_contract['certified_source_time'][i],command_diagnostics[i]['body_no_command_first_source_time'])
                        diag=command_diagnostics[i];diag['last_health']=health;gap=health['remote_receipt_gap_s']
                        diag['body_no_command_first_source_time']=health['body_no_command_first_source_time']
                        if gap is not None:
                            diag['maximum_remote_receipt_gap_s']=max(diag['maximum_remote_receipt_gap_s'],gap)
                            diag['remote_gap_over_0p4_checks']+=gap>.4
                        if health['body_confirmed_stale']:
                            diag['body_confirmed_stale_checks']+=1
                            fail('Local body confirms native command timeout for robot '+str(i))
                        elif health['reason']=='native_state_unconfirmed_for_2s':
                            diag['unconfirmed_over_2s_checks']+=1
                            fail('Native command and body command telemetry unconfirmed for 2s for robot '+str(i))
        if abort_event.is_set():stop_controls();return

def odom(m,i):
    caller=(getattr(m,'_connection_header',None) or {}).get('callerid')
    if caller!=actual_odometry_caller(i,a.central_telemetry):
        fail('Unexpected actual-body telemetry publisher for robot '+str(i));return
    q=m.pose.pose.position;r=m.pose.pose.orientation;xyz=[q.x,q.y,q.z]
    yaw=math.atan2(2*(r.w*r.z+r.x*r.y),1-2*(r.y*r.y+r.z*r.z))
    velocity=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z]
    if not all(math.isfinite(v) for v in xyz+velocity+[yaw]):fail('Nonfinite actual body odometry');return
    row=None
    with lock:
        now=time.monotonic();received[i]=now
        if poses[i] is not None:
            step=math.dist(poses[i],xyz)
            if step>1.:fail('Discontinuous actual simulated body position for robot '+str(i))
            length[i]+=step
        poses[i]=xyz;yaws[i]=yaw
        velocities[i]=velocity
        if first[i] is None:first[i]=list(xyz)
        if now-trace_last[i]>=.095:
            row=[time.time(),m.header.stamp.to_nsec(),i,*xyz,yaw];trace_last[i]=now
    if row is not None:
        with trace_lock:
            if not stream.closed:writer.writerow(row)

def estimator(m,i):estimate_received[i]=time.monotonic()
def sensor(m,i):ready[i]=m.data

def spline(m,i=None):
    try:
        broadcast=i is None
        if broadcast:i=int(m.drone_id)
        if i not in (0,1):raise ValueError('Unknown broadcast drone ID')
        caller=(getattr(m,'_connection_header',None) or {}).get('callerid')
        if caller!='/drone_%d_ego_planner_node'%i:raise ValueError('Unexpected Bspline publisher '+str(caller))
        start=m.start_time.to_nsec();order=int(m.order)
        if start<=0 or len(m.pos_pts)<order+1 or len(m.knots)<len(m.pos_pts)+order+1:raise ValueError('Empty or invalid Bspline')
        values=[v for point in m.pos_pts for v in (point.x,point.y,point.z)]+list(m.knots)
        if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite Bspline')
        if broadcast and abs(rospy.Time.now().to_nsec()-start)>.25e9:return # Same existing EGO receive-age gate; do not count stale traffic.
        with lock:(broadcast_splines if broadcast else local_splines)[i].append((int(m.traj_id),start,time.monotonic()))
    except Exception as error:fail('Bspline evidence error: '+str(error))

def command(m,i):
    try:
        caller=(getattr(m,'_connection_header',None) or {}).get('callerid')
        if caller!='/drone_%d_traj_server'%i:raise ValueError('Unexpected poscmd publisher '+str(caller))
        if m.header.stamp.to_nsec()<=0:raise ValueError('Nonpositive command stamp')
        values=[v for vector in (m.position,m.velocity,m.acceleration) for v in (vector.x,vector.y,vector.z)]+[m.yaw,m.yaw_dot]
        if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite PositionCommand')
        with lock:command_received[i]=time.monotonic();commands[i].append((int(m.trajectory_id),command_received[i]))
    except Exception as error:fail('PositionCommand evidence error: '+str(error))

def body_status(m,i):
    try:
        caller=(getattr(m,'_connection_header',None) or {}).get('callerid')
        if caller!='/drone_%d_continuous_body'%i:raise ValueError('Unexpected body-status publisher '+str(caller))
        value=json.loads(m.data)
        if value.get('drone_id')!=i:raise ValueError('Body telemetry robot identity differs')
        age=value.get('command_received_age_s');timeout=value.get('command_timeout_s');stamp=value.get('time')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<=0:raise ValueError('Invalid body telemetry time')
        if not isinstance(timeout,(int,float)) or not math.isfinite(timeout) or abs(timeout-.5)>1e-9:raise ValueError('Body native command timeout differs from .5s')
        if not isinstance(value.get('has_command'),bool) or not isinstance(value.get('stale_command_hold'),bool):raise ValueError('Missing body command-state flags')
        if age is not None and (not isinstance(age,(int,float)) or not math.isfinite(age) or age<0):raise ValueError('Invalid body local command age')
        if value['has_command'] and age is None:raise ValueError('Active command lacks local age')
        if value['stale_command_hold'] != (not value['has_command'] or age>.5):raise ValueError('Inconsistent local hold telemetry')
        with lock:
            if not -.25<=rospy.Time.now().to_sec()-stamp<=1.5:
                command_diagnostics[i]['ignored_body_status_source_age_messages']+=1
                return
            previous=body_statuses[i]
            if previous is not None and stamp<=previous['time']:return
            body_statuses[i]=value;body_status_received[i]=time.monotonic()
    except Exception as error:fail('Body command telemetry error: '+str(error))
subs=[]
for i in (0,1):
    suffix='lidar_odometry' if a.frontend=='lidar' else 'kimera_vio_ros/odometry'
    subs += [rospy.Subscriber(actual_odometry_topic(i,a.central_telemetry),Odometry,odom,callback_args=i,
                             queue_size=1 if a.central_telemetry else 20,tcp_nodelay=True),
             rospy.Subscriber('/sim_%d/'%i+suffix,Odometry,estimator,callback_args=i,queue_size=1),
             rospy.Subscriber('/sim_%d/sensors/ready'%i,Bool,sensor,callback_args=i,queue_size=1)]
    subs += [rospy.Subscriber('/drone_%d_planning/bspline'%i,Bspline,spline,callback_args=i,queue_size=50,tcp_nodelay=True),
             rospy.Subscriber('/drone_%d_planning/pos_cmd'%i,PositionCommand,command,callback_args=i,queue_size=100,tcp_nodelay=True),
             rospy.Subscriber('/sim_%d/body/status'%i,String,body_status,callback_args=i,queue_size=2,tcp_nodelay=True)]
subs.append(rospy.Subscriber('/broadcast_bspline',Bspline,spline,queue_size=100,tcp_nodelay=True))
watcher=threading.Thread(target=watchdog,daemon=True);watcher.start()

def report(final=False,error=None,goals=None):
    global minimum_separation
    with lock:
        if all(v is not None for v in poses):minimum_separation=min(minimum_separation,math.dist(*poses))
        data=dict(run_id=run_id,scenario='broad_loop',phase=phase,lap=lap,stage=stage_name,
            wall_time=time.time(),started_wall_time=started,final=final,laps_requested=laps,
            actual_body_positions=poses,actual_body_yaws_rad=yaws,actual_path_length_m=length,
            closure_error_from_recorded_start_m=[math.dist(first[i],poses[i]) if first[i] and poses[i] else None for i in (0,1)],
            minimum_sampled_center_separation_m=minimum_separation if math.isfinite(minimum_separation) else None,
            trajectory_csv=str(trace_path),trajectory_source=actual_odometry_description(a.central_telemetry),
            actual_body_telemetry=dict(central=a.central_telemetry,
                topics=[actual_odometry_topic(i,a.central_telemetry) for i in (0,1)],
                expected_callers=[actual_odometry_caller(i,a.central_telemetry) for i in (0,1)]),
            completed_stages=stage_history,completed_laps=lap_history,error=error,
            navigation_only=a.navigation_only,minimum_separation_guard_m=a.minimum_separation,
            minimum_separation_guard_basis=GUARD_BASIS,
            native_planning_motion_contract=stage_contract,planner_chain_preflight=last_chain_check,
            native_command_observation=command_diagnostics,body_command_telemetry=body_statuses,
            native_command_watchdog_policy=dict(remote_gap_0p4_s='diagnostic only',
                local_body_timeout_s=.5,body_status_freshness_s=1.5,unconfirmed_command_and_body_timeout_s=2.,
                body_report_must_follow_stage_certification=True,no_command_confirmation_source_span_s=2.,
                local_body_timeout_unchanged=True,command_or_trajectory_synthesis=False),
            control_node_stop=dict(stop_result),failure_latch=failure,
            execution_cutoff_and_hold_confirmed=stop_result.get('execution_cutoff_and_hold_confirmed'),
            all_control_nodes_shutdown_confirmed=stop_result.get('all_control_nodes_shutdown_confirmed'))
        if goals is not None:data.update(goals=goals,goal_errors_m=[math.dist(poses[i],goals[i]) if poses[i] else None for i in (0,1)])
        data=copy.deepcopy(data)
    with report_lock:
        tmp=a.output.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False));tmp.replace(a.output)
        with trace_lock:
            if not stream.closed:stream.flush()
        scene.publish(String(data=json.dumps(data,allow_nan=False)))
    return data

def execute_stage(stage):
    global phase,stage_name,stage_contract,last_chain_check
    checked()
    last_chain_check=inspect_planner_chain()
    if not last_chain_check['passed']:raise RuntimeError('Planner subscription/execution chain failed before stage')
    stage_name=stage['name'];phase='closed_loop_flying';goals=stage['targets']
    release_order=stage.get('release_order')
    if release_order is not None and (not isinstance(release_order,list) or len(release_order)!=2 or sorted(release_order)!=[0,1] or any(isinstance(i,bool) for i in release_order)):
        raise ValueError('release_order must explicitly be [0,1] or [1,0]')
    with lock:stage_contract=dict(began=[None,None],ros_ns=[None,None],origins=[None,None],released=[False,False],
        needs_motion=[None,None],certified=[False,False],certified_source_time=[None,None],release_events=[],release_order=release_order,
        gate_origins=[list(v) for v in poses],gate_evidence=None)
    if release_order is None:
        for i in (0,1):release_goal(i,goals,stage.get('goal_tolerance_m',.35))
    else:
        leader,follower=release_order;release_goal(leader,goals,stage.get('goal_tolerance_m',.35))
        deadline=stage_contract['began'][leader]+min(10.,a.motion_contract_seconds);next_report=0.
        while True:
            checked();now=time.monotonic()
            if now>=deadline:
                fail('Leader release gate timed out; follower goal was not sent');checked()
            with lock:
                release,evidence=leader_gate_ready(leader,follower,goals);stage_contract['gate_evidence']=evidence
            if release:
                release_goal(follower,goals,stage.get('goal_tolerance_m',.35));break
            if now>=next_report:report(goals=goals);next_report=now+.5
            time.sleep(.05)
    print(json.dumps(dict(lap=lap,stage=stage_name,event='goals_sent',goals=goals,wall=time.time())),flush=True)
    began=time.monotonic();settled=None;last_report=0.
    while not rospy.is_shutdown():
        checked()
        now=time.monotonic()
        with lock:
            if any(now-v>2. for v in received):raise RuntimeError('Actual body odometry stopped')
            errors=[math.dist(poses[i],goals[i]) for i in (0,1)]
        if now-last_report>=1.:report(goals=goals);last_report=now
        if max(errors)<stage.get('goal_tolerance_m',.35):
            if settled is None:settled=now
            if now-settled>=stage.get('settle_seconds',2.) and all(motion_certificates(now)):break
        else:settled=None
        if now-began>stage.get('timeout_seconds',120):raise RuntimeError('Both EGO goals not reached: '+stage_name+' '+str(errors))
        time.sleep(.1)
    if rospy.is_shutdown():raise RuntimeError('Mission interrupted')
    dwell=stage.get('dwell_seconds',stage.get('post_arrival_dwell_seconds',4.))
    phase='closed_loop_waypoint_hover';until=time.monotonic()+dwell
    while time.monotonic()<until and not rospy.is_shutdown():checked();report(goals=goals);time.sleep(.1)
    checked()
    with lock:row=dict(lap=lap,stage=stage_name,event='arrived',positions=[list(v) for v in poses],
        goal_errors_m=[math.dist(poses[i],goals[i]) for i in (0,1)],wall=time.time())
    stage_history.append(row);print(json.dumps(row),flush=True)

try:
    deadline=time.monotonic()+75
    while not rospy.is_shutdown():
        checked()
        now=time.monotonic()
        try:last_chain_check=inspect_planner_chain()
        except Exception as error:last_chain_check=dict(passed=False,error=repr(error),wall_time=time.time())
        mapping_ready=a.navigation_only or (all(ready) and all(now-v<2. for v in estimate_received))
        if all(v is not None for v in poses) and all(now-v<.6 for v in received) and mapping_ready and last_chain_check['passed']:break
        if now>deadline:raise RuntimeError('Two local bodies, estimators, sensors and planners required before flight')
        report();time.sleep(.5)
    for i in (0,1):
        if math.dist(poses[i],route['starts'][i])>.2:raise RuntimeError('Route starts differ from stationary initialization')
    time.sleep(2)
    for lap in range(1,laps+1):
        for stage in route['stages']:execute_stage(stage)
        with lock:lap_history.append(dict(lap=lap,wall=time.time(),closure_errors_m=[math.dist(poses[i],route['starts'][i]) for i in (0,1)],path_length_m=list(length)))
        phase='lap_complete';report()
        print(json.dumps(dict(event='position_loop_complete',**lap_history[-1])),flush=True)
    phase='closed_loops_complete_hovering';report(final=True)
    print(json.dumps(dict(event=phase,wall=time.time(),completed_laps=lap_history)),flush=True)
except Exception as e:
    fail(str(e));stop_controls();phase='closed_loop_needs_attention';report(final=True,error=str(e));raise
finally:
    finished.set();watcher.join(timeout=8.)
    with trace_lock:stream.close()
