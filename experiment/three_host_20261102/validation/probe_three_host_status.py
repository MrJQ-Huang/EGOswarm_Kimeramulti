#!/usr/bin/env python3
"""Time-bounded observer of seven small status topics; never reads raw geometry.

Does not publish, estimate poses, generate data, or subscribe to RGB-D, IMU,
point clouds or meshes. Counter progress here proves producer/status progress,
not native TSDF integration completion; use existing native completion logs too.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import threading
import time

from readonly_ros_graph import MASTER_URI,mesh_bus_delta,mesh_bus_snapshots,rpc,unwrap


def status_contracts():
    result={}
    for robot in (0,1):
        result['/sim_%d/sensors/ingress_status'%robot]=dict(kind='sensor_ingress',robot=robot,
            caller='/sim_%d_multimodal_sensors'%robot)
        result['/sim_%d/native_dense/local_sensor_status'%robot]=dict(kind='local_sensor',robot=robot,
            caller='/sim_%d/native_local_sensor_bridge'%robot)
        result['/sim_%d/native_dense/status'%robot]=dict(kind='hub_bridge',robot=robot,
            caller='/sim_%d/native_dense_bridge'%robot)
    result['/visualization/native_mesh/status']=dict(kind='global_display',caller='/native_dense_global_view')
    return result


FIELDS={
    'sensor_ingress':('lanes','fatal_error','max_frame_age_s','single_remote_sensor_receiver',
        'raw_data_unchanged','imu_independent_of_rgbd','records_raw_files','computes_pose_or_map'),
    'local_sensor':('native_local_gauge_verified','counts','timings','samples','maximum_input_age_s',
        'last_received_wall','pending_rgbd','maximum_pending_rgbd','max_pending_rgbd','max_rgbd_age_s',
        'cloud_rate_hz','gyro_maximum_sample_gap_s','gyro_maximum_used_gap_s',
        'native_vio_maximum_interpolated_gap_s','fatal_error','independent_process','consumes_mesh','consumes_global_optimizer'),
    'hub_bridge':('mode','atomic_rounds','coordinator_dispatch_seq','native_local_gauge_verified',
        'local_keyframes','graph_forwarded_through_key','counts','fatal_error',
        'native_original_priority_queue_length','native_original_priority_queue_max_length',
        'native_original_max_queue_delay_s','native_control_max_queue_delay_s','actual_gyro_max_queue_delay_s',
        'replaceable_full_mesh_overwrites','optimized_candidates_waiting','sensor_stream_in_independent_process',
        'native_deformation_eligible','mesh_transaction_phase','mesh_transaction_phase_timeout_s'),
    'coordinator':('counts','pending_native_pairs','active_dispatch_seq','active_native_request',
        'latest_committed_dispatch_seq','latest_committed_map_age_s','waiting_reason','robots_idle',
        'mesh_round_timeout_s','fatal_error'),
    'global_display':('fusion_ready','points','robot_point_counts','reason','archive_count','atomic_commit_mode',
        'committed_generation','committed_map_age_s','completed_round_serial','binding',
        'exact_shared_optimizer_round_id_available','snapshot_identity'),
}


def projected_status(value,kind):
    result={key:copy.deepcopy(value[key]) for key in FIELDS[kind] if key in value}
    if kind=='hub_bridge':
        receipt=value.get('accepted_joint_mesh_receipt') or {}
        proof=receipt.get('completed_round_proof') or {}
        deform=receipt.get('native_deformation_proof') or {}
        result['accepted_native_mesh']=dict(completed_round_serial=receipt.get('completed_round_serial'),
            optimizer_owner=receipt.get('optimizer_owner'),vertex_count=receipt.get('vertex_count'),
            accepted_wall=receipt.get('accepted_wall'),native_request_identity=proof.get('shared_native_request_identity'),
            paired_round_message_sha256=proof.get('paired_round_message_sha256'),
            final_native_weight_batch_sha256=receipt.get('final_native_weight_batch_sha256'),
            exact_shared_optimizer_round_id_available=receipt.get('exact_shared_optimizer_round_id_available'),
            historical_vertices_moved=deform.get('historical_vertices_moved'),
            maximum_vertex_motion_m=deform.get('maximum_vertex_motion_m'),
            native_path_echo_verified=deform.get('native_path_echo_verified'))
    if kind in ('coordinator','global_display'):
        errors=value.get('rejected_messages') or []
        result['rejected_messages_count']=len(errors);result['rejected_messages_tail']=copy.deepcopy(errors[-3:])
    return result


class StatusCollector:
    def __init__(self,run_id,cutoff_ns=None,max_status_age=5.):
        self.run_id,self.cutoff_ns,self.max_status_age=run_id,cutoff_ns,max_status_age
        self.contracts=status_contracts()
        self.streams={topic:dict(messages=0,bytes=0,rejections=0,latest=None,first_counts=None,
            first_lane_counts=None,first_receive_wall=None,last_receive_wall=None,errors=[])
            for topic in self.contracts}

    def receive(self,topic,data,caller,wall):
        if topic not in self.contracts:raise ValueError('Topic is outside the status whitelist')
        stream=self.streams[topic];contract=self.contracts[topic]
        stream['messages']+=1;stream['bytes']+=len(data.encode('utf-8'));stream['last_receive_wall']=wall
        stream['latest']=None
        try:
            if caller!=contract['caller']:raise ValueError('Unexpected status publisher')
            if len(data)>2**20:raise ValueError('Oversized status payload')
            def invalid_constant(value):raise ValueError('Nonfinite JSON constant '+value)
            value=json.loads(data,parse_constant=invalid_constant)
            if not isinstance(value,dict) or value.get('run_id')!=self.run_id:raise ValueError('Wrong status run ID')
            if self.cutoff_ns is not None and value.get('cutoff_ns')!=self.cutoff_ns:raise ValueError('Wrong status epoch cutoff')
            if 'robot' in contract and value.get('robot_id')!=contract['robot']:raise ValueError('Wrong status robot owner')
            producer_wall=value.get('wall')
            if not isinstance(producer_wall,(int,float)) or not math.isfinite(producer_wall):raise ValueError('Missing finite producer wall time')
            selected=projected_status(value,contract['kind'])
            selected.update(run_id=value['run_id'],cutoff_ns=value.get('cutoff_ns'),producer_wall=producer_wall,
                source_age_at_receive_s=wall-producer_wall)
            stream['latest']=selected
            if stream['first_receive_wall'] is None:stream['first_receive_wall']=wall
            if stream['first_counts'] is None and isinstance(selected.get('counts'),dict):
                stream['first_counts']=copy.deepcopy(selected['counts'])
            if stream['first_lane_counts'] is None and isinstance(selected.get('lanes'),dict):
                stream['first_lane_counts']=copy.deepcopy(selected['lanes'])
        except (ValueError,TypeError,KeyError) as error:
            stream['rejections']+=1;stream['errors'].append(dict(wall=wall,error=str(error)));del stream['errors'][:-8]

    def snapshot(self,wall):
        reports={};healthy=True
        for topic,stream in self.streams.items():
            current=copy.deepcopy(stream);latest=current['latest'];contract=self.contracts[topic]
            reasons=[]
            if latest is None:reasons.append('No currently valid status for this run')
            else:
                age=wall-latest['producer_wall'];current['status_age_s']=age
                if age>self.max_status_age:reasons.append('Producer status is stale')
                if age< -1.:reasons.append('Producer clock is ahead by more than one second')
                if latest.get('fatal_error') is not None:reasons.append('Producer reports a fatal error')
                if current['first_counts'] is not None and isinstance(latest.get('counts'),dict):
                    deltas={};resets=[]
                    for key,value in latest['counts'].items():
                        first=current['first_counts'].get(key,0)
                        if isinstance(value,(int,float)) and isinstance(first,(int,float)):
                            if value<first:resets.append(key)
                            else:deltas[key]=value-first
                    current['counter_deltas']=deltas;current['counter_resets']=resets
                    if resets:reasons.append('Some producer counters regressed within the probe window')
                if contract['kind']=='local_sensor':
                    stamp=(latest.get('samples') or {}).get('cloud_stamp_last_ns')
                    current['last_cloud_source_age_s']=wall-stamp*1e-9 if isinstance(stamp,(int,float)) else None
                if contract['kind']=='sensor_ingress':
                    lanes=latest.get('lanes') or {};ages={};deltas={};resets=[]
                    configured_limit=latest.get('max_frame_age_s',2.)
                    if not isinstance(configured_limit,(int,float)) or not math.isfinite(configured_limit) or configured_limit<=0:
                        reasons.append('Ingress frame-age limit is invalid');configured_limit=2.
                    age_limit=min(self.max_status_age,configured_limit)
                    for lane in ('rgbd','imu','lidar','truth'):
                        values=lanes.get(lane) or {};first=(current['first_lane_counts'] or {}).get(lane) or {}
                        stamp=values.get('last_republished_stamp_ns')
                        source_age=wall-stamp*1e-9 if isinstance(stamp,(int,float)) and stamp>0 else None
                        ages[lane]=source_age
                        if source_age is None:reasons.append('No accepted '+lane+' payload has been republished')
                        elif source_age>age_limit or source_age<-.5:
                            reasons.append('Ingress '+lane+' payload source is stale or ahead of the receiver clock')
                        total=values.get('envelopes_republished',0);initial=first.get('envelopes_republished',0)
                        numeric=lambda value:isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
                        if not numeric(total) or not numeric(initial):
                            reasons.append('Ingress '+lane+' republishing counter is invalid');total=initial=0
                        delta=total-initial
                        deltas[lane]=delta
                        if delta<0:resets.append(lane)
                        began=current['first_receive_wall']
                        elapsed=wall-began if began is not None else 0.
                        if elapsed>=2. and delta<=0:
                            reasons.append('Ingress '+lane+' payload made no republishing progress during the probe')
                    if resets:reasons.append('Ingress lane counters regressed within this probe')
                    if latest.get('single_remote_sensor_receiver') is not True:
                        reasons.append('Ingress does not report the single remote receiver contract')
                    current.update(lane_source_ages_s=ages,lane_republished_deltas=deltas,
                                   lane_counter_resets=resets,maximum_payload_age_s=age_limit)
            current.update(kind=contract['kind'],expected_caller=contract['caller'],
                stream_healthy=not reasons,not_ready_reasons=reasons)
            healthy=healthy and not reasons;reports[topic]=current
        gauges=[(reports['/sim_%d/native_dense/local_sensor_status'%r]['latest'] or {}).get('native_local_gauge_verified') for r in (0,1)]
        bridges=[reports['/sim_%d/native_dense/status'%r]['latest'] or {} for r in (0,1)]
        accepted=[(b.get('accepted_native_mesh') or {}).get('completed_round_serial') for b in bridges]
        accepted_receipts=[b.get('accepted_native_mesh') or {} for b in bridges]
        serials_match=accepted[0] is not None and accepted[0]==accepted[1]
        same_generation=serials_match and all(r.get('exact_shared_optimizer_round_id_available') is True for r in accepted_receipts)
        for key in ('native_request_identity','paired_round_message_sha256','final_native_weight_batch_sha256'):
            same_generation=same_generation and bool(accepted_receipts[0].get(key)) and accepted_receipts[0].get(key)==accepted_receipts[1].get(key)
        global_status=reports['/visualization/native_mesh/status']['latest'] or {}
        ingress_healthy=all(reports['/sim_%d/sensors/ingress_status'%r]['stream_healthy'] for r in (0,1))
        return dict(wall=wall,run_id=self.run_id,cutoff_ns=self.cutoff_ns,read_only=True,
            only_status_topics_subscribed=True,subscribed_topics=list(self.contracts),streams=reports,
            all_status_streams_healthy=bool(healthy),sensor_ingress_streams_healthy=ingress_healthy,
            local_gauges_verified=all(value is True for value in gauges),
            accepted_native_mesh_serials=accepted,native_mesh_serials_match=serials_match,same_native_mesh_generation=bool(same_generation),
            global_fusion_ready=global_status.get('fusion_ready') is True,
            limitation='Sensor-cloud counters are producer progress only; native TSDF completion and map geometry require separate evidence.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--cutoff',type=float)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--maximum-status-age',type=float,default=5.)
    parser.add_argument('--master-uri',default=os.environ.get('ROS_MASTER_URI',MASTER_URI))
    parser.add_argument('--output',type=Path)
    parser.add_argument('--mesh-bus',action='store_true',help='Read existing bus counters at window ends; no mesh subscription')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=60:parser.error('Status subscription window must be finite in (0,60] seconds')
    if not math.isfinite(args.maximum_status_age) or args.maximum_status_age<=0:parser.error('Status age limit must be finite and positive')
    if args.cutoff is not None and (not math.isfinite(args.cutoff) or args.cutoff<=0):parser.error('Cutoff must be finite and positive')
    if args.master_uri.rstrip('/')!=MASTER_URI:parser.error('Only the dedicated hub master .14:11431 is permitted')
    os.environ['ROS_MASTER_URI']=MASTER_URI
    collector=StatusCollector(args.run_id,round(args.cutoff*1e9) if args.cutoff is not None else None,args.maximum_status_age)
    try:unwrap(rpc(MASTER_URI,'getSystemState',timeout=2.))
    except Exception as error:
        report=collector.snapshot(time.time());report['master_not_ready']=str(error)
        serialized=json.dumps(report,indent=2,allow_nan=False)
        if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(serialized,encoding='utf-8')
        print(serialized);raise SystemExit(2)
    import rospy
    from std_msgs.msg import String
    # A master disappearing between the XMLRPC precheck and registration must
    # not leave rospy.init_node retrying indefinitely.
    watchdog=threading.Timer(5.,lambda:rospy.signal_shutdown('Status-probe registration deadline exceeded'))
    watchdog.daemon=True;watchdog.start()
    try:rospy.init_node('three_host_readonly_status_probe',anonymous=True,disable_signals=True)
    except Exception as error:
        report=collector.snapshot(time.time());report['registration_not_ready']=str(error)
        serialized=json.dumps(report,indent=2,allow_nan=False)
        if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(serialized,encoding='utf-8')
        print(serialized);raise SystemExit(2)
    finally:watchdog.cancel()
    lock=threading.Lock();subscriptions=[];before=None;after=None
    try:
        if args.mesh_bus:before=mesh_bus_snapshots(MASTER_URI)
        def receive(message,topic):
            caller=(getattr(message,'_connection_header',None) or {}).get('callerid')
            with lock:collector.receive(topic,message.data,caller,time.time())
        for topic in collector.contracts:
            subscriptions.append(rospy.Subscriber(topic,String,lambda m,t=topic:receive(m,t),
                queue_size=1,buff_size=2**20,tcp_nodelay=True))
        start=time.monotonic()
        while time.monotonic()-start<args.duration and not rospy.is_shutdown():time.sleep(.1)
        with lock:report=collector.snapshot(time.time())
        report['subscription_elapsed_s']=time.monotonic()-start
    finally:
        for subscription in subscriptions:subscription.unregister()
        rospy.signal_shutdown('Read-only status observation finished')
    if args.mesh_bus:
        try:after=mesh_bus_snapshots(MASTER_URI);report['mesh_bus_diagnostics']=mesh_bus_delta(before,after)
        except Exception as error:report['mesh_bus_diagnostics']=dict(diagnostic_only=True,error=str(error),creates_topic_subscriptions=False)
    serialized=json.dumps(report,indent=2,allow_nan=False)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(serialized,encoding='utf-8')
    print(serialized)
    raise SystemExit(0 if report['all_status_streams_healthy'] else 2)


if __name__=='__main__':main()
