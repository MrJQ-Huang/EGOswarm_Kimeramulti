#!/usr/bin/env python3
"""Read-only placement check of this three-computer, two-drone ROS graph.

Uses XMLRPC registry/URI/PID calls only. It never registers a ROS node,
subscribes to a topic, publishes, changes a parameter or calls a service.
Exit 0 = ready; 2 = not ready; 3 = wrong host/source/master configuration.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
from urllib.parse import urlparse

from readonly_ros_graph import (EDGE_IPS,HUB_IP,MASTER_URI,get_node_directory,
    mesh_bus_delta,mesh_bus_snapshots,rpc,state_indexes,unwrap,uri_placement)


def expected_nodes(require_rviz=True):
    result={}
    for robot,ip in EDGE_IPS.items():
        for suffix in ('kimera_vio_ros/kimera_vio_ros_node','native_tsdf','native_mesh_frontend','native_local_sensor_bridge'):
            result['/sim_%d/%s'%(robot,suffix)]=dict(expected_ip=HUB_IP,role='central_'+suffix,required=True)
        result['/sim_%d_multimodal_sensors'%robot]=dict(expected_ip=HUB_IP,role='single_sensor_ingress',required=True)
        result['/sim_%d_native_vio_camera_graph_adapter'%robot]=dict(expected_ip=HUB_IP,role='central_camera_graph_adapter',required=True)
        for name,role in [('/sim_%d_edge_sensors'%robot,'edge_sensors'),
                ('/sim_%d_sensor_uplink'%robot,'edge_uplink'),
                ('/sim_%d_peer_heartbeat'%robot,'local_peer_heartbeat'),
                ('/drone_%d_ego_planner_node'%robot,'local_EGO'),
                ('/drone_%d_traj_server'%robot,'local_EGO_trajectory_execution'),
                ('/drone_%d_continuous_body'%robot,'local_body'),
                ('/drone_%d_complex_sim_supervisor'%robot,'local_supervisor'),
                ('/drone_%d_ideal_range_sensor'%robot,'local_planning_range_sensor')]:
            result[name]=dict(expected_ip=ip,role=role,required=True)
        for suffix in ('distributed_loop_closure/distributed_loop_closure_node','dpgo_ros_node/agent','native_pgmo','native_dense_bridge'):
            result['/sim_%d/%s'%(robot,suffix)]=dict(expected_ip=HUB_IP,role='hub_'+suffix,required=True)
    for name in ('/native_dense_global_view','/live_swarm_trajectory_view','/two_pc_complex_world_and_view'):
        result[name]=dict(expected_ip=HUB_IP,role='hub_processing_or_display',required=True)
    result['/two_host_closed_loop_missions']=dict(expected_ip=HUB_IP,role='goal_scheduler_only',required=False)
    for name in ('/rviz_local0_hub','/rviz_local1_hub','/rviz_global_hub','/rviz_trajectory_hub'):
        result[name]=dict(expected_ip=HUB_IP,role='hub_RViz',required=require_rviz)
    return result


def raw_topic_contracts():
    result={}
    for robot,ip in EDGE_IPS.items():
        vio='/sim_%d/kimera_vio_ros/kimera_vio_ros_node'%robot
        sensor='/sim_%d/native_local_sensor_bridge'%robot
        for suffix in ('kimera/rgb/image_raw','kimera/depth/image_raw','kimera/depth/measured_mask',
                       'kimera/rgb/camera_info','kimera/depth/camera_info','kimera/imu',
                       'lidar/points','ground_truth/odometry','sensors/ready'):
            required=(['/live_swarm_trajectory_view','/two_pc_complex_world_and_view'] if suffix=='ground_truth/odometry' else
                      [sensor] if suffix=='kimera/rgb/camera_info' else
                      [sensor,vio] if suffix in ('kimera/rgb/image_raw','kimera/depth/image_raw','kimera/imu') else [])
            result['/sim_%d/%s'%(robot,suffix)]=dict(expected_ip=HUB_IP,
                expected_publisher='/sim_%d_multimodal_sensors'%robot,
                required_subscribers=required)
        for suffix in ('kimera/rgb/image_raw','kimera/depth/image_raw','kimera/depth/measured_mask',
                       'kimera/rgb/camera_info','kimera/depth/camera_info','kimera/imu',
                       'lidar/points','sensors/ready'):
            result['/edge/sim_%d/%s'%(robot,suffix)]=dict(expected_ip=ip,
                expected_publisher='/sim_%d_edge_sensors'%robot,
                required_subscribers=['/sim_%d_sensor_uplink'%robot],
                exact_subscribers=['/sim_%d_sensor_uplink'%robot])
        for lane in ('rgbd','imu','lidar','truth','ready'):
            result['/edge/sim_%d/uplink/%s'%(robot,lane)]=dict(expected_ip=ip,
                subscriber_ip=HUB_IP,expected_publisher='/sim_%d_sensor_uplink'%robot,
                required_subscribers=['/sim_%d_multimodal_sensors'%robot],
                exact_subscribers=['/sim_%d_multimodal_sensors'%robot])
    return result


def control_topic_contracts():
    """EGO alone creates trajectories; its real trajectory server commands the body."""
    result={}
    for robot in (0,1):
        planner='/drone_%d_ego_planner_node'%robot
        server='/drone_%d_traj_server'%robot
        body='/drone_%d_continuous_body'%robot
        result['/drone_%d_planning/bspline'%robot]=dict(publishers=[planner],subscribers=[server])
        result['/drone_%d_planning/pos_cmd'%robot]=dict(publishers=[server],subscribers=[body])
        result['/drone_%d_visual_slam/odom'%robot]=dict(publishers=[body],
            subscribers=[planner,'/sim_%d_sensor_uplink'%robot],subscriber_ip=EDGE_IPS[robot])
        result['/two_pc/drone_%d/goal'%robot]=dict(publishers=['/two_host_closed_loop_missions'],
            subscribers=[planner],publisher_optional=True)
    planners=['/drone_%d_ego_planner_node'%robot for robot in (0,1)]
    result['/broadcast_bspline']=dict(publishers=planners,subscribers=planners)
    return result


def check_control_topic(topic,spec,indexes,directory=None,resolver=None):
    publishers,subscribers=indexes[:2]
    pubs=publishers.get(topic,[]);subs=subscribers.get(topic,[])
    unexpected=sorted(set(pubs)-set(spec['publishers']))
    missing_pubs=[] if spec.get('publisher_optional') else sorted(set(spec['publishers'])-set(pubs))
    missing_subs=sorted(set(spec['subscribers'])-set(subs))
    participants=[]
    if spec.get('subscriber_ip'):
        for name in subs:
            participants.append(check_node(name,dict(expected_ip=spec['subscriber_ip'],
                role='edge_local_actual_body_consumer',required=True),directory or {},set(directory or {}),resolver))
    statuses={row['status'] for row in participants}
    return dict(topic=topic,status='wrong_host' if 'wrong_host' in statuses else 'wrong_source' if unexpected else
        'not_ready' if missing_pubs or missing_subs or 'not_ready' in statuses else 'ready',publishers=pubs,subscribers=subs,
        unexpected_publishers=unexpected,missing_required_publishers=missing_pubs,
        missing_required_subscribers=missing_subs,
        actual_body_consumers=participants,
        policy='EGO planner -> native B-spline -> native trajectory server -> continuous body; goals only from mission')


def check_node(name,spec,directory,registered,resolver=None):
    item=dict(name=name,**spec)
    if name not in registered:
        return dict(item,status='not_ready' if spec['required'] else 'optional_missing',reason='Node is not registered')
    observed=directory.get(name) or {}
    if not observed.get('lookup_ready'):
        return dict(item,status='not_ready',reason='Node lookup is not ready',lookup=observed)
    kw={} if resolver is None else dict(resolver=resolver)
    placement=uri_placement(observed['uri'],spec['expected_ip'],**kw)
    item.update(placement)
    item['pid']=observed.get('pid')
    if placement['status']=='ready' and observed.get('reachable') is False:
        item.update(status='not_ready',reason='Registered node XMLRPC is not reachable',error=observed.get('error'))
    return item


def check_raw_topic(topic,spec,indexes,directory,resolver=None):
    publishers,subscribers=indexes[:2]
    pubs=publishers.get(topic,[]);subs=subscribers.get(topic,[])
    participants=[]
    for direction,names in (('publisher',pubs),('subscriber',subs)):
        for name in names:
            expected_ip=spec.get('subscriber_ip',spec['expected_ip']) if direction=='subscriber' else spec['expected_ip']
            row=check_node(name,dict(expected_ip=expected_ip,role='raw_'+direction,required=True),directory,set(directory),resolver)
            row['direction']=direction;participants.append(row)
    missing_subs=sorted(set(spec['required_subscribers'])-set(subs))
    unexpected_pubs=sorted(set(pubs)-{spec['expected_publisher']})
    unexpected_subs=sorted(set(subs)-set(spec['exact_subscribers'])) if 'exact_subscribers' in spec else []
    missing_pub=spec['expected_publisher'] not in pubs
    statuses={row['status'] for row in participants}
    if 'wrong_host' in statuses:status='wrong_host'
    elif unexpected_pubs or unexpected_subs:status='wrong_source'
    elif 'not_ready' in statuses or missing_pub or missing_subs:status='not_ready'
    else:status='ready'
    return dict(topic=topic,expected_ip=spec['expected_ip'],status=status,
        expected_publisher=spec['expected_publisher'],missing_expected_publisher=missing_pub,
        missing_required_subscribers=missing_subs,unexpected_publishers=unexpected_pubs,
        unexpected_subscribers=unexpected_subs,
        publishers=pubs,subscribers=subs,participants=participants,
        evidence='ROS master registration + each participant advertised XMLRPC URI; no raw payload subscribed')


def build_report(master_uri,system_state,directory,advertised_master_uri,require_rviz=True,resolver=None):
    indexes,registered=state_indexes(system_state);registered=set(registered)
    expected=expected_nodes(require_rviz)
    # Every discovered RViz instance is subject to hub placement, including an
    # old/unexpected node that uses another name.
    for name in registered:
        if 'rviz' in name.rsplit('/',1)[-1].lower() and name not in expected:
            expected[name]=dict(expected_ip=HUB_IP,role='discovered_RViz',required=True)
    nodes=[check_node(name,spec,directory,registered,resolver) for name,spec in sorted(expected.items())]
    topics=[check_raw_topic(topic,spec,indexes,directory,resolver) for topic,spec in raw_topic_contracts().items()]
    controls=[check_control_topic(topic,spec,indexes,directory,resolver) for topic,spec in control_topic_contracts().items()]
    forbidden=[]
    for topic in ['/edge/sim_%d/ground_truth/odometry'%i for i in (0,1)]+['/map_generator/global_cloud']:
        pubs=indexes[0].get(topic,[]);subs=indexes[1].get(topic,[])
        # A source-authenticated local telemetry copy is the only hub-facing
        # body path. Planning occupancy is generated directly on each edge.
        active=bool(pubs or subs) if topic.startswith('/edge/') else bool(pubs)
        forbidden.append(dict(topic=topic,publishers=pubs,subscribers=subs,
            status='wrong_source' if active else 'ready'))
    for robot in (0,1):
        node='/drone_%d_ideal_range_sensor'%robot
        global_topic='/map_generator/global_cloud'
        if node in indexes[1].get(global_topic,[]):
            forbidden.append(dict(topic=global_topic,node=node,status='wrong_source',
                                   reason='Central profile requires an edge-local planning map'))
    master_place=uri_placement(advertised_master_uri,HUB_IP,**({} if resolver is None else dict(resolver=resolver)))
    if urlparse(advertised_master_uri).port!=11431:master_place.update(status='wrong_host',reason='Master is not on dedicated port 11431')
    master=dict(requested_uri=master_uri,expected_uri=MASTER_URI,advertised_uri=advertised_master_uri,**master_place)
    problems=[dict(kind='node',name=v['name'],status=v['status'],reason=v.get('reason')) for v in nodes if v['status'] not in ('ready','optional_missing')]
    problems.extend(dict(kind='raw_topic',name=v['topic'],status=v['status']) for v in topics if v['status']!='ready')
    problems.extend(dict(kind='control_topic',name=v['topic'],status=v['status']) for v in controls if v['status']!='ready')
    problems.extend(dict(kind='forbidden_duplicate_or_oracle_topic',name=v['topic'],status=v['status'])
                    for v in forbidden if v['status']!='ready')
    if master['status']!='ready':problems.append(dict(kind='master',name=master_uri,status=master['status'],reason=master.get('reason')))
    return dict(wall=time.time(),read_only=True,creates_ros_node=False,creates_topic_subscriptions=False,
        location_basis='Advertised ROS XMLRPC URI with optional PID reachability; not OS process attestation',
        logical_drones=2,physical_hosts=3,master=master,nodes=nodes,raw_topic_placement=topics,
        control_topic_ownership=controls,
        forbidden_duplicate_or_oracle_topics=forbidden,
        ready=not problems,wrong_host=[v for v in problems if v['status']=='wrong_host'],
        wrong_source=[v for v in problems if v['status']=='wrong_source'],
        not_ready=[v for v in problems if v['status']=='not_ready'],optional_missing=[v['name'] for v in nodes if v['status']=='optional_missing'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-uri',default=os.environ.get('ROS_MASTER_URI',MASTER_URI))
    parser.add_argument('--output',type=Path)
    parser.add_argument('--timeout',type=float,default=2.)
    parser.add_argument('--allow-missing-rviz',action='store_true')
    parser.add_argument('--skip-node-pid',action='store_true')
    parser.add_argument('--bandwidth-window',type=float,default=5.,
                        help='Seconds between existing bus-counter samples; 0 disables; never subscribes mesh')
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<=0 or not math.isfinite(args.bandwidth_window) or not 0<=args.bandwidth_window<=30:
        parser.error('Timeout positive; bandwidth window must be finite in [0,30]')
    uri=args.master_uri.rstrip('/')
    if uri!=MASTER_URI:
        report=dict(wall=time.time(),ready=False,read_only=True,creates_topic_subscriptions=False,
            master=dict(requested_uri=uri,expected_uri=MASTER_URI,status='wrong_host',reason='Refuse to inspect another master'),
            wrong_host=[dict(kind='master',name=uri,status='wrong_host')],not_ready=[])
    else:
        try:
            state=unwrap(rpc(uri,'getSystemState',timeout=args.timeout))
            advertised=unwrap(rpc(uri,'getUri',timeout=args.timeout))
            indexes,registered=state_indexes(state)
            required=expected_nodes(not args.allow_missing_rviz)
            names=set(registered).intersection(required)
            names.update(name for name in registered if 'rviz' in name.rsplit('/',1)[-1].lower())
            for topic in set(raw_topic_contracts())|set(control_topic_contracts()):
                names.update(indexes[0].get(topic,[]));names.update(indexes[1].get(topic,[]))
            directory=get_node_directory(uri,sorted(names),args.timeout,not args.skip_node_pid)
            report=build_report(uri,state,directory,advertised,not args.allow_missing_rviz)
            if args.bandwidth_window:
                try:
                    before=mesh_bus_snapshots(uri,directory,args.timeout)
                    time.sleep(args.bandwidth_window)
                    after=mesh_bus_snapshots(uri,directory,args.timeout)
                    report['mesh_bus_diagnostics']=mesh_bus_delta(before,after)
                except Exception as error:
                    report['mesh_bus_diagnostics']=dict(diagnostic_only=True,error=str(error),creates_topic_subscriptions=False)
        except Exception as error:
            report=dict(wall=time.time(),ready=False,read_only=True,creates_topic_subscriptions=False,
                master=dict(requested_uri=uri,expected_uri=MASTER_URI,status='not_ready',error=str(error)),
                wrong_host=[],not_ready=[dict(kind='master',name=uri,status='not_ready',reason=str(error))])
    serialized=json.dumps(report,indent=2,allow_nan=False)
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(serialized,encoding='utf-8')
    print(serialized)
    raise SystemExit(0 if report.get('ready') else 3 if report.get('wrong_host') or report.get('wrong_source') else 2)


if __name__=='__main__':main()
