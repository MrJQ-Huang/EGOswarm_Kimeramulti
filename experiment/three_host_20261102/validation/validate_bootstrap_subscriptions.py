#!/usr/bin/env python3
"""Read-only cold-start barrier for centralized VIO/mapping before edge sensors."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import socket
from urllib.parse import urlparse
import xmlrpc.client

MASTER='http://192.168.5.14:11431'
HUB='192.168.5.14'
EDGES={0:'192.168.5.6',1:'192.168.5.21'}
SENSOR_SUFFIXES=('kimera/rgb/image_raw','kimera/depth/image_raw','kimera/depth/measured_mask',
                 'kimera/rgb/camera_info','kimera/depth/camera_info','kimera/imu',
                 'lidar/points','ground_truth/odometry','sensors/ready')
PRIVATE_SENSOR_SUFFIXES=tuple(s for s in SENSOR_SUFFIXES if s!='ground_truth/odometry')
LANES=('rgbd','imu','lidar','truth','ready')


class Transport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host);connection.timeout=2.;return connection


def rpc(uri,method,*args):
    if method not in ('getUri','getSystemState','lookupNode','getSubscriptions','getBusInfo'):
        raise ValueError('Only read-only XMLRPC calls are allowed')
    with xmlrpc.client.ServerProxy(uri,transport=Transport()) as proxy:
        code,message,value=getattr(proxy,method)('/bootstrap_barrier_readonly',*args)
    if code!=1:raise RuntimeError(str(message))
    return value


def contracts():
    result=[]
    for robot,edge in EDGES.items():
        base='/sim_%d'%robot
        graph=base+'/kimera_vio_ros/pose_graph_incremental'
        adapter=base+'/kimera_camera_adapter/pose_graph_incremental'
        odom=base+'/kimera_vio_ros/odometry'
        raw=[base+'/kimera/'+suffix for suffix in ('rgb/image_raw','depth/image_raw','imu')]
        private=['/edge'+base+'/'+suffix for suffix in PRIVATE_SENSOR_SUFFIXES]
        body='/drone_%d_visual_slam/odom'%robot
        telemetry=base+'/ground_truth/odometry'
        uplinks=['/edge'+base+'/uplink/'+lane for lane in LANES]
        def add(node,host,topics,connected,disconnected=(),forbidden=()):
            result.append(dict(node=node,expected_ip=host,required_subscriptions=list(topics),
                required_connected_topics=list(connected),must_be_disconnected_topics=list(disconnected),
                forbidden_subscriptions=list(forbidden)))
        add(base+'_native_vio_camera_graph_adapter',HUB,[graph],[graph])
        add(base+'/native_local_sensor_bridge',HUB,[graph,odom,base+'/kimera/rgb/camera_info']+raw,
            [graph,odom,base+'/kimera/rgb/camera_info']+raw)
        add(base+'/native_dense_bridge',HUB,[graph,odom],[graph,odom],forbidden=raw)
        add(base+'/distributed_loop_closure/distributed_loop_closure_node',HUB,
            [adapter,base+'/kimera_vio_ros/vlc_frames',base+'/kimera_vio_ros/bow_query'],[adapter])
        add(base+'/kimera_vio_ros/kimera_vio_ros_node',HUB,raw,raw)
        add(base+'_multimodal_sensors',HUB,uplinks,uplinks,forbidden=private+[body])
        add(base+'_sensor_uplink',edge,private+[body],[body],disconnected=private,
            forbidden=['/edge'+telemetry])
        add('/drone_%d_ideal_range_sensor'%robot,edge,[body],[body],
            forbidden=['/map_generator/global_cloud'])
        for node in ('/live_swarm_trajectory_view','/two_pc_complex_world_and_view'):
            add(node,HUB,[telemetry],[telemetry],forbidden=[body])
    return result


def inspect(call=rpc,resolve=socket.gethostbyname):
    if str(call(MASTER,'getUri')).rstrip('/')!=MASTER:
        raise ValueError('Dedicated master identity differs')
    state=call(MASTER,'getSystemState')
    pub,sub=({topic:nodes for topic,nodes in entries} for entries in state[:2])
    checks=[]
    for robot in EDGES:
        base='/sim_%d'%robot
        ingress=base+'_multimodal_sensors';uplink=base+'_sensor_uplink'
        for suffix in PRIVATE_SENSOR_SUFFIXES:
            private='/edge'+base+'/'+suffix
            publishers=pub.get(private,[]);subscribers=sub.get(private,[])
            checks.append(dict(kind='edge_sensors_must_still_be_stopped',topic=private,
                status='not_ready' if publishers else 'ready',publishers=publishers))
            checks.append(dict(kind='single_private_sensor_consumer',topic=private,
                status='ready' if subscribers==[uplink] else 'wrong_source',
                subscribers=subscribers,expected_subscribers=[uplink]))
        old_truth='/edge'+base+'/ground_truth/odometry'
        checks.append(dict(kind='duplicate_private_truth_must_be_absent',topic=old_truth,
            status='wrong_source' if pub.get(old_truth) or sub.get(old_truth) else 'ready',
            publishers=pub.get(old_truth,[]),subscribers=sub.get(old_truth,[])))
        for suffix in SENSOR_SUFFIXES:
            canonical=base+'/'+suffix;publishers=pub.get(canonical,[])
            checks.append(dict(kind='unique_hub_sensor_ingress',topic=canonical,
                status='ready' if publishers==[ingress] else 'wrong_source',
                publishers=publishers,expected_publisher=ingress))
        for lane in LANES:
            topic='/edge'+base+'/uplink/'+lane;publishers=pub.get(topic,[]);subscribers=sub.get(topic,[])
            checks.append(dict(kind='single_compressed_uplink_connection',topic=topic,
                status='ready' if publishers==[uplink] and subscribers==[ingress] else 'wrong_source',
                publishers=publishers,expected_publisher=uplink,subscribers=subscribers,
                expected_subscribers=[ingress]))
        topic=base+'/kimera_vio_ros/pose_graph_incremental'
        expected=base+'/kimera_vio_ros/kimera_vio_ros_node'
        checks.append(dict(kind='unique_native_graph_publisher',topic=topic,publishers=pub.get(topic,[]),
                           status='ready' if pub.get(topic)==[expected] else 'not_ready'))
        body='/drone_%d_visual_slam/odom'%robot
        expected='/drone_%d_continuous_body'%robot
        checks.append(dict(kind='unique_actual_body_source',topic=body,publishers=pub.get(body,[]),
                           status='ready' if pub.get(body)==[expected] else 'not_ready'))
        for node in sub.get(body,[]):
            try:
                ip=resolve(urlparse(call(MASTER,'lookupNode',node)).hostname)
                checks.append(dict(kind='native_body_telemetry_stays_on_edge',topic=body,node=node,
                    expected_ip=EDGES[robot],actual_ip=ip,status='ready' if ip==EDGES[robot] else 'wrong_host'))
            except Exception as exc:
                checks.append(dict(kind='native_body_telemetry_stays_on_edge',topic=body,node=node,
                                   status='not_ready',error=str(exc)))
    checks.append(dict(kind='central_display_does_not_publish_planning_oracle',
        topic='/map_generator/global_cloud',publishers=pub.get('/map_generator/global_cloud',[]),
        status='wrong_source' if pub.get('/map_generator/global_cloud') else 'ready'))
    def one(contract):
        result=dict(contract);node=contract['node'];host=contract['expected_ip']
        topics=contract['required_subscriptions'];connected_topics=contract['required_connected_topics']
        try:
            uri=call(MASTER,'lookupNode',node);result['uri']=uri
            actual=resolve(urlparse(uri).hostname)
            if actual!=host:return dict(result,status='wrong_host',actual_ip=actual)
            actual_topics=dict(call(uri,'getSubscriptions'))
            buses=[entry for entry in call(uri,'getBusInfo')
                   if len(entry)>=6 and entry[2]=='i' and entry[5]]
            connected={entry[4] for entry in buses}
            missing_registered=[topic for topic in topics if node not in sub.get(topic,[])]
            missing_actual=[topic for topic in topics if topic not in actual_topics]
            missing_connected=[topic for topic in connected_topics if topic not in connected]
            forbidden=[topic for topic in contract['forbidden_subscriptions'] if topic in actual_topics]
            premature=[topic for topic in contract['must_be_disconnected_topics'] if topic in connected]
            wrong_peers=[]
            # Registry ownership alone cannot replace an actual live TCPROS link.
            # Check peer identities where a unique source is part of the contract.
            for topic in connected_topics:
                publishers=pub.get(topic,[])
                if len(publishers)!=1:continue
                expected=publishers[0];peer_uri=call(MASTER,'lookupNode',expected)
                peers=[str(entry[1]).rstrip('/') for entry in buses if entry[4]==topic]
                allowed={expected,peer_uri.rstrip('/')}
                if peers and any(peer not in allowed for peer in peers):
                    wrong_peers.append(dict(topic=topic,expected_publisher=expected,observed_peers=peers))
            wrong=forbidden or wrong_peers
            waiting=missing_registered or missing_actual or missing_connected or premature
            return dict(result,status='wrong_source' if wrong else 'not_ready' if waiting else 'ready',
                        missing_master_subscriptions=missing_registered,missing_node_subscriptions=missing_actual,
                        missing_incoming_connections=missing_connected,forbidden_actual_subscriptions=forbidden,
                        premature_private_sensor_connections=premature,wrong_connected_peers=wrong_peers)
        except Exception as exc:return dict(result,status='not_ready',error=str(exc))
    with ThreadPoolExecutor(max_workers=8) as pool:checks.extend(pool.map(one,contracts()))
    ready=all(row['status']=='ready' for row in checks)
    return dict(master_uri=MASTER,ready=ready,read_only=True,no_ROS_node_or_topic_subscriptions=True,
                barrier='Run after edge uplinks and hub ingress/VIO advertise and connect; before either edge sensors starts',
                publishes_or_starts_nodes=False,checks=checks)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:result=inspect()
    except Exception as exc:result=dict(master_uri=MASTER,ready=False,read_only=True,error=str(exc),checks=[])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    wrong=any(row['status'] in ('wrong_host','wrong_source') for row in result['checks'])
    raise SystemExit(0 if result['ready'] else 3 if wrong else 2)


if __name__=='__main__':main()
