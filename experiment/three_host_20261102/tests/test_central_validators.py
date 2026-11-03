#!/usr/bin/env python3
"""Read-only graph and source-age barriers for the centralized deployment."""
import copy
import json
from pathlib import Path
import sys
import unittest

VALIDATION = Path(__file__).resolve().parents[1] / 'validation'
sys.path.insert(0, str(VALIDATION))
import validate_bootstrap_subscriptions as bootstrap
import validate_three_host_topology as topology
from probe_three_host_status import StatusCollector


class Graph:
    def __init__(self):
        self.pub={};self.sub={};self.directory={};self.buses={}

    def node(self,name,ip):
        if name in self.directory:return
        self.directory[name]={'name':name,'uri':'http://%s:%d/' % (ip,23000+len(self.directory)),
                              'lookup_ready':True,'reachable':True,'pid':500+len(self.directory)}

    def add(self,index,topic,node):
        values=index.setdefault(topic,[])
        if node not in values:values.append(node)

    def state(self):
        return [list(self.pub.items()),list(self.sub.items()),
                [('/fake_service_'+str(i),[name]) for i,name in enumerate(self.directory)]]

    def call(self,uri,method,*args):
        if method=='getUri':return bootstrap.MASTER
        if method=='getSystemState':return self.state()
        if method=='lookupNode':return self.directory[args[0]]['uri']
        node=next(name for name,item in self.directory.items() if item['uri']==uri)
        if method=='getSubscriptions':return [(topic,'test/Message') for topic,names in self.sub.items() if node in names]
        if method=='getBusInfo':return self.buses.get(node,[])
        raise AssertionError('Unexpected or mutating XMLRPC method '+method)


def warm_graph(include_mission=False):
    graph=Graph()
    for node,spec in topology.expected_nodes().items():
        if spec['required'] or include_mission:graph.node(node,spec['expected_ip'])
    for topic,spec in topology.raw_topic_contracts().items():
        graph.add(graph.pub,topic,spec['expected_publisher'])
        for node in spec['required_subscribers']:graph.add(graph.sub,topic,node)
    for topic,spec in topology.control_topic_contracts().items():
        if not spec.get('publisher_optional') or include_mission:
            for node in spec['publishers']:graph.add(graph.pub,topic,node)
        for node in spec['subscribers']:graph.add(graph.sub,topic,node)
    return graph


def cold_graph():
    graph=Graph()
    for contract in bootstrap.contracts():
        graph.node(contract['node'],contract['expected_ip'])
        for topic in contract['required_subscriptions']:graph.add(graph.sub,topic,contract['node'])
    for robot in (0,1):
        base='/sim_%d'%robot;vio=base+'/kimera_vio_ros/kimera_vio_ros_node'
        body='/drone_%d_continuous_body'%robot
        graph.node(body,bootstrap.EDGES[robot])
        graph.add(graph.pub,'/drone_%d_visual_slam/odom'%robot,body)
        for suffix in bootstrap.SENSOR_SUFFIXES:graph.add(graph.pub,base+'/'+suffix,base+'_multimodal_sensors')
        for lane in bootstrap.LANES:graph.add(graph.pub,'/edge'+base+'/uplink/'+lane,base+'_sensor_uplink')
        for suffix in ('pose_graph_incremental','odometry','vlc_frames','bow_query'):
            graph.add(graph.pub,base+'/kimera_vio_ros/'+suffix,vio)
        graph.add(graph.pub,base+'/kimera_camera_adapter/pose_graph_incremental',base+'_native_vio_camera_graph_adapter')
    for contract in bootstrap.contracts():
        graph.buses.setdefault(contract['node'],[]).extend([
            [i,graph.pub[topic][0],'i','TCPROS',topic,True]
            for i,topic in enumerate(contract['required_connected_topics'])])
    return graph


class CentralTopologyContracts(unittest.TestCase):
    def report(self,graph):
        return topology.build_report(bootstrap.MASTER,graph.state(),graph.directory,bootstrap.MASTER,
                                     resolver=lambda host:[host])

    def test_complete_central_graph_is_ready_before_mission_exists(self):
        result=self.report(warm_graph())
        self.assertTrue(result['ready'],result)
        self.assertIn('/two_host_closed_loop_missions',result['optional_missing'])
        self.assertTrue(all(row['status']=='ready' for row in result['control_topic_ownership']))
        self.assertFalse(result['creates_topic_subscriptions'])

    def test_private_raw_and_wire_lanes_each_have_exactly_one_receiver(self):
        for topic in ('/edge/sim_0/kimera/rgb/image_raw','/edge/sim_1/uplink/rgbd'):
            graph=warm_graph();graph.node('/unexpected_recorder',topology.HUB_IP)
            graph.add(graph.sub,topic,'/unexpected_recorder')
            result=self.report(graph)
            self.assertFalse(result['ready'])
            row=next(row for row in result['raw_topic_placement'] if row['topic']==topic)
            self.assertEqual(row['unexpected_subscribers'],['/unexpected_recorder'])
            self.assertIn(row['status'],('wrong_source','wrong_host'))

    def test_edge_raw_recorder_is_rejected_even_when_it_uses_the_right_edge_host(self):
        graph=warm_graph();graph.node('/edge_raw_recorder','192.168.5.6')
        graph.add(graph.sub,'/edge/sim_0/kimera/depth/image_raw','/edge_raw_recorder')
        self.assertTrue(self.report(graph)['wrong_source'])

    def test_vio_and_optional_mission_cannot_remain_on_an_edge(self):
        for node in ('/sim_0/kimera_vio_ros/kimera_vio_ros_node','/two_host_closed_loop_missions'):
            graph=warm_graph(include_mission=True)
            graph.directory[node]['uri']='http://192.168.5.6:29999/'
            result=self.report(graph)
            self.assertFalse(result['ready'])
            self.assertIn(node,[row['name'] for row in result['wrong_host']])

    def test_a_hub_command_publisher_cannot_replace_or_join_the_real_ego_chain(self):
        graph=warm_graph();graph.add(graph.pub,'/drone_0_planning/pos_cmd','/hub_synthetic_commands')
        result=self.report(graph)
        self.assertFalse(result['ready'])
        self.assertIn('/drone_0_planning/pos_cmd',[row['name'] for row in result['wrong_source']])

    def test_hub_consumers_share_ingress_and_cannot_open_native_body_lan_streams(self):
        graph=warm_graph(include_mission=True)
        graph.add(graph.sub,'/drone_0_visual_slam/odom','/live_swarm_trajectory_view')
        result=self.report(graph)
        self.assertFalse(result['ready'])
        self.assertIn('/drone_0_visual_slam/odom',[row['name'] for row in result['wrong_host']])

    def test_an_unexpected_hub_recorder_of_native_body_is_also_rejected(self):
        graph=warm_graph();graph.node('/extra_recorder',topology.HUB_IP)
        graph.add(graph.sub,'/drone_1_visual_slam/odom','/extra_recorder')
        self.assertTrue(self.report(graph)['wrong_host'])

    def test_duplicate_sensor_truth_and_hub_planning_oracle_are_rejected(self):
        for topic in ('/edge/sim_0/ground_truth/odometry','/map_generator/global_cloud'):
            graph=warm_graph();graph.add(graph.pub,topic,'/legacy_producer')
            self.assertTrue(self.report(graph)['wrong_source'])


class BootstrapContracts(unittest.TestCase):
    def inspect(self,graph):return bootstrap.inspect(call=graph.call,resolve=lambda host:host)

    def test_ingress_advertises_standard_raw_topics_before_edge_sensors_start(self):
        result=self.inspect(cold_graph())
        self.assertTrue(result['ready'],[row for row in result['checks'] if row['status']!='ready'])
        self.assertTrue(result['no_ROS_node_or_topic_subscriptions'])

    def test_starting_private_raw_publishers_before_the_barrier_is_rejected(self):
        graph=cold_graph();graph.pub['/edge/sim_0/kimera/imu']=['/sim_0_edge_sensors']
        result=self.inspect(graph)
        self.assertFalse(result['ready'])
        self.assertTrue(any(row['kind']=='edge_sensors_must_still_be_stopped' and row['status']=='not_ready'
                            for row in result['checks']))

    def test_extra_remote_raw_subscriber_is_not_accepted_as_a_bootstrap_consumer(self):
        graph=cold_graph();graph.sub['/edge/sim_0/kimera/rgb/image_raw'].append('/hub_recorder')
        result=self.inspect(graph)
        self.assertFalse(result['ready'])
        self.assertTrue(any(row['kind']=='single_private_sensor_consumer' and row['status']=='wrong_source'
                            for row in result['checks']))

    def test_body_telemetry_is_connected_before_renderers_start(self):
        graph=cold_graph();node='/sim_0_sensor_uplink';topic='/drone_0_visual_slam/odom'
        contract=next(c for c in bootstrap.contracts() if c['node']==node)
        self.assertIn(topic,contract['required_connected_topics'])
        self.assertNotIn('/edge/sim_0/ground_truth/odometry',contract['required_subscriptions'])
        graph.buses[node]=[row for row in graph.buses[node] if row[4]!=topic]
        self.assertFalse(self.inspect(graph)['ready'])

    def test_bootstrap_rejects_direct_hub_body_stream_and_legacy_truth(self):
        graph=cold_graph();graph.add(graph.sub,'/drone_0_visual_slam/odom','/live_swarm_trajectory_view')
        result=self.inspect(graph)
        self.assertTrue(any(row['kind']=='native_body_telemetry_stays_on_edge' and row['status']=='wrong_host'
                            for row in result['checks']))
        graph=cold_graph();graph.add(graph.pub,'/edge/sim_0/ground_truth/odometry','/sim_0_edge_sensors')
        self.assertFalse(self.inspect(graph)['ready'])

    def test_missing_transport_or_wrong_peer_cannot_be_replaced_by_registry_entries(self):
        ingress='/sim_0_multimodal_sensors'
        for connected in (False,True):
            graph=cold_graph();row=graph.buses[ingress][0]
            if connected:row[1]='/foreign_uplink'
            else:row[5]=False
            result=self.inspect(graph)
            self.assertFalse(result['ready'])
            observed=next(row for row in result['checks'] if row.get('node')==ingress)
            self.assertEqual(observed['status'],'wrong_source' if connected else 'not_ready')

    def test_perception_on_old_edge_host_and_direct_hub_raw_subscriptions_are_rejected(self):
        graph=cold_graph();node='/sim_1/kimera_vio_ros/kimera_vio_ros_node'
        graph.directory[node]['uri']='http://192.168.5.21:29999/'
        self.assertTrue(any(row.get('node')==node and row['status']=='wrong_host'
                            for row in self.inspect(graph)['checks']))
        graph=cold_graph();dense='/sim_0/native_dense_bridge'
        graph.add(graph.sub,'/sim_0/kimera/rgb/image_raw',dense)
        self.assertTrue(any(row.get('node')==dense and row['status']=='wrong_source'
                            for row in self.inspect(graph)['checks']))


class StatusIngressContracts(unittest.TestCase):
    def populate(self,collector,wall,total=10):
        for topic,contract in collector.contracts.items():
            payload={'run_id':'current','cutoff_ns':100,'wall':wall,'fatal_error':None,
                     'native_local_gauge_verified':True}
            if 'robot' in contract:payload['robot_id']=contract['robot']
            if contract['kind']=='sensor_ingress':
                payload.update(single_remote_sensor_receiver=True,max_frame_age_s=2.,
                    lanes={lane:{'envelopes_republished':total,'last_republished_stamp_ns':round((wall-.1)*1e9)}
                           for lane in ('rgbd','imu','lidar','truth','ready')})
            collector.receive(topic,json.dumps(payload),contract['caller'],wall)

    def test_seven_small_status_streams_include_both_real_sensor_ingresses(self):
        collector=StatusCollector('current',100);self.populate(collector,100.)
        self.populate(collector,103.,total=30)
        result=collector.snapshot(103.1)
        self.assertEqual(len(result['subscribed_topics']),7)
        self.assertTrue(result['all_status_streams_healthy'])
        self.assertTrue(result['sensor_ingress_streams_healthy'])

    def test_fresh_status_heartbeat_cannot_hide_a_stalled_compressed_payload_lane(self):
        collector=StatusCollector('current',100);self.populate(collector,100.)
        self.populate(collector,103.,total=30)
        topic='/sim_0/sensors/ingress_status';payload=copy.deepcopy(collector.streams[topic]['latest'])
        payload.update(wall=103.1,robot_id=0)
        payload['lanes']['imu']['last_republished_stamp_ns']=round(100.*1e9)
        payload['lanes']['imu']['envelopes_republished']=10
        collector.receive(topic,json.dumps(payload),'/sim_0_multimodal_sensors',103.1)
        report=collector.snapshot(103.2)
        self.assertFalse(report['all_status_streams_healthy'])
        self.assertTrue(any('imu' in reason for reason in report['streams'][topic]['not_ready_reasons']))

    def test_old_epoch_or_foreign_ingress_status_is_rejected(self):
        for caller,cutoff in (('/sim_0_multimodal_sensors',99),('/foreign_ingress',100)):
            collector=StatusCollector('current',100);self.populate(collector,100.)
            topic='/sim_0/sensors/ingress_status'
            payload={'run_id':'current','cutoff_ns':cutoff,'robot_id':0,'wall':100.}
            collector.receive(topic,json.dumps(payload),caller,100.)
            self.assertIsNone(collector.streams[topic]['latest'])
            self.assertEqual(collector.streams[topic]['rejections'],1)
            self.assertFalse(collector.snapshot(100.1)['sensor_ingress_streams_healthy'])


if __name__=='__main__':unittest.main()
