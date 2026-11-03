#!/usr/bin/env python3
"""Small read-only ROS XMLRPC helpers. No topic subscription or ROS mutation."""
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import socket
import time
from urllib.parse import urlparse
import xmlrpc.client

CALLER='/three_host_graph_inspector_readonly'
MASTER_URI='http://192.168.5.14:11431'
HUB_IP='192.168.5.14'
EDGE_IPS={0:'192.168.5.6',1:'192.168.5.21'}
READ_METHODS={'getSystemState','getUri','lookupNode','getPid','getBusStats','getBusInfo'}
PRIMARY_SOURCES=[
    'https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/publication.cpp',
    'https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/transport_subscriber_link.cpp',
    'https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/topics.py',
]


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self,timeout):super().__init__();self.timeout=timeout
    def make_connection(self,host):
        connection=super().make_connection(host);connection.timeout=self.timeout;return connection


def rpc(uri,method,*args,timeout=2.):
    if method not in READ_METHODS:raise ValueError('Only read-only XMLRPC methods are allowed')
    if urlparse(uri).scheme!='http':raise ValueError('ROS XMLRPC URI must use http')
    with xmlrpc.client.ServerProxy(uri,transport=TimeoutTransport(timeout),allow_none=True) as proxy:
        return getattr(proxy,method)(CALLER,*args)


def unwrap(response):
    if not isinstance(response,(tuple,list)) or len(response)!=3 or not isinstance(response[0],int):
        raise ValueError('Unsupported ROS XMLRPC response envelope')
    code,message,value=response
    if code!=1:raise ValueError(str(message))
    return value


def resolve_ipv4(host):
    try:return [str(ipaddress.ip_address(host))]
    except ValueError:
        return sorted({row[4][0] for row in socket.getaddrinfo(host,None,socket.AF_INET,socket.SOCK_STREAM)})


def uri_placement(uri,expected_ip,resolver=resolve_ipv4):
    value=dict(uri=uri,expected_ip=expected_ip)
    try:
        parsed=urlparse(uri)
        if parsed.scheme!='http' or not parsed.hostname or not parsed.port:
            return dict(value,status='wrong_host',reason='Invalid advertised ROS node URI')
        ips=resolver(parsed.hostname)
        return dict(value,advertised_host=parsed.hostname,resolved_ips=ips,
            status='ready' if expected_ip in ips else 'wrong_host',
            reason=None if expected_ip in ips else 'Advertised node host differs from required physical host')
    except (OSError,ValueError) as error:
        return dict(value,status='not_ready',reason='Node URI hostname is not resolvable: '+str(error))


def state_indexes(state):
    if len(state)!=3:raise ValueError('getSystemState must return publishers, subscribers, services')
    indexes=[{str(topic):list(nodes) for topic,nodes in entries} for entries in state]
    names=sorted({name for index in indexes for nodes in index.values() for name in nodes})
    return indexes,names


def get_node_directory(master_uri,names,timeout=2.,probe_pid=True,call=rpc,workers=8):
    def one(name):
        try:
            uri=unwrap(call(master_uri,'lookupNode',name,timeout=timeout))
            item=dict(name=name,uri=uri,lookup_ready=True)
            if probe_pid:
                try:item['pid']=int(unwrap(call(uri,'getPid',timeout=timeout)));item['reachable']=True
                except Exception as error:item.update(reachable=False,error=str(error))
            return name,item
        except Exception as error:return name,dict(name=name,lookup_ready=False,error=str(error))
    with ThreadPoolExecutor(max_workers=workers) as pool:return dict(pool.map(one,names))


def bus_info_index(entries):
    result={}
    for entry in entries:
        if not isinstance(entry,(list,tuple)) or len(entry)<6:continue
        result[(str(entry[2]),str(entry[4]),int(entry[0]))]=dict(connection_id=int(entry[0]),
            peer_id=str(entry[1]),direction=str(entry[2]),transport=str(entry[3]),topic=str(entry[4]),
            connected=bool(entry[5]),transport_info=str(entry[6]) if len(entry)>6 else None)
    return result


def normalize_bus_stats(raw,info):
    # roscpp's Noetic implementation has a legacy unwrapped response on some
    # builds. Retain a diagnostic and accept it only if the topic arrays parse.
    if isinstance(raw,(list,tuple)) and len(raw)==3 and isinstance(raw[0],int):
        stats=unwrap(raw);envelope='standard_ros_response'
    elif isinstance(raw,(list,tuple)) and len(raw)>=2 and isinstance(raw[0],(list,tuple)):
        stats=raw;envelope='legacy_roscpp_unwrapped_response'
    else:raise ValueError('Unsupported getBusStats envelope')
    if not isinstance(stats,(list,tuple)) or len(stats)<2:raise ValueError('Malformed bus statistics')
    rows=[]
    for direction,topics in zip(('o','i'),stats[:2]):
        for entry in topics:
            if len(entry)==2:topic,connections=entry
            elif len(entry)==3:topic,_,connections=entry
            else:raise ValueError('Unknown publication/subscription statistics row')
            for values in connections:
                if len(values) not in (4,5):raise ValueError('Unknown connection statistics row')
                identifier=int(values[0]);cpp_pub=direction=='o' and len(values)==5
                byte_count=int(values[1])
                if cpp_pub:byte_count &= 0xffffffff
                # Native roscpp publishers use 5 columns; rospy publishers 4.
                # Receivers use 5 in rospy/roscpp; older documented 4-column
                # receiver rows expose no usable received-message counter.
                message_count=int(values[3] if cpp_pub else values[2]) if direction=='o' or len(values)==5 else None
                if cpp_pub:message_count &= 0xffffffff
                item=dict(info.get((direction,str(topic),identifier),{}),topic=str(topic),direction=direction,
                    connection_id=identifier,bytes=byte_count,messages=message_count,
                    counter_schema=('roscpp_publisher_5' if cpp_pub else 'connection_'+str(len(values))),
                    byte_semantics=('native_roscpp_enqueued_serialized_bytes' if cpp_pub else
                                    'received_transport_bytes' if direction=='i' else 'publisher_transport_counter'))
                if 'connected' not in item:item['connected']=None
                rows.append(item)
    return rows,envelope


def bus_snapshot(node,uri,topics,timeout=2.,call=rpc):
    value=dict(node=node,uri=uri,wall=time.time(),monotonic=time.monotonic(),diagnostics=[])
    try:value['pid']=int(unwrap(call(uri,'getPid',timeout=timeout)))
    except Exception as error:value['diagnostics'].append('getPid: '+str(error))
    try:
        info=bus_info_index(unwrap(call(uri,'getBusInfo',timeout=timeout)))
        value['connections']=[row for row in info.values() if row['topic'] in topics]
    except Exception as error:
        info={};value['connections']=[];value['diagnostics'].append('getBusInfo: '+str(error))
    try:
        rows,envelope=normalize_bus_stats(call(uri,'getBusStats',timeout=timeout),info)
        value['counters']=[row for row in rows if row['topic'] in topics]
        value['stats_envelope']=envelope
    except Exception as error:value['counters']=[];value['diagnostics'].append('getBusStats: '+str(error))
    return value


def mesh_bus_snapshots(master_uri,directory=None,timeout=2.,call=rpc):
    names=['/sim_%d/%s'%(r,role) for r in (0,1) for role in ('native_mesh_frontend','native_dense_bridge')]
    if directory is None:directory=get_node_directory(master_uri,names,timeout,False,call)
    def one(name):
        item=directory.get(name) or {}
        if not item.get('lookup_ready'):
            return name,dict(node=name,wall=time.time(),monotonic=time.monotonic(),counters=[],connections=[],
                diagnostics=['Node lookup is not ready: '+str(item.get('error'))])
        robot=0 if name.startswith('/sim_0/') else 1
        topics={'/sim_%d/native_mesh_frontend/full_mesh'%robot,
                '/sim_%d/native_mesh_frontend/mesh_graph_incremental'%robot}
        return name,bus_snapshot(name,item['uri'],topics,timeout,call)
    with ThreadPoolExecutor(max_workers=4) as pool:result=dict(pool.map(one,names))
    for name,value in result.items():
        robot=0 if name.startswith('/sim_0/') else 1
        peer='/sim_%d/%s'%(robot,'native_dense_bridge' if name.endswith('native_mesh_frontend') else 'native_mesh_frontend')
        peer_uri=(directory.get(peer) or {}).get('uri')
        for row in value.get('connections',[])+value.get('counters',[]):
            observed=row.get('peer_id')
            row['expected_existing_peer_node']=peer
            row['expected_existing_peer_uri']=peer_uri
            row['is_expected_existing_peer']=(observed==peer or
                (peer_uri is not None and isinstance(observed,str) and observed.rstrip('/')==peer_uri.rstrip('/')))
    return result


def counter_delta(first,second):
    delta=int(second)-int(first)
    if delta>=0:return delta,None
    if 0<=int(first)<=0xffffffff and 0<=int(second)<=0xffffffff:
        wrapped=(int(second)-int(first))&0xffffffff
        if wrapped<2**31:return wrapped,'32_bit_counter_wrapped'
    return None,'counter_regressed_or_reset'


def mesh_bus_delta(before,after):
    reports=[]
    for name,last in after.items():
        first=before.get(name) or {};elapsed=last.get('monotonic',0)-first.get('monotonic',0)
        report=dict(node=name,seconds=elapsed,connections_now=last.get('connections',[]),rates=[],
            diagnostics=list(first.get('diagnostics',[]))+list(last.get('diagnostics',[])))
        rows={(v['direction'],v['topic'],v['connection_id']):v for v in first.get('counters',[])}
        for row in last.get('counters',[]):
            old=rows.get((row['direction'],row['topic'],row['connection_id']))
            item=dict(row)
            if old is None:item['measurement_status']='new_connection_no_common_start_counter'
            elif elapsed<=0:item['measurement_status']='invalid_measurement_interval'
            elif first.get('pid') is not None and last.get('pid')!=first['pid']:
                item['measurement_status']='node_pid_changed'
            else:
                byte_delta,reason=counter_delta(old['bytes'],row['bytes'])
                item.update(measurement_status='ready' if byte_delta is not None else 'counter_reset',
                    bytes_delta=byte_delta,bytes_per_second=byte_delta/elapsed if byte_delta is not None else None,
                    counter_diagnostic=reason)
                if old.get('messages') is not None and row.get('messages') is not None:
                    count,note=counter_delta(old['messages'],row['messages'])
                    item.update(messages_delta=count,messages_per_second=count/elapsed if count is not None else None,
                        message_counter_diagnostic=note)
            report['rates'].append(item)
        report['actual_receive_connections']=[row for row in last.get('connections',[]) if row.get('direction')=='i' and row.get('connected')]
        reports.append(report)
    expected_rates=[dict(node=n['node'],**row) for n in reports for row in n['rates'] if row.get('is_expected_existing_peer')]
    return dict(nodes=reports,existing_frontend_bridge_connection_rates=expected_rates,creates_topic_subscriptions=False,
        limitation='Publisher counters may count enqueue attempts and queue discards; receiver transport bytes do not prove TSDF/PGMO callback completion or integration.',
        primary_sources=PRIMARY_SOURCES)
