#!/usr/bin/env python3
"""Hub-only result reception, exact archives and native mesh visualization.

No estimator, registration, ICP, mesh welding or pose fitting runs here.
Local maps remain in separate robot frames. Shared overlays require two native
receipts; their application snapshot IDs are not native atomic round IDs.
"""
import argparse
from collections import OrderedDict
import copy
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import socket
import struct
import threading
import time

from result_wire import (VERSION, STREAMS, MESH_STREAMS, BODY_PATH_STREAMS, WireError, MissingBase, identity,
    validate_metadata, sha256, json_bytes, receive_packet, send_control,
    receive_control, encode_packet, read_header)


def channel_configuration(mode):
    if mode == 'map': return ('map',), 'distributed_result_observer'
    if mode == 'telemetry': return ('telemetry',), 'distributed_telemetry_observer'
    if mode == 'all': return ('map','telemetry'), 'distributed_result_observer'
    raise ValueError('Unknown observer channel mode')


def canonical_header_hash(raw):
    """Preserve all source bytes except ROS's mutable top-level Header.seq."""
    if not isinstance(raw,bytes) or len(raw) < 4: raise WireError('Truncated canonical ROS message')
    digest = hashlib.sha256(b'\0\0\0\0'); digest.update(memoryview(raw)[4:])
    return digest.hexdigest()


def verified_shared_body_path(mesh_record, path_record):
    """Only share the exact native evidence path named by this mesh receipt.

    Boot/frame agreement alone cannot associate asynchronous path/mesh topics.
    This is an immutable content association, not an exact numerical PGMO pose
    acknowledgement or proof of an atomic distributed optimization generation.
    """
    if not isinstance(mesh_record,dict) or not isinstance(path_record,dict): return False
    try:
        mesh, path = mesh_record['meta'],path_record['meta']
        validate_metadata(mesh); validate_metadata(path)
        if mesh['shared_gauge'] is not True or path['stream'] not in BODY_PATH_STREAMS: return False
        if any(mesh[key] != path[key] for key in ('run_id','cutoff_ns','robot_id','boot_id','gauge_id')): return False
        if path['frame_id'].lstrip('/') != 'kimera_world': return False
        raw = path_record['raw']
        if len(raw) != path['raw_bytes'] or sha256(raw) != path['result_sha256']: return False
        stamp, frame = read_header(raw)
        if stamp != path['stamp_ns'] or frame != path['frame_id']: return False
        # A serialized nav_msgs/Path has its pose count immediately after Header.
        frame_bytes, = struct.unpack_from('<I',raw,12)
        offset = 16+frame_bytes
        count, = struct.unpack_from('<I',raw,offset)
        if count <= 0 or count > (len(raw)-offset-4)//72: return False
        receipt = mesh['native_receipt']
        expected = receipt.get('canonical_submitted_body_path_sha256') or receipt.get('submitted_body_path_sha256')
        legacy = receipt.get('submitted_body_path_sha256')
        if legacy is not None and legacy != expected: return False
        return expected is not None and canonical_header_hash(raw) == expected
    except (KeyError,TypeError,ValueError,AttributeError,struct.error):
        return False


class ObserverState:
    def __init__(self, run_id, cutoff_ns, gauge_id, stale_seconds=5., clock=time.monotonic,wall_clock=time.time_ns,
                 body_cache_versions=32,body_cache_max_bytes=128*1024*1024,channels=('map','telemetry')):
        identity(0, run_id, cutoff_ns)
        from result_wire import IDENTIFIER
        if not isinstance(gauge_id,str) or not IDENTIFIER.fullmatch(gauge_id): raise WireError('Invalid observer gauge ID')
        self.run_id, self.cutoff_ns, self.gauge_id = run_id, cutoff_ns, gauge_id
        self.stale_seconds, self.clock = stale_seconds, clock
        self.wall_clock = wall_clock
        if not channels or any(channel not in ('map','telemetry') for channel in channels):raise ValueError('Invalid managed observer channels')
        self.managed_channels=tuple(dict.fromkeys(channels))
        self.lock = threading.Lock(); self.records = {}; self.links = {}; self.boots = {}; self.retired = {0:set(),1:set()}
        if type(body_cache_versions) is not int or body_cache_versions < 1 or type(body_cache_max_bytes) is not int or body_cache_max_bytes < 1:
            raise ValueError('Body path cache bounds must be positive integers')
        self.body_cache_versions,self.body_cache_max_bytes = body_cache_versions,body_cache_max_bytes
        self.body_paths = {0:OrderedDict(),1:OrderedDict()}; self.body_cache_bytes = {0:0,1:0}

    def connected(self, robot, channel, boot):
        identity(robot, self.run_id, self.cutoff_ns, boot)
        with self.lock:
            previous = self.boots.get(robot)
            if boot in self.retired[robot]: raise WireError('A retired result sender boot reappeared')
            if previous and previous != boot:
                self.retired[robot].add(previous)
                self.body_paths[robot].clear();self.body_cache_bytes[robot] = 0
                # A bounded experiment must not silently accept endless process
                # restarts while retaining ambiguous historical sessions.
                if len(self.retired[robot]) > 16: raise WireError('Too many sender restarts; use a fresh run')
            self.boots[robot] = boot
            self.links[robot,channel] = dict(connected=True, boot_id=boot, last_receive=self.clock(), error=None)

    def disconnected(self, robot, channel, boot=None, error=None):
        with self.lock:
            current = self.links.get((robot,channel))
            if current and (boot is None or current['boot_id'] == boot):
                current.update(connected=False, error=str(error) if error else None)

    def ingest(self, meta, raw):
        validate_metadata(meta, dict(run_id=self.run_id, cutoff_ns=self.cutoff_ns, gauge_id=self.gauge_id))
        robot, channel, stream = meta['robot_id'], meta['channel'], meta['stream']
        body_hash = canonical_header_hash(raw) if stream in BODY_PATH_STREAMS else None
        with self.lock:
            if self.boots.get(robot) != meta['boot_id']: raise WireError('Result belongs to an inactive sender boot')
            link = self.links.get((robot,channel))
            if not link or link['boot_id'] != meta['boot_id']: raise WireError('Result has no current subscribed session')
            now = self.clock(); link.update(connected=True, last_receive=now, error=None)
            if stream == 'heartbeat': return False
            key = robot, stream; old = self.records.get(key)
            if old and old['meta']['boot_id'] == meta['boot_id']:
                before = old['meta']
                if meta['sequence'] < before['sequence']: raise WireError('Result sequence regressed')
                if meta['sequence'] == before['sequence']:
                    if meta['result_sha256'] != before['result_sha256']: raise WireError('Result sequence reused for different original bytes')
                    if meta['snapshot_id'] == before['snapshot_id']: return False
                    # Asynchronous native status can enrich the same immutable
                    # mesh with its matching receipt. It cannot revoke a known
                    # proof or replace geometry under the same sequence.
                    if stream not in MESH_STREAMS or (before['shared_gauge'] and not meta['shared_gauge']):
                        raise WireError('Invalid same-sequence metadata replacement')
                if meta['stamp_ns'] < before['stamp_ns']: raise WireError('Result source stamp regressed')
            self.records[key] = dict(meta=copy.deepcopy(meta), raw=raw, received=now)
            if body_hash is not None:
                cache = self.body_paths[robot]
                previous = cache.pop(body_hash,None)
                if previous: self.body_cache_bytes[robot] -= len(previous['raw'])
                if len(raw) <= self.body_cache_max_bytes:
                    cache[body_hash] = self.records[key];self.body_cache_bytes[robot] += len(raw)
                while len(cache) > self.body_cache_versions or self.body_cache_bytes[robot] > self.body_cache_max_bytes:
                    _,evicted = cache.popitem(last=False);self.body_cache_bytes[robot] -= len(evicted['raw'])
            return True

    def snapshot(self):
        with self.lock:
            return dict(self.records), copy.deepcopy(self.links), dict(self.boots)

    def shared_pair(self):
        records, _, boots = self.snapshot(); pair = []
        for robot in (0,1):
            choices = [records.get((robot, key)) for key in ('accepted_mesh','mesh')]
            eligible = [item for item in choices if item and item['meta']['boot_id'] == boots.get(robot) and item['meta']['shared_gauge']]
            if not eligible: return None
            record = max(eligible,key=lambda item:(item['meta']['stamp_ns'],item['received']))
            pair.append(record)
        if len({item['meta']['gauge_id'] for item in pair}) != 1: return None
        if len({item['meta']['frame_id'].lstrip('/') for item in pair}) != 1: return None
        return pair

    def shared_body_path(self,robot,mesh_record=None):
        """Look up the immutable receipted path, even if newer paths arrived first."""
        if mesh_record is None:
            pair = self.shared_pair()
            if pair is None: return None
            mesh_record = pair[robot]
        receipt = mesh_record.get('meta',{}).get('native_receipt') or {}
        if not isinstance(receipt,dict): return None
        expected = receipt.get('canonical_submitted_body_path_sha256') or receipt.get('submitted_body_path_sha256')
        if not isinstance(expected,str): return None
        with self.lock:
            # The dedicated latest verified stream must survive an arbitrary
            # number of newer pending approvals. Its immutable record is already
            # bounded to one per stream, separately from historical path cache.
            candidates = [self.records.get((robot, 'verified_body_path')),
                          self.body_paths[robot].get(expected)]
            current_boot = self.boots.get(robot)
        for record in candidates:
            if (record and record['meta']['boot_id'] == current_boot and
                    verified_shared_body_path(mesh_record,record)):
                return record
        return None

    def report(self):
        records, links, boots = self.snapshot(); now = self.clock(); robots = {}
        for robot in (0,1):
            streams = {}
            for (owner,key), record in records.items():
                if owner != robot: continue
                meta = record['meta']; link = links.get((robot,meta['channel']), {})
                age = max(0., now-record['received'])
                source_age = max(0.,(self.wall_clock()-meta['stamp_ns'])/1e9) if meta['stamp_ns'] else None
                mesh_stream = key in MESH_STREAMS
                # Native mesh Header.stamp is derived from vertex stamps. A
                # freshly deformed mesh can retain old vertex timestamps; that
                # age is neither publication age nor end-to-end map latency.
                source_stale = (not mesh_stream and source_age is not None and
                                source_age > self.stale_seconds)
                streams[key] = dict(snapshot_id=meta['snapshot_id'], boot_id=meta['boot_id'],
                    sequence=meta['sequence'], last_receive_age_s=age, source_stamp_ns=meta['stamp_ns'],
                    source_age_s=source_age,
                    source_stamp_semantics=('native vertex-stamp-derived header; not publish time or end-to-end latency'
                        if mesh_stream else 'source message header'),
                    source_age_used_for_freshness=not mesh_stream,
                    freshness_basis=('last_snapshot_receive_age_link_and_boot' if mesh_stream else
                        'source_header_age_and_receive_age_link_and_boot'),
                    frame_id=meta['frame_id'], shared_gauge=meta['shared_gauge'],
                    last_known=True, stale=(age > self.stale_seconds or source_stale or not link.get('connected') or
                                          meta['boot_id'] != boots.get(robot)),
                    native_atomic_round=False)
            channels = {}
            for channel in ('map','telemetry'):
                if channel not in self.managed_channels:
                    channels[channel]=dict(handled_by='independent_observer_process',status_available_here=False)
                    continue
                link = links.get((robot,channel), {})
                age = max(0.,now-link['last_receive']) if link else None
                channels[channel] = dict(connected=bool(link.get('connected')), last_receive_age_s=age,
                    stale=not link.get('connected') or age is None or age > self.stale_seconds,
                    error=link.get('error'), boot_id=link.get('boot_id'))
            robots[str(robot)] = dict(current_boot_id=boots.get(robot), streams=streams, channels=channels)
        pair = self.shared_pair()
        selected_paths = {str(robot):self.shared_body_path(robot,pair[robot]) if pair else None for robot in (0,1)}
        with self.lock:
            cache_report = {str(robot):dict(versions=len(self.body_paths[robot]),bytes=self.body_cache_bytes[robot],
                maximum_versions=self.body_cache_versions,maximum_bytes=self.body_cache_max_bytes) for robot in (0,1)}
        return dict(run_id=self.run_id, cutoff_ns=self.cutoff_ns, gauge_id=self.gauge_id,
            managed_channels=list(self.managed_channels),
            telemetry_handler='this_process' if 'telemetry' in self.managed_channels else 'independent /distributed_telemetry_observer process',
            robots=robots, shared_display_ready=pair is not None,
            shared_display_snapshot_ids=[item['meta']['snapshot_id'] for item in pair] if pair else [],
            shared_body_path_ready={robot:record is not None for robot,record in selected_paths.items()},
            shared_body_path_snapshot_ids={robot:record['meta']['snapshot_id'] if record else None for robot,record in selected_paths.items()},
            shared_body_path_cache=cache_report,
            shared_display_is_atomic_native_round=False, estimator=False, registration=False,
            geometry_operation='unchanged per-robot native coordinates; shared overlay only with receipts',
            last_known_maps_retained_on_disconnect=True)


def mesh_components(message):
    """Validate and expose every original point/color/triangle, without sampling."""
    vertices = list(message.vertices); faces = [tuple(triangle.vertex_indices) for triangle in message.triangles]
    if not vertices or not faces: raise WireError('Empty native mesh')
    if any(not all(math.isfinite(float(v)) for v in (point.x,point.y,point.z)) for point in vertices):
        raise WireError('Nonfinite native mesh vertex')
    if any(len(face) != 3 or min(face) < 0 or max(face) >= len(vertices) for face in faces):
        raise WireError('Native triangle index out of range')
    colors = list(message.vertex_colors)
    if colors and len(colors) != len(vertices): raise WireError('Incomplete native color array')
    if any(not all(math.isfinite(float(v)) and 0 <= v <= 1 for v in (c.r,c.g,c.b,c.a)) for c in colors):
        raise WireError('Invalid native color')
    return vertices, colors, faces


def export_ply(message, path):
    """Binary PLY with full double XYZ, original RGBA and complete topology.

    Standard uchar color fields aid ordinary viewers; native_* double fields
    retain the exact source float values without 8-bit color quantization. The
    accompanying ROS binary remains authoritative for every native metadata field.
    """
    vertices, colors, faces = mesh_components(message)
    header = ['ply','format binary_little_endian 1.0',
        'comment original ROS message stored alongside this export',
        'comment native_* preserves source RGBA; uchar channels are display derivatives',
        'element vertex %d' % len(vertices),'property double x','property double y','property double z']
    if colors:
        header += ['property uchar red','property uchar green','property uchar blue','property uchar alpha',
            'property double native_red','property double native_green','property double native_blue','property double native_alpha']
    header += ['element face %d' % len(faces),'property list uchar uint vertex_indices','end_header']
    with Path(path).open('wb') as target:
        target.write(('\n'.join(header)+'\n').encode('ascii'))
        for index, point in enumerate(vertices):
            target.write(struct.pack('<ddd', point.x, point.y, point.z))
            if colors:
                color = colors[index]; rgba = (color.r,color.g,color.b,color.a)
                target.write(struct.pack('<4B4d', *(round(value*255) for value in rgba), *rgba))
        for face in faces: target.write(struct.pack('<BIII', 3, *face))


class ArchiveWriter:
    """Only instantiated on the hub; raw source messages are never rewritten."""
    def __init__(self, directory,snapshot_ply=True):
        self.directory = Path(directory)
        self.snapshot_ply = snapshot_ply
        self.locks = {(robot,channel): threading.Lock() for robot in (0,1) for channel in ('map','telemetry')}

    def archive(self, meta, raw, mesh=None):
        if meta['stream'] == 'heartbeat': return
        validate_metadata(meta)
        base = self.directory / meta['run_id'] / ('sim_%d' % meta['robot_id']) / meta['boot_id']
        with self.locks[meta['robot_id'],meta['channel']]:
            base.mkdir(parents=True, exist_ok=True)
            if meta['channel'] == 'telemetry':
                with (base / (meta['stream']+'.frames')).open('ab') as target:
                    target.write(encode_packet(meta, raw, 0))
                return
            directory = base / meta['stream']; directory.mkdir(exist_ok=True)
            name = '%09d_%s' % (meta['sequence'],meta['snapshot_id'])
            (directory / (name+'.rosbin')).write_bytes(raw)
            (directory / (name+'.json')).write_bytes(json_bytes(meta)+b'\n')
            write_ply = self.snapshot_ply and mesh is not None
            if write_ply: export_ply(mesh, directory / (name+'.ply'))
            pointer = directory / 'latest.json.tmp'
            pointer.write_bytes(json_bytes(dict(snapshot_id=meta['snapshot_id'], original_rosbin=name+'.rosbin',
                metadata=name+'.json', ply=name+'.ply' if write_ply else None))+b'\n')
            pointer.replace(directory / 'latest.json')


class ResultClient:
    def __init__(self, state, robot, host, channel, port, archive, on_result=None,
                 stop=None, socket_timeout=10., validate_result=None):
        self.state, self.robot, self.host, self.channel, self.port = state, robot, host, channel, port
        self.archive, self.on_result, self.validate_result = archive, on_result, validate_result
        self.stop = stop or threading.Event(); self.socket_timeout = socket_timeout
        self.pending = {}

    def run(self):
        while not self.stop.is_set():
            boot = None
            try:
                with socket.create_connection((self.host,self.port), timeout=self.socket_timeout) as connection:
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection.settimeout(self.socket_timeout)
                    request = dict(op='subscribe', protocol=VERSION, robot_id=self.robot,
                        run_id=self.state.run_id, cutoff_ns=self.state.cutoff_ns, channel=self.channel)
                    send_control(connection,dict(request,accept_delta=True)); reply = receive_control(connection)
                    expected_reply = dict(request, op='subscribed')
                    if not isinstance(reply,dict) or any(reply.get(key) != value for key,value in expected_reply.items()):
                        raise WireError('Wrong result sender handshake')
                    if reply.get('mode') != 'full' or reply.get('base_required') is not False:
                        raise WireError('Result sender did not offer a full baseline')
                    boot = reply.get('boot_id'); self.state.connected(self.robot,self.channel,boot)
                    delta_enabled = reply.get('delta_enabled') is True
                    bases = {}
                    expected = dict(robot_id=self.robot,run_id=self.state.run_id,cutoff_ns=self.state.cutoff_ns,
                        boot_id=boot,channel=self.channel,gauge_id=self.state.gauge_id)
                    while not self.stop.is_set():
                        try: meta, raw = receive_packet(connection, expected,bases)
                        except MissingBase as missing:
                            if not delta_enabled: raise
                            send_control(connection,dict(op='full_required',stream=missing.meta['stream'],snapshot_id=missing.meta['snapshot_id']))
                            meta,raw = receive_packet(connection,expected)
                            if meta['mode'] != 'full' or meta['snapshot_id'] != missing.meta['snapshot_id']:
                                raise WireError('Full fallback is not the requested immutable snapshot')
                        if self.validate_result: self.validate_result(meta)
                        if delta_enabled and meta['stream'] != 'heartbeat':
                            bases[meta['stream']] = raw
                            send_control(connection,dict(op='ack',stream=meta['stream'],snapshot_id=meta['snapshot_id'],result_sha256=meta['result_sha256']))
                        changed = self.state.ingest(meta,raw)
                        if not changed and self.pending.get(meta['stream']) != meta['snapshot_id']: continue
                        if meta['stream'] == 'heartbeat': continue
                        self.pending[meta['stream']] = meta['snapshot_id']
                        mesh = self.on_result(meta,raw) if self.on_result else None
                        self.archive.archive(meta,raw,mesh)
                        self.pending.pop(meta['stream'],None)
            except Exception as error:
                self.state.disconnected(self.robot,self.channel,boot,error)
            self.stop.wait(.5)


class RosTelemetryObserver:
    """A separate lightweight ROS process: no mesh classes, markers or map IO."""
    def __init__(self):
        import rospy
        import roslib.message
        self.rospy=rospy
        self.kinds={key:roslib.message.get_message_class(STREAMS[key][1]) for key in ('actual','vio')}
        if any(kind is None for kind in self.kinds.values()):raise RuntimeError('Missing nav_msgs/Odometry on hub')
        self.publishers={(robot,key):rospy.Publisher('/observer/sim_%d/%s'%(robot,key),kind,
            queue_size=1,latch=False,tcp_nodelay=True) for robot in (0,1) for key,kind in self.kinds.items()}

    def validate(self,meta):
        if meta['stream']=='heartbeat':return
        if meta['channel']!='telemetry' or meta['stream'] not in self.kinds:raise WireError('Map result sent to telemetry-only observer')
        if self.kinds[meta['stream']]._md5sum!=meta['ros_md5']:raise WireError('Telemetry ROS message MD5 differs from native sender')

    def receive(self,meta,raw):
        self.validate(meta)
        self.publishers[meta['robot_id'],meta['stream']].publish(self.rospy.AnyMsg().deserialize(raw))
        return None


class RosObserver:
    def __init__(self,channels=('map',),snapshot_ply=False,all_mesh_faces=False):
        import rospy
        import roslib.message
        from std_msgs.msg import String
        self.rospy, self.String = rospy, String
        self.kinds = {key:roslib.message.get_message_class(spec[1]) for key,spec in STREAMS.items()
                      if key != 'heartbeat' and spec[0] in channels}
        if any(kind is None for kind in self.kinds.values()): raise RuntimeError('Missing ROS result message definitions on hub')
        self.publishers = {(robot,key):rospy.Publisher('/observer/sim_%d/%s' % (robot,key),kind,
            queue_size=1,latch=STREAMS[key][0]=='map',tcp_nodelay=True)
            for robot in (0,1) for key,kind in self.kinds.items()}
        from visualization_msgs.msg import Marker
        self.Marker = Marker
        self.snapshot_ply,self.all_mesh_faces = snapshot_ply,all_mesh_faces
        self.markers = {(robot,key):rospy.Publisher('/observer/sim_%d/%s_faces' % (robot,key),Marker,queue_size=1,latch=True)
            for robot in (0,1) for key in (MESH_STREAMS if all_mesh_faces else ('local_mesh',))}
        self.shared = {robot:rospy.Publisher('/observer/sim_%d/shared_mesh' % robot,Marker,queue_size=1,latch=True) for robot in (0,1)}
        self.labels = {robot:rospy.Publisher('/observer/sim_%d/freshness' % robot,Marker,queue_size=1,latch=True) for robot in (0,1)}
        self.shared_labels = {robot:rospy.Publisher('/observer/sim_%d/shared_freshness' % robot,Marker,queue_size=1,latch=True) for robot in (0,1)}
        self.shared_paths = {robot:rospy.Publisher('/observer/sim_%d/shared_body_path' % robot,self.kinds['body_path'],queue_size=1,latch=True) for robot in (0,1)}
        self.status = rospy.Publisher('/observer/status',String,queue_size=1,latch=True)
        # At most one geometry per fixed robot/mesh stream (six slots). Equal
        # native/accepted content aliases the same immutable marker arrays.
        self.mesh_markers = {}; self.lock = threading.Lock()
        self.shared_last = {};self.shared_path_last = {};self.face_last = {}

    def validate(self, meta):
        if meta['stream'] != 'heartbeat' and self.kinds[meta['stream']]._md5sum != meta['ros_md5']:
            raise WireError('Hub ROS message MD5 differs from the native sender')

    def marker(self, message, robot, stream,components=None):
        from geometry_msgs.msg import Point
        vertices, colors, faces = components if components is not None else mesh_components(message)
        marker = self.Marker(); marker.header = copy.copy(message.header)
        marker.ns = 'native_'+stream; marker.id = robot; marker.type = marker.TRIANGLE_LIST
        marker.action = marker.ADD; marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = 1.
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
        for face in faces:
            for index in face:
                point = vertices[index]
                # The deployed native type already stores geometry_msgs/Point.
                # Reuse immutable native vertices instead of allocating a new
                # Python Point for every triangle corner.
                marker.points.append(point if isinstance(point,Point) else Point(point.x,point.y,point.z))
                if colors: marker.colors.append(colors[index])
        return marker

    @staticmethod
    def geometry_identity(meta):
        return tuple(meta[key] for key in ('robot_id','run_id','boot_id','gauge_id','ros_type','ros_md5','content_sha256'))

    def mesh_entry(self,meta,raw,message=None):
        """Validate original geometry once per content, with six bounded slots."""
        identity = self.geometry_identity(meta)
        with self.lock:
            cached = next((entry for entry in self.mesh_markers.values() if entry['identity']==identity),None)
        if cached is None:
            if message is None:
                message = self.kinds[meta['stream']]();message.deserialize(raw)
            components=mesh_components(message)
            cached = dict(identity=identity,message=message,components=components,marker=None)
        with self.lock:
            self.mesh_markers[meta['robot_id'],meta['stream']] = cached
        return cached

    def geometry(self,meta,raw,message=None):
        """Build faces only for a selected view; aliases share immutable arrays."""
        cached=self.mesh_entry(meta,raw,message)
        if cached['marker'] is None:
            marker=self.marker(cached['message'],meta['robot_id'],meta['stream'],cached['components'])
            with self.lock:
                cached['marker']=marker;cached['components']=None
                if not self.snapshot_ply:cached['message']=None
        return cached['marker']

    @staticmethod
    def detached_marker(marker,namespace,robot):
        # rospy may mutate Header.seq while serializing; geometry/color arrays
        # are immutable and safely shared by differently named derived views.
        value=copy.copy(marker);value.header=copy.copy(marker.header)
        value.ns=namespace;value.id=robot
        return value

    def receive(self, meta, raw):
        key, robot = meta['stream'], meta['robot_id']
        # AnyMsg keeps original Header.seq and every other ROS byte intact.
        self.publishers[robot,key].publish(self.rospy.AnyMsg().deserialize(raw))
        if key not in MESH_STREAMS: return None
        cached=self.mesh_entry(meta,raw)
        if (robot,key) in self.markers:
            identity=self.geometry_identity(meta)
            if self.face_last.get((robot,key)) != identity:
                marker=self.geometry(meta,raw)
                self.markers[robot,key].publish(self.detached_marker(marker,'native_'+key,robot))
                self.face_last[robot,key]=identity
        # Native/accepted shared geometry is rendered lazily by refresh only if
        # selected by the strict pair verifier, not for every incoming mirror.
        return cached['message'] if self.snapshot_ply else None

    def refresh(self, state):
        report = state.report(); self.status.publish(self.String(data=json.dumps(report,allow_nan=False)))
        pair = state.shared_pair()
        for robot in (0,1):
            boot=report['robots'][str(robot)]['current_boot_id'];gauge=report['gauge_id']
            if pair:
                meta=pair[robot]['meta']
                signature=(meta['snapshot_id'],self.Marker.ADD,boot,gauge)
                if self.shared_last.get(robot) != signature:
                    marker=self.geometry(meta,pair[robot]['raw'])
                    self.shared[robot].publish(self.detached_marker(marker,'native_shared_observed',robot))
                    self.shared_last[robot]=signature
            else:
                signature=(None,self.Marker.DELETE,boot,gauge)
                if self.shared_last.get(robot) != signature:
                    marker=self.Marker();marker.header.frame_id='kimera_world';marker.ns='native_shared_observed'
                    marker.id=robot;marker.action=marker.DELETE
                    self.shared[robot].publish(marker);self.shared_last[robot]=signature
            streams = report['robots'][str(robot)]['streams']; local = streams.get('local_mesh')
            label = self.Marker(); label.header.frame_id = 'sim_%d/vio_odom' % robot
            label.header.stamp = self.rospy.Time.now(); label.ns = 'observer_freshness'; label.id = robot
            label.type = label.TEXT_VIEW_FACING; label.action = label.ADD; label.pose.orientation.w = 1.
            label.pose.position.z = 2.5; label.scale.z = .4; label.color.a = 1.
            stale = not local or local['stale']; label.color.r = 1. if stale else .3; label.color.g = .4 if stale else 1.
            label.text = 'robot %d | %s | last local snapshot received %s' % (robot,'STALE / last known' if stale else 'LIVE',
                ('%.1fs ago' % local['last_receive_age_s']) if local else 'unavailable')
            self.labels[robot].publish(label)
            shared_label = copy.deepcopy(label); shared_label.header.frame_id = 'kimera_world'
            shared_label.ns = 'observer_shared_freshness'; shared_label.pose.position.x = robot*9.
            if pair:
                selected = report['robots'][str(robot)]['streams'][pair[robot]['meta']['stream']]
                stale = selected['stale']; shared_label.color.r = 1. if stale else .3
                shared_label.color.g = .4 if stale else 1.
                shared_label.text = 'robot %d | %s | last native snapshot received %.1fs ago' % (robot,
                    'STALE / last known' if stale else 'LIVE',selected['last_receive_age_s'])
            else:
                shared_label.color.r = 1.; shared_label.color.g = .7
                shared_label.text = 'robot %d | waiting for shared gauge; use local map view' % robot
            self.shared_labels[robot].publish(shared_label)
            path = state.shared_body_path(robot,pair[robot]) if pair else None
            if path is not None:
                receipt=pair[robot]['meta']['native_receipt']
                path_hash=receipt.get('canonical_submitted_body_path_sha256') or receipt['submitted_body_path_sha256']
                signature=(path_hash,boot,gauge)
                if self.shared_path_last.get(robot) != signature:
                    self.shared_paths[robot].publish(self.rospy.AnyMsg().deserialize(path['raw']))
                    self.shared_path_last[robot]=signature
            else:
                # Clear the latched overlay when a new boot/gauge lacks proof.
                signature=(None,boot,gauge)
                if self.shared_path_last.get(robot) != signature:
                    empty = self.kinds['body_path'](); empty.header.frame_id = 'kimera_world'
                    self.shared_paths[robot].publish(empty);self.shared_path_last[robot]=signature


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer', action='append', metavar='ROBOT@IP', help='Repeat once per edge; defaults to the two known edges')
    parser.add_argument('--channels',choices=('map','telemetry','all'),default='map',
        help='Run map and telemetry as separate managed processes; all retains the legacy combined diagnostic mode')
    parser.add_argument('--run-id', required=True); parser.add_argument('--cutoff', type=float, required=True)
    parser.add_argument('--gauge-id', required=True); parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--map-port', type=int, default=11740); parser.add_argument('--telemetry-port', type=int, default=11741)
    parser.add_argument('--stale-seconds', type=float, default=5.); parser.add_argument('--socket-timeout', type=float, default=10.)
    parser.add_argument('--no-ros', action='store_true', help='Archive/status only; no ROS visualization or ROS type-MD5 check')
    parser.add_argument('--snapshot-ply',action='store_true',help='Explicitly export PLY per received mesh snapshot; default archives exact ROS bytes/metadata only')
    parser.add_argument('--all-mesh-faces',action='store_true',help='Also render native/accepted per-stream faces; default renders local and verified shared views only')
    return parser


def main(argv=None):
    parser = argument_parser(); args = parser.parse_args(argv)
    if args.no_ros and args.snapshot_ply:parser.error('Snapshot PLY export requires ROS message definitions')
    if not all(math.isfinite(value) and value > 0 for value in (args.cutoff,args.stale_seconds,args.socket_timeout)):
        parser.error('Cutoff, stale threshold and timeout must be positive and finite')
    peers = {}
    try:
        for item in args.peer or ('0@192.168.5.6','1@192.168.5.21'):
            robot,host = item.split('@'); robot = int(robot); ipaddress.IPv4Address(host)
            if robot not in (0,1) or robot in peers: raise ValueError('duplicate/unknown robot')
            peers[robot] = host
        if args.map_port == args.telemetry_port or not all(0 < p < 65536 for p in (args.map_port,args.telemetry_port)):
            raise ValueError('two distinct valid result ports required')
    except ValueError as error: parser.error(str(error))
    channels,node_name=channel_configuration(args.channels)
    state = ObserverState(args.run_id,round(args.cutoff*1e9),args.gauge_id,args.stale_seconds,channels=channels)
    sink = None
    if not args.no_ros:
        import rospy
        rospy.init_node(node_name)
        sink = RosTelemetryObserver() if args.channels=='telemetry' else RosObserver(channels,args.snapshot_ply,args.all_mesh_faces)
    stop = threading.Event(); archive = ArchiveWriter(args.output_directory,snapshot_ply=args.snapshot_ply)
    clients = [ResultClient(state,robot,host,channel,port,archive,
        sink.receive if sink else None,stop,args.socket_timeout,sink.validate if sink else None)
        for robot,host in peers.items() for channel,port in (('map',args.map_port),('telemetry',args.telemetry_port)) if channel in channels]
    threads = [threading.Thread(target=client.run,name='observe-%d-%s' % (client.robot,client.channel),daemon=True) for client in clients]
    for thread in threads: thread.start()
    try:
        while sink is None or not sink.rospy.is_shutdown():
            if sink and hasattr(sink,'refresh'): sink.refresh(state)
            elif sink is not None: pass  # Telemetry has no map/status publishers or mesh/PLY work.
            else: print(json.dumps(state.report(),allow_nan=False),flush=True)
            if stop.wait(1.): break
    except KeyboardInterrupt: pass
    finally:
        stop.set()
        for thread in threads: thread.join(timeout=.5)


if __name__ == '__main__': main()
