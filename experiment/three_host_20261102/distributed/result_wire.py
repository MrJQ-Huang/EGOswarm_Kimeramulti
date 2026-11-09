#!/usr/bin/env python3
"""Bounded full-snapshot transport. Payloads are unmodified ROS1 message bytes.

The application snapshot identity is not a native atomic optimization round.
Every reconnect starts with full snapshots. Optional byte-block deltas name an
exact baseline; a missing base requires a fresh full snapshot, never guessing.
This module intentionally has no ROS or third-party dependency.
"""
import hashlib
import json
import re
import struct
import zlib

MAGIC = b'KMRES01\0'
PREFIX = struct.Struct('!8sIQ')
VERSION = 1
MAX_HEADER_BYTES = 1024 * 1024
MAX_RAW_BYTES = 128 * 1024 * 1024
MAX_WIRE_BYTES = MAX_RAW_BYTES + 1024 * 1024
MAX_CONTROL_BYTES = 4096
MAX_PATCH_BYTES = MAX_RAW_BYTES + 1024 * 1024
BLOCK_SIZE = 65536
IDENTIFIER = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}')
HASH = re.compile(r'[a-f0-9]{64}')
MD5 = re.compile(r'[a-f0-9]{32}')

# (channel, ROS type, source topic template, source node template).
STREAMS = {
    'local_mesh': ('map', 'kimera_pgmo_msgs/KimeraPgmoMesh', '/sim_{r}/native_mesh_frontend/full_mesh', '/sim_{r}/native_mesh_frontend'),
    'mesh': ('map', 'kimera_pgmo_msgs/KimeraPgmoMesh', '/sim_{r}/native_dense/native_optimized_mesh', '/sim_{r}/native_dense_bridge'),
    'accepted_mesh': ('map', 'kimera_pgmo_msgs/KimeraPgmoMesh', '/sim_{r}/native_dense/accepted_joint_mesh', '/sim_{r}/native_dense_bridge'),
    'status': ('map', 'std_msgs/String', '/sim_{r}/native_dense/status', '/sim_{r}/native_dense_bridge'),
    'actual': ('telemetry', 'nav_msgs/Odometry', '/drone_{r}_visual_slam/odom', '/drone_{r}_continuous_body'),
    'vio': ('telemetry', 'nav_msgs/Odometry', '/sim_{r}/kimera_vio_ros/odometry', '/sim_{r}/kimera_vio_ros/kimera_vio_ros_node'),
    'optimized_path': ('map', 'nav_msgs/Path', '/sim_{r}/dpgo_ros_node/path', '/sim_{r}/dpgo_ros_node/agent'),
    'body_path': ('map', 'nav_msgs/Path', '/sim_{r}/native_dense/approved_body_path', '/sim_{r}/native_dense_bridge'),
    # Keep the v1 approved-path contract above. This separate latched stream
    # retains the exact path supporting last-verified gauge continuity while a
    # newer approved path is pending native PGMO observation.
    'verified_body_path': ('map', 'nav_msgs/Path', '/sim_{r}/native_dense/verified_body_path', '/sim_{r}/native_dense_bridge'),
    'optimized_graph': ('map', 'pose_graph_tools_msgs/PoseGraph', '/sim_{r}/kimera_distributed/optimized_nodes', '/sim_{r}/distributed_loop_closure/distributed_loop_closure_node'),
    'heartbeat': ('any', 'std_msgs/String', '/sim_{r}/result_stream/heartbeat', '/sim_{r}_result_stream'),
}
MESH_STREAMS = frozenset(('local_mesh', 'mesh', 'accepted_mesh'))
BODY_PATH_STREAMS = frozenset(('body_path', 'verified_body_path'))
FIELDS = frozenset(('protocol', 'run_id', 'cutoff_ns', 'robot_id', 'boot_id', 'channel',
    'stream', 'sequence', 'source_topic', 'source_caller', 'ros_type', 'ros_md5',
    'stamp_ns', 'stamp_basis','frame_id', 'mode', 'base_sha256', 'result_sha256', 'content_sha256',
    'raw_bytes', 'encoded_raw_bytes', 'codec', 'snapshot_id', 'gauge_id', 'shared_gauge', 'native_receipt'))


class WireError(ValueError):
    pass


class MissingBase(WireError):
    def __init__(self, meta):
        super().__init__('Exact byte-delta base is unavailable; request a full snapshot')
        self.meta = meta


def json_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def parse_json(raw):
    def constant(value): raise WireError('Nonfinite JSON value')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result: raise WireError('Duplicate JSON key')
            result[key] = value
        return result
    try:
        return json.loads(raw.decode('utf-8'), parse_constant=constant, object_pairs_hook=pairs)
    except (UnicodeError, ValueError, TypeError, RecursionError) as error:
        raise WireError('Malformed JSON: ' + str(error)) from error


def integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise WireError('Invalid ' + name)
    return value


def identity(robot, run_id, cutoff_ns, boot_id=None):
    if type(robot) is not int or robot not in (0, 1): raise WireError('Invalid robot ID')
    if not isinstance(run_id, str) or not IDENTIFIER.fullmatch(run_id): raise WireError('Invalid run ID')
    integer(cutoff_ns, 'cutoff', 1)
    if boot_id is not None and (not isinstance(boot_id, str) or not IDENTIFIER.fullmatch(boot_id)):
        raise WireError('Invalid boot ID')


def contract(robot, stream):
    try: channel, kind, topic, caller = STREAMS[stream]
    except (KeyError, TypeError): raise WireError('Unknown result stream')
    return dict(channel=channel, ros_type=kind, source_topic=topic.format(r=robot), source_caller=caller.format(r=robot))


def sha256(raw): return hashlib.sha256(raw).hexdigest()


def content_hash(raw, stream):
    # Ignore only the transport-written top-level Header.seq for mesh change
    # detection. The original sequence and every other byte are still retained.
    if stream in MESH_STREAMS:
        if len(raw) < 4: raise WireError('Truncated mesh header')
        digest = hashlib.sha256(b'\0\0\0\0'); digest.update(memoryview(raw)[4:])
        return digest.hexdigest()
    return sha256(raw)


def read_header(raw):
    if len(raw) < 16: raise WireError('Truncated ROS Header')
    seq, seconds, nanos, size = struct.unpack_from('<IIII', raw)
    if nanos >= 10**9 or size > 256 or 16 + size > len(raw): raise WireError('Invalid ROS Header')
    try: frame = raw[16:16+size].decode('utf-8')
    except UnicodeError as error: raise WireError('Invalid frame ID') from error
    return seconds * 10**9 + nanos, frame


def ros_string(raw):
    if len(raw) < 4: raise WireError('Truncated ROS String')
    size, = struct.unpack_from('<I', raw)
    if size != len(raw)-4: raise WireError('Invalid ROS String length')
    return parse_json(raw[4:])


def _receipt_proves_shared(receipt, canonical_hash, frame, gauge_id, run_id, robot, cutoff_ns):
    """Require matched native evidence, never a CLI gauge label by itself.

    The producer is trusted only after the fixed topic/caller/type handshake.
    This checks its immutable receipt association; it does not claim to turn
    observed native messages into an atomic distributed commit protocol.
    """
    if not isinstance(receipt, dict) or frame.lstrip('/') != 'kimera_world': return False
    if (receipt.get('run_id'), receipt.get('robot_id'), receipt.get('cutoff_ns')) != (run_id, robot, cutoff_ns): return False
    if receipt.get('canonical_native_mesh_sha256') != canonical_hash: return False
    if receipt.get('mesh_frame', '').lstrip('/') != frame.lstrip('/'): return False
    if receipt.get('shared_gauge_id', gauge_id) != gauge_id: return False
    if (receipt.get('shared_gauge_ready') is not True or receipt.get('native_path_echo_verified') is not True
            or receipt.get('native_publish_iterate') is not False): return False
    evidence = receipt.get('optimization_evidence') or receipt.get('completed_round_proof')
    if not isinstance(evidence, dict): return False
    # Support the existing observed proof and the online receipt. Both must
    # explicitly report a real accepted cross-robot constraint.
    cross = evidence.get('accepted_cross_constraints', 0)
    if type(cross) is not int or cross <= 0: return False
    completed = evidence.get('round') or evidence.get('completed_round') or {}
    return (isinstance(completed, dict) and set(completed.get('active_robots', [])) == {0, 1}
            and (completed.get('terminate_wall') is not None or evidence.get('native_completed') is True))


def receipt_proves_shared(receipt, canonical_hash, frame, gauge_id, run_id, robot, cutoff_ns):
    try:
        return _receipt_proves_shared(receipt,canonical_hash,frame,gauge_id,run_id,robot,cutoff_ns)
    except (AttributeError,TypeError,ValueError):
        return False


def snapshot_identity(meta):
    # Excludes transport codec; a re-encoded full snapshot has the same ID.
    return sha256(json_bytes({key: meta[key] for key in (
        'run_id', 'cutoff_ns', 'robot_id', 'boot_id', 'stream', 'sequence',
        'result_sha256', 'content_sha256','raw_bytes','source_topic','source_caller','ros_type','ros_md5',
        'stamp_ns','stamp_basis','frame_id','gauge_id', 'shared_gauge', 'native_receipt')}))


def validate_metadata(meta, expected=None):
    if not isinstance(meta, dict) or set(meta) != FIELDS: raise WireError('Unexpected result envelope schema')
    if type(meta['protocol']) is not int or meta['protocol'] != VERSION: raise WireError('Unknown result protocol')
    identity(meta['robot_id'], meta['run_id'], meta['cutoff_ns'], meta['boot_id'])
    integer(meta['sequence'], 'sequence', 1); integer(meta['stamp_ns'], 'source stamp')
    if meta['stamp_basis'] not in ('header','headerless','latest_graph_node'): raise WireError('Unknown result source stamp basis')
    if meta['stamp_basis'] == 'latest_graph_node' and meta['stream'] != 'optimized_graph': raise WireError('Graph-node stamp used for a different source')
    if (meta['stream'] in ('status','heartbeat')) != (meta['stamp_basis'] == 'headerless'): raise WireError('Unexpected headerless result identity')
    integer(meta['raw_bytes'], 'raw size', 1)
    if meta['raw_bytes'] > MAX_RAW_BYTES: raise WireError('Result exceeds raw-byte limit')
    spec = contract(meta['robot_id'], meta['stream'])
    for key in ('ros_type', 'source_topic', 'source_caller'):
        if meta[key] != spec[key]: raise WireError('Unexpected result ' + key)
    if meta['channel'] not in ('map', 'telemetry') or spec['channel'] not in ('any', meta['channel']):
        raise WireError('Result on wrong channel')
    if not isinstance(meta['ros_md5'], str) or not MD5.fullmatch(meta['ros_md5']): raise WireError('Invalid ROS MD5')
    for key in ('result_sha256', 'content_sha256', 'snapshot_id'):
        if not isinstance(meta[key], str) or not HASH.fullmatch(meta[key]): raise WireError('Invalid ' + key)
    integer(meta['encoded_raw_bytes'], 'encoded raw size', 1)
    if meta['encoded_raw_bytes'] > MAX_PATCH_BYTES: raise WireError('Oversized encoded result')
    if meta['mode'] == 'full':
        if meta['base_sha256'] is not None or meta['encoded_raw_bytes'] != meta['raw_bytes']:
            raise WireError('Invalid full-snapshot baseline/size')
    elif meta['mode'] == 'blocks':
        if meta['channel'] != 'map' or not isinstance(meta['base_sha256'],str) or not HASH.fullmatch(meta['base_sha256']):
            raise WireError('Invalid byte-delta baseline/channel')
    else: raise WireError('Unknown result mode')
    if meta['codec'] not in ('zlib', 'none'): raise WireError('Unknown result codec')
    if not isinstance(meta['frame_id'], str) or len(meta['frame_id']) > 256: raise WireError('Invalid result frame')
    if not isinstance(meta['gauge_id'], str) or not IDENTIFIER.fullmatch(meta['gauge_id']): raise WireError('Invalid gauge ID')
    if type(meta['shared_gauge']) is not bool: raise WireError('Invalid shared-gauge flag')
    if meta['stream'] not in ('status', 'heartbeat') and meta['stamp_ns'] < meta['cutoff_ns']:
        raise WireError('Previous-epoch result stamp')
    if meta['native_receipt'] is not None and not isinstance(meta['native_receipt'], dict): raise WireError('Invalid native receipt')
    if meta['shared_gauge'] and (meta['stream'] not in ('mesh', 'accepted_mesh') or not receipt_proves_shared(
            meta['native_receipt'], meta['content_sha256'], meta['frame_id'], meta['gauge_id'],
            meta['run_id'], meta['robot_id'], meta['cutoff_ns'])):
        raise WireError('Unproven shared gauge')
    if meta['snapshot_id'] != snapshot_identity(meta): raise WireError('Application snapshot ID mismatch')
    if expected:
        for key, value in expected.items():
            if meta.get(key) != value: raise WireError('Unexpected result identity: ' + key)
    return meta


def make_delta(base, raw):
    """Exact fixed-byte-block replacement, including historical and face bytes."""
    chunks = []
    for offset in range(0,len(raw),BLOCK_SIZE):
        block = raw[offset:offset+BLOCK_SIZE]
        if block != base[offset:offset+len(block)]: chunks.append((offset,block))
    result = bytearray(struct.pack('!QI',len(raw),len(chunks)))
    for offset,block in chunks:
        result.extend(struct.pack('!QI',offset,len(block))); result.extend(block)
    if len(result) > MAX_PATCH_BYTES: raise WireError('Byte delta too large')
    return bytes(result)


def apply_delta(base, patch, size):
    if len(patch) < 12: raise WireError('Truncated byte delta')
    final_size,count = struct.unpack_from('!QI',patch)
    if final_size != size or not 0 < final_size <= MAX_RAW_BYTES or count > (MAX_RAW_BYTES//BLOCK_SIZE+1):
        raise WireError('Invalid byte-delta dimensions')
    output = bytearray(base[:final_size])
    if len(output) < final_size: output.extend(b'\0'*(final_size-len(output)))
    cursor = 12; previous_end = 0
    for _ in range(count):
        if cursor+12 > len(patch): raise WireError('Truncated byte-delta block')
        offset,length = struct.unpack_from('!QI',patch,cursor); cursor += 12
        if not 0 < length <= BLOCK_SIZE or offset < previous_end or offset+length > final_size or cursor+length > len(patch):
            raise WireError('Invalid/overlapping byte-delta block')
        output[offset:offset+length] = patch[cursor:cursor+length]
        previous_end = offset+length; cursor += length
    if cursor != len(patch): raise WireError('Trailing byte-delta data')
    return bytes(output)


def encode_packet(meta, raw, compression_level=1, base=None):
    if not isinstance(raw, bytes): raise WireError('Result payload must be immutable bytes')
    if type(compression_level) is not int or compression_level not in range(10): raise WireError('Invalid compression level')
    meta = dict(meta, codec='zlib' if compression_level else 'none',mode='full',base_sha256=None,encoded_raw_bytes=len(raw))
    validate_metadata(meta)
    if len(raw) != meta['raw_bytes'] or sha256(raw) != meta['result_sha256'] or content_hash(raw, meta['stream']) != meta['content_sha256']:
        raise WireError('Result payload differs from metadata')
    body = zlib.compress(raw, compression_level) if compression_level else raw
    if base is not None:
        if not isinstance(base,bytes) or len(base) > MAX_RAW_BYTES: raise WireError('Invalid byte-delta base')
        patch = make_delta(base,raw)
        compressed = zlib.compress(patch,compression_level) if compression_level else patch
        if len(compressed)+64 < len(body):
            body = compressed; meta.update(mode='blocks',base_sha256=sha256(base),encoded_raw_bytes=len(patch))
    validate_metadata(meta)
    header = json_bytes(meta)
    if len(header) > MAX_HEADER_BYTES: raise WireError('Oversized result metadata')
    if len(body) > MAX_WIRE_BYTES: raise WireError('Oversized result wire payload')
    return PREFIX.pack(MAGIC, len(header), len(body)) + header + body


def decode_parts(header, body, expected=None, base=None):
    meta = validate_metadata(parse_json(header), expected)
    if meta['codec'] == 'zlib':
        try:
            decoder = zlib.decompressobj()
            raw = decoder.decompress(body, meta['encoded_raw_bytes'] + 1)
            if decoder.unconsumed_tail or decoder.unused_data or not decoder.eof: raise WireError('Invalid/bomb/trailing compressed result')
        except zlib.error as error: raise WireError('Invalid compressed result') from error
    else: raw = body
    if len(raw) != meta['encoded_raw_bytes']: raise WireError('Encoded result length differs')
    if meta['mode'] == 'blocks':
        if base is None or sha256(base) != meta['base_sha256']: raise MissingBase(meta)
        raw = apply_delta(base,raw,meta['raw_bytes'])
    if len(raw) != meta['raw_bytes'] or sha256(raw) != meta['result_sha256'] or content_hash(raw, meta['stream']) != meta['content_sha256']:
        raise WireError('Result length/hash mismatch')
    return meta, raw


def decode_packet(packet, expected=None, base=None):
    if len(packet) < PREFIX.size: raise WireError('Truncated result frame')
    magic, hsize, bsize = PREFIX.unpack_from(packet)
    check_lengths(magic, hsize, bsize)
    if len(packet) != PREFIX.size + hsize + bsize: raise WireError('Result framing length mismatch')
    return decode_parts(packet[PREFIX.size:PREFIX.size+hsize], packet[PREFIX.size+hsize:], expected, base)


def check_lengths(magic, hsize, bsize):
    if magic != MAGIC or not 0 < hsize <= MAX_HEADER_BYTES or not 0 < bsize <= MAX_WIRE_BYTES:
        raise WireError('Invalid result framing/size')


def recv_exact(connection, length):
    result = bytearray()
    while len(result) < length:
        chunk = connection.recv(min(65536, length-len(result)))
        if not chunk: raise EOFError('Result connection closed')
        result.extend(chunk)
    return bytes(result)


def receive_packet(connection, expected=None, bases=None):
    magic, hsize, bsize = PREFIX.unpack(recv_exact(connection, PREFIX.size))
    check_lengths(magic, hsize, bsize)
    header = recv_exact(connection, hsize)
    meta = validate_metadata(parse_json(header), expected)  # Reject identity before reading a large body.
    base = bases.get(meta['stream']) if bases is not None else None
    return decode_parts(header, recv_exact(connection, bsize), expected, base)


def send_control(connection, value):
    data = json_bytes(value)
    if not 0 < len(data) <= MAX_CONTROL_BYTES: raise WireError('Oversized control message')
    connection.sendall(struct.pack('!I', len(data)) + data)


def receive_control(connection):
    size, = struct.unpack('!I', recv_exact(connection, 4))
    if not 0 < size <= MAX_CONTROL_BYTES: raise WireError('Invalid control size')
    return parse_json(recv_exact(connection, size))
