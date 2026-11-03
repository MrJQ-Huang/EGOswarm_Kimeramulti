#!/usr/bin/env python3
"""Bounded, lossless ROS-message envelopes for one edge-to-hub sensor receiver.

The payload is the original ROS serialization, including its original Header.
Envelope clocks diagnose transport age; they never replace a sensor timestamp.
This module has no ROS dependency so the wire and cache contracts are testable.
"""
from collections import Counter
import hashlib
import io
import json
import re
import struct
import threading
import zlib


STREAMS = {
    'rgb': dict(suffix='kimera/rgb/image_raw', type='sensor_msgs/Image', lane='rgbd', latch=False),
    'depth': dict(suffix='kimera/depth/image_raw', type='sensor_msgs/Image', lane='rgbd', latch=False),
    'mask': dict(suffix='kimera/depth/measured_mask', type='sensor_msgs/Image', lane='rgbd', latch=False),
    'info': dict(suffix='kimera/rgb/camera_info', type='sensor_msgs/CameraInfo', lane='rgbd', latch=False),
    'depth_info': dict(suffix='kimera/depth/camera_info', type='sensor_msgs/CameraInfo', lane='rgbd', latch=False),
    'imu': dict(suffix='kimera/imu', type='sensor_msgs/Imu', lane='imu', latch=False),
    'lidar': dict(suffix='lidar/points', type='sensor_msgs/PointCloud2', lane='lidar', latch=False),
    'truth': dict(suffix='ground_truth/odometry', type='nav_msgs/Odometry', lane='truth', latch=False),
    'ready': dict(suffix='sensors/ready', type='std_msgs/Bool', lane='ready', latch=True),
}
LANES = ('rgbd', 'imu', 'lidar', 'truth', 'ready')
RGBD_KEYS = ('info', 'depth_info', 'mask', 'rgb', 'depth')
PROTOCOL_VERSION = 2
MAGIC = b'KMSENS2\0'
MAX_HEADER_BYTES = 8192
MAX_RAW_BYTES = 32 * 1024 * 1024
MAX_WIRE_BYTES = MAX_RAW_BYTES + 65536
RUN_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}')


class WireError(ValueError):
    pass


def _integer(value, label, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise WireError('Invalid ' + label)
    return value


def validate_identity(robot_id, run_id, cutoff_ns):
    _integer(robot_id, 'robot ID')
    if robot_id not in (0, 1): raise WireError('Unknown robot ID')
    if not isinstance(run_id, str) or not RUN_ID.fullmatch(run_id): raise WireError('Invalid run ID')
    _integer(cutoff_ns, 'epoch cutoff', 1)


def canonical_topic(robot, suffix):
    return '/sim_%d/%s' % (robot, suffix.lstrip('/'))


def edge_topic(robot, suffix):
    return '/edge/sim_%d/%s' % (robot, suffix.lstrip('/'))


def uplink_topic(robot, lane):
    if lane not in LANES: raise WireError('Unknown sensor lane')
    return '/edge/sim_%d/uplink/%s' % (robot, lane)


def sensor_remaps(robot):
    return [canonical_topic(robot, spec['suffix']) + ':=' + edge_topic(robot, spec['suffix'])
            for spec in STREAMS.values()]


def source_caller(robot): return '/sim_%d_edge_sensors' % robot
def body_caller(robot): return '/drone_%d_continuous_body' % robot
def uplink_caller(robot): return '/sim_%d_sensor_uplink' % robot
def ingress_caller(robot): return '/sim_%d_multimodal_sensors' % robot


def input_topic(robot, key):
    """One local input for each uplink stream; actual telemetry bypasses rendering."""
    if key == 'truth': return '/drone_%d_visual_slam/odom' % robot
    return edge_topic(robot, STREAMS[key]['suffix'])


def input_caller(robot, key):
    return body_caller(robot) if key == 'truth' else source_caller(robot)


def lane_source_caller(robot, lane):
    if lane not in LANES: raise WireError('Unknown sensor lane')
    return body_caller(robot) if lane == 'truth' else source_caller(robot)


def serialized(message):
    buffer = io.BytesIO(); message.serialize(buffer); return buffer.getvalue()


def stamp_ns(message):
    return int(message.header.stamp.to_nsec()) if hasattr(message, 'header') else 0


def lane_keys(lane):
    if lane == 'rgbd': return RGBD_KEYS
    if lane not in LANES: raise WireError('Unknown sensor lane')
    return (lane,)


def encode_packet(robot_id, run_id, cutoff_ns, lane, sequence, source_stamp_ns,
                  messages, compression_level=1):
    """Encode a complete lane; ``messages`` maps stream keys to original bytes."""
    validate_identity(robot_id, run_id, cutoff_ns)
    _integer(sequence, 'lane sequence', 1); _integer(source_stamp_ns, 'sensor stamp')
    if lane != 'ready' and source_stamp_ns < cutoff_ns: raise WireError('Pre-epoch sensor stamp')
    if lane == 'ready' and source_stamp_ns != 0: raise WireError('Ready must retain its headerless identity')
    if set(messages) != set(lane_keys(lane)): raise WireError('Incomplete or foreign lane streams')
    if isinstance(compression_level, bool) or compression_level not in range(10):
        raise WireError('Invalid compression level')
    chunks = []; descriptors = []
    for key in lane_keys(lane):
        raw = messages[key]
        if not isinstance(raw, bytes) or not raw: raise WireError('Empty/non-byte ROS serialization')
        chunks.append(raw); descriptors.append(dict(key=key, type=STREAMS[key]['type'], bytes=len(raw)))
    raw = b''.join(chunks)
    if len(raw) > MAX_RAW_BYTES: raise WireError('Sensor payload exceeds raw-byte bound')
    compressed = lane in ('rgbd', 'lidar') and compression_level > 0
    payload = zlib.compress(raw, compression_level) if compressed else raw
    metadata = dict(version=PROTOCOL_VERSION, robot_id=robot_id, run_id=run_id, cutoff_ns=cutoff_ns,
                    lane=lane, sequence=sequence, stamp_ns=source_stamp_ns,
                    source_caller=lane_source_caller(robot_id, lane), codec='zlib' if compressed else 'none',
                    raw_bytes=len(raw), streams=descriptors)
    header = json.dumps(metadata, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    if len(header) > MAX_HEADER_BYTES: raise WireError('Oversized envelope header')
    frame = MAGIC + struct.pack('<I', len(header)) + header + payload
    frame += hashlib.sha256(frame).digest()
    if len(frame) > MAX_WIRE_BYTES: raise WireError('Sensor envelope exceeds wire-byte bound')
    return frame


def decode_packet(packet, robot_id, run_id, cutoff_ns, lane=None):
    """Check integrity and allocation bounds before decompressing untrusted input."""
    validate_identity(robot_id, run_id, cutoff_ns)
    if not isinstance(packet, bytes): packet = bytes(packet)
    minimum = len(MAGIC) + 4 + 32
    if len(packet) < minimum or len(packet) > MAX_WIRE_BYTES: raise WireError('Invalid envelope size')
    if packet[:len(MAGIC)] != MAGIC: raise WireError('Unknown sensor envelope')
    if hashlib.sha256(packet[:-32]).digest() != packet[-32:]: raise WireError('Sensor envelope checksum differs')
    size = struct.unpack_from('<I', packet, len(MAGIC))[0]
    offset = len(MAGIC) + 4
    if not 0 < size <= MAX_HEADER_BYTES or offset + size >= len(packet)-32:
        raise WireError('Invalid envelope header length')
    try:
        def invalid_constant(value): raise WireError('Nonfinite envelope value')
        meta = json.loads(packet[offset:offset+size].decode('utf-8'), parse_constant=invalid_constant)
    except (UnicodeError, ValueError, TypeError) as error:
        raise WireError('Malformed envelope JSON: ' + str(error))
    required = {'version', 'robot_id', 'run_id', 'cutoff_ns', 'lane', 'sequence',
                'stamp_ns', 'source_caller', 'codec', 'raw_bytes', 'streams'}
    if not isinstance(meta, dict) or set(meta) != required: raise WireError('Unexpected envelope schema')
    if meta['version'] != PROTOCOL_VERSION or isinstance(meta['version'], bool): raise WireError('Unknown protocol version')
    validate_identity(meta['robot_id'], meta['run_id'], meta['cutoff_ns'])
    if (meta['robot_id'], meta['run_id'], meta['cutoff_ns']) != (robot_id, run_id, cutoff_ns):
        raise WireError('Sensor envelope epoch/robot differs')
    _integer(meta['sequence'], 'lane sequence', 1); _integer(meta['stamp_ns'], 'sensor stamp')
    if lane is not None and meta['lane'] != lane: raise WireError('Sensor lane differs from topic')
    keys = lane_keys(meta['lane'])
    if meta['source_caller'] != lane_source_caller(robot_id, meta['lane']):
        raise WireError('Foreign sensor source')
    if meta['lane'] != 'ready' and meta['stamp_ns'] < cutoff_ns: raise WireError('Pre-epoch sensor stamp')
    if meta['lane'] == 'ready' and meta['stamp_ns'] != 0: raise WireError('Headerless ready was stamped')
    length = _integer(meta['raw_bytes'], 'raw payload length', 1)
    if length > MAX_RAW_BYTES: raise WireError('Raw-byte bound exceeded')
    descriptors = meta['streams']
    if not isinstance(descriptors, list) or len(descriptors) != len(keys): raise WireError('Invalid lane stream list')
    total = 0
    for key, item in zip(keys, descriptors):
        if not isinstance(item, dict) or set(item) != {'key', 'type', 'bytes'}:
            raise WireError('Invalid stream descriptor')
        if item['key'] != key or item['type'] != STREAMS[key]['type']: raise WireError('Foreign stream/schema')
        total += _integer(item['bytes'], 'serialized stream length', 1)
    if total != length: raise WireError('Stream lengths differ from payload')
    payload = packet[offset+size:-32]
    if meta['codec'] == 'zlib':
        try:
            decoder = zlib.decompressobj(); raw = decoder.decompress(payload, length+1)
            if (len(raw) != length or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail):
                raise WireError('Compressed payload has wrong length or trailing data')
        except zlib.error as error: raise WireError('Invalid compressed payload: ' + str(error))
    elif meta['codec'] == 'none': raw = payload
    else: raise WireError('Unknown sensor codec')
    if len(raw) != length: raise WireError('Serialized payload length differs')
    result = {}; position = 0
    for item in descriptors:
        result[item['key']] = raw[position:position+item['bytes']]; position += item['bytes']
    return meta, result


class FrameBuffer:
    """Small exact-stamp cache; compression/publication run outside this lock."""
    def __init__(self, cutoff_ns, max_pending=8, max_age_ns=2_000_000_000):
        _integer(cutoff_ns, 'epoch cutoff', 1); _integer(max_pending, 'cache capacity', 1)
        _integer(max_age_ns, 'frame age bound', 1)
        if max_pending > 128: raise WireError('Cache capacity exceeds 128 frames')
        self.cutoff_ns, self.max_pending, self.max_age_ns = cutoff_ns, max_pending, max_age_ns
        self.pending = {}; self.last_taken_ns = 0; self.lock = threading.Lock()
        self.counts = Counter(); self.maximum_pending = 0

    def put(self, key, stamp, data):
        if key not in RGBD_KEYS: raise WireError('Not an RGB-D stream')
        _integer(stamp, 'frame stamp', 1)
        if not isinstance(data, bytes) or not 0 < len(data) <= MAX_RAW_BYTES:
            raise WireError('Invalid serialized frame message')
        with self.lock:
            if stamp < self.cutoff_ns or stamp <= self.last_taken_ns:
                self.counts['old_or_pre_epoch_parts_omitted'] += 1; return False
            frame = self.pending.setdefault(stamp, {})
            if key in frame:
                if frame[key] != data: raise WireError('Same frame part changed within its stamp')
                self.counts['duplicate_parts'] += 1; return False
            if sum(len(v) for v in frame.values()) + len(data) > MAX_RAW_BYTES:
                del self.pending[stamp]; raise WireError('RGB-D frame exceeds raw-byte bound')
            frame[key] = data; self.counts['parts_received'] += 1
            while len(self.pending) > self.max_pending:
                del self.pending[min(self.pending)]; self.counts['frames_omitted_cache_limit'] += 1
            self.maximum_pending = max(self.maximum_pending, len(self.pending)); return True

    def take(self, now_ns):
        with self.lock:
            for stamp in list(self.pending):
                if now_ns-stamp > self.max_age_ns:
                    del self.pending[stamp]; self.counts['frames_omitted_age_limit'] += 1
            ready = sorted(stamp for stamp, parts in self.pending.items() if set(parts) == set(RGBD_KEYS))
            if not ready: return None
            stamp = ready[0]; parts = self.pending.pop(stamp)
            for old in [s for s in self.pending if s < stamp]:
                del self.pending[old]; self.counts['incomplete_older_frames_omitted'] += 1
            self.last_taken_ns = stamp; self.counts['frames_taken'] += 1
            return stamp, parts

    def snapshot(self):
        with self.lock:
            return dict(pending_rgbd=len(self.pending), maximum_pending_rgbd=self.maximum_pending,
                        max_pending_rgbd=self.max_pending, counts=dict(self.counts))


class Receiver:
    """Independent per-lane synchronization, age and monotonicity enforcement."""
    def __init__(self, robot_id, run_id, cutoff_ns, max_age_ns=2_000_000_000):
        validate_identity(robot_id, run_id, cutoff_ns)
        _integer(max_age_ns, 'receiver age bound', 1)
        self.robot_id, self.run_id, self.cutoff_ns = robot_id, run_id, cutoff_ns
        self.max_age_ns = max_age_ns
        self.locks = {lane: threading.Lock() for lane in LANES}
        self.last_sequences = Counter(); self.last_stamps = Counter(); self.counts = {lane: Counter() for lane in LANES}

    def receive(self, lane, packet, caller, now_ns):
        if caller != uplink_caller(self.robot_id): raise WireError('Foreign uplink ROS publisher')
        if lane not in LANES: raise WireError('Unknown receiver lane')
        # Big RGB-D decompression cannot hold the small IMU lane's lock.
        with self.locks[lane]:
            meta, messages = decode_packet(packet, self.robot_id, self.run_id, self.cutoff_ns, lane)
            sequence, stamp = meta['sequence'], meta['stamp_ns']; count = self.counts[lane]
            if sequence <= self.last_sequences[lane]: raise WireError('Repeated or regressed lane sequence')
            if lane != 'ready':
                if stamp <= self.last_stamps[lane]: raise WireError('Regressed sensor stamp')
                if now_ns-stamp > self.max_age_ns: raise WireError('Sensor envelope is stale')
                if stamp-now_ns > 500_000_000: raise WireError('Sensor clock is more than 0.5 seconds ahead')
            previous = self.last_sequences[lane]
            if previous: count['sequence_gaps'] += sequence-previous-1
            self.last_sequences[lane] = sequence; self.last_stamps[lane] = stamp
            count['envelopes_accepted'] += 1; count['wire_bytes'] += len(packet)
            count['maximum_source_age_ns'] = max(count['maximum_source_age_ns'], now_ns-stamp if stamp else 0)
            count['last_source_stamp_ns'] = stamp; count['last_receive_wall_ns'] = now_ns
            return meta, messages

    def snapshot(self):
        result = {}
        for lane in LANES:
            with self.locks[lane]: result[lane] = dict(self.counts[lane])
        return result
