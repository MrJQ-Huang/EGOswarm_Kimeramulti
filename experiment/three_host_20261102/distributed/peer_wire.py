"""Bounded peer envelopes and reliable in-memory delivery ledgers; no ROS deps.

ACK means local publication has returned, not that every ROS consumer has
processed the message. Duplicate suppression survives socket reconnection in
the same process. A peer process boot change fails closed: it cannot provide
exactly-once application recovery after an optimizer or bridge restart.
"""
from collections import Counter, OrderedDict
import hashlib
import io
import json
import re
import struct
import threading

from peer_topics import LANES, topic_specs, validate_source


MAGIC = b'KMPEER1\0'
PREFIX = struct.Struct('!8sIQ')
MAX_HEADER = 16384
MAX_PAYLOAD = 32 * 1024 * 1024
IDENTITY = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}')
BOOT = re.compile(r'[A-Za-z0-9_-]{8,96}')
MD5 = re.compile(r'[a-f0-9]{32}')
SHA = re.compile(r'[a-f0-9]{64}')


class WireError(ValueError):
    pass


class QueueFull(WireError):
    pass


def integer(value, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise WireError('Invalid integer')
    return value


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=True, allow_nan=False).encode('ascii')


def canonical_payload_sha256(payload, has_header=False):
    if not isinstance(payload, bytes):
        raise WireError('Payload must be bytes')
    if has_header:
        if len(payload) < 4:
            raise WireError('Truncated ROS Header')
        payload = b'\0\0\0\0' + payload[4:]
    return hashlib.sha256(payload).hexdigest()


def native_message_sha256(message):
    """Ignore only a top-level Header.seq; never mutate the supplied message."""
    buffer = io.BytesIO()
    message.serialize(buffer)
    slots = getattr(message, '_slot_types', ())
    header = (getattr(message, '_has_header', False) or
              getattr(message, '_type', '') == 'std_msgs/Header' or
              bool(slots and slots[0] == 'std_msgs/Header'))
    return canonical_payload_sha256(buffer.getvalue(), header)


def validate_identity(robot, run_id, boot_id, cutoff_ns, lane):
    if isinstance(robot, bool) or robot not in (0, 1):
        raise WireError('Unknown robot')
    if not isinstance(run_id, str) or not IDENTITY.fullmatch(run_id):
        raise WireError('Invalid run ID')
    if not isinstance(boot_id, str) or not BOOT.fullmatch(boot_id):
        raise WireError('Invalid boot ID')
    integer(cutoff_ns, 1)
    if lane not in LANES:
        raise WireError('Unknown lane')


def encode_frame(header, payload=b''):
    if not isinstance(payload, bytes) or len(payload) > MAX_PAYLOAD:
        raise WireError('Oversized or non-byte payload')
    raw = canonical_json(header)
    if len(raw) > MAX_HEADER:
        raise WireError('Oversized header')
    return PREFIX.pack(MAGIC, len(raw), len(payload)) + raw + payload


def _exact(sock, count):
    parts = []
    remaining = count
    while remaining:
        part = sock.recv(min(remaining, 65536))
        if not part:
            raise EOFError('Peer closed the stream')
        parts.append(part)
        remaining -= len(part)
    return b''.join(parts)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise WireError('Duplicate metadata key')
        result[key] = value
    return result


def _frame_lengths(prefix):
    if len(prefix) != PREFIX.size:
        raise WireError('Truncated frame prefix')
    magic, header_size, payload_size = PREFIX.unpack(prefix)
    if magic != MAGIC or not 2 <= header_size <= MAX_HEADER or payload_size > MAX_PAYLOAD:
        raise WireError('Invalid frame prefix or length')
    return header_size, payload_size


def _decode_header(raw):
    try:
        header = json.loads(raw.decode('ascii'),
                            object_pairs_hook=_unique_object)
    except (UnicodeError, ValueError) as error:
        raise WireError('Invalid metadata JSON') from error
    if not isinstance(header, dict):
        raise WireError('Expected metadata object')
    return header


def read_frame(sock):
    header_size, payload_size = _frame_lengths(_exact(sock, PREFIX.size))
    header = _decode_header(_exact(sock, header_size))
    return header, _exact(sock, payload_size)


def decode_frame_bytes(data):
    """Decode exactly one in-process ROS envelope, including strict boundaries.

    A native-control envelope places provenance and the untouched serialized
    native ROS message in the same TCPROS message, avoiding a second receipt
    topic's independent scheduling and queue loss.
    """
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise WireError('Envelope must be bytes')
    if len(data) > PREFIX.size + MAX_HEADER + MAX_PAYLOAD:
        raise WireError('Envelope exceeds the bounded frame size')
    raw = bytes(data)
    header_size, payload_size = _frame_lengths(raw[:PREFIX.size])
    if len(raw) != PREFIX.size + header_size + payload_size:
        raise WireError('Truncated or concatenated native-control envelope')
    end = PREFIX.size + header_size
    return _decode_header(raw[PREFIX.size:end]), raw[end:]


def is_native_control_topic(topic, source_robot):
    return topic in ('/sim_%d/dpgo_ros_node/command' % source_robot,
                     '/sim_%d/dpgo_ros_node/measurement_weights' % source_robot)


def encode_native_control(receipt, payload, control_sequence):
    """Atomic metadata+payload for the two native controls consumed by dense.

    This counter is consecutive across command and measurement_weights only.
    The original peer control-lane sequence is retained separately as sequence.
    Subscriber-side gaps must fail explicitly; they must not wait forever for
    a receipt from an independently ordered status or ROS topic.
    """
    integer(control_sequence, 1)
    source = receipt.get('source_robot_id')
    if (source not in (0, 1) or not is_native_control_topic(receipt.get('topic'), source)
            or receipt.get('verified') is not True or receipt.get('lane') != 'control'
            or receipt.get('source_caller') != '/sim_%d/dpgo_ros_node/agent' % source
            or receipt.get('wire_sha256') != hashlib.sha256(payload).hexdigest()
            or receipt.get('payload_bytes') != len(payload)
            or receipt.get('sha256') != canonical_payload_sha256(payload, receipt.get('has_header', False))):
        raise WireError('Native control lacks matching verified provenance/payload')
    metadata = dict(receipt, kind='native_control', control_sequence=control_sequence,
                    local_boot_id=receipt['peer_boot_id'])
    return encode_frame(metadata, payload)


def base_header(kind, robot, run_id, boot_id, cutoff_ns, lane, peer_boot_id=None):
    validate_identity(robot, run_id, boot_id, cutoff_ns, lane)
    return dict(version=1, kind=kind, robot_id=robot, run_id=run_id,
                boot_id=boot_id, cutoff_ns=cutoff_ns, lane=lane,
                peer_boot_id=peer_boot_id)


def validate_base(header, *, kind, robot, run_id, boot_id, peer_boot_id, cutoff_ns, lane):
    validate_identity(header.get('robot_id'), header.get('run_id'),
                      header.get('boot_id'), header.get('cutoff_ns'), header.get('lane'))
    expected = dict(version=1, kind=kind, robot_id=robot, run_id=run_id,
                    boot_id=boot_id, peer_boot_id=peer_boot_id,
                    cutoff_ns=cutoff_ns, lane=lane)
    if any(header.get(k) != value for k, value in expected.items()):
        raise WireError('Peer session identity differs')


def data_header(robot, run_id, boot_id, cutoff_ns, lane, sequence, topic,
                md5, source_caller, created_ns, payload, has_header=False):
    spec = topic_specs(robot).get(topic)
    if spec is None or spec.lane != lane:
        raise WireError('Topic is outside the peer lane whitelist')
    validate_source(spec, source_caller)
    integer(sequence, 1)
    integer(created_ns, cutoff_ns)
    if not isinstance(md5, str) or not MD5.fullmatch(md5):
        raise WireError('Invalid ROS MD5')
    if not isinstance(payload, bytes) or len(payload) > MAX_PAYLOAD:
        raise WireError('Invalid ROS payload')
    header = base_header('data', robot, run_id, boot_id, cutoff_ns, lane)
    header.update(sequence=sequence, topic=topic, type=spec.type, md5=md5,
                  source_caller=source_caller, created_ns=created_ns,
                  payload_bytes=len(payload), has_header=bool(has_header),
                  wire_sha256=hashlib.sha256(payload).hexdigest(),
                  sha256=canonical_payload_sha256(payload, has_header))
    return header


def validate_data(header, payload, contracts):
    spec = topic_specs(header['robot_id']).get(header.get('topic'))
    if spec is None or spec.lane != header['lane'] or spec.type != header.get('type'):
        raise WireError('Unapproved peer topic/type/lane')
    try:
        validate_source(spec, header.get('source_caller'))
    except ValueError as error:
        raise WireError(str(error)) from error
    integer(header.get('sequence'), 1)
    integer(header.get('created_ns'), header['cutoff_ns'])
    contract = contracts.get(spec.type)
    if (contract is None or header.get('md5') != contract['md5'] or
            header.get('has_header') is not contract['has_header']):
        raise WireError('Peer ROS schema differs')
    if header.get('payload_bytes') != len(payload) or len(payload) > MAX_PAYLOAD:
        raise WireError('Payload length differs')
    if header.get('wire_sha256') != hashlib.sha256(payload).hexdigest():
        raise WireError('Wire payload integrity differs')
    if header.get('sha256') != canonical_payload_sha256(payload, contract['has_header']):
        raise WireError('Canonical ROS payload integrity differs')


def contract_digest(contracts):
    required = {spec.type for robot in (0, 1) for spec in topic_specs(robot).values()}
    if set(contracts) != required:
        raise WireError('Incomplete or foreign ROS contracts')
    for item in contracts.values():
        if (not isinstance(item.get('md5'), str) or not MD5.fullmatch(item['md5']) or
                not isinstance(item.get('has_header'), bool)):
            raise WireError('Invalid ROS contract')
    return hashlib.sha256(canonical_json(contracts)).hexdigest()


class ReliableOutbox:
    """Cumulative ACK queue: all peer optimizer messages remain until ACK."""
    def __init__(self, max_messages=512, max_bytes=64 * 1024 * 1024):
        self.max_messages = integer(max_messages, 1)
        self.max_bytes = integer(max_bytes, 1)
        self.items = OrderedDict()
        self.bytes = 0
        self.next_sequence = 1
        self.last_acked = self.highest_sent = 0
        self.lock = threading.RLock()
        self.counts = Counter()

    def enqueue(self, builder, payload):
        with self.lock:
            if len(self.items) >= self.max_messages or self.bytes + len(payload) > self.max_bytes:
                self.counts['overflow_fatal'] += 1
                raise QueueFull('Reliable peer queue exhausted; cold restart required')
            sequence = self.next_sequence
            header = builder(sequence)
            self.items[sequence] = (header, payload)
            self.bytes += len(payload)
            self.next_sequence += 1
            self.counts['enqueued'] += 1
            return sequence

    def next_after(self, cursor):
        with self.lock:
            for sequence, value in self.items.items():
                if sequence > cursor:
                    self.highest_sent = max(self.highest_sent, sequence)
                    return sequence, value
        return None

    def acknowledge(self, sequence):
        integer(sequence)
        with self.lock:
            if sequence > self.highest_sent:
                raise WireError('ACK exceeds the sent sequence')
            if sequence <= self.last_acked:
                return
            for key in list(self.items):
                if key > sequence:
                    break
                self.bytes -= len(self.items.pop(key)[1])
                self.counts['acked'] += 1
            self.last_acked = sequence

    def snapshot(self):
        with self.lock:
            return dict(pending=len(self.items), pending_bytes=self.bytes,
                        next_sequence=self.next_sequence, last_acked=self.last_acked,
                        highest_sent=self.highest_sent, counts=dict(self.counts))


class ReceiveLedger:
    """One ordered lane; callback completion precedes sequence acceptance."""
    def __init__(self, remembered=1024):
        self.last_sequence = 0
        self.remembered = integer(remembered, 1)
        self.fingerprints = OrderedDict()
        self.counts = Counter()
        self.lock = threading.RLock()
        self.state_lock = threading.Lock()

    def deliver(self, header, payload, callback):
        sequence = integer(header.get('sequence'), 1)
        fingerprint = hashlib.sha256(canonical_json(header)).hexdigest()
        with self.lock:
            if sequence <= self.last_sequence:
                if self.fingerprints.get(sequence) != fingerprint:
                    raise WireError('Unknown or conflicting replay')
                with self.state_lock:
                    self.counts['duplicate_suppressed'] += 1
                return False
            if sequence != self.last_sequence + 1:
                raise WireError('Reliable peer sequence has a gap')
            callback(header, payload)
            with self.state_lock:
                self.last_sequence = sequence
                self.counts['delivered'] += 1
            self.fingerprints[sequence] = fingerprint
            while len(self.fingerprints) > self.remembered:
                self.fingerprints.popitem(last=False)
            return True

    def snapshot(self):
        # Diagnostics must not wait for a slow native ROS publication while
        # holding the delivery serialization lock.
        with self.state_lock:
            return dict(last_sequence=self.last_sequence, counts=dict(self.counts))


def validate_ingress_receipt(receipt, *, run_id, cutoff_ns, local_robot_id,
                             peer_boot_id, topic, sha256):
    """Check bridge metadata; the subscriber must also check actual ROS caller."""
    peer = 1 - local_robot_id
    spec = topic_specs(peer).get(topic)
    if (spec is None or receipt.get('verified') is not True or
            receipt.get('run_id') != run_id or receipt.get('cutoff_ns') != cutoff_ns or
            receipt.get('local_robot_id') != local_robot_id or
            receipt.get('peer_robot_id') != peer or
            receipt.get('source_robot_id') != peer or
            receipt.get('source_boot_id') != peer_boot_id or
            receipt.get('topic') != topic or receipt.get('sha256') != sha256 or
            receipt.get('source_caller') not in spec.callers):
        raise WireError('Ingress receipt differs from native source/session')
    return True
