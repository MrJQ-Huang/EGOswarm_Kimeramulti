#!/usr/bin/env python3
"""Edge read-only result server; never records data or sends under a SLAM lock.

ROS AnyMsg callbacks retain one bounded immutable byte snapshot per stream.
Independent socket workers compress/send map results and 30 Hz telemetry.
A slow, absent, or reconnecting observer cannot backpressure native publishers.
"""
import argparse
from collections import Counter
import socket
import struct
import threading
import time
import uuid

from result_wire import (VERSION, STREAMS, MESH_STREAMS, MAX_RAW_BYTES, MAX_HEADER_BYTES,WireError,
    identity, contract, content_hash, sha256, read_header, ros_string, json_bytes,
    receipt_proves_shared, snapshot_identity, encode_packet, receive_control, send_control)


class LatestResults:
    """One snapshot per fixed stream, with no outgoing queue and no filesystem IO."""
    def __init__(self, robot, run_id, cutoff_ns, gauge_id, boot_id=None, now=time.time_ns):
        self.boot_id = boot_id or uuid.uuid4().hex
        identity(robot, run_id, cutoff_ns, self.boot_id)
        from result_wire import IDENTIFIER
        if not isinstance(gauge_id, str) or not IDENTIFIER.fullmatch(gauge_id): raise WireError('Invalid gauge ID')
        self.robot, self.run_id, self.cutoff_ns, self.gauge_id = robot, run_id, cutoff_ns, gauge_id
        self.now = now; self.lock = threading.Lock(); self.latest = {}; self.counts = Counter()
        self.native_status = None; self.heartbeat_sequences = Counter()

    def ingest(self, stream, raw, connection_header, graph_nodes=None):
        if stream == 'heartbeat': raise WireError('Heartbeat is generated locally')
        spec = contract(self.robot, stream)
        if not isinstance(raw, bytes): raw = bytes(raw)
        if not 0 < len(raw) <= MAX_RAW_BYTES: raise WireError('Oversized/empty source result')
        if (connection_header.get('callerid') != spec['source_caller'] or
                connection_header.get('type') != spec['ros_type']): raise WireError('Wrong native source caller/type')
        from result_wire import MD5
        md5 = connection_header.get('md5sum', '')
        if not MD5.fullmatch(md5): raise WireError('Missing native ROS MD5')
        if stream == 'status':
            if len(raw) > MAX_HEADER_BYTES: raise WireError('Native status exceeds metadata limit')
            status = ros_string(raw)
            if not isinstance(status, dict) or (status.get('run_id'), status.get('robot_id'), status.get('cutoff_ns')) != (
                    self.run_id, self.robot, self.cutoff_ns): raise WireError('Native status belongs to a different epoch/robot')
            stamp, frame, basis = 0, '', 'headerless'
        else:
            status = None; stamp, frame = read_header(raw)
            basis = 'header'
            if stream == 'optimized_graph' and stamp == 0:
                if not graph_nodes or any(type(robot) is not int or robot != self.robot or type(value) is not int
                        or value < self.cutoff_ns or value > self.now()+2*10**9 for robot,value in graph_nodes):
                    raise WireError('Zero-header optimized graph lacks verified current-epoch native node stamps')
                stamp = max(value for _,value in graph_nodes); basis = 'latest_graph_node'
            if stamp < self.cutoff_ns or stamp > self.now() + 2*10**9: raise WireError('Old/future native result stamp')
        fingerprint = content_hash(raw, stream)
        raw_hash = sha256(raw)
        # Hashing/validation happen outside the tiny replacement lock. Incoming
        # ROS bytes are immutable; this process shares no locks with SLAM.
        with self.lock:
            old = self.latest.get(stream)
            if old and old['content_sha256'] == fingerprint:
                self.counts[stream + '_unchanged'] += 1; return False
            sequence = old['sequence'] + 1 if old else 1
            self.latest[stream] = dict(raw=raw, sequence=sequence, ros_md5=md5,
                stamp_ns=stamp,stamp_basis=basis, frame_id=frame, content_sha256=fingerprint, result_sha256=raw_hash)
            if status is not None: self.native_status = status
            current = self.native_status or {}
            receipt = current.get('accepted_joint_mesh_receipt')
            if isinstance(receipt, dict):
                for key, entry in list(self.latest.items()):
                    if key in ('mesh', 'accepted_mesh') and receipt.get('canonical_native_mesh_sha256') == entry['content_sha256']:
                        # Keep the proof for this exact immutable snapshot even
                        # when a later status already describes the next mesh.
                        self.latest[key] = dict(entry, native_receipt=receipt)
            self.counts[stream + '_changed'] += 1
            return True

    def entries(self, channel):
        with self.lock:
            entries = [(key, value) for key, value in self.latest.items() if STREAMS[key][0] == channel]
            status = self.native_status
        return entries, status

    def metadata(self, stream, entry, status=None, channel=None):
        spec = contract(self.robot, stream)
        receipt = None; shared = False
        if stream in ('mesh', 'accepted_mesh') and isinstance(status, dict):
            candidate = entry.get('native_receipt') or status.get('accepted_joint_mesh_receipt')
            if isinstance(candidate, dict) and candidate.get('canonical_native_mesh_sha256') == entry['content_sha256']:
                receipt = candidate
                shared = (receipt.get('shared_gauge_id') == self.gauge_id and receipt_proves_shared(
                        receipt, entry['content_sha256'], entry['frame_id'], self.gauge_id,
                        self.run_id, self.robot, self.cutoff_ns))
        meta = dict(protocol=VERSION, run_id=self.run_id, robot_id=self.robot,
            cutoff_ns=self.cutoff_ns, boot_id=self.boot_id, stream=stream,
            channel=channel or spec.pop('channel'), sequence=entry['sequence'],
            stamp_ns=entry['stamp_ns'],stamp_basis=entry.get('stamp_basis','headerless' if stream=='heartbeat' else 'header'), frame_id=entry['frame_id'], mode='full',
            base_sha256=None, result_sha256=entry.get('result_sha256') or sha256(entry['raw']), content_sha256=entry['content_sha256'],
            raw_bytes=len(entry['raw']), encoded_raw_bytes=len(entry['raw']),codec='none', gauge_id=self.gauge_id,
            shared_gauge=shared, native_receipt=receipt,
            ros_md5=entry['ros_md5'], **{key:value for key,value in spec.items() if key != 'channel'})
        meta['snapshot_id'] = snapshot_identity(meta)
        return meta

    def heartbeat(self, channel):
        with self.lock:
            self.heartbeat_sequences[channel] += 1
            sequence = self.heartbeat_sequences[channel]
            data = json_bytes(dict(wall_ns=self.now(), counters=dict(self.counts),
                channel=channel, edge_data_persistence=False, queue_policy='one_latest_per_stream'))
        raw = struct.pack('<I', len(data)) + data
        entry = dict(raw=raw, sequence=sequence, ros_md5='992ce8a1687cec8c8bd883ec73ca41d1',
            stamp_ns=0, frame_id='', content_sha256=sha256(raw))
        return self.metadata('heartbeat', entry, channel=channel), raw


class ResultServer:
    """One observer connection per channel, independent of every other channel."""
    def __init__(self, latest, channel, bind, port, rate, stop=None, socket_timeout=10., compression_level=1, delta=False):
        if channel not in ('map', 'telemetry') or rate <= 0 or socket_timeout <= 0:
            raise ValueError('Invalid result server channel/rate/timeout')
        self.latest, self.channel, self.bind, self.port, self.rate = latest, channel, bind, port, rate
        self.stop = stop or threading.Event(); self.socket_timeout = socket_timeout
        self.compression_level = compression_level; self.counts = Counter(); self.ready = threading.Event()
        self.delta = delta
        self.bound_port = None; self.error = None

    def serve(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.bind, self.port)); listener.listen(1); listener.settimeout(.2)
                self.bound_port = listener.getsockname()[1]; self.ready.set()
                while not self.stop.is_set():
                    try: connection, _ = listener.accept()
                    except socket.timeout: continue
                    with connection:
                        connection.settimeout(self.socket_timeout)
                        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        try: self.stream(connection)
                        except (OSError, EOFError, ValueError): self.counts['disconnects_or_rejected_handshakes'] += 1
        except Exception as error:
            self.error = error; self.ready.set()

    def stream(self, connection):
        request = receive_control(connection)
        accept_delta = request.pop('accept_delta',False) if isinstance(request,dict) else False
        if type(accept_delta) is not bool: raise WireError('Invalid delta negotiation')
        expected = dict(op='subscribe', protocol=VERSION, robot_id=self.latest.robot,
            run_id=self.latest.run_id, cutoff_ns=self.latest.cutoff_ns, channel=self.channel)
        if request != expected: raise WireError('Observer subscribed to a different run/robot/channel')
        delta_enabled = bool(self.delta and accept_delta and self.channel == 'map')
        send_control(connection, dict(expected, op='subscribed', boot_id=self.latest.boot_id,
            mode='full', base_required=False, delta_enabled=delta_enabled,application_snapshot_is_native_atomic_round=False))
        self.counts['connections'] += 1
        sent = {}; encoded = {}; bases = {}; last_heartbeat = 0.
        while not self.stop.is_set():
            began = time.monotonic()
            entries, status = self.latest.entries(self.channel)
            # Native status goes first; it is small and provides diagnostics.
            for key, entry in sorted(entries, key=lambda item: item[0] != 'status'):
                meta = self.latest.metadata(key, entry, status)
                version = meta['snapshot_id']
                if sent.get(key) == version: continue
                # Compression, socket backpressure and wire hashes are outside
                # the replacement lock, and never inside a native SLAM process.
                cache_key = (entry['sequence'], meta['shared_gauge'], sha256(json_bytes(meta['native_receipt'])))
                if key not in encoded or encoded[key][0] != cache_key:
                    encoded[key] = (cache_key, encode_packet(meta, entry['raw'],
                        self.compression_level if self.channel == 'map' else 0,
                        bases.get(key) if delta_enabled else None))
                connection.sendall(encoded[key][1])
                if delta_enabled:
                    response = receive_control(connection)
                    if response == dict(op='full_required',stream=key,snapshot_id=version):
                        connection.sendall(encode_packet(meta,entry['raw'],self.compression_level))
                        self.counts['missing_base_full_resends'] += 1
                        response = receive_control(connection)
                    if response != dict(op='ack',stream=key,snapshot_id=version,result_sha256=meta['result_sha256']):
                        raise WireError('Result receiver did not acknowledge this exact snapshot')
                    bases[key] = entry['raw']
                sent[key] = version
                self.counts[key + '_sent'] += 1
            if time.monotonic() - last_heartbeat >= 1.:
                meta, raw = self.latest.heartbeat(self.channel)
                connection.sendall(encode_packet(meta, raw, 0)); last_heartbeat = time.monotonic()
            self.stop.wait(max(0., 1./self.rate - (time.monotonic()-began)))


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', type=int, choices=(0,1), required=True)
    parser.add_argument('--run-id', required=True); parser.add_argument('--cutoff', type=float, required=True)
    parser.add_argument('--gauge-id', required=True, help='Configuration identity only; never shared-gauge proof')
    parser.add_argument('--bind', required=True, help='Edge interface address, not the hub')
    parser.add_argument('--map-port', type=int, default=11740); parser.add_argument('--telemetry-port', type=int, default=11741)
    parser.add_argument('--map-rate', type=float, default=2.); parser.add_argument('--telemetry-rate', type=float, default=30.)
    parser.add_argument('--socket-timeout', type=float, default=10.)
    parser.add_argument('--compression-level', type=int, choices=range(10), default=1)
    parser.add_argument('--delta',action='store_true',help='Negotiate exact byte-block deltas after a full acknowledged baseline')
    return parser


def main(argv=None):
    import math
    parser = argument_parser()
    args = parser.parse_args(argv)
    if not all(math.isfinite(value) and value > 0 for value in
            (args.cutoff, args.map_rate, args.telemetry_rate, args.socket_timeout)):
        parser.error('Cutoff, rates and timeout must be positive and finite')
    if args.map_port == args.telemetry_port or not all(0 < p < 65536 for p in (args.map_port,args.telemetry_port)):
        parser.error('Two distinct valid result ports are required')
    import rospy
    import roslib.message
    latest = LatestResults(args.robot_id, args.run_id, round(args.cutoff*1e9), args.gauge_id)
    rospy.init_node('sim_%d_result_stream' % args.robot_id)
    stop = threading.Event(); subscribers = []
    for key, (channel, kind_name, _, _) in STREAMS.items():
        if key == 'heartbeat': continue
        kind = roslib.message.get_message_class(kind_name)
        if kind is None: raise RuntimeError('Missing local ROS result type: ' + kind_name)
        spec = contract(args.robot_id, key)
        def receive(message, stream=key, expected_md5=kind._md5sum,message_kind=kind):
            try:
                header = getattr(message, '_connection_header', {}) or {}
                if header.get('md5sum') != expected_md5: raise WireError('Native source message MD5 differs')
                graph_nodes = None
                if stream == 'optimized_graph' and read_header(message._buff)[0] == 0:
                    decoded = message_kind(); decoded.deserialize(message._buff)
                    graph_nodes = [(int(node.robot_id),int(node.header.stamp.to_nsec())) for node in decoded.nodes]
                latest.ingest(stream, message._buff, header,graph_nodes)
            except Exception as error:
                with latest.lock: latest.counts[stream + '_rejected'] += 1
                rospy.logwarn_throttle(2., 'Result source %s rejected: %s', stream, str(error))
        subscribers.append(rospy.Subscriber(spec['source_topic'], rospy.AnyMsg, receive,
            queue_size=1, buff_size=2**26 if key in MESH_STREAMS else 2**20, tcp_nodelay=True))
    servers = [ResultServer(latest, channel, args.bind, port, rate, stop,
                args.socket_timeout, args.compression_level,args.delta)
        for channel, port, rate in (('map',args.map_port,args.map_rate),('telemetry',args.telemetry_port,args.telemetry_rate))]
    threads = [threading.Thread(target=server.serve, name='result-'+server.channel, daemon=True) for server in servers]
    for thread in threads: thread.start()
    try:
        while not rospy.is_shutdown():
            if any(server.error for server in servers): raise RuntimeError(str([server.error for server in servers]))
            stop.wait(.2)
    finally:
        stop.set()
        for subscriber in subscribers: subscriber.unregister()
        for thread in threads: thread.join(timeout=.5)


if __name__ == '__main__': main()
