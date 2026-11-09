#!/usr/bin/env python3
"""Direct peer bridge between two independent ROS1 masters.

Robot zero dials robot one. Each of three independent TCP lanes is full duplex.
All optimizer topics share the ordered control lane. Transport ACKs confirm
proxy publication, not native optimizer consumption; native iteration/cluster
checks remain essential. Reconnection retains queues/dedup ledgers in RAM.
Process boot changes, control expiration or queue overflow require a new run.
No raw ROS payload is written to disk. This is not the author's UDP RTM.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import socket
import threading
import time
import uuid

from peer_topics import (LANES, proxy_caller, requires_native_subscriber,
                         topic_specs, validate_source)
from peer_wire import (QueueFull, ReceiveLedger, ReliableOutbox, WireError,
                       base_header, canonical_json, contract_digest, data_header,
                       encode_frame, encode_native_control, is_native_control_topic,
                       read_frame, validate_base, validate_data,
                       validate_identity)


class PeerTransport:
    """ROS-independent engine, also used by real-loopback integration tests."""
    def __init__(self, robot_id, run_id, cutoff_ns, contracts, on_message,
                 listen_host='0.0.0.0', listen_port=11640, peer_host=None,
                 peer_port=11640, boot_id=None, max_messages=512,
                 max_bytes=64 * 1024 * 1024, max_control_age=10.,
                 max_ego_age=1., health_timeout=5.):
        self.robot = robot_id
        self.peer = 1 - robot_id
        self.run_id = run_id
        self.cutoff_ns = cutoff_ns
        self.boot_id = boot_id or uuid.uuid4().hex
        validate_identity(robot_id, run_id, self.boot_id, cutoff_ns, 'control')
        if not peer_host:
            raise ValueError('A direct peer host is required')
        if not all(math.isfinite(v) and v > 0 for v in
                   (max_control_age, max_ego_age, health_timeout)):
            raise ValueError('Transport ages must be finite and positive')
        self.contracts = contracts
        self.schema_sha256 = contract_digest(contracts)
        self.on_message = on_message
        self.listen_host, self.listen_port = listen_host, listen_port
        self.peer_host, self.peer_port = peer_host, peer_port
        self.peer_ip = socket.gethostbyname(peer_host)
        self.max_control_age = max_control_age
        self.max_ego_age = max_ego_age
        self.health_timeout = health_timeout
        self.outboxes = {lane: ReliableOutbox(max_messages, max_bytes) for lane in LANES}
        self.ledgers = {lane: ReceiveLedger(max(1024, max_messages * 2)) for lane in LANES}
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.connections = {}
        self.peer_boot_id = None
        self.fatal = None
        self.counts = Counter()
        self.threads = []
        self.server = None

    def _thread(self, function, *args):
        thread = threading.Thread(target=function, args=args, daemon=True)
        with self.lock:
            self.threads = [item for item in self.threads if item.is_alive()]
            self.threads.append(thread)
            thread.start()

    def start(self):
        if self.server is not None:
            raise RuntimeError('Peer transport already started')
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.listen_host, self.listen_port))
        self.server.listen(8)
        self.server.settimeout(.25)
        self.listen_port = self.server.getsockname()[1]
        self._thread(self._accept)
        if self.robot == 0:
            for lane in LANES:
                self._thread(self._dial, lane)

    def _close(self, sock):
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def disconnect(self):
        """Drop only TCP connections; keep same-process delivery state."""
        with self.lock:
            connections = list(self.connections.values())
        for connection in connections:
            self._close(connection['socket'])

    def shutdown(self):
        self.stop_event.set()
        self.disconnect()
        if self.server is not None:
            self._close(self.server)
        for thread in list(self.threads):
            if thread is not threading.current_thread():
                thread.join(timeout=1.)

    def fail(self, error):
        with self.lock:
            if self.fatal is None:
                self.fatal = str(error)
                self.counts['fatal'] += 1
        self.disconnect()

    def enqueue(self, topic, payload, source_caller, md5, created_ns=None):
        with self.lock:
            if self.fatal:
                raise WireError('Bridge has failed; require a new run: ' + self.fatal)
        spec = topic_specs(self.robot).get(topic)
        if spec is None:
            raise WireError('Unapproved local peer export')
        validate_source(spec, source_caller)
        contract = self.contracts[spec.type]
        if md5 != contract['md5']:
            raise WireError('Local publisher MD5 differs')
        created_ns = time.time_ns() if created_ns is None else created_ns
        try:
            return self.outboxes[spec.lane].enqueue(
                lambda seq: data_header(self.robot, self.run_id, self.boot_id,
                    self.cutoff_ns, spec.lane, seq, topic, md5, source_caller,
                    created_ns, payload, contract['has_header']), payload)
        except QueueFull as error:
            self.fail(error)
            raise

    def _accept(self):
        while not self.stop_event.is_set():
            try:
                sock, address = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if self.robot != 1 or address[0] != self.peer_ip or self.fatal:
                self._close(sock)
                continue
            self._thread(self._serve, sock, None)

    def _dial(self, lane):
        while not self.stop_event.is_set() and not self.fatal:
            try:
                sock = socket.create_connection((self.peer_host, self.peer_port), timeout=2.)
                self._serve(sock, lane)
            except OSError:
                with self.lock:
                    self.counts['dial_failures'] += 1
            self.stop_event.wait(.25)

    def _hello(self, lane):
        value = base_header('hello', self.robot, self.run_id, self.boot_id,
                            self.cutoff_ns, lane, self.peer_boot_id)
        value['schema_sha256'] = self.schema_sha256
        return value

    def _check_hello(self, value, lane):
        if value.get('kind') != 'hello':
            raise WireError('Expected session HELLO')
        remote_boot = value.get('boot_id')
        validate_identity(value.get('robot_id'), value.get('run_id'), remote_boot,
                          value.get('cutoff_ns'), value.get('lane'))
        if (value.get('version') != 1 or value.get('robot_id') != self.peer or
                value.get('run_id') != self.run_id or value.get('cutoff_ns') != self.cutoff_ns or
                value.get('lane') != lane or value.get('schema_sha256') != self.schema_sha256 or
                value.get('peer_boot_id') not in (None, self.boot_id)):
            raise WireError('HELLO identity/schema differs; new run required')
        with self.lock:
            if self.peer_boot_id not in (None, remote_boot):
                raise WireError('Peer boot changed; optimizer state recovery is not implicit')
            self.peer_boot_id = remote_boot

    def _check(self, header, kind, lane):
        validate_base(header, kind=kind, robot=self.peer, run_id=self.run_id,
                      boot_id=self.peer_boot_id, peer_boot_id=self.boot_id,
                      cutoff_ns=self.cutoff_ns, lane=lane)

    def _send(self, connection, header, payload=b''):
        with connection['send_lock']:
            connection['socket'].sendall(encode_frame(header, payload))

    def _base(self, kind, lane):
        return base_header(kind, self.robot, self.run_id, self.boot_id,
                           self.cutoff_ns, lane, self.peer_boot_id)

    def _serve(self, sock, lane):
        connection = None
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(2.)
            # The listening end learns the lane before sending its HELLO.
            if lane is None:
                hello, payload = read_frame(sock)
                if payload:
                    raise WireError('HELLO must not contain a ROS payload')
                lane = hello.get('lane')
                self._check_hello(hello, lane)
                sock.sendall(encode_frame(self._hello(lane)))
            else:
                sock.sendall(encode_frame(self._hello(lane)))
                hello, payload = read_frame(sock)
                if payload:
                    raise WireError('HELLO must not contain a ROS payload')
                self._check_hello(hello, lane)
            # Mutual acknowledgement proves both sides checked the run/schema.
            sock.sendall(encode_frame(self._base('ready', lane)))
            ready, payload = read_frame(sock)
            self._check(ready, 'ready', lane)
            if payload:
                raise WireError('READY must not contain a payload')
            connection = dict(socket=sock, send_lock=threading.Lock(),
                              last_rx=time.monotonic(), done=threading.Event())
            with self.lock:
                previous = self.connections.get(lane)
                if previous is not None:
                    # Same authenticated process can reconnect while an old
                    # reader finishes a slow local publication. Its ledger lock
                    # orders acceptance and suppresses retransmission safely.
                    previous['done'].set()
                    self._close(previous['socket'])
                    self.counts['same_boot_lane_replaced'] += 1
                self.connections[lane] = connection
                self.counts['sessions_established'] += 1
            self._thread(self._write, connection, lane)
            while not self.stop_event.is_set() and not self.fatal:
                header, payload = read_frame(sock)
                kind = header.get('kind')
                if kind not in ('data', 'ack', 'ping'):
                    raise WireError('Unexpected peer frame kind')
                self._check(header, kind, lane)
                connection['last_rx'] = time.monotonic()
                if kind == 'data':
                    validate_data(header, payload, self.contracts)
                    def publish(metadata, original):
                        age = (time.time_ns() - metadata['created_ns']) / 1e9
                        if lane == 'control' and age > self.max_control_age:
                            raise WireError('Expired optimizer message; cold restart required')
                        if lane == 'ego' and age > self.max_ego_age:
                            with self.lock:
                                self.counts['ego_stale_omitted'] += 1
                            return
                        try:
                            self.on_message(metadata, original)
                        except Exception as error:
                            raise WireError('Local ROS delivery failed: ' + str(error)) from error
                    self.ledgers[lane].deliver(header, payload, publish)
                    ack = self._base('ack', lane)
                    ack['sequence'] = self.ledgers[lane].last_sequence
                    self._send(connection, ack)
                elif payload:
                    raise WireError('Control envelope has an unexpected payload')
                elif kind == 'ack':
                    self.outboxes[lane].acknowledge(header.get('sequence'))
        except WireError as error:
            self.fail(error)
        except (EOFError, OSError):
            with self.lock:
                self.counts['connections_closed'] += 1
        finally:
            if connection is not None:
                connection['done'].set()
                with self.lock:
                    if self.connections.get(lane) is connection:
                        self.connections.pop(lane)
            self._close(sock)

    def _write(self, connection, lane):
        cursor = 0
        ping_time = 0.
        try:
            while not self.stop_event.is_set() and not self.fatal and not connection['done'].is_set():
                if time.monotonic() - connection['last_rx'] > self.health_timeout:
                    raise OSError('Peer lane heartbeat timeout')
                # Bounded in-flight window, independent from retained queue size.
                if cursor - self.outboxes[lane].last_acked < 16:
                    item = self.outboxes[lane].next_after(cursor)
                else:
                    item = None
                if item is not None:
                    cursor, (header, payload) = item
                    header = dict(header, peer_boot_id=self.peer_boot_id)
                    self._send(connection, header, payload)
                    continue
                if time.monotonic() >= ping_time:
                    self._send(connection, self._base('ping', lane))
                    ping_time = time.monotonic() + .5
                connection['done'].wait(.01)
        except OSError:
            self._close(connection['socket'])

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            ages = {lane: max(0., now - item['last_rx'])
                    for lane, item in self.connections.items()}
            ready = (not self.fatal and set(ages) == set(LANES) and
                     all(age < self.health_timeout for age in ages.values()))
            value = dict(run_id=self.run_id, cutoff_ns=self.cutoff_ns,
                local_robot_id=self.robot, peer_robot_id=self.peer,
                local_boot_id=self.boot_id, peer_boot_id=self.peer_boot_id,
                session_ready=ready, fatal=self.fatal, lane_receive_ages_s=ages,
                counts=dict(self.counts), proxy_caller=proxy_caller(self.robot),
                ack_scope='local proxy publication returned, not native consumer completion',
                recovery_scope='same-process TCP reconnect; boot changes require a new run')
        value['outgoing'] = {lane: box.snapshot() for lane, box in self.outboxes.items()}
        value['incoming'] = {lane: ledger.snapshot() for lane, ledger in self.ledgers.items()}
        return value


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', type=int, choices=(0, 1), required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', type=float, required=True)
    parser.add_argument('--boot-id')
    parser.add_argument('--listen-host', default='0.0.0.0')
    parser.add_argument('--listen-port', type=int, default=11640)
    parser.add_argument('--peer-host', required=True)
    parser.add_argument('--peer-port', type=int, default=11640)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--max-messages', type=int, default=512)
    parser.add_argument('--max-bytes', type=int, default=64 * 1024 * 1024)
    parser.add_argument('--max-control-age', type=float, default=10.)
    parser.add_argument('--max-ego-age', type=float, default=1.)
    parser.add_argument('--health-timeout', type=float, default=5.)
    return parser


def run_ros(args):
    import rospy
    import roslib.message
    from std_msgs.msg import Header, String, UInt8MultiArray, UInt16MultiArray

    rospy.init_node(proxy_caller(args.robot_id).lstrip('/'))
    own, remote = topic_specs(args.robot_id), topic_specs(1 - args.robot_id)
    kinds = {spec.type: roslib.message.get_message_class(spec.type)
             for robot in (0, 1) for spec in topic_specs(robot).values()}
    if any(kind is None for kind in kinds.values()):
        raise RuntimeError('A required native ROS peer message is unavailable')
    contracts = {name: dict(md5=kind._md5sum,
        has_header=bool(getattr(kind, '_has_header', False) or name == 'std_msgs/Header'))
        for name, kind in kinds.items()}
    # Synchronous publication keeps cross-topic invocation order on this bridge.
    # ROS consumers still own their callback order and native iteration checks.
    publishers = {topic: rospy.Publisher(topic, kinds[spec.type], queue_size=None,
                                       latch=spec.latch, tcp_nodelay=True)
                  for topic, spec in remote.items()}
    base = '/sim_%d/peer_bridge' % args.robot_id
    status_pub = rospy.Publisher(base + '/status', String, queue_size=1, latch=True)
    receipt_pub = rospy.Publisher(base + '/ingress_receipt', String, queue_size=256)
    # Native payload and proof are one ROS message, synchronously published on
    # one connection. This is the authoritative dense input; the two separate
    # topics below remain diagnostic compatibility, never an authentication join.
    native_control_pub = rospy.Publisher(base + '/native_control', UInt8MultiArray,
                                         queue_size=None, tcp_nodelay=True)
    connected_pub = rospy.Publisher('/sim_%d/connected_peer_ids' % args.robot_id,
                                    UInt16MultiArray, queue_size=1, latch=True)
    heartbeat_pub = rospy.Publisher('/sim_%d/heartbeat' % args.robot_id,
                                    Header, queue_size=1)
    recent = deque(maxlen=128)
    recent_lock = threading.Lock()
    counters = Counter()
    native_control_sequence = [0]
    cutoff_ns = round(args.cutoff * 1e9)

    def receive(metadata, payload):
        # Check actual native payload decoding, without serializing it back or
        # changing Header.seq, matrix elements or covariance bits.
        kind = kinds[metadata['type']]
        kind().deserialize(payload)
        spec = remote[metadata['topic']]
        if requires_native_subscriber(spec, transport.peer):
            # Startup must connect the real native subscribers before key zero.
            # An absent native subscriber is an explicit failure, never a
            # successful ACK that loses an optimizer command on an empty topic.
            deadline = time.monotonic() + args.max_control_age
            while publishers[metadata['topic']].get_num_connections() == 0:
                if (rospy.is_shutdown() or time.monotonic() >= deadline or
                        time.time_ns() - metadata['created_ns'] > args.max_control_age * 1e9):
                    raise WireError('Native peer consumer absent: ' + metadata['topic'])
                time.sleep(.02)
        receipt = dict(metadata, verified=True, local_robot_id=args.robot_id,
            peer_robot_id=1 - args.robot_id, source_robot_id=metadata['robot_id'],
            source_boot_id=metadata['boot_id'], received_wall_ns=time.time_ns(),
            proxy_caller=proxy_caller(args.robot_id))
        if is_native_control_topic(metadata['topic'], transport.peer):
            deadline = time.monotonic() + args.max_control_age
            while native_control_pub.get_num_connections() == 0:
                if (rospy.is_shutdown() or time.monotonic() >= deadline or
                        time.time_ns() - metadata['created_ns'] > args.max_control_age * 1e9):
                    raise WireError('Dense atomic native-control subscriber absent')
                time.sleep(.02)
            native_control_sequence[0] += 1
            native_control_pub.publish(UInt8MultiArray(data=encode_native_control(
                receipt, payload, native_control_sequence[0])))
            counters['atomic_native_controls_published'] += 1
            # This small ring now contains only significant controls; public
            # pose traffic can no longer evict a unique TERMINATE receipt.
            receipt_pub.publish(String(data=json.dumps(receipt, allow_nan=False)))
            with recent_lock:
                recent.append(receipt)
        publishers[metadata['topic']].publish(rospy.AnyMsg().deserialize(payload))

    transport = PeerTransport(args.robot_id, args.run_id, cutoff_ns, contracts, receive,
        listen_host=args.listen_host, listen_port=args.listen_port,
        peer_host=args.peer_host, peer_port=args.peer_port, boot_id=args.boot_id,
        max_messages=args.max_messages, max_bytes=args.max_bytes,
        max_control_age=args.max_control_age, max_ego_age=args.max_ego_age,
        health_timeout=args.health_timeout)

    def send(message, topic):
        connection = getattr(message, '_connection_header', None) or {}
        spec = own[topic]
        try:
            validate_source(spec, connection.get('callerid'))
            if connection.get('type') != spec.type:
                raise WireError('Local ROS publisher type differs')
            transport.enqueue(topic, bytes(message._buff), connection['callerid'],
                              connection.get('md5sum'))
        except ValueError as error:
            counters['local_messages_rejected'] += 1
            # A received broadcast proxy is intentionally not exported again.
            if connection.get('callerid') == proxy_caller(args.robot_id):
                counters['proxy_loop_suppressed'] += 1
            else:
                transport.fail(error)
                rospy.logerr_throttle(1., 'Peer bridge source rejection: %s', error)

    subscriptions = [rospy.Subscriber(topic, rospy.AnyMsg, send, callback_args=topic,
        queue_size=None, buff_size=2 ** 26, tcp_nodelay=True) for topic in own]
    args.output_directory.mkdir(parents=True, exist_ok=True)
    transport.start()
    connected_pub.publish(UInt16MultiArray(data=[]))
    rate = rospy.Rate(2)
    sequence = 0
    try:
        while not rospy.is_shutdown():
            sequence += 1
            heartbeat_pub.publish(Header(seq=sequence, stamp=rospy.Time.now(),
                                         frame_id='sim_%d' % args.robot_id))
            value = transport.snapshot()
            with recent_lock:
                value['recent_ingress'] = list(recent)
            value.update(wall=time.time(), local_counts=dict(counters),
                         native_control_sequence=native_control_sequence[0],
                         native_control_authentication='atomic_metadata_and_original_ros_bytes',
                         disk_payload_recording=False)
            connected_pub.publish(UInt16MultiArray(data=[transport.peer]
                                  if value['session_ready'] else []))
            status_pub.publish(String(data=json.dumps(value, allow_nan=False)))
            target = args.output_directory / 'peer_bridge_status.json'
            temporary = target.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
            temporary.replace(target)
            rate.sleep()
    finally:
        connected_pub.publish(UInt16MultiArray(data=[]))
        transport.shutdown()
        for subscriber in subscriptions:
            subscriber.unregister()


def main():
    parser = argument_parser()
    args, _ros_args = parser.parse_known_args()
    if not math.isfinite(args.cutoff) or args.cutoff <= 0:
        parser.error('A positive finite cutoff is required')
    if not all(1 <= port <= 65535 for port in (args.listen_port, args.peer_port)):
        parser.error('TCP ports must lie in [1,65535]')
    run_ros(args)


if __name__ == '__main__':
    main()
