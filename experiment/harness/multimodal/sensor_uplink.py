#!/usr/bin/env python3
"""Send prioritized actual telemetry and lossless sensors from a simulation host.

This process records no sensor files and computes no pose or map. ROS/log output
is permitted. IMU transport has its own callback, sequence, publisher and lock;
it never waits for an RGB-D pair or its compression worker.
Actual telemetry comes directly from continuous_body, not from the renderer.
"""
import argparse
from collections import Counter
import json
import math
import socket
import threading
import time
import weakref

from sensor_wire import (FrameBuffer, LANES, RGBD_KEYS, STREAMS, WireError,
                         MAX_RAW_BYTES,
                         encode_packet, source_caller, input_topic, input_caller,
                         stamp_ns, uplink_topic, validate_identity)


class SendBufferError(RuntimeError):
    """The bounded outgoing sensor transport could not be guaranteed."""


LANE_TRANSPORT = {
    # Marking is best effort: the access point must honor DSCP for radio QoS.
    # IMU keeps its original queue so sampling continuity is not replaced by a
    # latest-frame policy. Actual telemetry is a single latest-state stream.
    'truth': dict(queue_size=1, send_buffer_bytes=4096, ip_tos=0xb8),
    'imu': dict(queue_size=100, send_buffer_bytes=None, ip_tos=0xb8),
    'rgbd': dict(queue_size=1, send_buffer_bytes=64*1024, ip_tos=0),
    'lidar': dict(queue_size=1, send_buffer_bytes=64*1024, ip_tos=0x20),
}


class SourceStampSampler:
    """Choose current original samples on a source-clock phase, without bursts.

    Advancing a phase rather than waiting a full period after every selected
    sample permits a 100 Hz source to supply 30 Hz instead of accidentally
    rounding that down to 25 Hz. Source Header stamps are never rewritten.
    """
    def __init__(self, rate_hz):
        if not math.isfinite(rate_hz) or not 1 <= rate_hz <= 100:
            raise ValueError('Telemetry rate must be finite and in [1,100] Hz')
        self.rate_hz = rate_hz
        self.period_ns = round(1e9/rate_hz)
        self.next_ns = None; self.last_seen_ns = None
        self.counts = Counter(); self.lock = threading.Lock()

    def allow(self, stamp):
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise ValueError('Telemetry source stamp must be a positive integer')
        with self.lock:
            self.counts['source_messages_observed'] += 1
            if self.last_seen_ns is not None and stamp <= self.last_seen_ns:
                self.counts['nonmonotonic_source_samples_omitted'] += 1
                return False
            self.last_seen_ns = stamp
            if self.next_ns is None: self.next_ns = stamp
            if stamp < self.next_ns:
                self.counts['source_samples_omitted_configured_rate'] += 1
                return False
            # Skipping source time never creates a debt of old samples to send.
            self.next_ns += ((stamp-self.next_ns)//self.period_ns+1)*self.period_ns
            self.counts['source_samples_selected'] += 1
            return True

    def snapshot(self):
        with self.lock:
            return dict(rate_hz=self.rate_hz, counts=dict(self.counts),
                        original_header_stamp_unchanged=True,
                        source_clock_phase_sampling=True)


class LatestFrameSendBuffer:
    """Noetic transport guard plus a check before each guarded-lane publish.

    _PublisherImpl.add_connection wraps TCPROSTransport in QueuedConnection,
    registers it under c_lock, then calls peer_subscribe with bound publish.
    Registration precedes the listener, so publish also holds publock/c_lock
    through configuration and enqueueing. This covers that race and reconnects
    without patching rospy or polling. Only the small TCPROS header precedes us.
    """
    def __init__(self, topic, on_failure, requested_bytes=64*1024,
                 expected_queue_size=1, ip_tos=None):
        if requested_bytes is not None and (isinstance(requested_bytes, bool) or
                not isinstance(requested_bytes, int) or requested_bytes < 4096):
            raise ValueError('Socket send buffer request must be at least 4096 bytes')
        if expected_queue_size not in (1, 100):
            raise ValueError('Only the configured latest-state and continuous-IMU queues are supported')
        if ip_tos is not None and (isinstance(ip_tos, bool) or not isinstance(ip_tos, int)
                                  or not 0 <= ip_tos <= 255 or ip_tos & 3):
            raise ValueError('Use an integer IP_TOS DSCP marking with ECN bits clear')
        self.topic = topic
        self.on_failure = on_failure
        self.requested_bytes = requested_bytes
        self.expected_queue_size = expected_queue_size
        self.ip_tos = ip_tos
        self._sockets = weakref.WeakKeyDictionary()
        self._socket_tos = weakref.WeakKeyDictionary()
        self._lock = threading.Lock()
        self._configured = 0
        self._failures = 0
        self._last_error = None

    def _configure(self, impl):
        if impl.resolved_name != self.topic:
            raise SendBufferError('Unexpected outgoing sensor topic ' + str(impl.resolved_name))
        for connection in impl.connections:
            if connection.done:
                continue
            try:
                # Noetic QueuedConnection proxies unknown attributes, but the
                # queue itself and its underlying transport are explicit here.
                if connection._queue_size != self.expected_queue_size:
                    raise SendBufferError('Sensor lane requires QueuedConnection queue_size=' +
                                          str(self.expected_queue_size))
                transport = connection._connection
                sock = transport.socket
                if transport.transport_type != 'TCPROS' or sock is None:
                    raise SendBufferError('Missing outgoing TCPROS sensor socket')
                with self._lock:
                    known = sock in self._sockets
                if known:
                    continue
                if self.requested_bytes is not None:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.requested_bytes)
                effective = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
                # Linux doubles SO_SNDBUF for bookkeeping. Never accept an
                # uncapped/autotuned multi-megabyte buffer after setsockopt.
                if self.requested_bytes is not None and not 0 < effective <= 2 * self.requested_bytes:
                    raise SendBufferError('Outgoing sensor SO_SNDBUF not bounded: ' + str(effective))
                effective_tos = None
                if self.ip_tos is not None:
                    sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, self.ip_tos)
                    effective_tos = sock.getsockopt(socket.IPPROTO_IP, socket.IP_TOS)
                    if effective_tos & 0xfc != self.ip_tos:
                        raise SendBufferError('Outgoing sensor IP_TOS marking differs: ' + str(effective_tos))
                with self._lock:
                    self._sockets[sock] = effective
                    self._socket_tos[sock] = effective_tos
                    self._configured += 1
            except Exception as error:
                # A transport already closing is ordinary reconnect churn.
                if connection.done:
                    continue
                connection.close()
                raise SendBufferError(self.topic + ': ' + str(error)) from error

    def _failed(self, error):
        with self._lock:
            self._failures += 1
            self._last_error = repr(error)
        self.on_failure(error)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        try:
            impl = topic_publish.__self__
            with impl.publock, impl.c_lock:
                self._configure(impl)
        except Exception as error:
            self._failed(error)
            raise

    def peer_unsubscribe(self, topic_name, num_peers):
        pass

    def publish(self, publisher, message):
        try:
            impl = publisher.impl
            with impl.publock, impl.c_lock:
                self._configure(impl)
                publisher.publish(message)
        except Exception as error:
            self._failed(error)
            raise

    def snapshot(self):
        with self._lock:
            return dict(requested_bytes=self.requested_bytes,
                        effective_bytes=list(self._sockets.values()),
                        expected_queue_size=self.expected_queue_size,
                        requested_ip_tos=self.ip_tos,
                        effective_ip_tos=list(self._socket_tos.values()),
                        ap_priority_enforcement_guaranteed=False,
                        connections_configured=self._configured,
                        failures=self._failures, last_error=self._last_error)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--max-frame-age', type=float, default=2.)
    parser.add_argument('--max-pending', type=int, default=8)
    parser.add_argument('--compression-level', type=int, choices=range(10), default=1)
    parser.add_argument('--telemetry-rate', type=float, default=30.,
                        help='Sample actual body odometry on its unchanged source stamps')
    args = parser.parse_args(argv)
    if not 0 < args.cutoff < float('inf') or not .1 <= args.max_frame_age <= 10:
        parser.error('Use a finite positive cutoff and frame-age limit in [0.1,10] seconds')
    validate_identity(args.robot_id, args.run_id, round(args.cutoff*1e9))
    if not 1 <= args.max_pending <= 128: parser.error('max-pending must be in [1,128]')
    if not math.isfinite(args.telemetry_rate) or not 1 <= args.telemetry_rate <= 100:
        parser.error('telemetry-rate must be finite and in [1,100] Hz')
    return args


def run_ros(args):
    import rospy
    import roslib.message
    from std_msgs.msg import String, UInt8MultiArray

    rospy.init_node('sim_%d_sensor_uplink' % args.robot_id)
    cutoff_ns = round(args.cutoff*1e9)
    frames = FrameBuffer(cutoff_ns, args.max_pending, round(args.max_frame_age*1e9))
    stop = threading.Event(); fatal = [None]
    telemetry_sampler = SourceStampSampler(args.telemetry_rate)

    def transport_failure(error):
        fatal[0] = repr(error)
        rospy.logfatal('Sensor uplink outgoing buffer failed: %s', fatal[0])
        rospy.signal_shutdown('Sensor uplink outgoing buffer failed')

    buffers = {lane: LatestFrameSendBuffer(uplink_topic(args.robot_id, lane), transport_failure,
                    requested_bytes=policy['send_buffer_bytes'],
                    expected_queue_size=policy['queue_size'], ip_tos=policy['ip_tos'])
               for lane, policy in LANE_TRANSPORT.items()}
    publishers = {lane: rospy.Publisher(uplink_topic(args.robot_id, lane), UInt8MultiArray,
                                      queue_size=LANE_TRANSPORT.get(lane, {}).get('queue_size', 1),
                                      latch=lane == 'ready', tcp_nodelay=True,
                                      subscriber_listener=buffers.get(lane)) for lane in LANES}
    status = rospy.Publisher('/edge/sim_%d/uplink/status' % args.robot_id,
                             String, queue_size=1, latch=True)
    sequences = Counter(); counts = {lane: Counter() for lane in LANES}
    locks = {lane: threading.Lock() for lane in LANES}
    kinds = {key: roslib.message.get_message_class(spec['type']) for key, spec in STREAMS.items()}
    if any(kind is None for kind in kinds.values()): raise RuntimeError('A ROS sensor message type is missing')

    def transmit(lane, stamp, parts):
        # A lock belongs to one lane only; large compression cannot stop IMU.
        with locks[lane]:
            sequences[lane] += 1
            packet = encode_packet(args.robot_id, args.run_id, cutoff_ns, lane,
                                   sequences[lane], stamp, parts, args.compression_level)
            message = UInt8MultiArray(data=packet)
            if lane in buffers:
                buffers[lane].publish(publishers[lane], message)
            else:
                publishers[lane].publish(message)
            counts[lane]['envelopes_published'] += 1
            counts[lane]['wire_bytes'] += len(packet)
            counts[lane]['original_ros_bytes'] += sum(map(len, parts.values()))

    def callback(message, key):
        lane = STREAMS[key]['lane']
        try:
            connection = getattr(message, '_connection_header', None) or {}
            caller = connection.get('callerid')
            if caller != input_caller(args.robot_id, key):
                raise WireError('Unexpected local source publisher ' + str(caller))
            kind = kinds[key]
            if connection.get('type') != STREAMS[key]['type'] or connection.get('md5sum') != kind._md5sum:
                raise WireError('Unexpected original ROS sensor type/MD5')
            # Capture the actual TCPROS bytes before decoding a temporary copy
            # for its original Header. Never regenerate the transmitted bytes.
            if not 0 < len(message._buff) <= MAX_RAW_BYTES: raise WireError('Original sensor message exceeds raw-byte bound')
            raw = bytes(message._buff); decoded = kind().deserialize(raw)
            stamp = stamp_ns(decoded)
            if key != 'ready':
                if stamp < cutoff_ns: raise WireError('Pre-epoch edge sensor message')
                if time.time_ns()-stamp > round(args.max_frame_age*1e9): raise WireError('Stale edge sensor message')
            if key == 'truth' and not telemetry_sampler.allow(stamp):
                with locks[lane]: counts[lane]['messages_omitted_telemetry_sampling'] += 1
                return
            if key in RGBD_KEYS: frames.put(key, stamp, raw)
            else: transmit(lane, stamp, {key: raw})
        except Exception as error:
            # Exclusions remain visible; timestamps are never shifted forward.
            with locks[lane]: counts[lane]['messages_rejected'] += 1
            rospy.logwarn_throttle(2., 'Sensor uplink omitted %s: %s', key, str(error))

    subscribers = []
    for key, spec in STREAMS.items():
        subscribers.append(rospy.Subscriber(input_topic(args.robot_id, key), rospy.AnyMsg,
            callback, callback_args=key, queue_size=200 if key == 'imu' else 1 if key == 'truth' else 2,
            buff_size=2**24 if key in RGBD_KEYS else 2**20, tcp_nodelay=True))

    def rgbd_worker():
        try:
            while not stop.is_set() and not rospy.is_shutdown():
                item = frames.take(time.time_ns())
                if item is None: stop.wait(.005)
                else: transmit('rgbd', item[0], item[1])
        except Exception as error:
            fatal[0] = repr(error); rospy.logfatal('Sensor uplink worker failed: %s', fatal[0])
            rospy.signal_shutdown('Sensor uplink worker failed')

    worker = threading.Thread(target=rgbd_worker, name='lossless_rgbd_uplink', daemon=True)
    worker.start()
    try:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            lanes = {}
            for lane in LANES:
                with locks[lane]: lanes[lane] = dict(counts[lane])
            value = dict(run_id=args.run_id, cutoff_ns=cutoff_ns, robot_id=args.robot_id,
                         wall=time.time(), renderer_source_caller=source_caller(args.robot_id),
                         source_inputs={key:dict(topic=input_topic(args.robot_id,key),
                                                caller=input_caller(args.robot_id,key)) for key in STREAMS},
                         telemetry_sampling=telemetry_sampler.snapshot(),
                         direct_body_telemetry=True,
                         lanes=lanes, **frames.snapshot(), fatal_error=fatal[0],
                         raw_sensor_files_recorded=False, estimator=False, computes_map=False,
                         imu_independent_of_rgbd=True, lossless_original_ros_serialization=True,
                         outgoing_send_buffers={lane: policy.snapshot() for lane, policy in buffers.items()})
            status.publish(String(data=json.dumps(value, allow_nan=False))); rate.sleep()
    finally:
        stop.set(); worker.join(timeout=3.)
        for subscriber in subscribers: subscriber.unregister()


if __name__ == '__main__':
    import rospy
    run_ros(arguments(rospy.myargv()[1:]))
