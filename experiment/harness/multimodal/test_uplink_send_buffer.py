#!/usr/bin/env python3
"""Outgoing socket bounds and Noetic connection/reconnection lifecycle."""
import importlib.util
import socket
import struct
import threading
from types import SimpleNamespace as S
import unittest

from sensor_uplink import LatestFrameSendBuffer, SendBufferError, LANE_TRANSPORT


TOPIC = '/edge/sim_0/uplink/rgbd'


def tcp_pair():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(('127.0.0.1', 0)); listener.listen(1)
    receiver = socket.create_connection(listener.getsockname(), timeout=2)
    sender, _ = listener.accept(); listener.close()
    sender.settimeout(2); receiver.settimeout(2)
    return sender, receiver


class Connection:
    def __init__(self, sock):
        self._queue_size = 1
        self._connection = S(socket=sock, transport_type='TCPROS')
        self.done = False

    def close(self):
        self.done = True
        self._connection.socket.close()


class Impl:
    def __init__(self, connections):
        self.resolved_name = TOPIC
        self.publock = threading.RLock(); self.c_lock = threading.RLock()
        self.connections = connections
        self.published = []

    def publish(self, message):
        self.published.append(message)
        for connection in self.connections:
            if not connection.done:
                connection._connection.socket.sendall(message)


class SendBufferContracts(unittest.TestCase):
    def test_real_tcp_socket_is_capped_at_subscribe_and_reconnect(self):
        failures = []; policy = LatestFrameSendBuffer(TOPIC, failures.append)
        for count in (1, 2):
            sender, receiver = tcp_pair()
            try:
                impl = Impl([Connection(sender)])
                policy.peer_subscribe(TOPIC, impl.publish, None)
                actual = sender.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
                self.assertGreater(actual, 0); self.assertLessEqual(actual, 128 * 1024)
                self.assertEqual(policy.snapshot()['connections_configured'], count)
                policy.publish(S(impl=impl, publish=impl.publish), b'unchanged\x00\xff')
                self.assertEqual(receiver.recv(100), b'unchanged\x00\xff')
                self.assertEqual(policy.snapshot()['connections_configured'], count)
            finally:
                sender.close(); receiver.close()
        self.assertFalse(failures)

    def test_publish_covers_connection_registered_before_listener_runs(self):
        sender, receiver = tcp_pair()
        try:
            impl = Impl([Connection(sender)])
            policy = LatestFrameSendBuffer(TOPIC, self.fail)
            # This is the actual Noetic race: super.add_connection has exposed
            # the new connection but peer_subscribe has not yet been called.
            policy.publish(S(impl=impl, publish=impl.publish), b'first-frame')
            self.assertLessEqual(sender.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF), 128 * 1024)
            self.assertEqual(receiver.recv(100), b'first-frame')
            policy.peer_subscribe(TOPIC, impl.publish, None)
            self.assertEqual(policy.snapshot()['connections_configured'], 1)
        finally:
            sender.close(); receiver.close()

    def test_real_telemetry_socket_is_small_and_priority_marks_are_read_back(self):
        for lane in ('truth','imu','rgbd','lidar'):
            sender,receiver=tcp_pair()
            try:
                settings=LANE_TRANSPORT[lane];connection=Connection(sender)
                connection._queue_size=settings['queue_size'];impl=Impl([connection])
                policy=LatestFrameSendBuffer(TOPIC,self.fail,
                    requested_bytes=settings['send_buffer_bytes'],
                    expected_queue_size=settings['queue_size'],ip_tos=settings['ip_tos'])
                initial=sender.getsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF)
                policy.peer_subscribe(TOPIC,impl.publish,None)
                self.assertEqual(sender.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)&0xfc,settings['ip_tos'])
                effective=sender.getsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF)
                if lane=='truth':self.assertLessEqual(effective,8192)
                elif lane=='imu':self.assertEqual(effective,initial)
                else:self.assertLessEqual(effective,131072)
                policy.publish(S(impl=impl,publish=impl.publish),b'original bytes\x00\xff')
                self.assertEqual(receiver.recv(100),b'original bytes\x00\xff')
                self.assertEqual(policy.snapshot()['expected_queue_size'],settings['queue_size'])
                self.assertFalse(policy.snapshot()['ap_priority_enforcement_guaranteed'])
            finally:sender.close();receiver.close()

    def test_unsupported_queue_or_socket_option_fails_closed_before_publish(self):
        for mode in ('wrong_queue', 'setsockopt_failure', 'ineffective_limit'):
            with self.subTest(mode=mode):
                class BadSocket:
                    def setsockopt(self, *args):
                        if mode == 'setsockopt_failure': raise OSError('setsockopt rejected')
                    def getsockopt(self, *args): return 4 * 1024 * 1024
                    def close(self): pass
                connection = Connection(BadSocket())
                if mode == 'wrong_queue': connection._queue_size = 100
                impl = Impl([connection]); failures = []
                policy = LatestFrameSendBuffer(TOPIC, failures.append)
                with self.assertRaises(SendBufferError):
                    policy.publish(S(impl=impl, publish=impl.publish), b'must-not-send')
                self.assertTrue(connection.done); self.assertFalse(impl.published)
                self.assertEqual(len(failures), 1)
                self.assertEqual(policy.snapshot()['failures'], 1)


@unittest.skipUnless(importlib.util.find_spec('rospy'), 'Run the actual Noetic lifecycle inside its container')
class NativeNoeticContracts(unittest.TestCase):
    def test_actual_queued_connection_socket_and_latest_frame_replacement(self):
        from rospy.topics import _PublisherImpl
        from rospy.impl.tcpros_base import TCPROSTransport
        from rospy.impl.tcpros_pubsub import TCPROSPub, QueuedConnection
        from std_msgs.msg import UInt8MultiArray
        topic='/edge/sim_0/uplink/truth'
        failures = []; policy = LatestFrameSendBuffer(topic, failures.append,
                                                    requested_bytes=4096,ip_tos=0xb8)
        impl = _PublisherImpl(topic, UInt8MultiArray)
        impl.set_queue_size(1); impl.add_subscriber_listener(policy)
        for number in (1, 2):
            sender, receiver = tcp_pair(); sender.settimeout(.5)
            raw = TCPROSTransport(TCPROSPub(topic, UInt8MultiArray), topic)
            raw.set_socket(sender, '/ingress_' + str(number))
            entered = threading.Event(); write = raw.write_data
            def sending(data):
                entered.set(); return write(data)
            raw.write_data = sending
            try:
                impl.add_connection(raw)
                queued = impl.connections[-1]
                self.assertIsInstance(queued, QueuedConnection)
                self.assertIs(queued._connection.socket, sender)
                self.assertLessEqual(sender.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),8192)
                self.assertEqual(sender.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)&0xfc,0xb8)
                publisher = S(impl=impl, publish=impl.publish)
                policy.publish(publisher, UInt8MultiArray(data=b'a' * (8 * 1024 * 1024)))
                self.assertTrue(entered.wait(1))
                policy.publish(publisher, UInt8MultiArray(data=b'old-pending'))
                policy.publish(publisher, UInt8MultiArray(data=b'latest-unchanged\x00\xff'))
                with queued._lock:
                    self.assertEqual(len(queued._queue), 1)
                    serialized = queued._queue[0]
                length = struct.unpack('<I', serialized[:4])[0]
                self.assertEqual(length, len(serialized) - 4)
                received = UInt8MultiArray().deserialize(serialized[4:])
                self.assertEqual(bytes(received.data), b'latest-unchanged\x00\xff')
            finally:
                receiver.close(); raw.close()
                queued._thread.join(2)
                self.assertFalse(queued._thread.is_alive())
        impl.close()
        self.assertFalse(failures)
        self.assertEqual(policy.snapshot()['connections_configured'], 2)

    def test_actual_noetic_imu_retains_ordered_queue_instead_of_latest_frame_replacement(self):
        from rospy.topics import _PublisherImpl
        from rospy.impl.tcpros_base import TCPROSTransport
        from rospy.impl.tcpros_pubsub import TCPROSPub
        from std_msgs.msg import UInt8MultiArray
        topic='/edge/sim_0/uplink/imu';failures=[]
        policy=LatestFrameSendBuffer(topic,failures.append,requested_bytes=None,
                                     expected_queue_size=100,ip_tos=0xb8)
        impl=_PublisherImpl(topic,UInt8MultiArray);impl.set_queue_size(100)
        impl.add_subscriber_listener(policy)
        sender,receiver=tcp_pair();sender.settimeout(.5)
        raw=TCPROSTransport(TCPROSPub(topic,UInt8MultiArray),topic)
        raw.set_socket(sender,'/imu_ingress');entered=threading.Event();write=raw.write_data
        def sending(data):entered.set();return write(data)
        raw.write_data=sending
        try:
            impl.add_connection(raw);queued=impl.connections[-1]
            publisher=S(impl=impl,publish=impl.publish)
            policy.publish(publisher,UInt8MultiArray(data=b'a'*(8*1024*1024)))
            self.assertTrue(entered.wait(1))
            policy.publish(publisher,UInt8MultiArray(data=b'imu 1\x00'))
            policy.publish(publisher,UInt8MultiArray(data=b'imu 2\xff'))
            with queued._lock:pending=list(queued._queue)
            self.assertEqual(len(pending),2)
            recovered=[bytes(UInt8MultiArray().deserialize(value[4:]).data) for value in pending]
            self.assertEqual(recovered,[b'imu 1\x00',b'imu 2\xff'])
            self.assertEqual(policy.snapshot()['requested_bytes'],None)
            self.assertEqual(policy.snapshot()['expected_queue_size'],100)
            self.assertEqual(sender.getsockopt(socket.IPPROTO_IP,socket.IP_TOS)&0xfc,0xb8)
        finally:
            receiver.close();raw.close();queued._thread.join(2);impl.close()
            self.assertFalse(queued._thread.is_alive())
        self.assertFalse(failures)


if __name__ == '__main__': unittest.main()
