from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'distributed'))
from peer_bridge import PeerTransport
from peer_topics import LANES, topic_specs
from peer_wire import WireError
from test_peer_wire import contracts


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for peer transport condition')


class PeerTransportTests(unittest.TestCase):
    def pair(self, on_zero=None, on_one=None, **options):
        self.received = [[], []]
        def receive(robot):
            return lambda h, p: self.received[robot].append((dict(h), p))
        cutoff = time.time_ns() - 60 * 10 ** 9
        self.one = PeerTransport(1, 'socket_test', cutoff, contracts(),
            on_one or receive(1), listen_host='127.0.0.1', listen_port=0,
            peer_host='127.0.0.1', boot_id='b' * 32, **options)
        self.one.start()
        self.zero = PeerTransport(0, 'socket_test', cutoff, contracts(),
            on_zero or receive(0), listen_host='127.0.0.1', listen_port=0,
            peer_host='127.0.0.1', peer_port=self.one.listen_port,
            boot_id='a' * 32, **options)
        self.zero.start()
        self.addCleanup(self.zero.shutdown)
        self.addCleanup(self.one.shutdown)
        wait_until(lambda: self.zero.snapshot()['session_ready'] and
                   self.one.snapshot()['session_ready'])

    def send(self, source, suffix, payload=b'opaque_native_matrix\x00\xff', robot=None):
        robot = source.robot if robot is None else robot
        topic = '/sim_%d/' % robot + suffix
        spec = topic_specs(robot)[topic]
        return source.enqueue(topic, payload, spec.callers[0], contracts()[spec.type]['md5'])

    def test_full_duplex_native_bytes_and_isolated_lanes(self):
        self.pair()
        self.send(self.zero, 'dpgo_ros_node/command')
        self.send(self.zero, 'dpgo_ros_node/measurement_weights', b'weights')
        self.send(self.one, 'dpgo_ros_node/public_poses', b'lifted_five_dimensional_matrix')
        self.send(self.one, 'kimera_vio_ros/bow_query', b'features')
        self.send(self.zero, 'mission/status', b'local stage barrier')
        wait_until(lambda: len(self.received[1]) == 3 and len(self.received[0]) == 2)
        controls = [h for h, _ in self.received[1] if h['lane'] == 'control']
        self.assertEqual([h['sequence'] for h in controls], [1, 2])
        command_payload = next(p for h, p in self.received[1]
                               if h['topic'] == '/sim_0/dpgo_ros_node/command')
        self.assertEqual(command_payload, b'opaque_native_matrix\x00\xff')
        wait_until(lambda: all(box['pending'] == 0
            for box in self.zero.snapshot()['outgoing'].values()))
        self.assertEqual(set(self.zero.snapshot()['lane_receive_ages_s']), set(LANES))

    def test_reconnect_after_local_delivery_before_ack_does_not_redeliver(self):
        seen = []
        def delivered(header, payload):
            seen.append((header['sequence'], payload))
            if len(seen) == 1:
                self.one.disconnect()  # Lose the ACK after native publication.
        self.pair(on_one=delivered)
        self.send(self.zero, 'dpgo_ros_node/command', b'execute once')
        wait_until(lambda: self.zero.snapshot()['outgoing']['control']['last_acked'] == 1)
        self.assertEqual(seen, [(1, b'execute once')])
        self.assertGreaterEqual(self.one.snapshot()['incoming']['control']['counts']
                                .get('duplicate_suppressed', 0), 1)

    def test_foreign_source_and_bad_local_md5_are_rejected(self):
        self.pair()
        with self.assertRaises(ValueError):
            self.zero.enqueue('/broadcast_bspline', b'loop', '/sim_0_peer_bridge', '0' * 32)
        spec = topic_specs(0)['/sim_0/dpgo_ros_node/command']
        with self.assertRaises(WireError):
            self.zero.enqueue(spec.topic, b'bad MD5', spec.callers[0], '0' * 32)
        self.assertEqual(self.received, [[], []])

    def test_reconnect_while_old_reader_completes_publication_is_serialized(self):
        entered, release = threading.Event(), threading.Event()
        seen = []
        def slow_publication(header, payload):
            seen.append((header['sequence'], payload))
            entered.set()
            if not release.wait(3.):
                raise RuntimeError('Test did not release local publication')
        self.pair(on_one=slow_publication)
        self.addCleanup(release.set)
        self.send(self.zero, 'dpgo_ros_node/command', b'one native publication')
        self.assertTrue(entered.wait(2.))
        self.one.disconnect()
        wait_until(lambda: self.one.snapshot()['counts'].get('sessions_established', 0) > 3)
        release.set()
        wait_until(lambda: self.zero.snapshot()['outgoing']['control']['last_acked'] == 1)
        self.assertEqual(seen, [(1, b'one native publication')])
        self.assertIsNone(self.one.fatal)

    def test_expired_optimizer_command_fails_the_session_instead_of_silent_drop(self):
        self.pair(max_control_age=.05)
        spec = topic_specs(0)['/sim_0/dpgo_ros_node/command']
        self.zero.enqueue(spec.topic, b'expired', spec.callers[0],
            contracts()[spec.type]['md5'], created_ns=time.time_ns() - 10 ** 9)
        wait_until(lambda: self.one.fatal is not None)
        self.assertIn('Expired optimizer', self.one.fatal)
        self.assertEqual(self.received[1], [])
        self.assertFalse(self.one.snapshot()['session_ready'])

    def test_ego_expiration_is_explicit_but_does_not_fail_optimizer_lanes(self):
        self.pair(max_ego_age=.05)
        spec = topic_specs(0)['/sim_0/mission/status']
        self.zero.enqueue(spec.topic, b'old stage', spec.callers[0],
            contracts()[spec.type]['md5'], created_ns=time.time_ns() - 10 ** 9)
        wait_until(lambda: self.zero.snapshot()['outgoing']['ego']['last_acked'] == 1)
        self.assertEqual(self.received[1], [])
        self.assertEqual(self.one.snapshot()['counts']['ego_stale_omitted'], 1)
        self.assertIsNone(self.one.fatal)

    def test_peer_process_boot_change_is_not_hidden_as_socket_recovery(self):
        self.pair()
        hello = self.one._hello('control')
        self.zero._check_hello(hello, 'control')
        with self.assertRaises(WireError):
            self.zero._check_hello(dict(hello, boot_id='c' * 32), 'control')


if __name__ == '__main__':
    unittest.main()
