import hashlib
import io
from pathlib import Path
import socket
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'distributed'))
from peer_topics import (LANES, proxy_caller, requires_native_subscriber,
                         topic_specs, validate_source)
from peer_wire import (MAX_PAYLOAD, MAGIC, PREFIX, QueueFull, ReceiveLedger,
    ReliableOutbox, WireError, canonical_payload_sha256, contract_digest,
    data_header, decode_frame_bytes, encode_frame, encode_native_control,
    is_native_control_topic, native_message_sha256, read_frame,
    validate_data, validate_ingress_receipt)


def contracts():
    return {s.type: dict(md5=hashlib.md5(s.type.encode()).hexdigest(), has_header=False)
            for robot in (0, 1) for s in topic_specs(robot).values()}


def message(sequence=1, topic='/sim_0/dpgo_ros_node/command', payload=b'original\x00matrix'):
    spec = topic_specs(0)[topic]
    header = data_header(0, 'test_run', 'a' * 32, 1, spec.lane, sequence,
                         topic, contracts()[spec.type]['md5'], spec.callers[0], 2, payload)
    header['peer_boot_id'] = 'b' * 32
    return header, payload


class PeerWireTests(unittest.TestCase):
    def test_atomic_control_provenance_and_native_bytes_are_one_bounded_message(self):
        header, payload = message()
        receipt = dict(header, verified=True, local_robot_id=1, peer_robot_id=0,
            source_robot_id=0, source_boot_id=header['boot_id'],
            proxy_caller='/sim_1_peer_bridge')
        observed, original = decode_frame_bytes(encode_native_control(receipt, payload, 1))
        self.assertEqual(original, payload)
        self.assertEqual(observed['kind'], 'native_control')
        self.assertEqual(observed['control_sequence'], 1)
        self.assertEqual(observed['sequence'], header['sequence'])
        self.assertEqual(observed['local_boot_id'], 'b' * 32)
        validate_data(observed, original, contracts())
        validate_ingress_receipt(observed, run_id='test_run', cutoff_ns=1,
            local_robot_id=1, peer_boot_id='a' * 32,
            topic=header['topic'], sha256=header['sha256'])

    def test_atomic_envelope_rejects_truncation_concatenation_and_payload_mismatch(self):
        header, payload = message()
        receipt = dict(header, verified=True, source_robot_id=0)
        packet = encode_native_control(receipt, payload, 1)
        for raw in (packet[:-1], packet + b'!', packet + packet, b''):
            with self.subTest(length=len(raw)), self.assertRaises(WireError):
                decode_frame_bytes(raw)
        with self.assertRaises(WireError):
            encode_native_control(receipt, payload + b'corrupted', 2)

    def test_public_pose_traffic_cannot_enter_the_atomic_control_stream(self):
        header, payload = message(topic='/sim_0/dpgo_ros_node/public_poses')
        receipt = dict(header, verified=True, source_robot_id=0)
        self.assertFalse(is_native_control_topic(header['topic'], 0))
        with self.assertRaises(WireError):
            encode_native_control(receipt, payload, 1)

    def test_custom_ros_serialization_survives_framing(self):
        header, payload = message()
        left, right = socket.socketpair()
        try:
            left.sendall(encode_frame(header, payload))
            observed, original = read_frame(right)
            self.assertEqual((observed, original), (header, payload))
            validate_data(observed, original, contracts())
        finally:
            left.close(); right.close()

    def test_frame_bound_checked_before_payload_allocation(self):
        left, right = socket.socketpair()
        try:
            left.sendall(PREFIX.pack(MAGIC, 2, MAX_PAYLOAD + 1))
            with self.assertRaises(WireError):
                read_frame(right)
        finally:
            left.close(); right.close()

    def test_corrupt_payload_source_md5_type_and_lane_rejected(self):
        header, payload = message()
        for change in ({'source_caller': '/sim_1_peer_bridge'}, {'md5': '0' * 32},
                       {'type': 'nav_msgs/Odometry'}, {'lane': 'feature'},
                       {'sha256': '0' * 64}, {'payload_bytes': len(payload) + 1}):
            with self.subTest(change=change), self.assertRaises(WireError):
                validate_data(dict(header, **change), payload, contracts())
        with self.assertRaises(WireError):
            validate_data(header, payload + b'changed', contracts())

    def test_header_seq_normalization_preserves_stamp_and_native_bits(self):
        original = struct.pack('<I', 32) + b'original_stamp_and_matrix_bits'
        changed = struct.pack('<I', 99) + original[4:]
        self.assertEqual(canonical_payload_sha256(original, True),
                         canonical_payload_sha256(changed, True))
        self.assertNotEqual(canonical_payload_sha256(original),
                            canonical_payload_sha256(changed))
        class Native:
            _has_header = True
            def serialize(self, buffer):
                buffer.write(original)
        self.assertEqual(native_message_sha256(Native()),
                         canonical_payload_sha256(original, True))
        self.assertNotEqual(canonical_payload_sha256(original, True),
                            canonical_payload_sha256(original[:-1] + b'!', True))

    def test_outbox_retains_commands_until_cumulative_ack(self):
        box = ReliableOutbox(max_messages=3, max_bytes=100)
        a, p = message()
        b, q = message(2, '/sim_0/dpgo_ros_node/measurement_weights')
        box.enqueue(lambda s: dict(a, sequence=s), p)
        box.enqueue(lambda s: dict(b, sequence=s), q)
        self.assertEqual(box.next_after(0)[0], 1)
        self.assertEqual(box.next_after(1)[0], 2)
        # Socket reconnect rewinds the sending cursor, not the retained queue.
        self.assertEqual(box.next_after(0)[0], 1)
        with self.assertRaises(WireError):
            box.acknowledge(3)
        box.acknowledge(1)
        self.assertEqual(box.next_after(0)[0], 2)
        box.acknowledge(2)
        self.assertEqual(box.snapshot()['pending'], 0)

    def test_bounded_queue_fails_instead_of_dropping_an_optimizer_command(self):
        box = ReliableOutbox(max_messages=1, max_bytes=100)
        header, payload = message()
        box.enqueue(lambda s: header, payload)
        with self.assertRaises(QueueFull):
            box.enqueue(lambda s: header, payload)
        self.assertEqual(box.snapshot()['pending'], 1)
        self.assertEqual(box.snapshot()['counts']['overflow_fatal'], 1)

    def test_cross_topic_order_and_reconnect_duplicate_suppression(self):
        ledger = ReceiveLedger()
        observed = []
        a, p = message()
        b, q = message(2, '/sim_0/dpgo_ros_node/measurement_weights')
        publish = lambda h, raw: observed.append((h['sequence'], h['topic'], raw))
        self.assertTrue(ledger.deliver(a, p, publish))
        self.assertFalse(ledger.deliver(a, p, publish))
        self.assertTrue(ledger.deliver(b, q, publish))
        self.assertEqual([item[0] for item in observed], [1, 2])
        with self.assertRaises(WireError):
            ledger.deliver(dict(a, topic=b['topic']), p, publish)
        with self.assertRaises(WireError):
            ledger.deliver(dict(b, sequence=4), q, publish)

    def test_callback_failure_is_not_accepted_or_acked(self):
        ledger = ReceiveLedger()
        header, payload = message()
        def failure(*_args):
            raise RuntimeError('native publication failed')
        with self.assertRaises(RuntimeError):
            ledger.deliver(header, payload, failure)
        self.assertEqual(ledger.last_sequence, 0)

    def test_whitelist_excludes_sensor_truth_and_local_services(self):
        for robot in (0, 1):
            specs = topic_specs(robot)
            self.assertNotIn('/sim_%d/ground_truth/odometry' % robot, specs)
            self.assertNotIn('/sim_%d/kimera/imu' % robot, specs)
            self.assertFalse(any('request_pose_graph' in p for p in specs))
            with self.assertRaises(ValueError):
                validate_source(specs['/broadcast_bspline'], proxy_caller(robot))
            self.assertEqual(specs['/sim_%d/mission/status' % robot].lane, 'ego')
        self.assertTrue(requires_native_subscriber(
            topic_specs(0)['/sim_0/dpgo_ros_node/command'], 0))
        self.assertFalse(requires_native_subscriber(
            topic_specs(0)['/sim_0/kimera_vio_ros/bow_query'], 0))
        self.assertTrue(requires_native_subscriber(
            topic_specs(1)['/sim_1/kimera_vio_ros/bow_query'], 1))

    def test_receipt_binds_native_source_to_the_same_peer_run_and_boot(self):
        header, payload = message()
        receipt = dict(header, verified=True, local_robot_id=1, peer_robot_id=0,
                       source_robot_id=0, source_boot_id='a' * 32)
        kwargs = dict(run_id='test_run', cutoff_ns=1, local_robot_id=1,
            peer_boot_id='a' * 32, topic=header['topic'], sha256=header['sha256'])
        self.assertTrue(validate_ingress_receipt(receipt, **kwargs))
        for change in ({'source_boot_id': 'b' * 32}, {'run_id': 'old'},
                       {'source_caller': '/sim_1/dpgo_ros_node/agent'},
                       {'verified': False}):
            with self.subTest(change=change), self.assertRaises(WireError):
                validate_ingress_receipt(dict(receipt, **change), **kwargs)

    def test_schema_digest_requires_every_native_message_type(self):
        value = contracts()
        self.assertEqual(len(contract_digest(value)), 64)
        value.pop('dpgo_ros/PublicPoses')
        with self.assertRaises(WireError):
            contract_digest(value)


if __name__ == '__main__':
    unittest.main()
