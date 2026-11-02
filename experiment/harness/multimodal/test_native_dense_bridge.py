#!/usr/bin/env python3
"""Contract regression tests; actual native integration is a separate gate."""
import copy
import json
from collections import Counter, deque
from types import SimpleNamespace as S
import threading
import tempfile
from pathlib import Path
import time
import unittest
from unittest.mock import patch

import numpy as np
import native_dense_bridge as dense_module

from native_dense_bridge import (KeyframeLedger, PoseHistory, RoundLedger, image_array,
    canonical_mesh_digest, native_readiness_query, optical_points, optimized_camera_to_body, path_error,
    path_geometry_digest, pose_matrix, publish_preserving_cached_message, require_source, require_stamp, retag_local_mesh,
    same_mesh_topology_and_stamps, set_pose, SIM_T_BODY_CAMERA, validate_mesh, NativeDenseBridge)


class FakeMessage(S):
    def serialize(self, stream):
        def simplify(value):
            if hasattr(value, 'to_nsec'): return value.to_nsec()
            if isinstance(value, S): return {k: simplify(v) for k, v in vars(value).items()}
            if isinstance(value, list): return [simplify(v) for v in value]
            return value
        stream.write(json.dumps(simplify(self), sort_keys=True).encode())


class Stamp:
    def __init__(self, value): self.value = value
    def to_nsec(self): return self.value
    def __eq__(self, other): return self.value == other.value


class Edge(FakeMessage):
    def serialize(self, stream):
        stream.write(np.array([self.pose.position.x]+self.covariance).tobytes())


def pose(x=0.):
    return S(position=S(x=x, y=0., z=0.), orientation=S(x=0., y=0., z=0., w=1.))


def node(key, stamp=None, robot=0):
    return S(key=key, robot_id=robot, header=S(stamp=Stamp(stamp or 100+key)), pose=pose(key))


def command(kind, owner=0):
    return S(command=kind, publishing_robot=owner, cluster_id=0, active_robots=[0, 1],
        REQUEST_POSE_GRAPH=0, UPDATE_WEIGHT=1, TERMINATE=2, HARD_TERMINATE=3)


def image(array, encoding):
    return S(width=array.shape[1], height=array.shape[0], encoding=encoding,
             is_bigendian=False, step=array.strides[0], data=array.tobytes())


class DenseContracts(unittest.TestCase):
    def test_source_and_epoch_fail_closed(self):
        require_source(S(_connection_header={'callerid': '/native'}), '/native')
        with self.assertRaises(ValueError): require_source(S(_connection_header={'callerid': '/truth'}), '/native')
        with self.assertRaises(ValueError): require_source(S(), '/native')
        with self.assertRaises(ValueError): require_stamp(Stamp(0), 10)
        with self.assertRaises(ValueError): require_stamp(Stamp(9), 10)
        self.assertEqual(require_stamp(Stamp(10), 10), 10)

    def test_original_index_and_timestamp_contract(self):
        ledger = KeyframeLedger(0, 100)
        ledger.ingest(S(nodes=[node(0), node(1)], edges=[]))
        self.assertEqual([n.key for n in ledger.contiguous([node(1), node(0)])], [0, 1])
        for invalid in ([node(1)], [node(0), node(0)], [node(0), node(2)],
                        [node(0), node(1, 999)], [node(0), node(1, robot=1)]):
            with self.assertRaises(ValueError): ledger.contiguous(invalid)
        with self.assertRaises(ValueError): ledger.ingest(S(nodes=[node(0, 999)], edges=[]))

    def test_complete_nodes_do_not_hide_a_missing_original_odometry_edge(self):
        ledger = KeyframeLedger(0, 100)
        ledger.ingest(S(nodes=[node(0), node(1), node(2)], edges=[]))
        with self.assertRaises(ValueError): ledger.require_odometry_through(2)
        ledger.odometry[0, 1] = S()
        with self.assertRaises(ValueError): ledger.require_odometry_through(2)
        ledger.odometry[1, 2] = S()
        ledger.require_odometry_through(2)

    def test_original_edge_values_and_unknown_covariance_are_preserved(self):
        ledger = KeyframeLedger(0, 100)
        # Real original messages are serializable; fixture serializer exercises
        # immutable measurement identity, rather than inventing covariance.
        edge = Edge(robot_from=0, robot_to=0, key_from=0, key_to=1,
            ODOM=0, type=0, header=S(stamp=Stamp(101)), pose=pose(1.), covariance=[0.]*36)
        ledger.ingest(S(nodes=[node(0), node(1)], edges=[edge]))
        self.assertEqual(ledger.odometry[0, 1].covariance, [0.]*36)
        self.assertEqual(edge.pose.position.x, 1.)
        edge.pose.position.x = 2.
        with self.assertRaises(ValueError): ledger.ingest(S(nodes=[], edges=[edge]))

    def test_optical_Z_conversion_invalid_depth_and_padding(self):
        rgb = np.array([[[10, 20, 30], [40, 50, 60]], [[70, 80, 90], [100, 110, 120]]], np.uint8)
        depth = np.array([[2., 0.], [np.nan, 3.]], '<f4')
        xyz, color = optical_points(image(rgb, 'rgb8'), image(depth, '32FC1'),
            S(K=[2., 0., 0., 0., 2., 0., 0., 0., 1.], width=2, height=2), 1, .3, 8.)
        np.testing.assert_array_equal(xyz, [[0., 0., 2.], [1.5, 1.5, 3.]])
        self.assertEqual(color.tolist(), [0x0A141E, 0x646E78])
        padded = S(width=1, height=2, encoding='32FC1', is_bigendian=False, step=8,
                   data=np.array([1., 999., 2., 999.], '<f4').tobytes())
        np.testing.assert_array_equal(image_array(padded), [[1.], [2.]])

    def test_native_pose_interpolation_and_gap_rejection(self):
        history = PoseHistory(100)
        history.put(100, pose()); history.put(200, pose(2.))
        np.testing.assert_allclose(history.at(150)[:3, 3], [1., 0., 0.])
        self.assertIsNone(history.at(99)); self.assertIsNone(history.at(201))
        history.put(400, pose(4.))
        with self.assertRaises(ValueError): history.at(300)

    def test_metadata_retag_preserves_all_geometry_keys_stamps(self):
        mesh = S(ns='0', header=S(frame_id='sim_1/vio_odom'),
            vertices=[S(x=1., y=2., z=3.)], triangles=[S(vertex_indices=[0, 0, 0])],
            vertex_indices=[7], vertex_stamps=[Stamp(100)], vertex_colors=[S(r=.2, g=.3, b=.4, a=1.)])
        before = copy.deepcopy(mesh); result = retag_local_mesh(mesh, 1)
        self.assertEqual(result.ns, '1'); self.assertEqual(mesh.ns, '0')
        result.ns = '0'
        self.assertEqual(vars(result.header), vars(before.header))
        self.assertEqual([vars(p) for p in result.vertices], [vars(p) for p in before.vertices])
        self.assertEqual(result.triangles[0].vertex_indices, before.triangles[0].vertex_indices)
        self.assertEqual(result.vertex_indices, before.vertex_indices)
        self.assertEqual(result.vertex_stamps, before.vertex_stamps)
        self.assertEqual([vars(c) for c in result.vertex_colors], [vars(c) for c in before.vertex_colors])
        mesh.ns = '2'
        with self.assertRaises(ValueError): retag_local_mesh(mesh, 1)

    def test_mesh_validity_requires_real_triangles_and_native_stamps(self):
        mesh = S(header=S(frame_id='sim_0/vio_odom'), vertices=[S(x=0., y=0., z=0.)],
            triangles=[S(vertex_indices=[0, 0, 0])], vertex_indices=[-1], vertex_stamps=[Stamp(100)])
        self.assertEqual(validate_mesh(mesh, 100, 'sim_0/vio_odom').shape, (1, 3))
        with self.assertRaises(ValueError): validate_mesh(mesh, 100, 'world')
        mesh.triangles[0].vertex_indices[-1] = 1
        with self.assertRaises(ValueError): validate_mesh(mesh, 100, 'sim_0/vio_odom')

    def test_round_rejects_incomplete_cached_and_pretermination_weights(self):
        ledger = RoundLedger(updates=2)
        path = S(poses=[S(pose=pose(1)), S(pose=pose(2))]); ledger.native_path(path, 1.)
        ledger.command(0, command(0), 2.)
        ledger.command(0, command(1), 3.); ledger.command(0, command(1), 4.)
        weight = S(src_robot_ids=[0], src_pose_ids=[1], dst_robot_ids=[1], dst_pose_ids=[2], weights=[1.], fixed_weights=[True])
        ledger.weights(0, weight, 4.5); ledger.command(0, command(2), 5.)
        ledger.native_path(path, 6.)  # cached timer echo must not authorize output
        self.assertIsNone(ledger.proof(7., True))
        ledger.native_path(S(poses=[S(pose=pose(1)), S(pose=pose(3))]), 6.5)
        self.assertIsNone(ledger.proof(7., True))  # no final native weights yet
        ledger.weights(0, weight, 6.8)
        self.assertEqual(ledger.proof(7., True)['accepted_cross_constraints'], 1)
        ledger.command(0, command(0), 8.)
        self.assertIsNone(ledger.proof(9., True))
        ledger.command(0, command(1), 9.); ledger.command(0, command(2), 10.)
        ledger.native_path(S(poses=[S(pose=pose(1)), S(pose=pose(4))]), 11.); ledger.weights(0, weight, 11.)
        self.assertIsNone(ledger.proof(12., True))  # missing one requested GNC update

    def test_cached_path_republication_is_not_changed_geometry(self):
        first = S(header=S(stamp=Stamp(100)), poses=[S(header=S(stamp=Stamp(100)), pose=pose(1.))])
        second = copy.deepcopy(first); second.header.stamp = Stamp(200); second.poses[0].header.stamp = Stamp(200)
        self.assertEqual(path_geometry_digest(first), path_geometry_digest(second))
        second.poses[0].pose.position.x = 1.1
        self.assertNotEqual(path_geometry_digest(first), path_geometry_digest(second))

    def test_body_camera_path_conversion_is_not_independent_alignment(self):
        body = np.eye(4); body[:3, 3] = [4., -2., 1.]
        camera_pose = pose(); set_pose(camera_pose, body @ SIM_T_BODY_CAMERA)
        np.testing.assert_allclose(optimized_camera_to_body(camera_pose), body, atol=1e-12)
        expected = S(poses=[S(pose=pose(4.))])
        actual = copy.deepcopy(expected); actual.poses[0].pose.position.x += .5
        self.assertAlmostEqual(path_error(actual, expected)['max_translation_m'], .5)

    def test_readiness_query_has_no_measurement_and_retains_actual_key_zero(self):
        original = S(header=S(frame_id='sim_0/map'), nodes=[node(8), node(9)], edges=[S(type=0)])
        zero = node(0)
        query = native_readiness_query(original, zero)
        self.assertEqual(query.edges, [])
        self.assertEqual([n.key for n in query.nodes], [0])
        self.assertEqual(query.nodes[0].header.stamp, zero.header.stamp)
        np.testing.assert_array_equal(pose_matrix(query.nodes[0].pose), pose_matrix(zero.pose))
        self.assertEqual(len(original.edges), 1)
        self.assertEqual([n.key for n in original.nodes], [8, 9])
        with self.assertRaises(ValueError): native_readiness_query(original, node(1))

    def test_same_fixed_mesh_proof_rejects_new_topology_or_time(self):
        original = S(vertices=[S(x=0., y=0., z=0.)], triangles=[S(vertex_indices=[0, 0, 0])], vertex_stamps=[Stamp(100)])
        changed_geometry = copy.deepcopy(original); changed_geometry.vertices[0].x = .5
        self.assertTrue(same_mesh_topology_and_stamps(original, changed_geometry))
        changed_geometry.vertex_stamps[0] = Stamp(101)
        self.assertFalse(same_mesh_topology_and_stamps(original, changed_geometry))

    def test_rospy_header_sequence_mutation_cannot_invalidate_mesh_cache(self):
        retained = S(header=S(seq=0), vertices=[S(x=-0., y=1., z=2.)])
        received = []
        def publish(message):
            message.header.seq = 42
            received.append(message)
        publisher = S(publish=publish)
        publish_preserving_cached_message(publisher, retained)
        publish_preserving_cached_message(publisher, retained)
        self.assertEqual(retained.header.seq, 0)
        self.assertEqual([message.header.seq for message in received], [42, 42])
        self.assertIsNot(received[0], received[1])

    def test_accepted_mesh_hash_ignores_only_transport_sequence(self):
        first = FakeMessage(header=S(seq=1, frame_id='kimera_world', stamp_ns=100),
                            vertices=[S(x=0., y=1., z=2.)], triangles=[[0, 0, 0]])
        second = copy.deepcopy(first); second.header.seq = 99
        self.assertEqual(canonical_mesh_digest(first), canonical_mesh_digest(second))
        second.vertices[0].x = .001
        self.assertNotEqual(canonical_mesh_digest(first), canonical_mesh_digest(second))

    def test_anchor_waits_for_all_frozen_native_keys_not_only_DPGO_prefix(self):
        node = object.__new__(NativeDenseBridge); published = []
        node.pub = {'full': S(publish=lambda message: published.append(message))}
        node.transaction = dict(phase='await_native_seed', expected_native_body_keys=141,
            fixed_mesh=S(header=S(seq=0)), phase_started_wall=time.time())
        node.on_pgmo_path(S(poses=[None]*130))
        self.assertEqual(node.transaction['phase'], 'await_native_seed')
        self.assertEqual(published, [])
        node.on_pgmo_path(S(poses=[None]*141))
        self.assertEqual(node.transaction['phase'], 'await_baseline')
        self.assertEqual(len(published), 1)

    def test_native_transaction_deadline_is_per_phase_and_finite(self):
        node = object.__new__(NativeDenseBridge); events = []
        node.a = S(mesh_phase_timeout=30.)
        node.counts = Counter(); node.event = lambda *args, **kwargs: events.append((args, kwargs))
        node.transaction = dict(phase='await_baseline', started_wall=time.time()-50.,
            phase_started_wall=time.time(), waiting_full_mesh_ack=True)
        node.transaction_tick()
        self.assertIsNotNone(node.transaction)  # previous native phase can take time
        node.transaction['phase_started_wall'] = time.time()-31.
        node.transaction_tick()
        self.assertIsNone(node.transaction)
        self.assertEqual(node.counts['mesh_transactions_not_accepted'], 1)

    def test_unapproved_optimized_snapshot_cannot_fail_on_future_originals(self):
        bridge = object.__new__(NativeDenseBridge)
        bridge.transaction = None; bridge.keys = KeyframeLedger(0, 100)
        bridge.keys.ingest(S(nodes=[node(0)], edges=[]))
        bridge.rounds = S(proof=lambda *args: None); bridge.a = S(require_cross=True)
        bridge.counts = Counter(); bridge.camera_graph_contract = None
        bridge.on_optimized_graph(FakeMessage(nodes=[node(0), node(1)]))
        self.assertEqual(bridge.counts['optimized_graphs_without_verified_camera_graph_contract'], 1)
        # A missing original is asynchronous transport; foreign owners, old
        # epochs and invalid complete-array indices remain malformed data.
        for invalid in ([node(0), node(1, robot=1)], [node(0), node(2)], [node(0), node(1, 99)]):
            with self.assertRaises(ValueError): bridge.on_optimized_graph(FakeMessage(nodes=invalid))

    def test_eligible_candidate_waits_for_actual_original_and_revalidates_stamp(self):
        bridge = object.__new__(NativeDenseBridge); events = []
        bridge.transaction = None; bridge.keys = KeyframeLedger(0, 100)
        bridge.keys.ingest(S(nodes=[node(0)], edges=[]))
        proof = {'round': {'serial': 7, 'owner': 0, 'active_robots': [0, 1]}}
        bridge.rounds = S(proof=lambda *args: proof); bridge.a = S(require_cross=True, robot_id=0)
        bridge.counts = Counter(); bridge.event = lambda *args, **kw: events.append((args, kw))
        bridge.missing_original_since = {}; bridge.optimized_candidate_wait = 60.
        bridge.camera_graph_contract = {'native_key_zero_stamp_ns': 100}
        bridge.pending_optimized = None
        candidate = dict(nodes=[node(0), node(1)], proof=proof, signature='actual-candidate',
            raw=b'native-original-serialization', received_wall=time.time(), last_missing=None)
        bridge.optimized_candidates = deque([candidate]); bridge.advance_optimized_candidates()
        self.assertIsNone(bridge.pending_optimized)
        self.assertEqual(bridge.optimized_candidates[0]['last_missing'], 1)
        bridge.keys.ingest(S(nodes=[node(1)], edges=[])); bridge.advance_optimized_candidates()
        self.assertEqual(bridge.pending_optimized[2:], ('actual-candidate', b'native-original-serialization'))
        self.assertEqual([x.key for x in bridge.pending_optimized[0]], [0, 1])
        # A late original with another stamp must fail, never authorize the path.
        bridge.keys = KeyframeLedger(0, 100); bridge.keys.ingest(S(nodes=[node(0), node(1, 999)], edges=[]))
        bridge.optimized_candidates = deque([candidate])
        with self.assertRaises(ValueError): bridge.advance_optimized_candidates()

    def test_missing_original_deadline_survives_replacement_generations(self):
        bridge = object.__new__(NativeDenseBridge)
        bridge.transaction = None; bridge.keys = KeyframeLedger(0, 100)
        bridge.keys.ingest(S(nodes=[node(0)], edges=[]))
        proof = {'round': {'serial': 7, 'owner': 0, 'active_robots': [0, 1]}}
        bridge.rounds = S(proof=lambda *args: proof); bridge.a = S(require_cross=True, robot_id=0)
        bridge.counts = Counter(); bridge.event = lambda *args, **kw: None
        bridge.missing_original_since = {1: time.monotonic()-61.}
        bridge.optimized_candidate_wait = 60.; bridge.pending_optimized = None
        bridge.optimized_candidates = deque([dict(nodes=[node(0), node(1)], proof=proof,
            signature='newer-generation', raw=b'new', received_wall=time.time(), last_missing=None)])
        bridge.advance_optimized_candidates()
        self.assertEqual(len(bridge.optimized_candidates), 0)
        self.assertIsNone(bridge.pending_optimized)
        self.assertEqual(bridge.counts['optimized_candidates_expired_missing_originals'], 1)

    def test_eligible_candidate_can_wait_for_original_key_zero_without_epoch_fit(self):
        bridge = object.__new__(NativeDenseBridge)
        bridge.transaction = None; bridge.keys = KeyframeLedger(0, 100)
        proof = {'round': {'serial': 7, 'owner': 0, 'active_robots': [0, 1]}}
        bridge.rounds = S(proof=lambda *args: proof); bridge.a = S(require_cross=True, robot_id=0)
        bridge.camera_graph_contract = {'native_key_zero_stamp_ns': 100}
        bridge.counts = Counter(); bridge.event = lambda *args, **kw: None
        bridge.missing_original_since = {}; bridge.optimized_candidate_wait = 60.
        bridge.pending_optimized = None
        candidate = dict(nodes=[node(0)], proof=proof, signature='native-key0',
            raw=b'native-key0', received_wall=time.time(), last_missing=None)
        bridge.optimized_candidates = deque([candidate]); bridge.advance_optimized_candidates()
        self.assertIsNone(bridge.pending_optimized)
        bridge.keys.ingest(S(nodes=[node(0)], edges=[])); bridge.advance_optimized_candidates()
        self.assertEqual(bridge.pending_optimized[2], 'native-key0')
        bridge.camera_graph_contract['native_key_zero_stamp_ns'] = 999
        bridge.optimized_candidates = deque([candidate])
        with self.assertRaises(ValueError): bridge.advance_optimized_candidates()

    def test_priority_original_retention_does_not_wait_for_mesh_state_lock(self):
        bridge = object.__new__(NativeDenseBridge)
        bridge.a = S(robot_id=0, robot_name='sim_0'); bridge.cutoff_ns = 100
        bridge.lock = threading.RLock(); bridge.native_input_lock = threading.Lock()
        bridge.native_inputs = deque(); bridge.native_input_max_queue = 0; bridge.native_input_max_delay = 0.
        bridge.subs = []; bridge.counts = Counter(); bridge.fatal = None; bridge.missing_original_since = {}
        bridge.keys = KeyframeLedger(0, 100); bridge.graph_queue = deque(); bridge.graph_sent = -1
        callbacks = []; kwargs_seen = []
        def subscriber(topic, kind, callback, **kwargs):
            callbacks.append(callback); kwargs_seen.append(kwargs); return S()
        received = threading.Event(); errors = []
        bridge.fail = lambda error: errors.append(str(error))
        with patch.object(dense_module, 'rospy', S(Subscriber=subscriber), create=True):
            bridge.subscribe('/original', object, bridge.on_graph, '/native', None, priority=True)
        def ingress():
            for key in range(80):
                callbacks[0](S(_connection_header={'callerid': '/native'},
                    header=S(frame_id='sim_0/map'), nodes=[node(key)], edges=[]))
            received.set()
        with bridge.lock:
            worker = threading.Thread(target=ingress); worker.start()
            self.assertTrue(received.wait(2.), 'Native ingress must retain originals while mesh lock is busy')
            self.assertEqual(len(bridge.native_inputs), 80)
            bridge.drain_native_inputs()
        worker.join(2.)
        self.assertIsNone(kwargs_seen[0]['queue_size'])
        self.assertEqual(set(bridge.keys.nodes), set(range(80)))
        self.assertEqual(len(bridge.graph_queue), 80)
        self.assertEqual(errors, [])

    def test_shutdown_publisher_exception_cannot_overwrite_primary_fatal(self):
        bridge = object.__new__(NativeDenseBridge); events = []; shutdowns = []
        bridge.lock = threading.RLock(); bridge.fatal = None; bridge.a = S(run_id='native-test')
        bridge.event = lambda *args, **kw: events.append((args, kw))
        with tempfile.TemporaryDirectory() as directory:
            bridge.evidence = Path(directory)
            with patch.object(dense_module, 'rospy', S(logfatal=lambda *args: None,
                    signal_shutdown=lambda reason: shutdowns.append(reason)), create=True):
                bridge.fail(ValueError('Original native stamp mismatch'))
                bridge.fail(RuntimeError('publish() to a closed topic'))
            self.assertEqual(bridge.fatal, 'Original native stamp mismatch')
            self.assertEqual(json.loads((bridge.evidence/'fatal.json').read_text())['error'], bridge.fatal)
        self.assertEqual(len(events), 1); self.assertEqual(shutdowns, ['Original native stamp mismatch'])

    def test_control_fifo_uses_ingress_times_while_mesh_state_lock_is_busy(self):
        bridge = object.__new__(NativeDenseBridge)
        bridge.lock = threading.RLock(); bridge.native_input_lock = threading.Lock()
        bridge.native_inputs = deque(); bridge.native_input_max_queue = 0
        bridge.native_input_max_delay = bridge.native_control_max_delay = 0.
        bridge.subs = []; bridge.counts = Counter(); bridge.fatal = None
        bridge.rounds = RoundLedger(updates=2); callbacks = []; done = threading.Event()
        bridge.fail = lambda error: self.fail(str(error))
        def subscriber(topic, kind, callback, **kwargs): callbacks.append(callback); return S()
        with patch.object(dense_module, 'rospy', S(Subscriber=subscriber), create=True):
            bridge.subscribe('/command', object, lambda m, t: bridge.rounds.command(0, m, t),
                '/native', None, priority='control', with_receive_wall=True)
            bridge.subscribe('/weights', object, lambda m, t: bridge.rounds.weights(0, m, t),
                '/native', None, priority='control', with_receive_wall=True)
            bridge.subscribe('/path', object, lambda m, t: bridge.rounds.native_path(m, t),
                '/native', None, priority='control', with_receive_wall=True)
        weight = S(src_robot_ids=[0], src_pose_ids=[0], dst_robot_ids=[1], dst_pose_ids=[0],
            weights=[1.], fixed_weights=[True], _connection_header={'callerid': '/native'})
        cached_path = S(poses=[S(pose=pose())], _connection_header={'callerid': '/native'})
        bridge.rounds.native_path(cached_path, .5)
        def ingress():
            for kind in (0, 1, 1, 2):
                message = command(kind); message._connection_header = {'callerid': '/native'}; callbacks[0](message)
            callbacks[1](weight); callbacks[2](cached_path); done.set()
        with bridge.lock:
            with patch.object(dense_module.time, 'time', side_effect=[1., 2., 3., 4., 5., 6.]):
                thread = threading.Thread(target=ingress); thread.start()
                self.assertTrue(done.wait(2.)); thread.join(2.)
            self.assertIsNone(bridge.rounds.completed)  # state lock work not run yet
            bridge.drain_native_inputs()
        proof = bridge.rounds.proof(10., True)
        self.assertEqual(proof['round']['terminate_wall'], 4.)
        self.assertEqual(proof['weights']['received_wall'], 5.)
        self.assertEqual(proof['upstream_dpgo_path']['received_wall'], 6.)
        self.assertFalse(proof['upstream_dpgo_path']['numerical_change_from_previous_cached_path'])

    def test_posttermination_graph_waits_for_independent_round_metadata_delivery(self):
        bridge = object.__new__(NativeDenseBridge); bridge.transaction = None
        bridge.keys = KeyframeLedger(0, 100); bridge.keys.ingest(S(nodes=[node(0)], edges=[]))
        bridge.a = S(require_cross=True, robot_id=0); bridge.base = '/sim_0'
        bridge.rounds = RoundLedger(updates=2); bridge.rounds.command(0, command(0), 1.)
        bridge.rounds.command(0, command(1), 2.); bridge.rounds.command(0, command(1), 3.)
        bridge.rounds.command(0, command(2), 4.)
        bridge.camera_graph_contract = {'native_key_zero_stamp_ns': 100}
        bridge.counts = Counter(); bridge.event = lambda *args, **kwargs: None
        bridge.optimized_candidates = deque(); bridge.optimized_candidate_limit = 4
        bridge.optimized_candidate_wait = 60.; bridge.missing_original_since = {}
        bridge.last_optimized_sha = None; bridge.pending_optimized = None
        master = S(getSystemState=lambda: (1, '', [[], [('/sim_0/kimera_camera_adapter/pose_graph_incremental',
            ['/sim_0/distributed_loop_closure/distributed_loop_closure_node'])], []]))
        with patch.object(dense_module, 'rospy', S(get_master=lambda: master, get_param=lambda *args: False), create=True):
            bridge.on_optimized_graph(FakeMessage(nodes=[node(0), node(1)]), received_wall=5.)
        self.assertEqual(len(bridge.optimized_candidates), 1)
        self.assertIsNone(bridge.pending_optimized)
        weight = S(src_robot_ids=[0], src_pose_ids=[0], dst_robot_ids=[1], dst_pose_ids=[0], weights=[1.], fixed_weights=[True])
        bridge.rounds.weights(0, weight, 6.); bridge.rounds.native_path(S(poses=[S(pose=pose())]), 7.)
        bridge.keys.ingest(S(nodes=[node(1)], edges=[])); bridge.advance_optimized_candidates()
        self.assertIsNotNone(bridge.pending_optimized)
        self.assertEqual(bridge.pending_optimized[1]['round']['terminate_wall'], 4.)

    def test_transaction_freezes_with_live_originals_without_prequiet_interval(self):
        bridge = object.__new__(NativeDenseBridge); sent = []; bridge.transaction = None
        bridge.pgmo_initialized = True; bridge.last_graph_wall = time.time()
        bridge.latest_full_mesh = FakeMessage(header=S(seq=0), vertices=[S(x=0., y=0., z=0.)])
        bridge.latest_full_mesh_bytes = b'exact-native-fixed-mesh'; bridge.latest_full_mesh_sha = 'fixed-native-sha'
        proof = {'round': {'serial': 7}}; bridge.rounds = S(proof=lambda *args: proof)
        bridge.graph_sent = 2; bridge.pending_optimized = ([node(0), node(1)], proof, 'native-graph', b'native-graph-bytes')
        bridge.pub = {'path': S(get_num_connections=lambda: 1), 'full': S(publish=lambda msg: sent.append(msg))}
        bridge.a = S(require_cross=True, global_frame='kimera_world'); bridge.counts = Counter()
        bridge.camera_graph_contract = {'native_key_zero_stamp_ns': 100}; bridge.event = lambda *args, **kwargs: None
        ros = S(Time=S(now=lambda: Stamp(1000)))
        with tempfile.TemporaryDirectory() as directory:
            bridge.evidence = Path(directory)
            with patch.multiple(dense_module, rospy=ros, RosPath=lambda header: FakeMessage(header=header, poses=[]),
                    Header=lambda **kwargs: S(**kwargs), PoseStamped=lambda **kwargs: S(**kwargs), create=True):
                bridge.approve_path()
            self.assertEqual(bridge.transaction['phase'], 'await_native_seed')
            self.assertEqual(bridge.transaction['expected_native_body_keys'], 3)
            self.assertEqual(len(bridge.transaction['path'].poses), 2)
            self.assertEqual(len(sent), 1)
            self.assertEqual((Path(directory)/'native_transaction_0001_fixed_input_mesh.rosbin').read_bytes(),
                             b'exact-native-fixed-mesh')

    def test_deferred_round_receipt_cannot_bypass_active_robot_membership(self):
        bridge = object.__new__(NativeDenseBridge); bridge.transaction = None
        bridge.keys = KeyframeLedger(0, 100); bridge.keys.ingest(S(nodes=[node(0)], edges=[]))
        proof = {'round': {'serial': 7, 'owner': 0, 'active_robots': [1], 'terminate_wall': 4.}}
        bridge.rounds = S(proof=lambda *args: proof); bridge.a = S(require_cross=True, robot_id=0)
        bridge.counts = Counter(); bridge.event = lambda *args, **kwargs: None
        bridge.optimized_candidate_wait = 60.; bridge.missing_original_since = {}; bridge.pending_optimized = None
        bridge.optimized_candidates = deque([dict(nodes=[node(0)], proof=None, signature='native', raw=b'native',
            received_wall=5., received_monotonic=time.monotonic(), last_missing=None)])
        bridge.advance_optimized_candidates()
        self.assertEqual(bridge.counts['optimized_candidates_outside_active_cluster'], 1)
        self.assertIsNone(bridge.pending_optimized)

    def test_native_mesh_factors_wait_for_initial_metadata_ack_without_moving_quiet(self):
        bridge = object.__new__(NativeDenseBridge); sent = []
        bridge.transaction = None; bridge.gauge_checked = True; bridge.graph_sent = 8
        bridge.keys = KeyframeLedger(0, 100); bridge.keys.ingest(S(nodes=[node(0)], edges=[]))
        bridge.a = S(robot_id=0); bridge.native_body_metadata_ack_has_optimized_key_zero = False
        bridge.native_body_metadata_ack = False; bridge.last_graph_wall = time.time()
        bridge.graph_queue = deque(); bridge.mesh_graph_queue = deque([S(factor=1), S(factor=2)])
        bridge.counts = Counter(); bridge.pub = {'graph': S(get_num_connections=lambda: 1),
            'mesh_graph': S(get_num_connections=lambda: 1, publish=lambda message: sent.append(message.factor))}
        bridge.publish_graphs(); self.assertEqual(sent, [])
        bridge.on_pgmo_graph(S(nodes=[], edges=[]))  # native callback completion even before new_values optimized
        bridge.last_graph_wall = time.time(); bridge.publish_graphs()
        self.assertEqual(sent, [1, 2])
        self.assertEqual(len(bridge.mesh_graph_queue), 0)
        self.assertFalse(bridge.native_body_metadata_ack_has_optimized_key_zero)
        bridge.on_pgmo_graph(S(nodes=[node(0)], edges=[]))
        self.assertTrue(bridge.native_body_metadata_ack_has_optimized_key_zero)
        with self.assertRaises(ValueError): bridge.on_pgmo_graph(S(nodes=[node(0, 999)], edges=[]))


if __name__ == '__main__': unittest.main(verbosity=2)
