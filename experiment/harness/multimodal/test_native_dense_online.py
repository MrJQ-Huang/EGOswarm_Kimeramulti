#!/usr/bin/env python3
"""Offline production contracts; no ROS master, deployment or native solver."""
import copy
from collections import Counter, deque
import hashlib
import json
import io
from pathlib import Path
import struct
import time
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

import native_dense_bridge as dense
import native_local_sensor_bridge as local_sensor
from test_native_dense_bridge import Edge, FakeMessage, Stamp, command, node, pose


class SerializedCommand(FakeMessage):
    _type = 'dpgo_ros/Command'
    _md5sum = 'c'*32
    _has_header = False

    def deserialize(self, payload):
        self.__dict__.update(json.loads(payload)); return self


def random_path(count, rng):
    items = []
    for key in range(count):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        p = rng.uniform(-100., 100., size=3)
        value = S(position=S(**dict(zip(('x','y','z'), map(float,p)))),
                  orientation=S(**dict(zip(('x','y','z','w'), map(float,q)))))
        items.append(S(header=S(seq=key,stamp=Stamp(100+key),frame_id='kimera_world'),pose=value))
    return FakeMessage(header=S(seq=7,stamp=Stamp(1234),frame_id='kimera_world'),poses=items)


def inverse_reference_error(actual, expected):
    """Independent original numerical reference: invert each target SE(3)."""
    if len(actual.poses) < len(expected.poses): return None
    translations, rotations = [0.], [0.]
    for a, e in zip(actual.poses,expected.poses):
        delta = np.linalg.inv(dense.pose_matrix(e.pose)) @ dense.pose_matrix(a.pose)
        translations.append(float(np.linalg.norm(delta[:3,3])))
        rotations.append(float(np.arccos(np.clip((np.trace(delta[:3,:3])-1.)/2.,-1.,1.))))
    return dict(max_translation_m=max(translations),max_rotation_rad=max(rotations))


class BatchedPoseResiduals(unittest.TestCase):
    def test_random_finite_rigid_transforms_match_original_inverse_reference_without_mutation(self):
        rng = np.random.default_rng(7248)
        for count in (1,7,257,2049):
            with self.subTest(count=count):
                actual, expected = random_path(count,rng), random_path(count,rng)
                before = (dense.message_bytes(actual),dense.message_bytes(expected))
                reference = inverse_reference_error(actual,expected)
                result = dense.path_error(actual,expected)
                np.testing.assert_allclose([result[k] for k in reference],list(reference.values()),rtol=1e-10,atol=5e-8)
                self.assertEqual(before,(dense.message_bytes(actual),dense.message_bytes(expected)))

    def test_identity_antipodal_tiny_and_pi_rotation_boundaries(self):
        axis = np.array([1.,2.,-3.]); axis /= np.linalg.norm(axis)
        for angle in (0.,1e-12,1e-8,.0001,np.pi-1e-9,np.pi):
            for sign in (1.,-1.):
                with self.subTest(angle=angle,sign=sign):
                    expected = S(poses=[S(pose=pose())]); actual = copy.deepcopy(expected)
                    q = sign*np.r_[axis*np.sin(angle/2.),np.cos(angle/2.)]
                    for k,v in zip(('x','y','z','w'),q):setattr(actual.poses[0].pose.orientation,k,float(v))
                    actual.poses[0].pose.position = S(x=-3.,y=4.,z=12.)
                    result = dense.path_error(actual,expected); reference = inverse_reference_error(actual,expected)
                    self.assertAlmostEqual(result['max_translation_m'],13.,places=12)
                    self.assertAlmostEqual(result['max_rotation_rad'],angle,places=12)
                    self.assertAlmostEqual(result['max_rotation_rad'],reference['max_rotation_rad'],delta=5e-8)

    def test_invalid_pose_boundaries_and_original_prefix_semantics(self):
        original = S(poses=[S(pose=pose())])
        for field,value in [('position.x',float('nan')),('position.z',float('inf')),
                            ('orientation.w',0.),('orientation.w',1.0010001)]:
            invalid=copy.deepcopy(original);parent,key=field.split('.');setattr(getattr(invalid.poses[0].pose,parent),key,value)
            with self.subTest(field=field,value=value):
                with self.assertRaises(ValueError):dense.path_error(invalid,original)
                with self.assertRaises(ValueError):inverse_reference_error(invalid,original)
        almost=copy.deepcopy(original);almost.poses[0].pose.orientation.w=1.0009999
        self.assertEqual(dense.path_error(almost,original)['max_rotation_rad'],0.)
        self.assertIsNone(dense.path_error(S(poses=[]),original))
        self.assertEqual(dense.path_error(original,S(poses=[])),dict(max_translation_m=0.,max_rotation_rad=0.))
        # The native full path can extend beyond the submitted prefix. Keep
        # original prefix-comparison behavior; source ingress validates all rows.
        extended=copy.deepcopy(original);extra=copy.deepcopy(original.poses[0]);extra.pose.position.x=float('nan')
        extended.poses.append(extra)
        self.assertEqual(dense.path_error(extended,original),inverse_reference_error(extended,original))

    def test_extrinsic_inverse_is_reused_and_exact_diagnostic_metric_is_preserved(self):
        rng=np.random.default_rng(625)
        native=random_path(80,rng)
        references=[dense.pose_matrix(p.pose) @ np.linalg.inv(dense.SIM_T_BODY_CAMERA) for p in native.poses]
        before=dense.message_bytes(native)
        with patch.object(dense.np.linalg,'inv',side_effect=AssertionError('constant extrinsic inverse recomputed')):
            for p,reference in zip(native.poses,references):
                np.testing.assert_array_equal(dense.optimized_camera_to_body(p.pose),reference)
        self.assertFalse(dense.SIM_T_CAMERA_BODY.flags.writeable)
        self.assertEqual(dense.message_bytes(native),before)
        other=random_path(80,rng)
        self.assertEqual(dense.diagnostic_path_error(native,other),inverse_reference_error(native,other))


def mesh(frame='kimera_world'):
    return WireMesh(ns='0', header=S(seq=0, frame_id=frame, stamp=Stamp(123)),
        vertices=[S(x=0., y=0., z=0.), S(x=1., y=0., z=0.), S(x=0., y=1., z=0.)],
        triangles=[S(vertex_indices=[0, 1, 2])], vertex_indices=[0, 1, 2],
        vertex_stamps=[Stamp(123)]*3,
        vertex_colors=[S(r=.2, g=.3, b=.4, a=1.)]*3)


class WireMesh(FakeMessage):
    """Fake geometry with the ROS top Header.seq four-byte wire contract."""
    def serialize(self, stream):
        normalized = dense.snapshot_message_header(self); normalized.header.seq = 0
        stream.write(struct.pack('<I', self.header.seq)); FakeMessage.serialize(normalized, stream)


class SerializedBytes:
    def deserialize(self, payload):
        self._buff = payload; return self
    def serialize(self, stream):
        stream.write(self._buff)


class FakeProcessProbe:
    def __init__(self, operation):
        self.value = dict(identity=dict(node='/sim_0/native_pgmo',uri='http://native:1234/',pid=123),
            error=None,checked_monotonic=time.monotonic())
    def snapshot(self):
        return copy.deepcopy(self.value)
    def close(self): pass


def completed(ledger, active=(0, 1), cross=True):
    request = command(0); request.active_robots = list(active)
    ledger.command(0, request, 1.)
    ledger.command(0, command(1), 2.)  # one observed update, not all 25
    ledger.weights(0, S(src_robot_ids=[0], src_pose_ids=[0], dst_robot_ids=[1 if cross else 0],
        dst_pose_ids=[1], weights=[1.], fixed_weights=[True]), 2.5)
    ledger.command(0, command(2), 3.)
    ledger.native_path(S(poses=[S(pose=pose()), S(pose=pose(1.))]), 4.)


class OnlineDense(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(); self.addCleanup(self.folder.cleanup)
        self.publishers, self.subscribers = {}, {}

        def publisher(topic, kind, **options):
            value = S(messages=[], options=options, get_num_connections=lambda: 1)
            value.publish = value.messages.append; self.publishers[topic] = value
            return value

        def subscriber(topic, kind, callback, **options):
            self.subscribers[topic] = callback
            return S()

        lcd = '/sim_0/distributed_loop_closure/distributed_loop_closure_node'
        self.ros = S(Publisher=publisher, Subscriber=subscriber, is_shutdown=lambda: False,
            AnyMsg=SerializedBytes,
            on_shutdown=lambda callback: None,
            Duration=lambda duration: duration, Timer=lambda *args: S(),
            Time=S(now=lambda: Stamp(1000)), get_param=lambda *args: False,
            get_master=lambda: S(getSystemState=lambda: (1, '', [[], [
                ('/sim_0/kimera_camera_adapter/pose_graph_incremental', [lcd]),
                ('/sim_0/dpgo_ros_node/path', [lcd])], []])))
        types = {name: S for name in ('PoseGraph', 'KimeraPgmoMesh', 'String', 'Odometry', 'Imu',
            'CameraInfo', 'Image', 'Command', 'RelativeMeasurementWeights', 'Header', 'PoseStamped', 'UInt8MultiArray')}
        types.update(RosPath=lambda **kw: FakeMessage(poses=[], **kw),
            PointField=S(FLOAT32=7, UINT32=6), PointCloud2=S)
        self.patch = patch.multiple(dense, rospy=self.ros,
            tf2_ros=S(TransformBroadcaster=lambda: S()), NativeProcessProbe=FakeProcessProbe, create=True, **types)
        self.patch.start(); self.addCleanup(self.patch.stop)
        self.args = S(robot_id=0, robot_name='sim_0', run_id='online-test', cutoff=1e-7,
            mode='joint', execution_mode='online', no_persist=True, global_frame='kimera_world',
            shared_gauge_id='test-gauge', external_sensor_stream=True, max_gyro_gap=.05,
            gyro_sample_convention='left_endpoint_interval_equivalent', sensor_rotation_interpolation='gyro_endpoint',
            max_odom_gap=.6, weight_updates=25, output_directory=Path(self.folder.name)/'bridge',
            recorded_native_bag=None, recorded_native_bag_sha256=None, replay_caller='/native_dense_replay',
            require_cross=True, mesh_phase_timeout=30., remote_bridge_caller=None)
        self.bridge = dense.NativeDenseBridge(self.args)
        self.addCleanup(self.bridge.events.close)
        self.bridge.make_cloud = lambda xyz, color, stamp, frame: S(xyz=xyz.copy(), frame=frame)
        self.bridge.camera_graph_contract = {'native_key_zero_stamp_ns': 100}
        self.bridge.keys.ingest(S(nodes=[node(0), node(1)], edges=[]))
        self.bridge.gauge_checked = self.bridge.native_body_metadata_ack = True
        self.bridge.graph_sent = 1
        self.bridge.on_pgmo_graph(S(nodes=[node(0), node(1)], edges=[]))
        self.bridge.on_full_mesh(mesh('sim_0/vio_odom'))

    def submit(self, offset=0.):
        optimized = [node(0), node(1)]
        optimized[1].pose.position.x += offset
        for n in optimized: dense.set_pose(n.pose, dense.pose_matrix(n.pose) @ dense.SIM_T_BODY_CAMERA)
        self.bridge.on_optimized_graph(FakeMessage(nodes=optimized), received_wall=4.1)
        self.bridge.approve_path()

    def messages(self, suffix): return self.publishers['/sim_0/native_dense/'+suffix].messages

    def native_path_response(self):
        value = copy.deepcopy(self.bridge.online_submission['path'])
        # Match the actual pinned native publisher: only the aggregate stamp
        # is populated, individual PoseStamped keys/stamps are absent.
        value.header.stamp = Stamp(101)
        for item in value.poses: item.header.seq = 0; item.header.stamp = Stamp(0)
        return value

    def test_incomplete_observer_update_count_does_not_block_online_completion(self):
        ledger = dense.RoundLedger(25); completed(ledger)
        self.assertIsNone(ledger.proof(5., True))
        proof = ledger.online_proof(5., True)
        self.assertEqual(proof['accepted_cross_constraints'], 1)
        self.assertEqual(proof['observed_weight_update_count'], 1)
        self.assertFalse(proof['diagnostic_complete_weight_update_count'])
        ledger.command(0, command(0), 6.)
        ledger.command(0, command(3), 7.)
        self.assertIsNone(ledger.online_proof(8., True))

    def test_new_round_weights_cannot_relabel_the_previous_completed_solution(self):
        ledger = dense.RoundLedger(25); completed(ledger)
        accepted = ledger.online_proof(5., True)
        ledger.command(0, command(0), 6.)
        ledger.weights(0, S(src_robot_ids=[0], src_pose_ids=[0], dst_robot_ids=[1],
            dst_pose_ids=[1], weights=[0.], fixed_weights=[True]), 7.)
        ledger.native_path(S(poses=[S(pose=pose()), S(pose=pose(1.))]), 8.)
        retained = ledger.online_proof(9., True)
        self.assertEqual(retained['weights']['sha256'], accepted['weights']['sha256'])
        self.assertEqual(retained['round']['serial'], accepted['round']['serial'])

    def test_native_unanchored_mesh_is_published_without_claiming_shared_gauge(self):
        value = mesh(); before = dense.message_bytes(value)
        self.bridge.on_optimized_mesh(value)
        self.assertEqual(len(self.messages('native_optimized_mesh')), 1)
        self.assertEqual(self.messages('accepted_joint_mesh'), [])
        self.assertEqual(dense.message_bytes(value), before)
        self.assertEqual(self.bridge.online_latest_mesh_gauge, 'local_unanchored')

    def test_online_mesh_serializes_once_and_reuses_unchanged_bytes_without_legacy_clouds(self):
        value = mesh(); value.header.seq = 73
        self.bridge.on_optimized_mesh(value); completed(self.bridge.rounds); self.submit()
        self.bridge.on_pgmo_path(self.native_path_response())
        before = dense.message_bytes(value)
        with patch.object(dense, 'message_bytes', wraps=dense.message_bytes) as serialize, \
                patch.object(self.bridge, 'make_cloud', side_effect=AssertionError('legacy PointCloud produced online')):
            self.bridge.on_optimized_mesh(value)
        self.assertEqual(serialize.call_count, 1)
        native = self.messages('native_optimized_mesh')[-1]; accepted = self.messages('accepted_joint_mesh')[-1]
        self.assertIs(native._buff, accepted._buff)
        self.assertEqual(native._buff, before)
        self.assertEqual(dense.message_bytes(value), before)
        self.assertEqual(self.bridge.accepted_joint_mesh_receipt['canonical_native_mesh_sha256'], dense.canonical_mesh_digest(value))
        for suffix in ('display_vertices','accepted_joint_vertices','raw_local_vertices'):
            self.assertNotIn('/sim_0/native_dense/'+suffix, self.publishers)
        for suffix in ('validate','serialize','canonical_hash','proof_hashes','publish_native','publish_accepted'):
            self.assertIn('online_mesh:'+suffix, self.bridge.timings)

    def test_online_full_mesh_reuses_cached_native_bytes_for_every_pgmo_forward(self):
        value = mesh('sim_0/vio_odom'); value.header.seq = 91
        self.bridge.pgmo_initialized = True
        before = dense.message_bytes(value)
        with patch.object(dense, 'message_bytes', wraps=dense.message_bytes) as serialize, \
                patch.object(self.bridge, 'make_cloud', side_effect=AssertionError('legacy PointCloud produced online')):
            self.bridge.on_full_mesh(value); self.bridge.tick(None)
            self.bridge.remesh_after = 0.; self.bridge.tick(None)
        self.assertEqual(serialize.call_count, 1)
        forwarded = self.messages('full_mesh')
        self.assertEqual(len(forwarded), 2)
        self.assertIs(forwarded[0]._buff, forwarded[1]._buff)
        self.assertEqual(forwarded[0]._buff, before)
        self.assertEqual(dense.message_bytes(value), before)

    def test_explicit_online_legacy_flag_and_diagnostic_keep_vertex_displays(self):
        self.args.online_legacy_vertices = True
        online = dense.NativeDenseBridge(self.args); self.addCleanup(online.events.close)
        online.make_cloud = self.bridge.make_cloud
        online.on_full_mesh(mesh('sim_0/vio_odom')); online.on_optimized_mesh(mesh())
        for suffix in ('display_vertices','accepted_joint_vertices','raw_local_vertices'):
            self.assertIn('/sim_0/native_dense/'+suffix, self.publishers)
        self.assertEqual(len(self.messages('display_vertices')), 1)
        self.assertEqual(len(self.messages('raw_local_vertices')), 1)
        self.args.execution_mode = 'diagnostic'; self.args.no_persist = False; self.args.online_legacy_vertices = False
        self.args.mode = 'local'
        diagnostic = dense.NativeDenseBridge(self.args); self.addCleanup(diagnostic.events.close)
        diagnostic.make_cloud = self.bridge.make_cloud
        self.assertTrue(diagnostic.legacy_vertices_enabled())
        diagnostic.on_full_mesh(mesh('sim_0/vio_odom')); diagnostic.on_optimized_mesh(mesh('sim_0/vio_odom'))
        self.assertEqual(len(self.messages('display_vertices')), 1)
        self.assertEqual(len(self.messages('raw_local_vertices')), 1)

    def test_zero_deformation_is_valid_after_native_completed_path_echo(self):
        value = mesh(); self.bridge.on_optimized_mesh(value)
        completed(self.bridge.rounds); self.submit()
        self.assertIsNone(self.bridge.transaction)
        path = self.bridge.online_submission['path']
        np.testing.assert_allclose(dense.pose_matrix(path.poses[1].pose), dense.pose_matrix(pose(1.)), atol=1e-12)
        self.assertEqual([p.header.seq for p in path.poses], [0, 1])
        self.assertEqual([p.header.stamp.to_nsec() for p in path.poses], [100, 101])
        self.bridge.on_optimized_mesh(value)  # no echo: still not accepted
        self.assertEqual(self.messages('accepted_joint_mesh'), [])
        self.bridge.on_pgmo_path(self.native_path_response())
        self.bridge.on_optimized_mesh(value)  # exact same mesh as before
        self.assertEqual(len(self.messages('accepted_joint_mesh')), 1)
        receipt = self.bridge.accepted_joint_mesh_receipt
        self.assertEqual(receipt['canonical_native_mesh_sha256'], dense.canonical_mesh_digest(value))
        self.assertTrue(receipt['native_path_echo_verified'])
        self.assertTrue(receipt['shared_gauge_ready'])
        self.assertFalse(receipt['exact_optimizer_mesh_generation_proven'])
        self.assertFalse(receipt['exact_shared_optimizer_round_id_available'])
        self.assertFalse(receipt['fixed_mesh_deformation_diagnostic_performed'])
        self.assertFalse(receipt['native_path_exact_pose_echo'])
        self.bridge.tick(None)
        status = json.loads(self.messages('status')[-1].data)
        self.assertFalse(status['persistence_enabled']); self.assertFalse(status['pgmo_inputs_frozen'])
        self.assertIsNone(status['native_deformation_eligible'])
        self.assertEqual(sorted(p.name for p in self.args.output_directory.iterdir()), ['bridge_events.jsonl'])

    def test_joint_local_only_completed_solution_is_not_submitted_or_accepted(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds, active=(0,), cross=False)
        self.submit()
        self.assertEqual(len(self.messages('approved_body_path')), 0)
        self.assertIsNone(self.bridge.online_submission)
        self.bridge.on_optimized_mesh(mesh())
        self.assertEqual(self.messages('accepted_joint_mesh'), [])

    def establish_shared_gauge(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds); self.submit()
        self.bridge.on_pgmo_path(self.native_path_response()); self.bridge.on_optimized_mesh(mesh())
        return self.bridge.online_verified_submission

    def test_new_proven_shared_submission_preserves_last_verified_gauge_without_claiming_new_echo(self):
        verified = self.establish_shared_gauge()
        old_hash = verified['submission']['metadata']['path_sha256']
        old_raw = dense.message_bytes(self.messages('verified_body_path')[-1])
        old_round = verified['submission']['metadata']['proof']['round']['serial']
        completed(self.bridge.rounds); self.submit(.25)
        pending_hash = self.bridge.online_submission['metadata']['path_sha256']
        self.assertNotEqual(pending_hash,old_hash); self.assertIsNone(self.bridge.online_path_echo)
        self.assertIs(self.bridge.online_verified_submission,verified)
        value = mesh(); value.vertices[0].z = .2
        self.bridge.on_optimized_mesh(value)
        receipt = self.bridge.accepted_joint_mesh_receipt
        self.assertEqual(len(self.messages('accepted_joint_mesh')),2)
        self.assertEqual(receipt['submitted_body_path_sha256'],old_hash)
        self.assertEqual(receipt['last_verified_body_path_sha256'],old_hash)
        self.assertEqual(receipt['completed_round_proof']['round']['serial'],old_round)
        self.assertEqual(receipt['pending_submitted_body_path_sha256'],pending_hash)
        self.assertTrue(receipt['newer_path_pending']); self.assertFalse(receipt['native_path_echo_covers_latest_submission'])
        self.assertFalse(receipt['current_mesh_path_generation_verified'])
        self.assertFalse(receipt['exact_optimizer_mesh_generation_proven'])
        self.assertEqual(dense.message_bytes(self.messages('verified_body_path')[-1]),old_raw)
        self.bridge.on_pgmo_path(self.native_path_response()); self.bridge.on_optimized_mesh(value)
        updated = self.bridge.accepted_joint_mesh_receipt
        self.assertEqual(len(self.messages('verified_body_path')),2)
        self.assertEqual(updated['submitted_body_path_sha256'],pending_hash)
        self.assertFalse(updated['newer_path_pending']); self.assertTrue(updated['native_path_echo_covers_latest_submission'])

    def test_first_shared_path_still_requires_its_native_observation(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds); self.submit()
        self.bridge.on_optimized_mesh(mesh())
        self.assertEqual(self.messages('verified_body_path'),[])
        self.assertEqual(self.messages('accepted_joint_mesh'),[])
        self.assertIsNone(self.bridge.online_verified_submission)

    def test_unshared_new_path_revokes_continuity_and_never_overwrites_native_priors(self):
        self.establish_shared_gauge(); count=len(self.messages('approved_body_path'))
        completed(self.bridge.rounds,active=(0,),cross=False); self.submit(.2)
        self.assertEqual(len(self.messages('approved_body_path')),count)
        self.assertIsNone(self.bridge.online_verified_submission)
        self.assertIsNone(self.bridge.accepted_joint_mesh_receipt)
        self.bridge.on_optimized_mesh(mesh())
        self.assertEqual(len(self.messages('accepted_joint_mesh')),1)

    def test_shared_cluster_without_accepted_cross_weight_cannot_replace_verified_priors(self):
        self.establish_shared_gauge(); count=len(self.messages('approved_body_path'))
        completed(self.bridge.rounds,cross=False); self.submit(.2)
        self.assertEqual(len(self.messages('approved_body_path')),count)
        self.assertIsNone(self.bridge.online_verified_submission)

    def test_peer_boot_change_cannot_inherit_verified_gauge(self):
        self.bridge.peer_controls=S(lock=threading.Lock(),status=dict(local_boot_id='local-A',peer_boot_id='peer-A'))
        self.establish_shared_gauge(); self.bridge.peer_controls.status['peer_boot_id']='peer-B'
        with self.assertRaisesRegex(ValueError,'epoch changed'):self.bridge.on_optimized_mesh(mesh())
        self.assertIsNone(self.bridge.online_verified_submission)
        self.assertIsNone(self.bridge.accepted_joint_mesh_receipt)

    def assert_process_change_rejected(self,field,value):
        self.establish_shared_gauge();self.bridge.pgmo_process_probe.value['identity'][field]=value
        with self.assertRaisesRegex(ValueError,'process changed'):self.bridge.on_optimized_mesh(mesh())
        self.assertIsNone(self.bridge.online_verified_submission)
        self.assertIsNone(self.bridge.accepted_joint_mesh_receipt)

    def test_native_pgmo_pid_change_requires_fresh_run(self):self.assert_process_change_rejected('pid',124)

    def test_native_pgmo_uri_change_requires_fresh_run(self):self.assert_process_change_rejected('uri','http://native:5678/')

    def test_identity_temporarily_unavailable_keeps_native_output_but_suspends_acceptance(self):
        verified=self.establish_shared_gauge();probe=self.bridge.pgmo_process_probe
        old=copy.deepcopy(probe.value);probe.value.update(identity=None,error='timeout')
        native_count=len(self.messages('native_optimized_mesh'));self.bridge.on_optimized_mesh(mesh())
        self.assertEqual(len(self.messages('native_optimized_mesh')),native_count+1)
        self.assertEqual(len(self.messages('accepted_joint_mesh')),1)
        self.assertIs(self.bridge.online_verified_submission,verified)
        probe.value=old;self.bridge.on_optimized_mesh(mesh())
        self.assertEqual(len(self.messages('accepted_joint_mesh')),2)

    def test_native_graph_prefix_regression_reset_or_gap_invalidates_shared_gauge(self):
        self.establish_shared_gauge()
        for nodes in ([node(0)],[],[node(0),node(2)],[node(0),node(0)]):
            with self.subTest(keys=[n.key for n in nodes]):
                with self.assertRaisesRegex(ValueError,'prefix regressed/reset'):
                    self.bridge.on_pgmo_graph(S(nodes=nodes,edges=[]))
        self.assertIsNone(self.bridge.online_verified_submission)
        self.assertIsNone(self.bridge.accepted_joint_mesh_receipt)

    def test_soft_lmo_residual_is_observed_without_becoming_a_pose_copy_gate(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds); self.submit()
        value = self.native_path_response(); value.poses[1].pose.position.x += .015
        self.bridge.on_pgmo_path(value); self.bridge.on_optimized_mesh(mesh())
        self.assertEqual(len(self.messages('accepted_joint_mesh')), 1)
        self.assertAlmostEqual(self.bridge.online_path_echo['max_translation_m'], .015)

    def test_native_path_waits_for_verified_body_key_metadata_and_rejects_bad_frame(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds); self.submit()
        self.bridge.pgmo_body_key_stamps.clear()
        value = self.native_path_response(); self.bridge.on_pgmo_path(value)
        self.assertIsNone(self.bridge.online_path_echo)
        # Cross-topic path/metadata ordering is asynchronous; the sole path
        # observation must survive until its original-key metadata arrives.
        self.bridge.on_pgmo_graph(S(nodes=[node(0), node(1)], edges=[]))
        self.assertIsNotNone(self.bridge.online_path_echo)
        value.header.frame_id = 'fake_global_frame'
        with self.assertRaises(ValueError): self.bridge.on_pgmo_path(value)
        with self.assertRaises(ValueError): self.bridge.on_pgmo_graph(S(nodes=[node(1, 999)], edges=[]))

    def test_confirmed_cached_native_path_avoids_matrix_work_but_still_checks_finite_poses(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds); self.submit()
        value = self.native_path_response(); self.bridge.on_pgmo_path(value)
        with patch.object(dense, 'path_error', side_effect=AssertionError('recomputed old full-path residual')):
            self.bridge.on_pgmo_path(value)
            invalid = copy.deepcopy(value); invalid.poses[1].pose.position.x = float('nan')
            with self.assertRaises(ValueError): self.bridge.on_pgmo_path(invalid)
        self.assertEqual(self.bridge.counts['online_path_prior_metadata_reused'],1)

    def test_online_does_not_freeze_graphs_even_if_diagnostic_transaction_is_present(self):
        self.bridge.transaction = dict(phase='await_baseline')
        graph = FakeMessage(nodes=[node(0)], edges=[])
        self.bridge.graph_queue.append(graph)
        self.bridge.mesh_graph_queue.append(graph)
        self.bridge.publish_graphs()
        self.assertEqual(len(self.messages('pose_graph_incremental')), 1)
        self.assertEqual(len(self.messages('mesh_graph_incremental')), 1)
        self.bridge.pgmo_initialized = True; completed(self.bridge.rounds)
        self.submit()
        self.assertEqual(len(self.messages('approved_body_path')), 1)
        self.assertEqual(self.bridge.counts['optimized_graphs_retained_during_mesh_transaction'], 1)

    def test_diagnostic_transaction_retains_the_latest_optimization_for_after_unfreeze(self):
        self.args.execution_mode = 'diagnostic'; self.args.no_persist = False
        self.bridge.transaction = dict(phase='await_baseline')
        completed(self.bridge.rounds)
        proof = self.bridge.rounds.online_proof(5., True)
        self.bridge.rounds.proof = lambda *args: proof
        first = FakeMessage(nodes=[node(0), node(1)])
        second = copy.deepcopy(first); second.nodes[1].pose.position.x = 1.2
        self.bridge.on_optimized_graph(first, received_wall=4.1)
        self.bridge.on_optimized_graph(second, received_wall=4.2)
        self.assertIsNone(self.bridge.pending_optimized)
        self.assertEqual(len(self.bridge.optimized_candidates), 2)
        self.bridge.transaction = None
        self.bridge.advance_optimized_candidates()
        self.assertEqual(self.bridge.pending_optimized[2], dense.digest(second))
        self.assertEqual(self.bridge.pending_optimized[0][1].pose.position.x, 1.2)

    def test_native_key_and_source_checks_still_reject_foreign_or_nonexpanded_paths(self):
        completed(self.bridge.rounds)
        with self.assertRaises(ValueError):
            self.bridge.on_optimized_graph(FakeMessage(nodes=[node(0), node(2)]))
        with self.assertRaises(ValueError):
            self.bridge.on_optimized_graph(FakeMessage(nodes=[node(0), node(1, 999)]))
        received = FakeMessage(_connection_header={'callerid': '/pretend_vio'}, nodes=[node(0)],
                               header=S(frame_id='sim_0/map'), edges=[])
        errors = []; self.bridge.fail = lambda error: errors.append(str(error))
        self.subscribers['/sim_0/kimera_vio_ros/pose_graph_incremental'](received)
        self.assertIn('Rejected caller', errors[0])

    def test_actual_remote_subscription_requires_proxy_and_matching_late_receipt(self):
        self.args.remote_bridge_caller = '/sim_0_peer_bridge'
        self.args.remote_control_transport = 'receipt'  # Explicit compatibility diagnostic.
        bridge = dense.NativeDenseBridge(self.args); self.addCleanup(bridge.events.close)
        errors = []; bridge.fail = lambda error: errors.append(str(error))
        topic = '/sim_1/dpgo_ros_node/command'
        request = FakeMessage(**vars(command(0, owner=1)), _connection_header={'callerid': '/sim_0_peer_bridge'})
        self.subscribers[topic](request)
        bridge.drain_native_inputs(); self.assertIsNone(bridge.rounds.active)
        status = dict(run_id='online-test', cutoff_ns=100, local_robot_id=0, peer_robot_id=1,
            local_boot_id='local-boot', peer_boot_id='peer-boot', session_ready=True)
        self.subscribers['/sim_0/peer_bridge/status'](S(data=json.dumps(status),
            _connection_header={'callerid': '/sim_0_peer_bridge'}))
        receipt = dict(status, topic=topic, sha256=bridge.remote_control_digest(request), verified=True,
            source_robot_id=1, source_boot_id='peer-boot', source_caller='/sim_1/dpgo_ros_node/agent',
            proxy_caller='/sim_0_peer_bridge')
        self.subscribers['/sim_0/peer_bridge/ingress_receipt'](S(data=json.dumps(receipt),
            _connection_header={'callerid': '/sim_0_peer_bridge'}))
        bridge.drain_native_inputs(); self.assertEqual(bridge.rounds.active['owner'], 1)
        self.assertEqual(errors, [])
        request._connection_header['callerid'] = '/rogue_proxy'
        self.subscribers[topic](request)
        self.assertIn('Rejected caller', errors[-1])

    def test_atomic_control_burst_bypasses_slow_slam_lock_without_receipt_topics(self):
        self.args.remote_bridge_caller = '/sim_0_peer_bridge'
        self.subscribers.clear()
        with patch.object(dense, 'Command', SerializedCommand): bridge = dense.NativeDenseBridge(self.args)
        self.addCleanup(bridge.events.close)
        from peer_wire import encode_frame
        self.assertNotIn('/sim_1/dpgo_ros_node/command', self.subscribers)
        self.assertNotIn('/sim_0/peer_bridge/ingress_receipt', self.subscribers)
        self.assertEqual(bridge.a.remote_control_transport, 'atomic')
        errors = []; bridge.fail = lambda error: errors.append(str(error))
        proxy = {'callerid': '/sim_0_peer_bridge'}
        status = dict(run_id='online-test', cutoff_ns=100, local_robot_id=0, peer_robot_id=1,
            local_boot_id='local-boot', peer_boot_id='peer-boot', session_ready=True)
        done = threading.Event()
        def ingress():
            self.subscribers['/sim_0/peer_bridge/status'](S(data=json.dumps(status), _connection_header=proxy))
            for sequence in range(1, 1025):
                value = SerializedCommand(**vars(command(0 if sequence == 1 else 2 if sequence == 1024 else 1, owner=1)))
                payload = dense.message_bytes(value); sha = hashlib.sha256(payload).hexdigest()
                receipt = dict(status, topic='/sim_1/dpgo_ros_node/command', kind='native_control',
                    sha256=sha, wire_sha256=sha, verified=True, proxy_caller='/sim_0_peer_bridge',
                    source_robot_id=1, source_boot_id='peer-boot', source_caller='/sim_1/dpgo_ros_node/agent',
                    has_header=False, type=value._type, md5=value._md5sum, control_sequence=sequence)
                self.subscribers['/sim_0/peer_bridge/native_control'](
                    S(data=encode_frame(receipt, payload), _connection_header=proxy))
            done.set()
        # This lock models an arbitrarily slow on_full_mesh/optimized-graph
        # callback. Both authentication streams must still finish independently.
        with bridge.lock:
            worker = threading.Thread(target=ingress); worker.start()
            self.assertTrue(done.wait(5.), 'Provenance/control ingress must not wait for the SLAM state lock')
            self.assertEqual(errors, [])
            self.assertEqual(bridge.peer_controls.atomic_delivered, 1024)
            self.assertEqual(len(bridge.peer_controls.atomic_pending), 0)
            self.assertEqual(len(bridge.native_inputs), 1024)
            self.assertIsNone(bridge.rounds.online_completed)
            bridge.drain_native_inputs()
        worker.join(1.)
        self.assertIsNotNone(bridge.rounds.online_completed)
        self.assertEqual(len(bridge.rounds.online_completed['weight_update_walls']), 1022)
        self.assertEqual(bridge.peer_controls.atomic_sequence, 1024)

    def test_only_header_sequence_is_ignored_for_online_expanded_graph_deduplication(self):
        self.bridge.on_optimized_mesh(mesh()); completed(self.bridge.rounds)
        graph = FakeMessage(header=S(seq=1, stamp=Stamp(0), frame_id='kimera_world'), nodes=[node(0), node(1)])
        self.bridge.on_optimized_graph(graph, received_wall=4.1)
        self.bridge.approve_path()
        before = self.bridge.counts['submitted_native_paths']
        repeated = copy.deepcopy(graph); repeated.header.seq = 999
        with patch.object(self.bridge.keys, 'optimized_structure', side_effect=AssertionError('duplicate traversed all native keys')):
            self.bridge.on_optimized_graph(repeated, received_wall=4.2)
        self.assertEqual(self.bridge.counts['optimized_graph_timer_duplicates_suppressed'], 1)
        self.assertEqual(self.bridge.counts['submitted_native_paths'], before)
        repeated.nodes[1].pose.position.x += .1
        self.bridge.on_optimized_graph(repeated, received_wall=4.3)
        self.assertIsNotNone(self.bridge.pending_optimized)

    def test_online_original_graph_burst_is_batched_without_replaying_or_dropping_keys(self):
        bridge = self.bridge; bridge.keys = dense.KeyframeLedger(0, 100)
        bridge.graph_sent = -1; bridge.graph_queue.clear()
        for key in range(80):
            edges = [] if key == 0 else [Edge(robot_from=0, robot_to=0, key_from=key-1, key_to=key,
                ODOM=0, type=0, header=S(stamp=Stamp(100+key)), pose=pose(1.), covariance=[0.]*36)]
            bridge.on_graph(FakeMessage(header=S(frame_id='sim_0/map'), nodes=[node(key)], edges=edges))
        bridge.publish_graphs()
        self.assertEqual(len(self.messages('pose_graph_incremental')), 32)
        self.assertEqual(len(bridge.graph_queue), 48)
        bridge.publish_graphs(); bridge.publish_graphs()
        forwarded = self.messages('pose_graph_incremental')
        self.assertEqual([m.nodes[0].key for m in forwarded], list(range(80)))
        self.assertEqual([(m.edges[0].key_from,m.edges[0].key_to) for m in forwarded[1:]],
                         [(i,i+1) for i in range(79)])
        self.assertIsNone(self.publishers['/sim_0/native_dense/pose_graph_incremental'].options['queue_size'])
        bridge.on_pgmo_graph(S(nodes=[node(key) for key in range(80)], edges=[]))
        self.assertEqual(bridge.pgmo_verified_through, 79)
        self.assertEqual(bridge.graph_sent-bridge.pgmo_verified_through, 0)


class RemoteControlAuthentication(unittest.TestCase):
    def setUp(self):
        self.gate = dense.PeerControlIngress(0, 'test', 100, '/sim_0_peer_bridge')
        self.topic = '/sim_1/dpgo_ros_node/command'; self.sha = 'a'*64
        self.status = dict(run_id='test', cutoff_ns=100, local_robot_id=0, peer_robot_id=1,
            local_boot_id='hub-boot', peer_boot_id='peer-boot', session_ready=True)
        self.receipt = dict(run_id='test', cutoff_ns=100, local_robot_id=0, peer_robot_id=1,
            topic=self.topic, sha256=self.sha, verified=True, proxy_caller='/sim_0_peer_bridge',
            source_robot_id=1, source_boot_id='peer-boot', source_caller='/sim_1/dpgo_ros_node/agent')

    def test_unique_terminate_waits_for_late_receipt_and_status(self):
        terminate = command(2, owner=1)
        self.gate.enqueue(self.topic, self.sha, 'callback', terminate, 2., 10., True)
        self.assertEqual(self.gate.drain(11.), [])
        self.gate.receipt(self.receipt); self.assertEqual(self.gate.drain(12.), [])
        self.gate.set_status(self.status); items = self.gate.drain(13.)
        self.assertEqual(len(items), 1); self.assertIs(items[0][1], terminate)
        self.assertEqual(items[0][2], 2.)  # preserve actual source callback arrival

    def test_wrong_original_source_run_or_boot_cannot_authorize_native_control(self):
        for key, value in [('source_caller', '/sim_1/fake'), ('run_id', 'another'), ('verified', False)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.gate.receipt(dict(self.receipt, **{key: value}))
        self.gate.set_status(self.status)
        self.gate.receipt(dict(self.receipt, source_boot_id='old-boot'))
        self.gate.enqueue(self.topic, self.sha, None, command(2, owner=1), 2., 10., True)
        self.assertEqual(self.gate.drain(12.), [])
        self.assertEqual(self.gate.drain(16.), [])
        self.assertTrue(self.gate.overdue)
        # Preserve the unique termination while local mapping keeps running.
        self.gate.receipt(self.receipt)
        self.assertEqual(len(self.gate.drain(20.)), 1)
        self.assertFalse(self.gate.overdue)

    def test_bounded_pending_cache_and_native_fifo_are_enforced(self):
        gate = dense.PeerControlIngress(0, 'test', 100, '/sim_0_peer_bridge', maximum_pending=1)
        gate.enqueue(self.topic, self.sha, None, command(2), 1., 10., True)
        with self.assertRaises(RuntimeError):
            gate.enqueue(self.topic, self.sha, None, command(2), 2., 11., True)

    def atomic_fixture(self, sequence=1):
        message = SerializedCommand(**vars(command(0, owner=1)))
        payload = dense.message_bytes(message); sha = hashlib.sha256(payload).hexdigest()
        receipt = dict(self.receipt, kind='native_control', has_header=False,
            sha256=sha, wire_sha256=sha, local_boot_id='hub-boot', control_sequence=sequence,
            type=message._type, md5=message._md5sum)
        return receipt, payload, message

    def test_atomic_envelope_integrity_original_source_type_boot_and_sequence_remain_strict(self):
        self.gate.set_status(self.status)
        receipt, payload, message = self.atomic_fixture()
        for fields in ({'sha256':'b'*64}, {'source_caller':'/sim_1/fake'}, {'md5':'0'*32},
                       {'source_boot_id':'old-boot'}, {'local_boot_id':'old-local-boot'}, {'control_sequence':2}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.gate.atomic(dict(receipt,**fields),payload,message,None,1.,1.)
        with self.assertRaises(ValueError): self.gate.atomic(receipt,payload+b'x',message,None,1.,1.)
        events = self.gate.atomic(receipt,payload,message,None,1.,1.)
        self.assertEqual(len(events),1)
        self.assertEqual(self.gate.atomic(receipt,payload,message,None,2.,2.),[])
        self.assertEqual(self.gate.atomic_duplicates,1)

    def test_atomic_message_survives_initial_status_without_peer_boot_and_late_identity(self):
        self.gate.set_status(dict(self.status,peer_boot_id=None,session_ready=False))
        receipt, payload, message = self.atomic_fixture()
        self.assertEqual(self.gate.atomic(receipt,payload,message,None,1.,1.),[])
        self.gate.set_status(self.status)
        self.assertEqual(len(self.gate.drain(30.)),1)
        self.assertEqual(self.gate.diagnostic(31.)['atomic_reason'],'all_received_controls_authenticated')


class ProcessIdentityProbe(unittest.TestCase):
    def test_slow_xmlrpc_observation_never_holds_the_snapshot_or_slam_lock(self):
        began, release = threading.Event(),threading.Event()
        def operation():
            began.set();release.wait(1.);return dict(node='/native',uri='http://native:1/',pid=5)
        probe=dense.NativeProcessProbe(operation,period=.01);self.addCleanup(probe.close);self.addCleanup(release.set)
        self.assertTrue(began.wait(.5))
        start=time.monotonic();snapshot=probe.snapshot();self.assertLess(time.monotonic()-start,.05)
        self.assertIsNone(snapshot['identity'])
        release.set();deadline=time.monotonic()+.5
        while probe.snapshot()['identity'] is None and time.monotonic()<deadline:time.sleep(.005)
        self.assertEqual(probe.snapshot()['identity']['pid'],5)

    def test_xmlrpc_timeout_and_invalid_response_never_turn_into_a_process_identity(self):
        master=S(lookupNode=lambda *a:(1,'','http://native:123/'))
        native=S(getPid=lambda *a:(1,'',234))
        class Proxy:
            def __init__(self,value):self.value=value
            def __enter__(self):return self.value
            def __exit__(self,*a):pass
        with patch.object(dense.xmlrpc.client,'ServerProxy',side_effect=[Proxy(master),Proxy(native)]):
            self.assertEqual(dense.native_node_identity('http://master:1/','/native','/probe'),
                             dict(node='/native',uri='http://native:123/',pid=234))
        for response in ((-1,'missing',0),(1,'',0),(1,'',True)):
            bad=S(getPid=lambda *a,v=response:v)
            with patch.object(dense.xmlrpc.client,'ServerProxy',side_effect=[Proxy(master),Proxy(bad)]):
                with self.assertRaises(ValueError):dense.native_node_identity('http://master:1/','/native','/probe')
        with patch.object(dense.xmlrpc.client,'ServerProxy',side_effect=TimeoutError('bounded RPC')):
            with self.assertRaises(TimeoutError):dense.native_node_identity('http://master:1/','/native','/probe')
        connection=dense.BoundedXmlRpcTransport(.25).make_connection('localhost:11311')
        self.assertEqual(connection.timeout,.25);connection.close()


class NativeRosMeshWire(unittest.TestCase):
    def test_real_noetic_typed_publish_preserves_native_serialized_mesh_and_source(self):
        try:
            import rospy
            from rospy.msg import args_kwds_to_message, serialize_message
            from geometry_msgs.msg import Point
            from kimera_pgmo_msgs.msg import KimeraPgmoMesh, TriangleIndices
            from std_msgs.msg import ColorRGBA
        except ImportError as error:
            self.skipTest('Native Noetic message environment required: '+str(error))
        value = KimeraPgmoMesh(); value.header.seq = 879; value.header.stamp = rospy.Time(123,456)
        value.header.frame_id = 'kimera_world'; value.ns = '1'
        value.vertices = [Point(0.,0.,0.),Point(1.,0.,0.),Point(0.,1.,0.)]
        value.triangles = [TriangleIndices(vertex_indices=[0,1,2])]
        value.vertex_stamps = [rospy.Time(123+i, i+5) for i in range(3)]
        value.vertex_indices = [3,7,11]
        value.vertex_colors = [ColorRGBA(.2,.3,.4,1.),ColorRGBA(.5,.6,.7,.8),ColorRGBA(.9,1.,.1,.2)]
        before = dense.message_bytes(value); normalized = dense.canonical_mesh_digest(value)
        captured = []
        # These are the actual typed Publisher argument conversion and ROS
        # transport serialization functions, without a ROS node or master.
        def publish(message):
            converted = args_kwds_to_message(KimeraPgmoMesh,(message,),{})
            stream = io.BytesIO(); serialize_message(stream,999,converted)
            data = stream.getvalue(); self.assertEqual(struct.unpack_from('<I',data)[0],len(before))
            captured.append(data[4:])
        with patch.object(dense,'rospy',rospy,create=True):
            dense.publish_serialized_message(S(publish=publish),before)
            dense.publish_serialized_message(S(publish=publish),before)
        self.assertEqual(captured,[before,before])
        self.assertEqual(dense.canonical_serialized_mesh_digest(before),normalized)
        decoded = KimeraPgmoMesh().deserialize(captured[-1])
        self.assertEqual(dense.message_bytes(decoded),before)
        self.assertEqual(dense.message_bytes(value),before)
        self.assertEqual(decoded.header.seq,879)
        with self.assertRaises(ValueError): dense.canonical_serialized_mesh_digest(b'\0\0\0')


class PersistenceLaunchContracts(unittest.TestCase):
    def test_no_persist_disables_every_native_automatic_artifact_path(self):
        folder = Path(__file__).parent
        for filename in ('hub_backend.launch', 'local_frontend.launch', 'backend.launch'):
            tree = ET.parse(folder/filename)
            self.assertEqual(tree.find("arg[@name='no_persist']").get('default'), 'false')
            self.assertIsNotNone(tree.find("arg[@name='persistent_outputs']"))
        backend = ET.parse(folder/'hub_backend.launch')
        pgmo = backend.find(".//node[@name='native_pgmo']")
        self.assertEqual(pgmo.find("param[@name='log_output']").get('value'), '$(arg persistent_outputs)')
        self.assertIn("else ''", pgmo.find("param[@name='output_prefix']").get('value'))
        lcd = ET.parse(folder/'backend.launch').find(".//node[@name='distributed_loop_closure_node']")
        self.assertEqual(lcd.find("param[@name='log_output_path']").get('if'), '$(arg persistent_outputs)')
        self.assertEqual(lcd.find("rosparam[@param='log_output_path']").get('command'), 'delete')

    def test_local_sensor_no_persist_keeps_ros_status_without_writing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            args = S(robot_id=0, robot_name='sim_0', run_id='test', cutoff=1e-7,
                max_gyro_gap=.05, max_odom_gap=.6, no_persist=True,
                output_directory=Path(directory)/'must-not-be-created')
            pubs = []
            def publisher(*a, **kw):
                return S(get_num_connections=lambda: 0, publish=pubs.append)
            ros = S(Publisher=publisher, Subscriber=lambda *a, **kw: S(),
                Timer=lambda *a: S(), Duration=lambda d: d, is_shutdown=lambda: False)
            types = {name: S for name in ('PointCloud2', 'String', 'Odometry', 'PoseGraph', 'Imu', 'CameraInfo', 'Image')}
            with patch.multiple(local_sensor, rospy=ros, tf2_ros=S(TransformBroadcaster=lambda: S()), create=True, **types):
                bridge = local_sensor.LocalSensorBridge(args)
                args.max_pending_rgbd=32; args.max_rgbd_age=2.; args.cloud_rate=5.
                bridge.tick(None)
            self.assertFalse(args.output_directory.exists())
            self.assertTrue(pubs)


if __name__ == '__main__': unittest.main()
