#!/usr/bin/env python3
"""Receipt/geometry contracts; these do not replace two-host native ROS tests."""
import copy
import json
from types import SimpleNamespace as S
import unittest

import numpy as np

from native_dense_global_view import COLORS, NativeMeshPair, NotReady, mesh_content, sha256


class Stamp:
    def __init__(self, value): self.value = value
    def to_nsec(self): return self.value


class Mesh(S):
    def serialize(self, stream):
        def simple(value):
            if isinstance(value, Stamp): return value.value
            if isinstance(value, S): return {k: simple(v) for k, v in vars(value).items()}
            if isinstance(value, list): return [simple(v) for v in value]
            return value
        stream.write(json.dumps(simple(self), sort_keys=True, separators=(',', ':')).encode())


def mesh(robot, shift=0.):
    return Mesh(ns=str(robot), header=S(seq=7, frame_id='kimera_world', stamp=Stamp(2000)),
        vertices=[S(x=robot*10.+shift, y=0., z=1.), S(x=robot*10.+1.+shift, y=1., z=1.),
                  S(x=robot*10.+2.+shift, y=0., z=2.)],
        vertex_stamps=[Stamp(1500), Stamp(1600), Stamp(1700)], vertex_indices=[], vertex_colors=[],
        triangles=[S(vertex_indices=[0, 1, 2])])


def status(robot, native_mesh, serial=7, owner=0, run='run'):
    content = mesh_content(native_mesh, 1000, 'kimera_world'); del content['xyz']
    rows = [[0, 3, 1, 5, 1., True]]
    weight_hash = sha256(json.dumps(rows, separators=(',', ':')).encode())
    proof = dict(round=dict(owner=owner, serial=serial, request_wall=10., terminate_wall=13.,
        active_robots=[0, 1], weight_update_walls=[10.1+i*.1 for i in range(25)]),
        weights=dict(owner=owner, received_wall=13.1, rows=rows, sha256=weight_hash), accepted_cross_constraints=1)
    receipt = dict(content, run_id=run, cutoff_ns=1000, robot_id=robot, immutable_until_next_accepted_transaction=True,
        optimizer_owner=owner, completed_round_serial=serial, final_native_weight_batch_sha256=weight_hash,
        completed_round_proof=proof, submitted_body_path_sha256='path',
        native_deformation_proof=dict(native_path_echo_verified=True,
            same_native_input_mesh_topology_and_vertex_stamps=True, historical_vertices_moved=3,
            maximum_vertex_motion_m=.1, submitted_path_sha256='path'))
    return dict(run_id=run, cutoff_ns=1000, robot_id=robot, mode='joint', fatal_error=None,
        native_deformation_eligible=True, accepted_joint_mesh_receipt=receipt)


def add(pair, robot, value=None, metadata=None):
    value = mesh(robot) if value is None else value
    pair.ingest_mesh(robot, value, pair.expected_caller(robot))
    pair.ingest_status(robot, status(robot, value) if metadata is None else metadata, pair.expected_caller(robot))


class PairContracts(unittest.TestCase):
    def setUp(self): self.pair = NativeMeshPair('run', 1000)

    def test_exact_native_XYZ_and_two_source_colors(self):
        add(self.pair, 0); add(self.pair, 1)
        xyz, colors, robots, report = self.pair.evaluate()
        expected = np.array([[p.x, p.y, p.z] for r in (0, 1) for p in mesh(r).vertices], '<f4')
        np.testing.assert_array_equal(xyz, expected)
        np.testing.assert_array_equal(colors, [COLORS[0]]*3+[COLORS[1]]*3)
        np.testing.assert_array_equal(robots, [0]*3+[1]*3)
        self.assertEqual(report['ordered_xyz_float32_le_sha256'], sha256(expected.tobytes()))
        self.assertFalse(report['uses_icp']); self.assertFalse(report['uses_reprojection'])

    def test_status_before_mesh_waits_for_exact_hash(self):
        for robot in (0, 1): self.pair.ingest_status(robot, status(robot, mesh(robot)), self.pair.expected_caller(robot))
        with self.assertRaises(NotReady): self.pair.evaluate()
        self.pair.ingest_mesh(0, mesh(0), self.pair.expected_caller(0))
        with self.assertRaises(NotReady): self.pair.evaluate()
        self.pair.ingest_mesh(1, mesh(1), self.pair.expected_caller(1))
        self.assertTrue(self.pair.evaluate()[3]['fusion_ready'])

    def test_wrong_caller_frame_epoch_and_ownership_are_rejected(self):
        with self.assertRaises(ValueError): self.pair.ingest_mesh(0, mesh(0), '/custom_mapper')
        wrong = mesh(0); wrong.header.frame_id = 'world'
        with self.assertRaises(ValueError): self.pair.ingest_mesh(0, wrong, self.pair.expected_caller(0))
        wrong = mesh(0); wrong.vertex_stamps[0] = Stamp(999)
        with self.assertRaises(ValueError): self.pair.ingest_mesh(0, wrong, self.pair.expected_caller(0))
        with self.assertRaises(ValueError): self.pair.ingest_mesh(0, mesh(1), self.pair.expected_caller(0))
        add(self.pair, 0); add(self.pair, 1)
        self.pair.ingest_status(1, status(1, mesh(1), run='old'), self.pair.expected_caller(1))
        with self.assertRaises(NotReady): self.pair.evaluate()

    def test_latest_generation_does_not_fall_back_to_cached_passing_pair(self):
        add(self.pair, 0); add(self.pair, 1); self.pair.evaluate()
        next_mesh = mesh(0, .25)
        self.pair.ingest_status(0, status(0, next_mesh, serial=8), self.pair.expected_caller(0))
        with self.assertRaises(NotReady): self.pair.evaluate()  # no old mesh fallback
        self.pair.ingest_mesh(0, next_mesh, self.pair.expected_caller(0))
        with self.assertRaises(NotReady): self.pair.evaluate()  # one side still round7

    def test_mixed_owner_weights_or_round_time_cannot_merge(self):
        add(self.pair, 0)
        add(self.pair, 1, metadata=status(1, mesh(1), owner=1))
        with self.assertRaises(NotReady): self.pair.evaluate()
        different = status(1, mesh(1)); different['accepted_joint_mesh_receipt']['completed_round_proof']['round']['request_wall'] -= 2.
        self.pair.ingest_status(1, different, self.pair.expected_caller(1))
        with self.assertRaises(NotReady): self.pair.evaluate()
        different = status(1, mesh(1)); proof = different['accepted_joint_mesh_receipt']['completed_round_proof']
        proof['weights']['rows'][0][4] = .9
        weight_hash = sha256(json.dumps(proof['weights']['rows'], separators=(',', ':')).encode())
        proof['weights']['sha256'] = weight_hash
        different['accepted_joint_mesh_receipt']['final_native_weight_batch_sha256'] = weight_hash
        self.pair.ingest_status(1, different, self.pair.expected_caller(1))
        with self.assertRaises(NotReady): self.pair.evaluate()

    def test_geometry_receipt_hashes_and_actual_cross_weights_are_verified(self):
        add(self.pair, 0); add(self.pair, 1)
        wrong = status(1, mesh(1)); wrong['accepted_joint_mesh_receipt']['vertex_xyz_float32_le_sha256'] = 'wrong'
        self.pair.ingest_status(1, wrong, self.pair.expected_caller(1))
        with self.assertRaises(NotReady): self.pair.evaluate()
        wrong = status(1, mesh(1)); proof = wrong['accepted_joint_mesh_receipt']['completed_round_proof']
        proof['weights']['rows'][0][2] = 0
        weight_hash = sha256(json.dumps(proof['weights']['rows'], separators=(',', ':')).encode())
        proof['weights']['sha256'] = weight_hash
        wrong['accepted_joint_mesh_receipt']['final_native_weight_batch_sha256'] = weight_hash
        self.pair.ingest_status(1, wrong, self.pair.expected_caller(1))
        with self.assertRaises(NotReady): self.pair.evaluate()  # claimed count is insufficient

    def test_latched_heartbeat_and_Header_seq_are_not_new_geometry(self):
        add(self.pair, 0); add(self.pair, 1)
        identity = self.pair.evaluate()[3]['snapshot_identity']
        repeated = mesh(0); repeated.header.seq = 999
        self.assertFalse(self.pair.ingest_mesh(0, repeated, self.pair.expected_caller(0)))
        self.pair.ingest_status(0, status(0, repeated), self.pair.expected_caller(0))
        self.assertEqual(identity, self.pair.evaluate()[3]['snapshot_identity'])
        self.assertEqual(len(self.pair.meshes[0]), 1)

    def test_two_mesh_versions_per_robot_are_bounded(self):
        for index in range(5): self.pair.ingest_mesh(0, mesh(0, index), self.pair.expected_caller(0))
        self.assertEqual(len(self.pair.meshes[0]), 2)

    def test_malformed_latest_status_invalidates_prior_good_status(self):
        add(self.pair, 0); add(self.pair, 1); self.pair.evaluate()
        with self.assertRaises(ValueError): self.pair.ingest_status(0, [], self.pair.expected_caller(0))
        with self.assertRaises(NotReady): self.pair.evaluate()


if __name__ == '__main__': unittest.main(verbosity=2)
