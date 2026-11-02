#!/usr/bin/env python3
"""Offline SE(3), native identity, covariance and ordered bootstrap tests.

Fixtures mirror the installed rosmsg schema; no ROS or ground truth input is
required. Run the adapter in ROS to capture its actual class MD5/schema.
"""
import copy
import math
from types import SimpleNamespace as NS
import unittest

import numpy as np

from native_vio_graph_adapter import (GraphConverter, OrderedOutput,
    SIM_T_BODY_CAMERA, adjoint, pose_matrix, rotate_covariance, set_pose,
    skew, validate_transform)


class Stamp:
    def __init__(self, nanoseconds): self.nanoseconds = int(nanoseconds)
    def to_nsec(self): return self.nanoseconds


def header(stamp, frame='native_map', seq=17):
    return NS(stamp=Stamp(stamp), frame_id=frame, seq=seq)


def pose(transform):
    result = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    set_pose(result, transform); return result


def transform(rotation_vector, translation):
    vector = np.asarray(rotation_vector); theta = float(np.linalg.norm(vector)); cross = skew(vector)
    rotation = np.eye(3)+cross if theta < 1e-12 else np.eye(3)+math.sin(theta)/theta*cross+(1-math.cos(theta))/theta**2*(cross@cross)
    result = np.eye(4); result[:3, :3] = rotation; result[:3, 3] = translation
    return result


def se3_exp(twist):
    translation, rotation = np.asarray(twist[:3]), np.asarray(twist[3:])
    angle = float(np.linalg.norm(rotation)); cross = skew(rotation)
    if angle < 1e-8:
        jacobian = np.eye(3)+.5*cross+(cross@cross)/6
    else:
        jacobian = np.eye(3)+(1-math.cos(angle))/angle**2*cross+(angle-math.sin(angle))/angle**3*(cross@cross)
    return transform(rotation, jacobian@translation)


def small_log(matrix):
    rotation = matrix[:3, :3]
    return np.r_[matrix[:3, 3], .5*np.array([rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0], rotation[1, 0]-rotation[0, 1]])]


def fixture(first=np.eye(4), second=None, robot=1):
    if second is None: second = transform([0., 0., 0.], [1., 0., 0.])
    nodes = [NS(header=header(1000000001), robot_id=robot, key=0, pose=pose(first)),
             NS(header=header(2000000002), robot_id=robot, key=1, pose=pose(second))]
    edge = NS(header=header(2000000002), key_from=0, key_to=1,
        robot_from=robot, robot_to=robot, type=0, pose=pose(np.linalg.inv(first)@second), covariance=[0.]*36)
    return NS(header=header(2000000002), nodes=nodes, edges=[edge])


class AdapterTests(unittest.TestCase):
    def assertMatrix(self, left, right, tolerance=1e-10):
        np.testing.assert_allclose(left, right, atol=tolerance, rtol=0.)

    def test_body_forward_x_is_camera_forward_z(self):
        result = GraphConverter(1).convert(fixture())
        self.assertMatrix(pose_matrix(result.edges[0].pose)[:3, 3], [0., 0., 1.])
        self.assertMatrix(pose_matrix(result.nodes[1].pose), transform([0.,0.,0.], [1.,0.,0.])@SIM_T_BODY_CAMERA)

    def test_random_node_edge_consistency_and_inverse_direction(self):
        rng = np.random.default_rng(782)
        for _ in range(100):
            calibration = transform(rng.normal(size=3), rng.normal(size=3))
            first = transform(rng.normal(size=3), rng.normal(size=3))
            second = transform(rng.normal(size=3), rng.normal(size=3))
            result = GraphConverter(1, calibration).convert(fixture(first, second))
            expected = np.linalg.inv(first@calibration)@(second@calibration)
            actual = pose_matrix(result.edges[0].pose)
            self.assertMatrix(actual, expected)
            self.assertMatrix(np.linalg.inv(actual), np.linalg.inv(second@calibration)@(first@calibration))

    def test_native_identity_headers_and_input_unchanged(self):
        original = fixture(); before = copy.deepcopy(original)
        result = GraphConverter(1).convert(original)
        for a, b in zip(result.nodes, original.nodes):
            self.assertEqual((a.key, a.robot_id, a.header.seq, a.header.frame_id, a.header.stamp.to_nsec()),
                             (b.key, b.robot_id, b.header.seq, b.header.frame_id, b.header.stamp.to_nsec()))
        self.assertEqual((result.edges[0].key_from, result.edges[0].key_to, result.edges[0].robot_from, result.edges[0].robot_to, result.edges[0].type), (0,1,1,1,0))
        self.assertMatrix(pose_matrix(original.nodes[0].pose), pose_matrix(before.nodes[0].pose))
        self.assertMatrix(pose_matrix(original.nodes[1].pose), pose_matrix(before.nodes[1].pose))
        self.assertEqual(result.header.stamp.to_nsec(), original.header.stamp.to_nsec())
        self.assertEqual(result.header.frame_id, original.header.frame_id)
        self.assertEqual(result.edges[0].covariance, [0.]*36)

    def test_bootstrap_waits_for_subscriber_and_is_once_before_original(self):
        stream = OrderedOutput(GraphConverter(1), '/native', bootstrap_spacing=.05)
        graph = fixture(); stream.ingest(graph, '/native', now=1.)
        emitted = []
        self.assertFalse(stream.drain_one(False, emitted.append, now=2.))
        self.assertEqual(emitted, [])
        self.assertTrue(stream.drain_one(True, emitted.append, now=3.))
        self.assertEqual(len(emitted[0].nodes), 1); self.assertEqual(emitted[0].nodes[0].key, 0)
        self.assertEqual(emitted[0].edges, [])
        self.assertEqual(emitted[0].header.stamp.to_nsec(), 1000000001)
        self.assertEqual(emitted[0].header.frame_id, graph.header.frame_id)
        self.assertFalse(stream.drain_one(True, emitted.append, now=3.02))
        self.assertTrue(stream.drain_one(True, emitted.append, now=3.06))
        self.assertEqual(len(emitted[1].nodes), 2)
        self.assertEqual(emitted[1].header.stamp.to_nsec(), 2000000002)
        stream.ingest(graph, '/native', now=4.)
        stream.drain_one(True, emitted.append, now=4.)
        self.assertEqual(stream.bootstrap_count, 1)
        self.assertEqual(len(emitted), 3)

    def test_wrong_source_no_key_zero_and_overflow_fail_without_drops(self):
        stream = OrderedOutput(GraphConverter(1), '/native', max_pending=2)
        with self.assertRaisesRegex(ValueError, 'Rejected graph caller'):
            stream.ingest(fixture(), '/sdk')
        missing = fixture(); missing.nodes[0].key = 9
        with self.assertRaisesRegex(ValueError, 'key-zero'):
            stream.ingest(missing, '/native')
        stream.ingest(fixture(), '/native')
        with self.assertRaisesRegex(RuntimeError, 'overflow'):
            stream.ingest(fixture(), '/native')
        self.assertEqual(stream.received, 1); self.assertEqual(len(stream.pending), 2)

    def test_covariance_adjoint_matches_finite_difference_nonzero_lever_arm(self):
        calibration = transform([.3,-.2,.1], [.25,-.1,.05]); inverse = np.linalg.inv(calibration)
        edge = transform([-.4,.2,.6], [1.,2.,-.1]); mapped = inverse@edge@calibration
        numerical = np.empty((6,6)); epsilon = 1e-6
        for column in range(6):
            delta = np.zeros(6); delta[column] = epsilon
            plus = inverse@edge@se3_exp(delta)@calibration
            minus = inverse@edge@se3_exp(-delta)@calibration
            numerical[:,column] = (small_log(np.linalg.inv(mapped)@plus)-small_log(np.linalg.inv(mapped)@minus))/(2*epsilon)
        self.assertMatrix(adjoint(inverse), numerical, 5e-9)
        rng = np.random.default_rng(42); factor = rng.normal(size=(6,6)); covariance = factor@factor.T
        converted = np.array(rotate_covariance(covariance.reshape(-1), adjoint(inverse))).reshape(6,6)
        self.assertMatrix(converted, numerical@covariance@numerical.T, 2e-8)
        self.assertMatrix(converted, converted.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(converted).min(), -1e-12)

    def test_covariance_order_anisotropy_and_unknown_zero(self):
        covariance = np.diag([1.,2.,3.,4.,5.,6.]); converter = GraphConverter(1)
        result = np.array(rotate_covariance(covariance.reshape(-1), converter.jacobian)).reshape(6,6)
        self.assertMatrix(result, np.diag([2.,3.,1.,5.,6.,4.]))
        self.assertEqual(rotate_covariance([0.]*36, converter.jacobian), [0.]*36)
        permutation = [3,4,5,0,1,2]
        self.assertMatrix(adjoint(converter.camera_body, 'rotation_translation'), converter.jacobian[np.ix_(permutation,permutation)])

    def test_invalid_calibration_covariance_and_other_robot_rejected(self):
        invalid = np.eye(4); invalid[0,0] = 2
        with self.assertRaises(ValueError): validate_transform(invalid)
        with self.assertRaises(ValueError): GraphConverter(0).convert(fixture())
        graph = fixture(); graph.edges[0].covariance[0] = -1
        with self.assertRaisesRegex(ValueError, 'positive semidefinite'): GraphConverter(1).convert(graph)
        graph = fixture(); graph.edges[0].type = 5
        with self.assertRaisesRegex(ValueError, 'Only ODOM'): GraphConverter(1).convert(graph)

    def test_quaternion_roundtrip_near_half_turns(self):
        for axis in np.eye(3):
            for angle in (0., 1e-10, math.pi-1e-10, math.pi):
                value = transform(axis*angle, [1.,2.,3.])
                self.assertMatrix(pose_matrix(pose(value)), value)


if __name__ == '__main__':
    unittest.main(verbosity=2)
