#!/usr/bin/env python3
"""Local TSDF continuity and display separation; native replay is a separate gate."""
import copy
from collections import Counter
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import numpy as np

import native_dense_bridge as dense
from test_native_dense_bridge import FakeMessage, Stamp, image, node


class PointField(S):
    FLOAT32 = 7
    UINT32 = 6


class LocalDenseStreams(unittest.TestCase):
    def setUp(self):
        self.types = patch.multiple(dense, Header=S, PointCloud2=S,
                                   PointField=PointField, TransformStamped=S, create=True)
        self.types.start()
        self.addCleanup(self.types.stop)

    def test_external_sensor_mode_has_one_sensor_producer_and_keeps_native_backend_inputs(self):
        raw_topics = {'/sim_0/kimera/'+suffix for suffix in
                      ('imu', 'rgb/camera_info', 'rgb/image_raw', 'depth/image_raw')}
        backend_topics = {'/sim_0/'+suffix for suffix in
                          ('kimera_vio_ros/odometry', 'kimera_vio_ros/pose_graph_incremental',
                           'native_mesh_frontend/full_mesh', 'native_pgmo/optimized_mesh',
                           'kimera_distributed/optimized_nodes')}
        backend_topics.update('/sim_%d/dpgo_ros_node/%s' % (robot, kind)
                              for robot in (0, 1) for kind in ('command', 'measurement_weights'))
        for external in (False, True):
            with self.subTest(external_sensor_stream=external), tempfile.TemporaryDirectory() as folder:
                publishers, subscriptions = {}, {}

                def publisher(topic, kind, **options):
                    messages = []
                    value = S(messages=messages, publish=messages.append, get_num_connections=lambda: 0)
                    publishers[topic] = value
                    return value

                def subscriber(topic, kind, callback, **options):
                    subscriptions[topic] = S(callback=callback, **options)
                    return subscriptions[topic]

                ros = S(Publisher=publisher, Subscriber=subscriber, is_shutdown=lambda: False,
                        Duration=lambda seconds: seconds, Timer=lambda duration, callback: S(callback=callback))
                args = S(robot_id=0, robot_name='sim_0', run_id='separate_sensor_process', cutoff=1.,
                         mode='joint', global_frame='kimera_world', max_gyro_gap=.05,
                         gyro_sample_convention='left_endpoint_interval_equivalent',
                         sensor_rotation_interpolation='gyro_endpoint', max_odom_gap=.6,
                         weight_updates=25, output_directory=Path(folder), recorded_native_bag=None,
                         recorded_native_bag_sha256=None, replay_caller='/native_dense_replay',
                         require_cross=False, mesh_phase_timeout=30.)
                # Omitting the option exercises compatibility with callers that
                # construct the older argument namespace rather than argparse.
                if external: args.external_sensor_stream = True
                types = {name: S for name in ('PoseGraph', 'KimeraPgmoMesh', 'RosPath', 'String',
                                              'Odometry', 'Imu', 'CameraInfo', 'Image', 'Command',
                                              'RelativeMeasurementWeights')}
                with patch.multiple(dense, rospy=ros,
                                    tf2_ros=S(TransformBroadcaster=lambda: S()), create=True, **types):
                    bridge = dense.NativeDenseBridge(args)
                    try:
                        self.assertTrue(backend_topics.issubset(subscriptions))
                        self.assertIsInstance(bridge.rounds, dense.RoundLedger)
                        self.assertEqual(raw_topics.intersection(subscriptions), set() if external else raw_topics)
                        self.assertEqual('/sim_0/native_dense/rgbd_points' in publishers, not external)
                        bridge.tick(None)
                        self.assertIsNone(bridge.fatal)
                        status = publishers['/sim_0/native_dense/status'].messages[-1]
                        self.assertEqual(json.loads(status.data)['sensor_stream_in_independent_process'], external)
                        if external:
                            # Even an accidentally queued frame cannot make this
                            # backend publish a second TSDF cloud or consume it.
                            frame_pair = object()
                            bridge.gauge_checked = True; bridge.camera = S()
                            bridge.pending_images[2_000_000_000] = frame_pair
                            for phase in ('await_native_seed', 'await_baseline', 'await_path_echo', 'await_deformed_mesh'):
                                bridge.transaction = dict(phase=phase)
                                bridge.publish_sensor()
                                self.assertIs(bridge.pending_images[2_000_000_000], frame_pair)
                                self.assertEqual(bridge.last_rgbd_ns, 0)
                                self.assertEqual(bridge.counts['sensor_clouds'], 0)
                    finally:
                        bridge.events.close()

    def test_sensor_continues_in_every_pgmo_phase_without_changing_frozen_mesh(self):
        for phase in ('await_native_seed', 'await_baseline', 'await_path_echo', 'await_deformed_mesh'):
            with self.subTest(phase=phase):
                bridge = object.__new__(dense.NativeDenseBridge)
                frozen = FakeMessage(header=S(seq=0), vertices=[S(x=1., y=2., z=3.)])
                frozen_bytes = dense.message_bytes(frozen)
                bridge.transaction = dict(phase=phase, fixed_mesh=frozen)
                bridge.gauge_checked = True
                bridge.a = S(cloud_rate=1., pixel_stride=1, max_depth=8.)
                bridge.last_rgbd_ns = 0
                bridge.local_frame = 'sim_0/vio_odom'
                bridge.optical_frame = 'sim_0/native_dense_optical'
                bridge.camera = S(K=[2., 0., 0., 0., 2., 0., 0., 0., 1.], width=2, height=2)
                rgb = image(np.full((2, 2, 3), [10, 20, 30], np.uint8), 'rgb8')
                depth = image(np.array([[2., 0.], [np.nan, 3.]], '<f4'), '32FC1')
                rgb.header = depth.header = S(stamp=Stamp(2_000_000_000))
                bridge.pending_images = {2_000_000_000: dict(rgb=rgb, depth=depth)}
                bridge.keys = S(nodes={0: node(0)})
                bridge.history = S(at=lambda stamp: np.eye(4))
                bridge.counts = Counter()
                points, transforms = [], []
                bridge.pub = {'points': S(get_num_connections=lambda: 1, publish=points.append)}
                bridge.tf = S(sendTransform=transforms.append)
                # TransformStamped normally supplies its nested transform.
                transform_type = lambda **kw: S(transform=S(translation=S(), rotation=None), **kw)
                with patch.object(dense, 'TransformStamped', transform_type):
                    bridge.publish_sensor()
                self.assertEqual(len(points), 1)
                cloud = points[0]
                self.assertEqual(cloud.header.frame_id, bridge.optical_frame)
                self.assertEqual(cloud.header.stamp.to_nsec(), 2_000_000_000)
                self.assertEqual(transforms[0].header.frame_id, bridge.local_frame)
                actual = np.frombuffer(cloud.data, dtype=[('xyz', '<f4', (3,)), ('rgb', '<u4')])
                np.testing.assert_array_equal(actual['xyz'], [[0., 0., 2.], [1.5, 1.5, 3.]])
                self.assertEqual(bridge.counts['sensor_clouds_during_mesh_transaction'], 1)
                self.assertEqual(bridge.counts['sensor_clouds'], 1)
                self.assertEqual(bridge.pending_images, {})
                self.assertIs(bridge.transaction['fixed_mesh'], frozen)
                self.assertEqual(dense.message_bytes(frozen), frozen_bytes)
                # Native PGMO graph publication remains frozen in this phase.
                bridge.publish_graphs()
                self.assertEqual(bridge.transaction['phase'], phase)

    def test_raw_frontend_display_retains_local_xyz_stamp_and_input_mesh(self):
        bridge = object.__new__(dense.NativeDenseBridge)
        bridge.a = S(robot_id=1)
        bridge.cutoff_ns = 100
        bridge.local_frame = 'sim_1/vio_odom'
        bridge.counts = Counter()
        frozen = FakeMessage(header=S(seq=7), vertices=[S(x=99., y=0., z=0.)])
        bridge.transaction = dict(phase='await_deformed_mesh', fixed_mesh=frozen)
        frozen_bytes = dense.message_bytes(frozen)
        mesh = FakeMessage(ns='0', header=S(seq=5, frame_id=bridge.local_frame, stamp=Stamp(123)),
            vertices=[S(x=1., y=2., z=3.), S(x=4., y=2., z=3.), S(x=1., y=5., z=3.)],
            triangles=[S(vertex_indices=[0, 1, 2])], vertex_indices=[0, 1, 2],
            vertex_stamps=[Stamp(123)]*3, vertex_colors=[S(r=.2, g=.3, b=.4, a=1.)]*3)
        original = dense.message_bytes(mesh)
        clouds = []
        bridge.pub = {'raw_local_vertices': S(publish=clouds.append)}
        bridge.on_full_mesh(mesh)
        self.assertEqual(dense.message_bytes(mesh), original)
        self.assertEqual(dense.message_bytes(frozen), frozen_bytes)
        self.assertEqual(bridge.latest_full_mesh.ns, '1')
        self.assertEqual(bridge.latest_full_mesh.header.frame_id, bridge.local_frame)
        self.assertEqual(len(clouds), 1)
        cloud = clouds[0]
        self.assertEqual(cloud.header.frame_id, bridge.local_frame)
        self.assertEqual(cloud.header.stamp.to_nsec(), 123)
        actual = np.frombuffer(cloud.data, dtype=[('xyz', '<f4', (3,)), ('rgb', '<u4')])
        np.testing.assert_array_equal(actual['xyz'], [[1., 2., 3.], [4., 2., 3.], [1., 5., 3.]])
        np.testing.assert_array_equal(actual['rgb'], [0xFFA04B]*3)
        self.assertEqual(bridge.counts['raw_local_mesh_displays'], 1)
        # A global/deformed mesh must never be relabeled into a local window.
        invalid = copy.deepcopy(mesh)
        invalid.header.frame_id = 'kimera_world'
        with self.assertRaises(ValueError): bridge.on_full_mesh(invalid)
        self.assertEqual(len(clouds), 1)


if __name__ == '__main__': unittest.main(verbosity=2)
