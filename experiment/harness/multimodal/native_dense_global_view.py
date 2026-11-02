#!/usr/bin/env python3
"""Read-only two-color display of two exactly receipted native PGMO meshes.

Only immutable accepted_joint_mesh and latest authoritative bridge status are
read. No alignment, interpolation, reprojection, downsampling or map estimation
is performed. Native vertices are concatenated exactly in kimera_world. Native
messages lack a shared optimizer round ID, so matched native evidence is labelled
an observational binding and is never advertised as an atomic distributed map.
"""
import argparse
from collections import OrderedDict
import copy
import hashlib
import json
from pathlib import Path
import threading
import time

import numpy as np

from native_dense_bridge import canonical_mesh_digest, validate_mesh


COLORS = (0x34D8ED, 0xFFA04B)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def mesh_content(message, cutoff_ns, world_frame):
    xyz = validate_mesh(message, cutoff_ns, world_frame)
    return dict(canonical_native_mesh_sha256=canonical_mesh_digest(message),
        vertex_xyz_float32_le_sha256=sha256(xyz.astype('<f4').tobytes()),
        mesh_header_stamp_ns=message.header.stamp.to_nsec(), mesh_frame=message.header.frame_id,
        vertex_count=len(message.vertices), triangle_count=len(message.triangles),
        vertex_stamps_uint64_le_sha256=sha256(np.asarray([s.to_nsec() for s in message.vertex_stamps], dtype='<u8').tobytes()),
        triangles_uint32_le_sha256=sha256(np.asarray([t.vertex_indices for t in message.triangles], dtype='<u4').tobytes()),
        xyz=xyz)


class NotReady(ValueError):
    pass


class NativeMeshPair:
    def __init__(self, run_id, cutoff_ns, world_frame='kimera_world', wall_tolerance=1., weight_updates=25):
        self.run_id, self.cutoff_ns, self.frame = run_id, cutoff_ns, world_frame
        self.wall_tolerance, self.weight_updates = wall_tolerance, weight_updates
        self.meshes = {0: OrderedDict(), 1: OrderedDict()}; self.statuses = {0: None, 1: None}
        self.mesh_messages = [0, 0]; self.status_messages = [0, 0]

    @staticmethod
    def expected_caller(robot): return '/sim_%d/native_dense_bridge' % robot

    def ingest_mesh(self, robot, message, caller):
        if caller != self.expected_caller(robot): raise ValueError('Wrong native accepted-mesh publisher')
        if message.ns != str(robot): raise ValueError('Native mesh namespace ownership mismatch')
        content = mesh_content(message, self.cutoff_ns, self.frame)
        identity = content['canonical_native_mesh_sha256']; self.mesh_messages[robot] += 1
        if identity in self.meshes[robot]: return False
        self.meshes[robot][identity] = content
        while len(self.meshes[robot]) > 2: self.meshes[robot].popitem(last=False)
        return True

    def ingest_status(self, robot, value, caller):
        # Clear first: the newest expected status is authoritative, even when
        # malformed or not yet eligible. Never seek an older passing generation.
        self.statuses[robot] = None
        if caller != self.expected_caller(robot): raise ValueError('Wrong native bridge-status publisher')
        self.status_messages[robot] += 1
        if not isinstance(value, dict): raise ValueError('Native bridge status must be an object')
        self.statuses[robot] = copy.deepcopy(value)

    def receipt(self, robot):
        status = self.statuses[robot]
        if status is None: raise NotReady('Waiting for latest robot%d status' % robot)
        if (status.get('run_id') != self.run_id or status.get('cutoff_ns') != self.cutoff_ns or
                status.get('robot_id') != robot or status.get('mode') != 'joint'):
            raise NotReady('Robot%d latest status epoch/mode mismatch' % robot)
        if status.get('fatal_error') is not None or status.get('native_deformation_eligible') is not True:
            raise NotReady('Robot%d latest native map is not deformation eligible' % robot)
        receipt = status.get('accepted_joint_mesh_receipt')
        if not isinstance(receipt, dict): raise NotReady('Robot%d has no immutable accepted mesh receipt' % robot)
        if (receipt.get('run_id') != self.run_id or receipt.get('cutoff_ns') != self.cutoff_ns or
                receipt.get('robot_id') != robot or receipt.get('mesh_frame') != self.frame or
                receipt.get('immutable_until_next_accepted_transaction') is not True):
            raise NotReady('Robot%d immutable receipt epoch/frame/ownership mismatch' % robot)
        proof = receipt.get('completed_round_proof') or {}
        native_round, weight = proof.get('round') or {}, proof.get('weights') or {}
        if (receipt.get('optimizer_owner') != native_round.get('owner') or
                receipt.get('completed_round_serial') != native_round.get('serial') or
                receipt.get('final_native_weight_batch_sha256') != weight.get('sha256')):
            raise NotReady('Robot%d receipt optimizer evidence is inconsistent' % robot)
        request, terminate = native_round.get('request_wall'), native_round.get('terminate_wall')
        updates = native_round.get('weight_update_walls') or []
        if (not isinstance(request, (float, int)) or not isinstance(terminate, (float, int)) or
                not np.isfinite([request, terminate]).all() or not request < terminate or
                len(updates) != self.weight_updates or not np.isfinite(updates).all() or
                updates != sorted(updates) or not all(request < t < terminate for t in updates)):
            raise NotReady('Robot%d lacks the complete finite native GNC round' % robot)
        if set(native_round.get('active_robots', [])) != {0, 1}:
            raise NotReady('Robot%d native round does not include both robots' % robot)
        weight_time = weight.get('received_wall', -1)
        if not isinstance(weight_time, (int, float)) or not np.isfinite(weight_time) or weight_time < terminate:
            raise NotReady('Robot%d weight evidence precedes native termination' % robot)
        rows = weight.get('rows') or []
        encoded = json.dumps(rows, separators=(',', ':'), allow_nan=False).encode()
        if sha256(encoded) != weight.get('sha256'):
            raise NotReady('Robot%d final native weight contents/hash mismatch' % robot)
        accepted_cross = [r for r in rows if len(r) == 6 and r[0] != r[2] and np.isfinite(r[4]) and r[4] >= .5]
        if not accepted_cross or proof.get('accepted_cross_constraints', 0) <= 0:
            raise NotReady('Robot%d has no accepted native inter-robot constraint' % robot)
        deform = receipt.get('native_deformation_proof') or {}
        if (deform.get('native_path_echo_verified') is not True or
                deform.get('same_native_input_mesh_topology_and_vertex_stamps') is not True or
                deform.get('historical_vertices_moved', 0) <= 0 or
                deform.get('maximum_vertex_motion_m', 0.) <= 1e-5 or
                deform.get('submitted_path_sha256') != receipt.get('submitted_body_path_sha256')):
            raise NotReady('Robot%d lacks actual same-input native deformation proof' % robot)
        identity = receipt.get('canonical_native_mesh_sha256')
        content = self.meshes[robot].get(identity)
        if content is None: raise NotReady('Waiting for robot%d exact newly receipted mesh' % robot)
        for name, value in content.items():
            if name != 'xyz' and receipt.get(name) != value:
                raise NotReady('Robot%d native mesh/receipt %s mismatch' % (robot, name))
        return receipt, content

    def evaluate(self):
        selected = [self.receipt(robot) for robot in (0, 1)]
        first, second = selected[0][0], selected[1][0]
        for name in ('optimizer_owner', 'completed_round_serial', 'final_native_weight_batch_sha256'):
            if first[name] != second[name]: raise NotReady('Latest native meshes have mixed optimizer generations: '+name)
        for name in ('request_wall', 'terminate_wall'):
            difference = abs(first['completed_round_proof']['round'][name]-second['completed_round_proof']['round'][name])
            if difference > self.wall_tolerance: raise NotReady('Native round %s differs by %.3fs' % (name, difference))
        xyz = np.concatenate([item[1]['xyz'] for item in selected]).astype('<f4', copy=False)
        colors = np.concatenate([np.full(len(item[1]['xyz']), COLORS[robot], '<u4') for robot, item in enumerate(selected)])
        sources = np.concatenate([np.full(len(item[1]['xyz']), robot, np.uint8) for robot, item in enumerate(selected)])
        source_hashes = [item[0]['canonical_native_mesh_sha256'] for item in selected]
        identity = sha256(json.dumps([self.run_id, self.cutoff_ns, source_hashes,
            first['optimizer_owner'], first['completed_round_serial'], first['final_native_weight_batch_sha256']], separators=(',', ':')).encode())
        report = dict(run_id=self.run_id, cutoff_ns=self.cutoff_ns, fusion_ready=True,
            geometry_operation='exact_concatenation_of_native_optimized_mesh_vertices',
            estimator=False, uses_icp=False, uses_reprojection=False, uses_voxel_filter=False,
            frame=self.frame, cloud_stamp_ns=max(item[0]['mesh_header_stamp_ns'] for item in selected),
            points=len(xyz), robot_point_counts=[len(item[1]['xyz']) for item in selected],
            packed_source_colors=[hex(color) for color in COLORS],
            ordered_xyz_float32_le_sha256=sha256(xyz.tobytes()),
            ordered_colors_uint32_le_sha256=sha256(colors.tobytes()), snapshot_identity=identity,
            exact_native_mesh_hashes=source_hashes, optimizer_owner=first['optimizer_owner'],
            completed_round_serial=first['completed_round_serial'],
            final_native_weight_batch_sha256=first['final_native_weight_batch_sha256'],
            native_receipts=[copy.deepcopy(item[0]) for item in selected],
            binding='matched_native_observations_no_shared_atomic_round_id',
            no_fallback_to_older_passing_generation=True)
        return xyz, colors, sources, report


def packed_cloud(xyz, colors, stamp, world_frame):
    values = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    if len(xyz): values['x'], values['y'], values['z'] = xyz.T
    values['rgb'] = colors
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32 if i < 3 else PointField.UINT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    return PointCloud2(header=Header(stamp=stamp, frame_id=world_frame), height=1, width=len(xyz), fields=fields,
        is_bigendian=False, point_step=16, row_step=16*len(xyz), data=values.tobytes(), is_dense=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--world-frame', choices=('kimera_world',), default='kimera_world')
    parser.add_argument('--round-wall-tolerance', type=float, default=1.)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.cutoff <= 0 or not 0 < args.round_wall_tolerance <= 1.: parser.error('Invalid epoch or native round tolerance')
    args.output_directory.mkdir(parents=True, exist_ok=True)
    rospy.init_node('native_dense_global_view', anonymous=False)
    verifier = NativeMeshPair(args.run_id, round(args.cutoff*1e9), args.world_frame, args.round_wall_tolerance)
    lock = threading.RLock(); errors = []; received = dict(mesh=[0, 0], status=[0, 0])
    cloud = rospy.Publisher('/visualization/native_mesh/global_cloud', PointCloud2, queue_size=1, latch=True)
    status = rospy.Publisher('/visualization/native_mesh/status', String, queue_size=1, latch=True)
    state = dict(last_identity=None, displayed=False, archives=0, archive_receipt={})

    def receive(robot, kind, message):
        with lock:
            try:
                caller = (getattr(message, '_connection_header', None) or {}).get('callerid')
                if kind == 'mesh': verifier.ingest_mesh(robot, message, caller)
                else: verifier.ingest_status(robot, json.loads(message.data), caller)
                received[kind][robot] += 1
            except Exception as error:
                if kind == 'status': verifier.statuses[robot] = None
                errors.append(dict(wall=time.time(), robot=robot, input=kind, error=str(error)))
                del errors[:-50]

    subscriptions = []
    for robot in (0, 1):
        subscriptions.append(rospy.Subscriber('/sim_%d/native_dense/accepted_joint_mesh' % robot, KimeraPgmoMesh,
            lambda m, r=robot: receive(r, 'mesh', m), queue_size=2, buff_size=2**27, tcp_nodelay=True))
        subscriptions.append(rospy.Subscriber('/sim_%d/native_dense/status' % robot, String,
            lambda m, r=robot: receive(r, 'status', m), queue_size=2, buff_size=2**24, tcp_nodelay=True))

    def update(_):
        with lock:
            try:
                xyz, colors, sources, report = verifier.evaluate()
                if report['snapshot_identity'] != state['last_identity']:
                    state['archives'] += 1
                    prefix = args.output_directory/('native_pair_%04d' % state['archives'])
                    np.savez_compressed(str(prefix)+'.npz', xyz=xyz, packed_colors=colors, robot_ids=sources)
                    archived = np.load(str(prefix)+'.npz')
                    if sha256(archived['xyz'].astype('<f4').tobytes()) != report['ordered_xyz_float32_le_sha256']:
                        raise ValueError('Archived native vertex XYZ differs from verified actual display')
                    state['archive_receipt'] = dict(archive_npz=prefix.name+'.npz', archive_json=prefix.name+'.json')
                    report.update(state['archive_receipt'])
                    prefix.with_suffix('.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
                    ns = report['cloud_stamp_ns']
                    cloud.publish(packed_cloud(xyz, colors, rospy.Time(secs=ns//10**9, nsecs=ns % 10**9), args.world_frame))
                    state['last_identity'] = report['snapshot_identity']; state['displayed'] = True
                elif not state['displayed']:
                    ns = report['cloud_stamp_ns']
                    cloud.publish(packed_cloud(xyz, colors, rospy.Time(secs=ns//10**9, nsecs=ns % 10**9), args.world_frame))
                    state['displayed'] = True
                report.update(state['archive_receipt'])
            except (NotReady, ValueError, TypeError, KeyError, IndexError) as error:
                report = dict(run_id=args.run_id, cutoff_ns=verifier.cutoff_ns, fusion_ready=False,
                    frame=args.world_frame, points=0, reason=str(error),
                    no_fallback_to_older_passing_generation=True,
                    binding='matched_native_observations_no_shared_atomic_round_id')
                if state['displayed']:
                    cloud.publish(packed_cloud(np.empty((0, 3), '<f4'), np.empty(0, '<u4'), rospy.Time.now(), args.world_frame))
                    state['displayed'] = False
            report.update(wall=time.time(), subscriber_only_geometry_inputs=True,
                received=received, rejected_messages=errors, archive_count=state['archives'])
            status.publish(String(data=json.dumps(report, allow_nan=False)))
            (args.output_directory/'status.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    timer = rospy.Timer(rospy.Duration(.5), update)
    rospy.spin()
    timer.shutdown()


if __name__ == '__main__':
    import rospy
    from kimera_pgmo_msgs.msg import KimeraPgmoMesh
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header, String
    main()
