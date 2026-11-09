#!/usr/bin/env python3
"""Thin ROS adapters for the pinned native Voxblox -> PGMO geometry chain.

Geometry production is native tsdf_server, voxblox_mesh_frontend_node and PGMO
7a67674b677468e6c28152596c9aae66cd24ee4b. This file only converts calibrated RGB-D
to optical point messages, interpolates native VIO for TF, preserves native graph
keys/measurements, converts optimized camera poses back to body poses, and exposes
native mesh vertices for display. It does not reconstruct or deform a map.

Start before VIO and sensors in a fresh epoch. The default diagnostic execution
retains the fixed-input deformation experiment. Explicit online execution never
freezes native graph/mesh inputs and accepts numerically unchanged solutions.
The pinned native messages have no shared round ID: completion and path echoes
are observational evidence, not an atomic optimizer/mesh receipt. Network
compression and transmission belong in a separate process.
"""
import argparse
import bisect
from collections import Counter, deque
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import re
import sys
import threading
import time
import xmlrpc.client

import numpy as np

from native_vio_graph_adapter import SIM_T_BODY_CAMERA, pose_matrix, set_pose, rotate_covariance
from native_imu_pose_interpolation import GyroCoverageError, GyroRotationHistory


# The calibrated extrinsic is fixed for this deployed simulator. Compute its
# inverse once, rather than solving the same 4x4 system for every expanded key.
SIM_T_CAMERA_BODY = np.linalg.inv(SIM_T_BODY_CAMERA)
SIM_T_CAMERA_BODY.setflags(write=False)


def require_source(message, expected):
    caller = (getattr(message, '_connection_header', None) or {}).get('callerid')
    if caller != expected:
        raise ValueError('Rejected caller %r; expected %r' % (caller, expected))


def require_stamp(stamp, cutoff_ns):
    value = int(stamp.to_nsec())
    if value < cutoff_ns or value <= 0:
        raise ValueError('Zero or previous-epoch sensor/keyframe timestamp')
    return value


def frame(value):
    return str(value).lstrip('/')


def message_bytes(message):
    stream = io.BytesIO(); message.serialize(stream); return stream.getvalue()


def digest(message):
    return hashlib.sha256(message_bytes(message)).hexdigest()


def publish_preserving_cached_message(publisher, message):
    # rospy/msg.py serialize_message mutates only the top-level Header's seq
    # (and defaults a None frame_id). Native geometry/pose arrays serialize
    # read-only. Own the envelope/header without walking every mesh object.
    publisher.publish(snapshot_message_header(message))


def snapshot_message_header(message):
    output = copy.copy(message)
    if hasattr(message, 'header'): output.header = copy.copy(message.header)
    return output


def canonical_mesh_digest(message):
    """Full native mesh content hash, ignoring only ROS transport Header.seq."""
    return hashlib.sha256(canonical_mesh_bytes(message)).hexdigest()


def canonical_mesh_bytes(message):
    """Serialize an immutable native mesh once, normalizing transport seq only."""
    normalized = snapshot_message_header(message)
    normalized.header.seq = 0
    return message_bytes(normalized)


def canonical_serialized_mesh_digest(payload):
    """Hash native ROS bytes, ignoring only the top Header.seq uint32."""
    if len(payload) < 4: raise ValueError('Truncated native mesh Header.seq')
    value = hashlib.sha256(); value.update(b'\0'*4); value.update(memoryview(payload)[4:])
    return value.hexdigest()


def publish_serialized_message(publisher, payload):
    # The publisher remains registered with the native ROS type and MD5.
    # AnyMsg.serialize forwards its immutable buffer without a second walk of
    # the native vertices/colors/faces or rospy rewriting the retained header.
    publisher.publish(rospy.AnyMsg().deserialize(payload))


def mesh_proof_hashes(message, xyz):
    return dict(vertex_xyz_float32_le_sha256=hashlib.sha256(xyz.astype('<f4').tobytes()).hexdigest(),
        vertex_stamps_uint64_le_sha256=hashlib.sha256(np.asarray([s.to_nsec() for s in message.vertex_stamps], dtype='<u8').tobytes()).hexdigest(),
        triangles_uint32_le_sha256=hashlib.sha256(np.asarray([t.vertex_indices for t in message.triangles], dtype='<u4').tobytes()).hexdigest())


class BoundedXmlRpcTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__(); self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host); connection.timeout = self.timeout
        return connection


def native_node_identity(master_uri, node_name, caller, timeout=.5):
    """Observe a native ROS process incarnation, without trusting node name alone."""
    with xmlrpc.client.ServerProxy(master_uri, transport=BoundedXmlRpcTransport(timeout)) as master:
        code, reason, uri = master.lookupNode(caller, node_name)
    if code != 1 or not isinstance(uri, str) or not uri.startswith('http://'):
        raise ValueError('Cannot resolve native PGMO process identity: '+str(reason))
    with xmlrpc.client.ServerProxy(uri, transport=BoundedXmlRpcTransport(timeout)) as native:
        code, reason, pid = native.getPid(caller)
    if code != 1 or type(pid) is not int or pid <= 0:
        raise ValueError('Cannot verify native PGMO process PID: '+str(reason))
    return dict(node=node_name, uri=uri, pid=pid)


class NativeProcessIdentityUnavailable(ValueError):
    pass


class NativeProcessProbe:
    """Bounded local XMLRPC observations, independent of the SLAM state lock."""
    def __init__(self, operation, period=.5):
        self.operation, self.period = operation, period
        self.lock = threading.Lock(); self.stopped = threading.Event()
        self.value = dict(identity=None, error='not_yet_observed', checked_monotonic=None)
        self.thread = threading.Thread(target=self.run, name='native-pgmo-process-probe', daemon=True)
        self.thread.start()

    def run(self):
        while not self.stopped.is_set():
            try: identity, error = self.operation(), None
            except Exception as failure: identity, error = None, str(failure)
            with self.lock:
                self.value = dict(identity=identity, error=error, checked_monotonic=time.monotonic())
            self.stopped.wait(self.period)

    def snapshot(self):
        with self.lock: return copy.deepcopy(self.value)

    def close(self):
        self.stopped.set()


def canonical_graph_digest(message):
    """Ignore transport sequence only; retain every native key/stamp/pose bit."""
    if not hasattr(message, 'header'): return digest(message)
    normalized = snapshot_message_header(message); normalized.header.seq = 0
    return digest(normalized)


def path_geometry_digest(message):
    # DPGO recreates ROS header stamps on every cached-path retransmission.
    # Hash only numerical poses to avoid treating that timer echo as an update.
    values = [pose_matrix(stamped.pose).reshape(-1).tolist() for stamped in message.poses]
    if not values: raise ValueError('Empty native optimized path')
    return hashlib.sha256(json.dumps(values, separators=(',', ':')).encode()).hexdigest()


def image_array(message):
    encodings = {'rgb8': (np.dtype('u1'), 3), 'bgr8': (np.dtype('u1'), 3),
                 '32FC1': (np.dtype('>f4' if message.is_bigendian else '<f4'), 1)}
    if message.encoding not in encodings:
        raise ValueError('Unsupported calibrated camera encoding: '+message.encoding)
    dtype, channels = encodings[message.encoding]
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width*channels*dtype.itemsize:
        raise ValueError('Invalid image dimensions/stride')
    if len(message.data) < step*height:
        raise ValueError('Truncated image buffer')
    shape = (height, width, channels) if channels > 1 else (height, width)
    strides = (step, channels*dtype.itemsize, dtype.itemsize) if channels > 1 else (step, dtype.itemsize)
    result = np.ndarray(shape=shape, dtype=dtype, buffer=message.data, strides=strides)
    return result[..., ::-1] if message.encoding == 'bgr8' else result


def optical_points(rgb, depth, calibration, stride, minimum, maximum):
    """A pinhole sensor-message conversion; no world pose or map is involved."""
    rgb, depth = image_array(rgb), image_array(depth)
    if rgb.shape[:2] != depth.shape:
        raise ValueError('RGB/depth size mismatch')
    k = np.asarray(calibration.K, dtype=float).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or
            calibration.width != depth.shape[1] or calibration.height != depth.shape[0]):
        raise ValueError('Invalid or mismatched camera calibration')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(np.float64)
    valid = np.isfinite(z) & (z >= minimum) & (z <= maximum)
    xyz = np.column_stack(((u[valid]-k[0, 2])*z[valid]/k[0, 0],
                           (v[valid]-k[1, 2])*z[valid]/k[1, 1], z[valid])).astype('<f4')
    color = rgb[::stride, ::stride][valid].astype(np.uint32)
    packed = (color[:, 0] << 16) | (color[:, 1] << 8) | color[:, 2]
    return xyz, packed


def interpolate_transform(first, second, fraction):
    """Shortest-arc unit-quaternion interpolation, with linear translation."""
    qa = np.array([first.orientation.x, first.orientation.y, first.orientation.z, first.orientation.w])
    qb = np.array([second.orientation.x, second.orientation.y, second.orientation.z, second.orientation.w])
    ta, tb = pose_matrix(first), pose_matrix(second)
    if np.dot(qa, qb) < 0: qb = -qb
    dot = float(np.clip(np.dot(qa, qb), -1., 1.))
    if dot > .9995:
        q = qa+fraction*(qb-qa)
    else:
        theta = np.arccos(dot)
        q = (np.sin((1-fraction)*theta)*qa+np.sin(fraction*theta)*qb)/np.sin(theta)
    q /= np.linalg.norm(q)
    result_pose = copy.deepcopy(first)
    result_pose.orientation.x, result_pose.orientation.y, result_pose.orientation.z, result_pose.orientation.w = q
    result = pose_matrix(result_pose)
    result[:3, 3] = (1-fraction)*ta[:3, 3]+fraction*tb[:3, 3]
    return result


class NativeOdometryGap(ValueError):
    pass


class PoseHistory:
    def __init__(self, maximum_gap_ns, maximum_samples=2400, gyros=None):
        self.maximum_gap_ns, self.maximum_samples = maximum_gap_ns, maximum_samples
        self.gyros = gyros
        self.values = {}; self.maximum_interpolated_gap_ns = 0; self.maximum_rejected_gap_ns = 0

    def put(self, stamp, pose):
        pose_matrix(pose)
        self.values[stamp] = copy.deepcopy(pose)
        for key in sorted(self.values)[:-self.maximum_samples]: del self.values[key]

    def at(self, stamp):
        keys = sorted(self.values)
        if stamp in self.values: return pose_matrix(self.values[stamp])
        index = bisect.bisect_left(keys, stamp)
        if not index or index == len(keys): return None
        lo, hi = keys[index-1:index+1]
        if hi-lo > self.maximum_gap_ns:
            self.maximum_rejected_gap_ns = max(self.maximum_rejected_gap_ns, hi-lo)
            raise NativeOdometryGap('Cannot interpolate native VIO gap %.6fs exceeding %.6fs' %
                                    ((hi-lo)*1e-9, self.maximum_gap_ns*1e-9))
        self.maximum_interpolated_gap_ns = max(self.maximum_interpolated_gap_ns, hi-lo)
        result = interpolate_transform(self.values[lo], self.values[hi], (stamp-lo)/(hi-lo))
        if self.gyros is not None:
            try:
                result[:3, :3] = self.gyros.rotation_at(pose_matrix(self.values[lo])[:3, :3],
                    pose_matrix(self.values[hi])[:3, :3], lo, hi, stamp)
            except GyroCoverageError as error:
                if error.kind == 'future': return None
                raise
        return result


class KeyframeLedger:
    def __init__(self, robot_id, cutoff_ns):
        self.robot_id, self.cutoff_ns = robot_id, cutoff_ns
        self.nodes, self.odometry = {}, {}

    def ingest(self, message):
        for node in message.nodes:
            if int(node.robot_id) != self.robot_id: raise ValueError('Foreign node in native local graph')
            key, stamp = int(node.key), require_stamp(node.header.stamp, self.cutoff_ns)
            pose_matrix(node.pose)
            old = self.nodes.get(key)
            if old is not None and int(old.header.stamp.to_nsec()) != stamp:
                raise ValueError('Native key reused across epochs')
            self.nodes[key] = copy.deepcopy(node)
        for edge in message.edges:
            if edge.robot_from != self.robot_id or edge.robot_to != self.robot_id:
                raise ValueError('Foreign edge in native local graph')
            pose_matrix(edge.pose); rotate_covariance(edge.covariance, np.eye(6))
            if int(edge.type) != int(edge.ODOM): continue
            a, b = int(edge.key_from), int(edge.key_to)
            if b != a+1: raise ValueError('Native ODOM key sequence is not consecutive')
            stamp = require_stamp(edge.header.stamp, self.cutoff_ns)
            if b in self.nodes and stamp != self.nodes[b].header.stamp.to_nsec():
                raise ValueError('Original ODOM edge stamp differs from destination key')
            old = self.odometry.get((a, b))
            if old is not None and digest(old) != digest(edge):
                raise ValueError('Native original ODOM measurement changed for an existing key')
            self.odometry[a, b] = copy.deepcopy(edge)

    def optimized_structure(self, nodes):
        selected = sorted(nodes, key=lambda item: int(item.key))
        if not selected or [int(n.key) for n in selected] != list(range(len(selected))):
            raise ValueError('PGMO MESH requires unique complete original keys 0..N without gaps')
        for node in selected:
            if int(node.robot_id) != self.robot_id:
                raise ValueError('Foreign robot owner in native optimized graph')
            require_stamp(node.header.stamp, self.cutoff_ns); pose_matrix(node.pose)
            original = self.nodes.get(int(node.key))
            if original is not None and node.header.stamp.to_nsec() != original.header.stamp.to_nsec():
                raise ValueError('Optimized key timestamp differs from original')
        return selected

    def missing_originals(self, nodes):
        return [int(node.key) for node in nodes if int(node.key) not in self.nodes]

    def contiguous(self, nodes):
        selected = self.optimized_structure(nodes)
        if self.missing_originals(selected):
            raise ValueError('Optimized key has no native local original')
        return selected

    def require_odometry_through(self, last_key):
        if set(range(last_key+1))-set(self.nodes):
            raise ValueError('Original native body graph is missing key nodes')
        if any((key, key+1) not in self.odometry for key in range(last_key)):
            raise ValueError('Original native body graph is missing an ODOM edge; refusing to invent a connection')


class RoundLedger:
    """Observed REQUEST -> exact GNC weights -> TERMINATE, never mapper status."""
    def __init__(self, updates=25):
        self.updates = updates; self.serial = 0; self.active = None; self.completed = None
        self.latest_weights = {}; self.path_event = None; self.last_path_sha = None
        self.online_completed = None; self.online_path_event = None; self.online_weights = None

    def command(self, owner, message, now):
        if int(message.publishing_robot) != owner:
            raise ValueError('DPGO command owner differs from native publisher')
        if message.command == message.REQUEST_POSE_GRAPH:
            self.serial += 1
            self.active = dict(serial=self.serial, owner=owner, cluster_id=int(message.cluster_id),
                active_robots=list(message.active_robots), request_wall=now, weight_update_walls=[],
                previous_dpgo_path_sha256=self.last_path_sha)
            self.path_event = None
        elif self.active is not None and owner == self.active['owner']:
            if message.command == message.UPDATE_WEIGHT:
                self.active['weight_update_walls'].append(now)
            elif message.command == message.HARD_TERMINATE:
                self.active = None; self.completed = None; self.path_event = None
                self.online_completed = None; self.online_path_event = None; self.online_weights = None
            elif message.command == message.TERMINATE:
                # Production does not require observing every GNC update. A
                # native successful termination remains distinct from a hard
                # termination, an iterate, or our own diagnostic update count.
                self.online_completed = dict(self.active, terminate_wall=now)
                self.online_path_event = None
                weight = self.latest_weights.get(owner)
                self.online_weights = (copy.deepcopy(weight) if weight is not None
                    and weight['received_wall'] >= self.active['request_wall'] else None)
                if len(self.active['weight_update_walls']) == self.updates:
                    self.completed = dict(self.active, terminate_wall=now)
                else:
                    self.completed = None
                self.active = None

    def weights(self, owner, message, now):
        arrays = [message.src_robot_ids, message.src_pose_ids, message.dst_robot_ids,
                  message.dst_pose_ids, message.weights, message.fixed_weights]
        if len(set(map(len, arrays))) != 1: raise ValueError('DPGO weight array lengths differ')
        rows = [list(row) for row in zip(*arrays)]
        if not all(np.isfinite(row[4]) for row in rows): raise ValueError('Nonfinite native weight')
        self.latest_weights[owner] = dict(owner=owner, received_wall=now, rows=rows,
            sha256=hashlib.sha256(json.dumps(rows, separators=(',', ':')).encode()).hexdigest())
        if (self.active is None and self.online_completed is not None
                and owner == self.online_completed['owner'] and now >= self.online_completed['request_wall']):
            self.online_weights = copy.deepcopy(self.latest_weights[owner])

    def native_path(self, message, now):
        signature = path_geometry_digest(message)
        self.last_path_sha = signature
        if self.online_completed is not None and now > self.online_completed['terminate_wall']:
            self.online_path_event = dict(received_wall=now, sha256=signature,
                poses=len(message.poses), round_serial=self.online_completed['serial'])
        if self.completed is None or self.active is not None: return
        if now <= self.completed['terminate_wall']: return
        # With publish_iterate=false, native PGOAgentROS writes mCachedPoses
        # only in TERMINATE. A numerically unchanged completed result remains
        # valid; cached visualization repeats are annotated, not invented
        # generations. The messages still do not expose an atomic round ID.
        self.path_event = dict(received_wall=now, sha256=signature, poses=len(message.poses),
            round_serial=self.completed['serial'], numerical_change_from_previous_cached_path=
            signature != self.completed['previous_dpgo_path_sha256'])

    def proof(self, now, require_cross):
        if self.active is not None or self.completed is None or self.path_event is None: return None
        if self.path_event['round_serial'] != self.completed['serial']: return None
        if not self.completed['terminate_wall'] < self.path_event['received_wall'] <= now: return None
        weight = self.latest_weights.get(self.completed['owner'])
        if weight is None or not self.completed['terminate_wall'] <= weight['received_wall'] <= now: return None
        accepted = [r for r in weight['rows'] if r[0] != r[2] and r[4] >= .5]
        if require_cross and not accepted: return None
        return dict(round=copy.deepcopy(self.completed), upstream_dpgo_path=dict(self.path_event),
                    weights=copy.deepcopy(weight), accepted_cross_constraints=len(accepted),
                    binding='observed_native_command_path_graph_window_no_shared_message_round_id')

    def online_proof(self, now, require_cross):
        completed, path = self.online_completed, self.online_path_event
        if completed is None or path is None or path['round_serial'] != completed['serial']:
            return None
        if not completed['terminate_wall'] < path['received_wall'] <= now: return None
        weight = self.online_weights
        # Native weights may be delivered before TERMINATE. This is an observed
        # request/termination window, not a +/- wall-time or all-updates gate.
        if weight is None or not completed['request_wall'] <= weight['received_wall'] <= now:
            return None
        active = set(completed['active_robots'])
        accepted = [r for r in weight['rows'] if r[0] != r[2] and r[4] >= .5
                    and r[0] in active and r[2] in active]
        if require_cross and not accepted: return None
        return dict(round=copy.deepcopy(completed), upstream_dpgo_path=dict(path),
            weights=copy.deepcopy(weight), accepted_cross_constraints=len(accepted),
            observed_weight_update_count=len(completed['weight_update_walls']),
            diagnostic_complete_weight_update_count=len(completed['weight_update_walls']) == self.updates,
            binding='observed_native_termination_cached_path_and_weights_no_shared_atomic_round_id',
            exact_shared_optimizer_round_id_available=False)


class PeerControlIngress:
    """Authenticate proxied native controls by exact per-message provenance.

    Receipt and native ROS topics are independently ordered. Keep a bounded
    pending cache rather than losing a unique TERMINATE when its receipt lags.
    A proxy caller alone never authorizes a native control event.
    """
    def __init__(self, robot_id, run_id, cutoff_ns, caller, maximum_pending=256, maximum_wait=5.):
        if not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*', caller):
            raise ValueError('Remote bridge caller must be one absolute ROS node name')
        self.robot_id, self.peer = robot_id, 1-robot_id
        self.run_id, self.cutoff_ns, self.caller = run_id, cutoff_ns, caller
        self.maximum_pending, self.maximum_wait = maximum_pending, maximum_wait
        self.status = None; self.receipts = {}; self.pending = deque(); self.lock = threading.Lock()
        self.overdue = False; self.discarded_previous_boot_controls = 0
        self.atomic_pending = deque(); self.atomic_sequence = 0; self.atomic_last_sha = None
        self.atomic_delivered = 0; self.atomic_duplicates = 0

    def set_status(self, value):
        if (value.get('run_id') != self.run_id or value.get('local_robot_id', value.get('robot_id')) != self.robot_id
                or value.get('peer_robot_id') != self.peer or value.get('cutoff_ns') != self.cutoff_ns):
            raise ValueError('Peer bridge status epoch/ownership mismatch')
        with self.lock:
            if self.status and self.status.get('peer_boot_id') and value.get('peer_boot_id') and (self.status.get('local_boot_id'), self.status.get('peer_boot_id')) != (
                    value.get('local_boot_id'), value.get('peer_boot_id')):
                self.receipts.clear()
                # Do not silently carry unproven controls across a peer restart.
                self.discarded_previous_boot_controls += len(self.pending); self.pending.clear()
                if self.atomic_sequence:
                    raise ValueError('Authenticated native control peer boot changed; use a fresh experiment run')
            self.status = value
            for receipt in value.get('recent_ingress', []): self._receipt(receipt)

    def _receipt(self, value):
        if value.get('topic') not in ('/sim_%d/dpgo_ros_node/command' % self.peer,
                                      '/sim_%d/dpgo_ros_node/measurement_weights' % self.peer):
            return
        if (value.get('run_id') != self.run_id or value.get('cutoff_ns') != self.cutoff_ns
                or value.get('local_robot_id') != self.robot_id or value.get('peer_robot_id') != self.peer
                or value.get('verified') is not True or value.get('proxy_caller') != self.caller
                or value.get('source_robot_id') != self.peer
                or value.get('source_caller') != '/sim_%d/dpgo_ros_node/agent' % self.peer
                or not re.fullmatch(r'[0-9a-f]{64}', str(value.get('sha256', '')))):
            raise ValueError('Untrusted remote native control provenance')
        key = (value['topic'], value['sha256'])
        self.receipts[key] = dict(value)
        while len(self.receipts) > self.maximum_pending*2:
            del self.receipts[next(iter(self.receipts))]

    def receipt(self, value):
        with self.lock: self._receipt(value)

    def enqueue(self, topic, sha256, callback, message, received_wall, received_monotonic, with_receive_wall):
        with self.lock:
            if len(self.pending) >= self.maximum_pending:
                raise RuntimeError('Remote native control authentication queue overflow: ' +
                    json.dumps(self._diagnostic(received_monotonic), sort_keys=True))
            self.pending.append((topic, sha256, callback, message, received_wall, received_monotonic, with_receive_wall))

    def atomic(self, receipt, payload, message, callback, wall, monotonic):
        """Payload and provenance are indivisible on the production ROS topic."""
        from peer_wire import canonical_payload_sha256
        if receipt.get('kind') != 'native_control': raise ValueError('Wrong atomic native control envelope')
        if receipt.get('wire_sha256') != hashlib.sha256(payload).hexdigest():
            raise ValueError('Atomic native control original bytes failed SHA256')
        if receipt.get('sha256') != canonical_payload_sha256(payload, receipt.get('has_header') is True):
            raise ValueError('Atomic native control canonical bytes failed SHA256')
        if getattr(message, '_type', None) != receipt.get('type') or not isinstance(receipt.get('type'), str):
            raise ValueError('Atomic native control ROS type mismatch')
        if (getattr(message, '_md5sum', None) != receipt.get('md5')
                or not re.fullmatch(r'[0-9a-f]{32}', str(receipt.get('md5', '')))):
            raise ValueError('Atomic native control ROS schema mismatch')
        slots = getattr(message, '_slot_types', ())
        has_header = bool(getattr(message, '_has_header', False) or (slots and slots[0] == 'std_msgs/Header'))
        if type(receipt.get('has_header')) is not bool or receipt['has_header'] != has_header:
            raise ValueError('Atomic native control Header schema mismatch')
        sequence = receipt.get('control_sequence')
        if type(sequence) is not int or sequence < 1: raise ValueError('Invalid atomic control sequence')
        with self.lock:
            if receipt.get('topic') not in ('/sim_%d/dpgo_ros_node/command' % self.peer,
                                            '/sim_%d/dpgo_ros_node/measurement_weights' % self.peer):
                raise ValueError('Unexpected atomic native control topic')
            self._receipt(receipt)  # Same strict run/source/proxy/robot authentication.
            if self.status and self.status.get('peer_boot_id') and (
                    receipt.get('local_boot_id') != self.status.get('local_boot_id') or
                    receipt.get('source_boot_id') != self.status['peer_boot_id']):
                raise ValueError('Atomic native control boot identity mismatch')
            if sequence == self.atomic_sequence:
                if self.atomic_last_sha != receipt['sha256']: raise ValueError('Atomic sequence reused for changed native bytes')
                self.atomic_duplicates += 1; return []
            if sequence != self.atomic_sequence+1:
                raise ValueError('Atomic control sequence gap: expected %d got %d' % (self.atomic_sequence+1, sequence))
            self.atomic_sequence = sequence; self.atomic_last_sha = receipt['sha256']
            self.atomic_pending.append((receipt, callback, message, wall, monotonic))
            if len(self.atomic_pending) > self.maximum_pending:
                raise RuntimeError('Atomic native control awaiting boot identity overflow: '+json.dumps(self._diagnostic(monotonic)))
            return self._drain_atomic()

    def _drain_atomic(self):
        events = []; status = self.status or {}
        while self.atomic_pending:
            receipt, callback, message, wall, monotonic = self.atomic_pending[0]
            # Per-message authenticated session/boots establish provenance;
            # aggregate feature/ego lane health is reported independently and
            # cannot invalidate an already authenticated control packet.
            if not status.get('local_boot_id') or not status.get('peer_boot_id'): break
            if (receipt.get('local_boot_id') != status['local_boot_id'] or
                    receipt.get('source_boot_id') != status['peer_boot_id']):
                raise ValueError('Atomic native control boot identity mismatch')
            self.atomic_pending.popleft(); self.atomic_delivered += 1
            events.append((callback, message, wall, monotonic, True, 'control'))
        return events

    def _diagnostic(self, now):
        head = self.pending[0] if self.pending else None
        status = self.status or {}; receipt = self.receipts.get((head[0], head[1])) if head else None
        reason = ('no_pending' if head is None else 'missing_receipt' if receipt is None
                  else 'session_not_ready' if status.get('session_ready') is not True
                  else 'missing_boot_identity' if not status.get('peer_boot_id') or not status.get('local_boot_id')
                  else 'source_boot_mismatch' if receipt.get('source_boot_id') != status['peer_boot_id'] else 'matched')
        return dict(pending=len(self.pending), head_topic=head[0] if head else None,
            head_sha256=head[1] if head else None, head_age_s=max(0., now-head[5]) if head else 0.,
            receipt_present=receipt is not None, reason=reason, receipt_cache_size=len(self.receipts),
            atomic_pending=len(self.atomic_pending), atomic_last_sequence=self.atomic_sequence,
            atomic_delivered=self.atomic_delivered, atomic_duplicates=self.atomic_duplicates,
            atomic_head_topic=self.atomic_pending[0][0].get('topic') if self.atomic_pending else None,
            atomic_head_sha256=self.atomic_pending[0][0].get('sha256') if self.atomic_pending else None,
            atomic_head_age_s=max(0., now-self.atomic_pending[0][4]) if self.atomic_pending else 0.,
            atomic_reason='awaiting_initial_boot_identity' if self.atomic_pending else 'all_received_controls_authenticated')

    def diagnostic(self, now):
        with self.lock: return self._diagnostic(now)

    def drain(self, now):
        authenticated = []
        with self.lock:
            authenticated.extend(self._drain_atomic())
            while self.pending:
                topic, sha, callback, message, wall, monotonic, with_wall = self.pending[0]
                receipt = self.receipts.get((topic, sha)); status = self.status or {}
                ready = (receipt is not None and status.get('session_ready') is True
                         and bool(status.get('peer_boot_id')) and bool(status.get('local_boot_id'))
                         and receipt.get('source_boot_id') == status['peer_boot_id'])
                if not ready:
                    # A missing/late receipt is a waiting condition, not a
                    # reason to kill local LMO or lose a unique TERMINATE. The
                    # queue remains bounded; actual overflow still fails closed.
                    self.overdue = now-monotonic > self.maximum_wait
                    break
                self.pending.popleft()
                self.overdue = False
                if getattr(message, '_type', None) is not None and receipt.get('type') != message._type:
                    raise ValueError('Peer receipt/native control ROS type mismatch')
                if getattr(message, '_md5sum', None) is not None and receipt.get('md5') != message._md5sum:
                    raise ValueError('Peer receipt/native control ROS schema mismatch')
                authenticated.append((callback, message, wall, monotonic, with_wall, 'control'))
        return authenticated


def validate_mesh(message, cutoff_ns, expected_frame):
    if frame(message.header.frame_id) != frame(expected_frame): raise ValueError('Native mesh frame mismatch')
    count = len(message.vertices)
    if not count or not message.triangles: raise ValueError('Native mesh has no vertices or triangles')
    if len(message.vertex_stamps) != count: raise ValueError('Native mesh missing per-vertex timestamps')
    if message.vertex_indices and len(message.vertex_indices) != count:
        raise ValueError('Native mesh graph-index correspondence is incomplete')
    xyz = np.asarray([[p.x, p.y, p.z] for p in message.vertices], dtype='<f4')
    if not np.isfinite(xyz).all(): raise ValueError('Nonfinite native mesh vertex')
    for stamp in message.vertex_stamps: require_stamp(stamp, cutoff_ns)
    for triangle in message.triangles:
        indices = list(triangle.vertex_indices)
        if len(indices) != 3 or min(indices) < 0 or max(indices) >= count:
            raise ValueError('Native mesh triangle index outside vertex array')
    return xyz


def retag_local_mesh(message, robot_id):
    """Only the known local frontend's missing robot_id-parser metadata bug."""
    if message.ns not in ('0', str(robot_id)):
        raise ValueError('Unexpected ownership namespace from native local mesh frontend')
    # Geometry is an immutable received snapshot. Only the scalar ownership
    # field changes here; retained publications own their outer header before
    # rospy can mutate it, so sharing geometry avoids a full object walk.
    output = copy.copy(message)
    output.ns = str(robot_id)
    return output


def same_mesh_topology_and_stamps(first, second):
    return (len(first.vertices) == len(second.vertices) and
        [list(t.vertex_indices) for t in first.triangles] == [list(t.vertex_indices) for t in second.triangles] and
        [s.to_nsec() for s in first.vertex_stamps] == [s.to_nsec() for s in second.vertex_stamps])


def native_readiness_query(last_native_graph, original_key_zero):
    """No edge measurements and no new key: native PGMO path publication only."""
    if int(original_key_zero.key) != 0: raise ValueError('Readiness query requires actual native key zero')
    output = copy.deepcopy(last_native_graph)
    output.nodes = [copy.deepcopy(original_key_zero)]; output.edges = []
    return output


def optimized_camera_to_body(camera_pose):
    return pose_matrix(camera_pose) @ SIM_T_CAMERA_BODY


def diagnostic_path_error(actual, expected):
    """Compatibility metric for the explicit fixed-mesh diagnostic gates."""
    if len(actual.poses) < len(expected.poses): return None
    translation, rotation = 0., 0.
    for measured, target in zip(actual.poses, expected.poses):
        delta = np.linalg.inv(pose_matrix(target.pose)) @ pose_matrix(measured.pose)
        translation = max(translation, float(np.linalg.norm(delta[:3, 3])))
        rotation = max(rotation, float(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1., 1.))))
    return dict(max_translation_m=translation, max_rotation_rad=rotation)


def pose_value_arrays(poses):
    """Copy and validate native values; never mutate the ROS pose objects."""
    poses = list(poses)
    if not poses: return np.empty((0, 3)), np.empty((0, 4))
    positions = np.asarray([(p.position.x, p.position.y, p.position.z) for p in poses], dtype=float)
    quaternions = np.asarray([(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                              for p in poses], dtype=float)
    norms = np.linalg.norm(quaternions, axis=1)
    if (not np.isfinite(positions).all() or not np.isfinite(quaternions).all()
            or np.any(np.abs(norms-1.) > 1e-3)):
        raise ValueError('Native graph contains a nonfinite pose or nonunit quaternion')
    return positions, quaternions/norms[:, None]


def validate_pose_values(poses):
    """Validate a native pose array without allocating N unused SE(3) matrices."""
    pose_value_arrays(poses)


def path_error(actual, expected):
    """Batched SE(3) residuals, with no per-pose matrix construction/inversion.

    For T_target^-1 T_actual the translation norm equals ||p_actual-p_target||
    because the target rotation is orthogonal. The relative quaternion's vector
    norm and absolute scalar recover the shortest rotation angle with atan2,
    including stable identity, antipodal and pi rotations. This is an online
    observation metric; the fixed-mesh diagnostic retains its previous metric.
    """
    count = len(expected.poses)
    if len(actual.poses) < count: return None
    if not count: return dict(max_translation_m=0., max_rotation_rad=0.)
    measured, qm = pose_value_arrays(item.pose for item in actual.poses[:count])
    target, qt = pose_value_arrays(item.pose for item in expected.poses)
    translation = np.linalg.norm(measured-target, axis=1)
    vector = qt[:, 3, None]*qm[:, :3]-qm[:, 3, None]*qt[:, :3]-np.cross(qt[:, :3], qm[:, :3])
    scalar = np.sum(qt*qm, axis=1)
    rotation = 2*np.arctan2(np.linalg.norm(vector, axis=1), np.abs(scalar))
    return dict(max_translation_m=float(translation.max()), max_rotation_rad=float(rotation.max()))


class NativeDenseBridge:
    def __init__(self, args):
        args.external_sensor_stream = getattr(args, 'external_sensor_stream', False)
        args.execution_mode = getattr(args, 'execution_mode', 'diagnostic')
        args.no_persist = getattr(args, 'no_persist', False)
        args.remote_bridge_caller = getattr(args, 'remote_bridge_caller', None)
        args.remote_control_transport = getattr(args, 'remote_control_transport', 'atomic')
        args.shared_gauge_id = getattr(args, 'shared_gauge_id', args.run_id)
        args.status_period = getattr(args, 'status_period', 1.)
        args.online_legacy_vertices = getattr(args, 'online_legacy_vertices', False)
        if args.no_persist and args.execution_mode != 'online':
            raise ValueError('No-persist requires online execution; fixed-mesh diagnostics archive their evidence')
        self.a = args; self.base = '/'+args.robot_name; self.lock = threading.RLock()
        self.native_input_lock = threading.Lock(); self.native_inputs = deque()
        self.timing_lock = threading.Lock(); self.timings = {}
        self.native_input_max_queue = 0; self.native_input_max_delay = 0.; self.native_control_max_delay = 0.
        self.native_imu_max_delay = 0.
        self.replaceable_lock = threading.Lock(); self.replaceable_inputs = {}; self.replaceable_overwrites = Counter()
        self.optimized_candidates = deque(); self.optimized_candidate_wait = 60.; self.optimized_candidate_limit = 4
        self.missing_original_since = {}
        self.local_frame = args.robot_name+'/vio_odom'; self.optical_frame = args.robot_name+'/native_dense_optical'
        self.output_frame = self.local_frame if args.mode == 'local' else args.global_frame
        self.cutoff_ns = round(args.cutoff*1e9)
        self.gyros = (GyroRotationHistory(round(args.max_gyro_gap*1e9),
            sample_convention=args.gyro_sample_convention) if args.sensor_rotation_interpolation == 'gyro_endpoint' else None)
        self.history = PoseHistory(round(args.max_odom_gap*1e9), gyros=self.gyros)
        self.keys = KeyframeLedger(args.robot_id, self.cutoff_ns); self.rounds = RoundLedger(args.weight_updates)
        self.camera = None; self.pending_images = {}; self.graph_queue = deque(); self.mesh_graph_queue = deque()
        self.first_native_odom = None; self.original_first_pose = None; self.gauge_checked = False
        self.graph_sent = -1; self.last_graph_wall = 0.; self.last_rgbd_ns = 0; self.latest_full_mesh = None
        self.latest_full_mesh_bytes = None; self.latest_full_mesh_sha = None
        self.approved = None; self.pending_optimized = None; self.last_optimized_sha = None
        self.transaction = None; self.pgmo_initialized = False; self.last_forwarded_graph = None
        self.pgmo_graph_stats = None; self.native_body_metadata_ack = False
        self.native_body_metadata_ack_has_optimized_key_zero = False
        self.eligible = None; self.camera_graph_contract = None
        self.accepted_joint_mesh_receipt = None; self.latest_submitted_body_path_sha256 = None
        self.online_submission = None; self.online_path_echo = None; self.online_mesh_sequence = 0
        self.online_verified_submission = None
        self.online_pgmo_process_identity = None
        self.pgmo_graph_largest_key = -1
        self.pgmo_process_probe = None; self.pgmo_process_identity_error = None
        self.online_latest_mesh_gauge = 'local_unanchored'
        self.last_rejected_control_envelope = None
        self.pgmo_body_key_stamps = {}; self.pgmo_verified_through = -1; self.online_pending_pgmo_path = None
        self.last_status_wall = 0.
        self.peer_controls = (PeerControlIngress(args.robot_id, args.run_id, self.cutoff_ns,
            args.remote_bridge_caller) if args.remote_bridge_caller else None)
        if self.peer_controls:
            protocol_folder = Path(__file__).resolve().parents[2]/'three_host_20261102'/'distributed'
            if str(protocol_folder) not in sys.path: sys.path.insert(0, str(protocol_folder))
            from peer_wire import native_message_sha256
            self.remote_control_digest = native_message_sha256
            self.remote_control_types = {
                '/sim_%d/dpgo_ros_node/command' % (1-args.robot_id): (Command, lambda m, t: self.rounds.command(1-args.robot_id, m, t)),
                '/sim_%d/dpgo_ros_node/measurement_weights' % (1-args.robot_id): (RelativeMeasurementWeights, lambda m, t: self.rounds.weights(1-args.robot_id, m, t))}
        self.remesh_after = None; self.fatal = None; self.counts = Counter(); self.evidence = args.output_directory
        self.evidence.mkdir(parents=True, exist_ok=True)
        self.events = (self.evidence/'bridge_events.jsonl').open('a', buffering=1)
        self.tf = tf2_ros.TransformBroadcaster()
        topics = {'points': ('rgbd_points', PointCloud2, 1, False),
            'graph': ('pose_graph_incremental', PoseGraph, 1000, False),
            'mesh_graph': ('mesh_graph_incremental', PoseGraph, 1000, False),
            'full': ('full_mesh', KimeraPgmoMesh, 1, False),
            'path': ('approved_body_path', RosPath, 1, True),
            'verified_path': ('verified_body_path', RosPath, 1, True),
            'native_mesh': ('native_optimized_mesh', KimeraPgmoMesh, 1, True),
            'accepted_mesh': ('accepted_joint_mesh', KimeraPgmoMesh, 1, True),
            'accepted_vertices': ('accepted_joint_vertices', PointCloud2, 1, True),
            'raw_local_vertices': ('raw_local_vertices', PointCloud2, 1, True),
            'vertices': ('display_vertices', PointCloud2, 1, True),
            'status': ('status', String, 1, True)}
        if args.external_sensor_stream: del topics['points']
        if not self.legacy_vertices_enabled():
            for key in ('vertices', 'accepted_vertices', 'raw_local_vertices'): del topics[key]
        self.pub = {k: rospy.Publisher(self.base+'/native_dense/'+v[0], v[1],
                    queue_size=None if self.is_online() and k in ('graph', 'mesh_graph') else v[2], latch=v[3])
                    for k, v in topics.items()}
        self.subs = []
        native = args.replay_caller if args.recorded_native_bag else self.base+'/kimera_vio_ros/kimera_vio_ros_node'
        # These callbacks retain every authenticated original immediately, before
        # contending with expensive mesh serialization/validation under lock.
        # None disables rospy's per-receive-batch queue truncation; application
        # overflow is explicit failure, never silently discarded native keys.
        self.subscribe(self.base+'/kimera_vio_ros/odometry', Odometry, self.on_odom, native, None, priority=True)
        self.subscribe(self.base+'/kimera_vio_ros/pose_graph_incremental', PoseGraph, self.on_graph, native, None, priority=True)
        sensor = args.replay_caller if args.recorded_native_bag else '/'+args.robot_name+'_multimodal_sensors'
        if not args.external_sensor_stream:
            if self.gyros is not None:
                self.subscribe(self.base+'/kimera/imu', Imu, self.on_imu, sensor, None, priority='imu')
            self.subscribe(self.base+'/kimera/rgb/camera_info', CameraInfo, self.on_camera, sensor, 2)
            self.subscribe(self.base+'/kimera/rgb/image_raw', Image, lambda m: self.on_image('rgb', m), sensor, 20)
            self.subscribe(self.base+'/kimera/depth/image_raw', Image, lambda m: self.on_image('depth', m), sensor, 20)
        frontend = self.base+'/native_mesh_frontend'
        self.subscribe(frontend+'/mesh_graph_incremental', PoseGraph, self.on_mesh_graph, frontend, 1000)
        self.subscribe(frontend+'/full_mesh', KimeraPgmoMesh, self.on_full_mesh, frontend, 1, replaceable=True)
        self.subscribe(self.base+'/native_pgmo/optimized_mesh', KimeraPgmoMesh, self.on_optimized_mesh,
                       self.base+'/native_pgmo', 1, replaceable=True)
        self.subscribe(self.base+'/native_pgmo/optimized_path', RosPath, self.on_pgmo_path,
                       self.base+'/native_pgmo', None, priority='control')
        self.subscribe(self.base+'/native_pgmo/pose_graph', PoseGraph, self.on_pgmo_graph,
                       self.base+'/native_pgmo', None, priority='control')
        if args.mode == 'joint':
            self.subscribe(self.base+'/kimera_camera_adapter/diagnostics', String, self.on_camera_graph_contract,
                '/'+args.robot_name+'_native_vio_camera_graph_adapter', 2)
            for robot in (0, 1):
                owner = '/sim_%d/dpgo_ros_node/agent' % robot
                remote = bool(self.peer_controls and robot != args.robot_id)
                if remote and args.remote_control_transport == 'atomic': continue
                caller = args.remote_bridge_caller if remote else owner
                self.subscribe('/sim_%d/dpgo_ros_node/command' % robot, Command,
                    lambda m, t, r=robot: self.rounds.command(r, m, t), caller, None,
                    priority='control', with_receive_wall=True, remote_control=remote)
                self.subscribe('/sim_%d/dpgo_ros_node/measurement_weights' % robot, RelativeMeasurementWeights,
                    lambda m, t, r=robot: self.rounds.weights(r, m, t), caller, None,
                    priority='control', with_receive_wall=True, remote_control=remote)
            if self.peer_controls:
                self.subscribe(self.base+'/peer_bridge/status', String, self.on_peer_status,
                    args.remote_bridge_caller, 2, independent=True)
                if args.remote_control_transport == 'atomic':
                    self.subscribe(self.base+'/peer_bridge/native_control', UInt8MultiArray, self.on_peer_native_control,
                        args.remote_bridge_caller, None, independent=True, with_receive_wall=True)
                else:
                    self.subscribe(self.base+'/peer_bridge/ingress_receipt', String, self.on_peer_receipt,
                        args.remote_bridge_caller, None, independent=True)
            self.subscribe(self.base+'/dpgo_ros_node/path', RosPath,
                lambda m, t: self.rounds.native_path(m, t), self.base+'/dpgo_ros_node/agent', None,
                priority='control', with_receive_wall=True)
            self.subscribe(self.base+'/kimera_distributed/optimized_nodes', PoseGraph, self.on_optimized_graph,
                self.base+'/distributed_loop_closure/distributed_loop_closure_node', None,
                priority='control', with_receive_wall=True)
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)
        if self.is_online() and args.mode == 'joint':
            self.pgmo_process_probe = NativeProcessProbe(lambda: self.timed('ros_pgmo_process_identity',
                native_node_identity, os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                self.base+'/native_pgmo', rospy.get_name()))
            rospy.on_shutdown(self.pgmo_process_probe.close)

    def is_online(self):
        return getattr(getattr(self, 'a', None), 'execution_mode', 'diagnostic') == 'online'

    def legacy_vertices_enabled(self):
        return not self.is_online() or getattr(self.a, 'online_legacy_vertices', False)

    @staticmethod
    def shared_online_proof(proof):
        return (isinstance(proof, dict) and set(proof.get('round', {}).get('active_robots', [])) == {0, 1}
            and proof.get('accepted_cross_constraints', 0) > 0
            and proof.get('round', {}).get('terminate_wall') is not None)

    def online_gauge_epoch(self):
        peer_boots = None
        if self.peer_controls:
            with self.peer_controls.lock:
                status = self.peer_controls.status or {}
                local, peer = status.get('local_boot_id'), status.get('peer_boot_id')
            if not local or not peer: raise ValueError('Shared gauge lacks authenticated peer boot identity')
            peer_boots = dict(local=local, peer=peer)
        return dict(run_id=self.a.run_id, cutoff_ns=self.cutoff_ns, robot_id=self.a.robot_id,
            output_frame=self.output_frame, native_key_zero_stamp_ns=self.keys.nodes[0].header.stamp.to_nsec(),
            peer_boots=peer_boots)

    def observe_pgmo_process_identity(self):
        observed = self.pgmo_process_probe.snapshot() if self.pgmo_process_probe else {}
        checked = observed.get('checked_monotonic')
        if observed.get('identity') is None or checked is None or time.monotonic()-checked > 2.:
            self.pgmo_process_identity_error = observed.get('error') or 'native PGMO process identity observation is stale'
            self.counts['online_shared_gauge_waiting_process_identity'] += 1
            raise NativeProcessIdentityUnavailable(self.pgmo_process_identity_error)
        identity = observed['identity']; self.pgmo_process_identity_error = None
        previous = self.online_pgmo_process_identity
        if previous is not None and identity != previous:
            self.invalidate_online_gauge('native_pgmo_process_changed')
            raise ValueError('Native PGMO process changed; shared-gauge continuity requires a fresh experiment run')
        self.online_pgmo_process_identity = identity
        return identity

    def invalidate_online_gauge(self, reason):
        if self.online_verified_submission is not None:
            self.counts['online_shared_gauge_invalidations'] += 1
            self.event('online_shared_gauge_invalidated', reason=reason)
        self.online_verified_submission = None; self.accepted_joint_mesh_receipt = None

    def validate_pgmo_graph_prefix(self, message):
        keys = sorted(int(node.key) for node in message.nodes)
        largest = keys[-1] if keys else -1
        if keys != list(range(largest+1)) or largest < self.pgmo_graph_largest_key:
            self.invalidate_online_gauge('native_pgmo_graph_prefix_regressed_or_reset')
            raise ValueError('Native PGMO graph prefix regressed/reset; use a fresh experiment run')
        self.pgmo_graph_largest_key = largest

    def verified_online_gauge(self):
        verified = self.online_verified_submission
        if verified is None: return None
        if verified['epoch'] != self.online_gauge_epoch():
            self.invalidate_online_gauge('run_key_zero_frame_or_peer_boot_changed')
            raise ValueError('Shared-gauge epoch changed; use a fresh experiment run')
        try: self.observe_pgmo_process_identity()
        except NativeProcessIdentityUnavailable: return None
        return verified

    def timed(self, label, operation, *args):
        if not hasattr(self, 'timing_lock'): return operation(*args)
        began = time.monotonic()
        try: return operation(*args)
        finally:
            elapsed = time.monotonic()-began
            with self.timing_lock:
                value = self.timings.setdefault(label, dict(calls=0, total_s=0., max_s=0., last_s=0.))
                value.update(calls=value['calls']+1, total_s=value['total_s']+elapsed,
                    max_s=max(value['max_s'], elapsed), last_s=elapsed)

    def timing_snapshot(self):
        if not hasattr(self, 'timing_lock'): return {}
        with self.timing_lock: return copy.deepcopy(self.timings)

    def persist_bytes(self, path, data):
        if not getattr(self.a, 'no_persist', False): path.write_bytes(data)

    def persist_json(self, path, value):
        if not getattr(self.a, 'no_persist', False):
            path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')

    def completion_proof(self):
        # A completed local-only solution can still improve its native local
        # output. Cross constraints gate shared-gauge acceptance, not live LMO.
        if self.is_online(): return self.rounds.online_proof(time.time(), False)
        return self.rounds.proof(time.time(), self.a.require_cross)

    def on_peer_status(self, message):
        self.peer_controls.set_status(json.loads(message.data))

    def on_peer_receipt(self, message):
        self.peer_controls.receipt(json.loads(message.data))

    def queue_authenticated_controls(self, events):
        if not events: return
        with self.native_input_lock:
            if len(self.native_inputs)+len(events) > 20000:
                raise RuntimeError('Authenticated native input processing queue overflow: '+
                    json.dumps(self.peer_controls.diagnostic(time.monotonic()), sort_keys=True))
            self.native_inputs.extend(events)
            self.native_input_max_queue = max(self.native_input_max_queue, len(self.native_inputs))

    def on_peer_native_control(self, envelope, received_wall=None):
        from peer_wire import decode_frame_bytes
        received_wall = time.time() if received_wall is None else received_wall
        monotonic = time.monotonic()
        receipt, payload = decode_frame_bytes(bytes(envelope.data))
        topic = receipt.get('topic')
        if topic not in self.remote_control_types: raise ValueError('Unrecognized atomic native control topic')
        kind, callback = self.remote_control_types[topic]
        try:
            message = kind(); message.deserialize(payload)
            events = self.peer_controls.atomic(receipt, payload, message, callback, received_wall, monotonic)
        except Exception as error:
            self.last_rejected_control_envelope = dict(topic=topic, sha256=receipt.get('sha256'),
                control_sequence=receipt.get('control_sequence'), error=str(error))
            raise
        self.queue_authenticated_controls(events)

    def subscribe(self, topic, kind, callback, expected, queue, priority=False, with_receive_wall=False,
                  replaceable=False, remote_control=False, independent=False):
        def receive(message):
            try:
                received_wall, received_monotonic = time.time(), time.monotonic()
                require_source(message, expected)
                if independent:
                    # Authentication metadata and coupled native controls may
                    # never contend with expensive SLAM/mesh state work.
                    if with_receive_wall: self.timed('independent:'+callback.__name__, callback, message, received_wall)
                    else: self.timed('independent:'+callback.__name__, callback, message)
                    return
                if remote_control:
                    self.peer_controls.enqueue(topic, self.remote_control_digest(message), callback, message,
                        received_wall, received_monotonic, with_receive_wall)
                    return
                if priority:
                    if priority is True and callback == self.on_graph:
                        if frame(message.header.frame_id) != self.a.robot_name+'/map':
                            raise ValueError('Unexpected original native graph frame at ingress')
                        for node in message.nodes:
                            if int(node.robot_id) != self.a.robot_id: raise ValueError('Foreign original robot at ingress')
                            require_stamp(node.header.stamp, self.cutoff_ns)
                        for edge in message.edges:
                            if edge.robot_from != self.a.robot_id or edge.robot_to != self.a.robot_id:
                                raise ValueError('Foreign original edge at ingress')
                            require_stamp(edge.header.stamp, self.cutoff_ns)
                    elif priority == 'imu':
                        if frame(message.header.frame_id) != self.a.robot_name+'/imu':
                            raise ValueError('Unexpected actual body gyro frame at ingress')
                        require_stamp(message.header.stamp, self.cutoff_ns)
                        omega = message.angular_velocity
                        if not np.isfinite([omega.x, omega.y, omega.z]).all():
                            raise ValueError('Nonfinite actual gyro at ingress')
                    elif priority is True:
                        if (frame(message.header.frame_id) != self.local_frame or
                                frame(message.child_frame_id) != self.a.robot_name+'/imu'):
                            raise ValueError('Unexpected native body odometry frames at ingress')
                        require_stamp(message.header.stamp, self.cutoff_ns)
                    with self.native_input_lock:
                        if len(self.native_inputs) >= 20000:
                            raise RuntimeError('Lossless native original input queue overflow; no keys discarded')
                        self.native_inputs.append((callback, message, received_wall, received_monotonic,
                            with_receive_wall, 'original' if priority is True else str(priority)))
                        self.native_input_max_queue = max(self.native_input_max_queue, len(self.native_inputs))
                    return
                if replaceable:
                    with self.replaceable_lock:
                        if topic in self.replaceable_inputs: self.replaceable_overwrites[topic] += 1
                        self.replaceable_inputs[topic] = (callback, message)
                    return
                with self.lock:
                    self.drain_native_inputs()
                    if self.fatal is None: callback(message)
            except Exception as error: self.fail(error)
        self.subs.append(rospy.Subscriber(topic, kind, receive, queue_size=queue, buff_size=2**26, tcp_nodelay=True))

    def drain_native_inputs(self):
        # Caller holds the state lock. Detach the finite batch under a separate
        # short lock so native transport callbacks never wait for mesh work.
        with self.native_input_lock:
            batch = list(self.native_inputs); self.native_inputs.clear()
        if getattr(self, 'peer_controls', None):
            batch.extend(self.peer_controls.drain(time.monotonic()))
            batch.sort(key=lambda entry: entry[3])
        for callback, message, received_wall, received_monotonic, with_receive_wall, category in batch:
            delay = time.monotonic()-received_monotonic
            if category == 'original': self.native_input_max_delay = max(self.native_input_max_delay, delay)
            elif category == 'imu': self.native_imu_max_delay = max(self.native_imu_max_delay, delay)
            else: self.native_control_max_delay = max(self.native_control_max_delay, delay)
            if with_receive_wall: self.timed('callback:'+callback.__name__, callback, message, received_wall)
            else: self.timed('callback:'+callback.__name__, callback, message)
            self.counts[{'original': 'priority_native_originals_processed',
                         'imu': 'priority_actual_imu_samples_processed'}.get(category,
                         'priority_native_control_events_processed')] += 1

    def drain_replaceable_inputs(self):
        with self.replaceable_lock:
            batch = list(self.replaceable_inputs.values()); self.replaceable_inputs.clear()
        for callback, message in batch:
            self.drain_native_inputs()
            self.timed('callback:'+callback.__name__, callback, message)

    def event(self, kind, **fields):
        self.events.write(json.dumps(dict(event=kind, run_id=self.a.run_id, wall=time.time(), **fields), allow_nan=False)+'\n')

    def fail(self, error):
        with self.lock:
            if self.fatal is not None: return
            # SIGINT can close rospy publishers while an already-running timer
            # finishes its callback. Only this exact closed-publisher exception
            # during actual ROS shutdown is benign; runtime failures still fail.
            if (str(error) == 'publish() to a closed topic' and
                    getattr(rospy, 'is_shutdown', lambda: False)()):
                return
            self.fatal = str(error)
            provenance = self.peer_controls.diagnostic(time.monotonic()) if getattr(self, 'peer_controls', None) else None
            self.event('fatal', error=str(error), remote_control_provenance=provenance,
                timings=self.timing_snapshot(),
                last_rejected_control_envelope=getattr(self, 'last_rejected_control_envelope', None))
            rospy.logfatal('Native dense bridge: %s', error)
            # Keep the primary cause even if an in-flight timer encounters
            # closed publishers during rospy shutdown.
            self.persist_json(self.evidence/'fatal.json', dict(error=self.fatal, wall=time.time(),
                run_id=self.a.run_id))
        rospy.signal_shutdown(str(error))

    def on_odom(self, message):
        if frame(message.header.frame_id) != self.local_frame or frame(message.child_frame_id) != self.a.robot_name+'/imu':
            raise ValueError('Expected native VIO body odometry frames')
        stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        if self.first_native_odom is None: self.first_native_odom = (stamp, pose_matrix(message.pose.pose))
        self.history.put(stamp, message.pose.pose); self.counts['native_odometry'] += 1

    def on_imu(self, message):
        if frame(message.header.frame_id) != self.a.robot_name+'/imu':
            raise ValueError('Expected authenticated actual body gyro frame')
        stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        omega = message.angular_velocity
        self.gyros.put(stamp, [omega.x, omega.y, omega.z])
        self.counts['actual_gyro_samples'] += 1

    def on_graph(self, message):
        if frame(message.header.frame_id) != self.a.robot_name+'/map':
            raise ValueError('Native original body graph must retain its native sim_N/map frame declaration')
        self.keys.ingest(message)
        for node in message.nodes: self.missing_original_since.pop(int(node.key), None)
        if not self.graph_queue and self.graph_sent < 0:
            if 0 not in self.keys.nodes: raise ValueError('Start dense bridge before native key zero')
            self.original_first_pose = pose_matrix(self.keys.nodes[0].pose)
        self.graph_queue.append(copy.deepcopy(message)); self.counts['native_graphs'] += 1
        if len(self.graph_queue) > 1000: raise RuntimeError('Lossless native graph queue overflow')

    def on_camera(self, message):
        require_stamp(message.header.stamp, self.cutoff_ns)
        if frame(message.header.frame_id) != self.a.robot_name+'/camera_optical':
            raise ValueError('Unexpected calibrated optical frame')
        if self.camera is not None and (message.K != self.camera.K or message.width != self.camera.width or message.height != self.camera.height):
            raise ValueError('Camera calibration changed within epoch')
        self.camera = message

    def on_image(self, kind, message):
        stamp = require_stamp(message.header.stamp, self.cutoff_ns)
        if frame(message.header.frame_id) != self.a.robot_name+'/camera_optical': raise ValueError('Unexpected image frame')
        if kind == 'depth' and message.encoding != '32FC1': raise ValueError('Depth must be optical-Z 32FC1 metres')
        self.pending_images.setdefault(stamp, {})[kind] = message
        for key in sorted(self.pending_images)[:-100]:
            del self.pending_images[key]; self.counts['unprocessed_rgbd_dropped'] += 1

    def on_mesh_graph(self, message):
        output = copy.deepcopy(message)
        owners = {int(n.robot_id) for n in output.nodes}
        owners.update(int(e.robot_from) for e in output.edges); owners.update(int(e.robot_to) for e in output.edges)
        if owners and owners not in ({0}, {self.a.robot_id}): raise ValueError('Mixed owner graph from local native mesh frontend')
        for node in output.nodes:
            require_stamp(node.header.stamp, self.cutoff_ns); pose_matrix(node.pose); node.robot_id = self.a.robot_id
        for edge in output.edges:
            edge.robot_from = edge.robot_to = self.a.robot_id
        self.mesh_graph_queue.append(output); self.counts['native_mesh_graphs'] += 1
        if len(self.mesh_graph_queue) > 1000: raise RuntimeError('Lossless native mesh graph queue overflow')

    def on_full_mesh(self, message):
        xyz = self.timed('online_full_mesh:validate', validate_mesh, message, self.cutoff_ns, self.local_frame) if self.is_online() else validate_mesh(message, self.cutoff_ns, self.local_frame)
        self.latest_full_mesh = retag_local_mesh(message, self.a.robot_id)
        if self.is_online():
            self.latest_full_mesh_bytes = self.timed('online_full_mesh:serialize', message_bytes, self.latest_full_mesh)
            self.latest_full_mesh_sha = self.timed('online_full_mesh:hash', canonical_serialized_mesh_digest, self.latest_full_mesh_bytes)
        else:
            self.latest_full_mesh_bytes = message_bytes(self.latest_full_mesh)
            self.latest_full_mesh_sha = hashlib.sha256(self.latest_full_mesh_bytes).hexdigest()
        self.counts['native_full_meshes'] += 1
        # The native frontend mesh is still in the VIO integration frame. Keep
        # that geometry observable independently of PGMO's global deformation.
        # Only XYZ extraction and a display ownership color happen here.
        if self.legacy_vertices_enabled():
            color = 0x34D8ED if self.a.robot_id == 0 else 0xFFA04B
            self.pub['raw_local_vertices'].publish(
                self.make_cloud(xyz, color, message.header.stamp, self.local_frame))
            self.counts['raw_local_mesh_displays'] += 1

    def on_optimized_graph(self, message, received_wall=None):
        if self.transaction is not None:
            self.counts['optimized_graphs_retained_during_mesh_transaction'] += 1
        signature = canonical_graph_digest(message) if self.is_online() else digest(message)
        proof = self.completion_proof()
        previous = getattr(self, 'approved', None)
        same_completion = (proof is None or previous is None or
            (proof['round']['owner'], proof['round']['serial']) ==
            (previous['proof']['round']['owner'], previous['proof']['round']['serial']))
        if ((signature == getattr(self, 'last_optimized_sha', None) and same_completion) or
                any(candidate['signature'] == signature for candidate in getattr(self, 'optimized_candidates', []))):
            self.counts['optimized_graph_timer_duplicates_suppressed'] += 1
            return
        nodes = self.keys.optimized_structure(message.nodes)
        if not nodes: return
        if self.camera_graph_contract is None:
            self.counts['optimized_graphs_without_verified_camera_graph_contract'] += 1; return
        code, _, states = self.timed('ros_get_system_state', lambda: rospy.get_master().getSystemState())
        subscribers = dict(states[1]) if code == 1 else {}
        if self.base+'/distributed_loop_closure/distributed_loop_closure_node' not in subscribers.get(
                self.base+'/kimera_camera_adapter/pose_graph_incremental', []):
            raise ValueError('Native LCD is not subscribed to the verified camera-convention graph')
        if self.is_online() and self.base+'/distributed_loop_closure/distributed_loop_closure_node' not in subscribers.get(
                self.base+'/dpgo_ros_node/path', []):
            raise ValueError('Native LCD is not subscribed to the completed native DPGO path')
        if proof is not None and self.a.robot_id not in proof['round']['active_robots']:
            self.counts['optimized_graphs_outside_active_cluster'] += 1; return
        if self.timed('ros_get_publish_iterate', lambda: rospy.get_param(self.base+'/dpgo_ros_node/agent/publish_iterate', None)) is not False:
            raise ValueError('Joint dense mode requires native DPGO publish_iterate=false')
        if len(self.optimized_candidates) >= self.optimized_candidate_limit:
            old = self.optimized_candidates.popleft(); self.counts['optimized_candidates_superseded'] += 1
            self.event('optimized_candidate_superseded', graph_sha256=old['signature'])
        self.optimized_candidates.append(dict(nodes=copy.deepcopy(nodes), proof=proof,
            signature=signature, raw=message_bytes(message), received_wall=received_wall or time.time(),
            received_monotonic=time.monotonic(), last_missing=None))
        if proof is None: self.counts['optimized_graphs_waiting_round_receipt'] += 1
        self.advance_optimized_candidates()

    def advance_optimized_candidates(self):
        if self.transaction is not None and not self.is_online(): return
        now = time.time(); current = self.completion_proof()
        retained = deque()
        while self.optimized_candidates:
            candidate = self.optimized_candidates.popleft(); nodes = candidate['nodes']
            if candidate['proof'] is None:
                if current is None:
                    if time.monotonic()-candidate['received_monotonic'] <= self.optimized_candidate_wait:
                        retained.append(candidate)
                    else: self.counts['optimized_candidates_expired_missing_round_receipt'] += 1
                    continue
                # Native cached poses are only committed by TERMINATE. Retain
                # independent-topic deliveries until the observed round ledger
                # catches up, while rejecting genuinely earlier stale graphs.
                if not self.is_online() and candidate['received_wall'] < current['round']['terminate_wall']-1.:
                    self.counts['optimized_candidates_predating_completed_round'] += 1; continue
                candidate['proof'] = current
            if (not self.is_online() and (current is None or
                    current['round']['serial'] != candidate['proof']['round']['serial'] or
                    current['round']['owner'] != candidate['proof']['round']['owner'])):
                self.counts['optimized_candidates_obsolete_round'] += 1; continue
            if self.a.robot_id not in candidate['proof']['round']['active_robots']:
                self.counts['optimized_candidates_outside_active_cluster'] += 1; continue
            missing = self.keys.missing_originals(nodes)
            monotonic_now = time.monotonic()
            for key in missing: self.missing_original_since.setdefault(key, monotonic_now)
            missing_wait = max((monotonic_now-self.missing_original_since[key] for key in missing), default=0.)
            if missing and missing_wait > self.optimized_candidate_wait:
                self.counts['optimized_candidates_expired_missing_originals'] += 1
                self.event('optimized_candidate_missing_originals_expired', graph_sha256=candidate['signature'],
                    wait_s=missing_wait, missing_count=len(missing),
                    first_missing_key=missing[0], local_key_count=len(self.keys.nodes))
                continue
            if missing:
                if candidate['last_missing'] is None:
                    self.counts['optimized_candidates_waiting_originals'] += 1
                    self.event('optimized_candidate_waiting_originals', graph_sha256=candidate['signature'],
                        missing_count=len(missing), first_missing_key=missing[0], local_key_count=len(self.keys.nodes))
                candidate['last_missing'] = len(missing); retained.append(candidate); continue
            # Revalidate every exact native stamp after the original connection
            # catches up. Missing data is never fabricated, extrapolated or fit.
            self.keys.contiguous(nodes)
            if self.camera_graph_contract.get('native_key_zero_stamp_ns') != self.keys.nodes[0].header.stamp.to_nsec():
                raise ValueError('Camera graph adapter native key-zero epoch differs from dense original graph')
            self.pending_optimized = (nodes, candidate['proof'], candidate['signature'], candidate['raw'])
            self.counts['optimized_candidates_originals_verified'] += 1
        self.optimized_candidates = retained

    def on_camera_graph_contract(self, message):
        report = json.loads(message.data)
        if (report.get('adapter_role') != 'coordinate_transform_of_actual_native_Kimera_VIO_graph' or
                report.get('uses_simulator_ground_truth') is not False or
                report.get('uses_sdk_or_lidar_odometry') is not False or
                report.get('input_topic') != self.base+'/kimera_vio_ros/pose_graph_incremental' or
                report.get('output_topic') != self.base+'/kimera_camera_adapter/pose_graph_incremental' or
                report.get('expected_input_caller') != self.base+'/kimera_vio_ros/kimera_vio_ros_node' or
                report.get('fatal_error') is not None or
                not np.array_equal(np.asarray(report.get('T_body_camera')), SIM_T_BODY_CAMERA)):
            raise ValueError('Joint camera/body graph convention has not been verified against native adapter')
        self.camera_graph_contract = report

    def make_cloud(self, xyz, colors, stamp, cloud_frame):
        data = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        if len(xyz): data['x'], data['y'], data['z'] = xyz.T
        data['rgb'] = colors
        fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32 if i<3 else PointField.UINT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
        return PointCloud2(header=Header(stamp=stamp, frame_id=cloud_frame), height=1, width=len(xyz),
            fields=fields, is_bigendian=False, point_step=16, row_step=16*len(xyz), data=data.tobytes(), is_dense=True)

    def publish_sensor(self):
        # A PGMO transaction freezes its own graph/full-mesh inputs, not the
        # independent VIO -> TSDF stream. Pausing this producer used to discard
        # camera coverage while waiting for native deformation receipts.
        if getattr(self.a, 'external_sensor_stream', False): return
        if not self.gauge_checked or self.camera is None or not self.pub['points'].get_num_connections(): return
        for stamp in sorted(list(self.pending_images)):
            if stamp <= self.last_rgbd_ns or (stamp-self.last_rgbd_ns)*1e-9 < 1./self.a.cloud_rate:
                del self.pending_images[stamp]; continue
            pair = self.pending_images[stamp]
            if set(pair) != {'rgb', 'depth'}: continue
            try:
                body = self.history.at(stamp)
            except NativeOdometryGap:
                del self.pending_images[stamp]; self.counts['rgbd_omitted_native_odom_gap'] += 1
                continue
            except GyroCoverageError as error:
                # Never replace a missing inertial interval with a constant
                # angular velocity curve. Count an explicit omitted RGB-D frame.
                del self.pending_images[stamp]
                self.counts['rgbd_omitted_actual_gyro_'+error.kind] += 1
                continue
            if body is None:
                if self.history.values and stamp < min(self.history.values):
                    del self.pending_images[stamp]; self.counts['rgbd_omitted_no_native_lower_bracket'] += 1
                continue
            xyz, colors = optical_points(pair['rgb'], pair['depth'], self.camera, self.a.pixel_stride, .3, self.a.max_depth)
            camera = body @ SIM_T_BODY_CAMERA
            pose = copy.deepcopy(self.keys.nodes[0].pose); set_pose(pose, camera)
            tf = TransformStamped(header=Header(stamp=pair['rgb'].header.stamp, frame_id=self.local_frame), child_frame_id=self.optical_frame)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = camera[:3, 3]
            tf.transform.rotation = pose.orientation
            self.tf.sendTransform(tf)
            self.pub['points'].publish(self.make_cloud(xyz, colors, pair['rgb'].header.stamp, self.optical_frame))
            self.last_rgbd_ns = stamp; del self.pending_images[stamp]; self.counts['sensor_clouds'] += 1
            if self.transaction is not None: self.counts['sensor_clouds_during_mesh_transaction'] += 1
            break

    def publish_graphs(self):
        if self.transaction is not None and not self.is_online(): return
        if self.gauge_checked and self.pub['graph'].get_num_connections():
            budget = 32 if self.is_online() else len(self.graph_queue)
            while self.graph_queue and budget:
                budget -= 1
                message = self.graph_queue.popleft(); output = copy.deepcopy(message)
                output.edges = [e for e in output.edges if int(e.type) == int(e.ODOM)]
                self.keys.require_odometry_through(max((int(n.key) for n in output.nodes), default=self.graph_sent))
                self.timed('publish_native_original_graph', self.pub['graph'].publish, output)
                self.last_forwarded_graph = copy.deepcopy(output)
                self.graph_sent = max(self.graph_sent, max((int(n.key) for n in output.nodes), default=-1))
                self.last_graph_wall = time.time(); self.counts['original_body_graphs_forwarded'] += 1
        # Native mesh controls attach immediately to native unconnected body
        # metadata. Its first pose_graph response is sent only after that body
        # callback completed. Once initialized, the native queue retains a final
        # body node and accepts asynchronous increments; moving quiet is neither
        # required nor a receipt that the native callback ran.
        if self.gauge_checked and self.native_body_metadata_ack and self.graph_sent >= 0 and self.pub['mesh_graph'].get_num_connections():
            budget = 32 if self.is_online() else len(self.mesh_graph_queue)
            while self.mesh_graph_queue and budget:
                budget -= 1
                self.timed('publish_native_mesh_graph', self.pub['mesh_graph'].publish, self.mesh_graph_queue.popleft())
                self.counts['mesh_graphs_forwarded'] += 1

    def approve_path(self):
        if self.is_online():
            self.approve_online_path()
            return
        if self.transaction is not None or not self.pgmo_initialized or self.latest_full_mesh is None: return
        if self.pending_optimized is None or self.graph_sent < len(self.pending_optimized[0])-1: return
        if not self.pub['path'].get_num_connections(): return
        nodes, proof, signature, raw = self.pending_optimized
        current = self.rounds.proof(time.time(), self.a.require_cross)
        if current is None or current['round']['serial'] != proof['round']['serial']:
            self.pending_optimized = None; return
        path = RosPath(header=Header(stamp=rospy.Time.now(), frame_id=self.a.global_frame))
        for node in nodes:
            body = optimized_camera_to_body(node.pose)
            value = PoseStamped(header=copy.deepcopy(node.header), pose=copy.deepcopy(node.pose))
            value.header.frame_id = self.a.global_frame; value.header.seq = int(node.key); set_pose(value.pose, body)
            path.poses.append(value)
        index = self.counts['native_mesh_transactions_started']+1
        self.counts['native_mesh_transactions_started'] = index
        prefix = self.evidence/('native_transaction_%04d' % index)
        prefix.with_suffix('.rosbin').write_bytes(raw)
        metadata = dict(graph_sha256=signature, key_count=len(nodes), pose_convention='T_global_camera_to_T_global_body',
            original_key_stamps_ns=[int(n.header.stamp.to_nsec()) for n in nodes], proof=proof,
            path_sha256=digest(path), prepared_wall=time.time(), mesh_round_receipt=False,
            camera_graph_adapter_native_key_zero_stamp_ns=self.camera_graph_contract['native_key_zero_stamp_ns'])
        prefix.with_suffix('.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        # Freeze PGMO graph/full-mesh producers. Establish a native baseline on
        # the exact same fixed mesh before submitting changed priors. The local
        # TSDF stream continues; newer frontend meshes remain in the latest
        # cache and cannot replace the transaction's captured fixed mesh.
        raw_mesh = snapshot_message_header(self.latest_full_mesh)
        self.transaction = dict(phase='await_native_seed', started_wall=time.time(), phase_started_wall=time.time(), path=path,
            metadata=metadata, prefix=prefix, fixed_mesh=raw_mesh,
                    fixed_mesh_sha256=self.latest_full_mesh_sha, last_query_wall=0., echo_receipts=[],
            expected_native_body_keys=self.graph_sent+1, waiting_full_mesh_ack=True,
            last_full_mesh_send_wall=time.time(), stable_baseline_count=0)
        prefix.with_name(prefix.name+'_fixed_input_mesh.rosbin').write_bytes(self.latest_full_mesh_bytes)
        self.last_optimized_sha = signature; self.pending_optimized = None
        publish_preserving_cached_message(self.pub['full'], raw_mesh)
        self.event('native_fixed_mesh_baseline_requested', mesh_sha256=self.latest_full_mesh_sha)

    def approve_online_path(self):
        """Submit a complete native-key path without pausing live PGMO inputs."""
        if (self.pending_optimized is None or not self.gauge_checked or not self.pgmo_initialized
                or not self.native_body_metadata_ack or self.latest_full_mesh is None
                or not self.pub['path'].get_num_connections()): return
        nodes, proof, signature, _raw = self.pending_optimized
        if self.graph_sent < len(nodes)-1: return
        # The input is the original-key expansion produced by native LCD, not
        # the sparse DPGO submap path. Never re-index a submap path into PGMO.
        self.keys.contiguous(nodes)
        active = set(proof['round']['active_robots'])
        if self.a.robot_id not in active: raise ValueError('Native completion excludes this robot')
        if self.a.mode == 'joint' and not self.shared_online_proof(proof):
            # A local-only new path must never replace native shared priors
            # while an earlier shared-gauge proof is retained for continuity.
            self.invalidate_online_gauge('new_path_has_no_completed_shared_gauge_proof')
            self.online_submission = None; self.online_path_echo = None; self.online_pending_pgmo_path = None
            self.approved = None; self.pending_optimized = None
            self.counts['online_paths_rejected_without_shared_gauge_proof'] += 1
            self.event('online_path_not_submitted', reason='joint_path_requires_completed_active_pair_and_cross_constraints')
            return
        epoch = self.online_gauge_epoch() if self.a.mode == 'joint' else None
        if self.online_verified_submission is not None and self.online_verified_submission['epoch'] != epoch:
            self.invalidate_online_gauge('submission_epoch_changed')
            raise ValueError('Shared-gauge submission epoch changed; use a fresh experiment run')
        path = RosPath(header=Header(stamp=rospy.Time.now(), frame_id=self.a.global_frame))
        for node in nodes:
            value = PoseStamped(header=copy.deepcopy(node.header), pose=copy.deepcopy(node.pose))
            value.header.frame_id = self.a.global_frame; value.header.seq = int(node.key)
            set_pose(value.pose, optimized_camera_to_body(node.pose)); path.poses.append(value)
        metadata = dict(graph_sha256=signature, key_count=len(nodes),
            pose_convention='T_global_camera_to_T_global_body',
            original_key_stamps_ns=[int(n.header.stamp.to_nsec()) for n in nodes],
            path_sha256=digest(path), prepared_wall=time.time(), proof=copy.deepcopy(proof),
            original_key_expansion_source=self.base+'/kimera_distributed/optimized_nodes',
            native_publish_iterate=False, exact_shared_optimizer_round_id_available=False,
            gauge_epoch=epoch,
            camera_graph_adapter_native_key_zero_stamp_ns=self.camera_graph_contract['native_key_zero_stamp_ns'])
        self.online_submission = dict(path=path, metadata=metadata)
        self.online_path_echo = None; self.online_pending_pgmo_path = None; self.approved = metadata
        self.latest_submitted_body_path_sha256 = metadata['path_sha256']
        self.timed('publish_native_optimized_body_path', publish_preserving_cached_message, self.pub['path'], path)
        self.last_optimized_sha = signature; self.pending_optimized = None
        # Full meshes continue independently; this flag merely schedules a live
        # latest-mesh callback so native PGMO applies its newly submitted priors.
        self.remesh_after = time.time()
        self.counts['submitted_native_paths'] += 1
        self.event('online_native_body_path_submitted', graph_sha256=signature,
            path_sha256=metadata['path_sha256'], native_key_count=len(nodes))

    def on_online_optimized_mesh(self, message, xyz):
        """Relay native geometry unchanged; attach honest observational evidence."""
        serialized = self.timed('online_mesh:serialize', message_bytes, message)
        self.timed('online_mesh:publish_native', publish_serialized_message, self.pub['native_mesh'], serialized)
        vertices = None
        if self.legacy_vertices_enabled():
            color = 0x34D8ED if self.a.robot_id == 0 else 0xFFA04B
            vertices = self.make_cloud(xyz, color, message.header.stamp, self.output_frame)
            self.pub['vertices'].publish(vertices)
        self.counts['native_pgmo_meshes'] += 1
        submission = self.online_submission
        verified = self.verified_online_gauge() if self.a.mode == 'joint' else None
        if self.a.mode != 'joint' or submission is None or verified is None:
            self.online_latest_mesh_gauge = ('local_unanchored' if submission is None else
                'awaiting_native_process_identity' if self.online_verified_submission is not None else 'awaiting_native_path_echo')
            self.counts['online_unanchored_or_awaiting_path_echo_meshes'] += 1
            return
        # The verified path establishes continuity of the same native PGMO
        # coordinate gauge. A newer proven shared path may be pending its own
        # observation; it does not turn the persistent native graph local.
        submission = verified['submission']; echo = verified['echo']
        proof = submission['metadata']['proof']; active = set(proof['round']['active_robots'])
        # A configured gauge ID is just a namespace. A shared result requires
        # actual native cross-robot constraints and a completed active cluster.
        if active != {0, 1} or proof.get('accepted_cross_constraints', 0) <= 0:
            self.online_latest_mesh_gauge = 'local_unanchored'
            self.counts['online_meshes_without_shared_gauge_evidence'] += 1
            return
        self.online_mesh_sequence += 1
        latest = self.online_submission['metadata']
        newer_pending = latest['path_sha256'] != submission['metadata']['path_sha256']
        canonical_sha256 = self.timed('online_mesh:canonical_hash', canonical_serialized_mesh_digest, serialized)
        proof_hashes = self.timed('online_mesh:proof_hashes', mesh_proof_hashes, message, xyz)
        receipt = dict(run_id=self.a.run_id, cutoff_ns=self.cutoff_ns, robot_id=self.a.robot_id,
            receipt_kind='online_native_pgmo_mesh', execution_mode='online',
            native_geometry_source='native_PGMO_online_live_mesh',
            canonical_native_mesh_sha256=canonical_sha256,
            mesh_header_stamp_ns=message.header.stamp.to_nsec(), mesh_frame=message.header.frame_id,
            vertex_count=len(message.vertices), triangle_count=len(message.triangles),
            native_optimized_graph_sha256=submission['metadata']['graph_sha256'],
            submitted_body_path_sha256=submission['metadata']['path_sha256'],
            native_path_echo_verified=True, native_path_echo=dict(echo),
            native_path_echo_basis='last_verified_submission_in_same_native_pgmo_gauge_session',
            native_path_echo_covers_latest_submission=not newer_pending,
            latest_submitted_body_path_sha256=latest['path_sha256'], newer_path_pending=newer_pending,
            pending_submitted_body_path_sha256=latest['path_sha256'] if newer_pending else None,
            last_verified_body_path_sha256=submission['metadata']['path_sha256'],
            last_verified_native_optimized_graph_sha256=submission['metadata']['graph_sha256'],
            last_verified_path_observed_wall=echo['received_wall'],
            native_pgmo_process_identity=dict(verified['process_identity']),
            shared_gauge_epoch=copy.deepcopy(verified['epoch']),
            native_path_exact_pose_echo=False,
            path_echo_verification='keys_stamps_and_finite_native_poses',
            path_echo_key_stamps_source=self.base+'/native_pgmo/pose_graph',
            path_echo_per_pose_stamps_available=False,
            native_publish_iterate=False, shared_gauge_ready=True,
            shared_gauge_id=self.a.shared_gauge_id, gauge_id_is_configuration_identity_only=True,
            shared_gauge_evidence='completed_native_DPGO_active_cluster_accepted_cross_weights_and_PGMO_path_observation',
            optimizer_owner=proof['round']['owner'], completed_round_serial=proof['round']['serial'],
            final_native_weight_batch_sha256=proof['weights']['sha256'],
            completed_round_proof=copy.deepcopy(proof), accepted_wall=time.time(),
            native_mesh_sequence=self.online_mesh_sequence, immutable_snapshot=True,
            fixed_mesh_deformation_diagnostic_performed=False,
            exact_optimizer_mesh_generation_proven=False, exact_shared_optimizer_round_id_available=False,
            current_mesh_path_generation_verified=False,
            binding='persistent_native_shared_gauge_from_last_verified_path_observation_no_atomic_generation', **proof_hashes)
        self.accepted_joint_mesh_receipt = receipt
        self.online_latest_mesh_gauge = 'shared_gauge_observed_non_atomic'
        self.timed('online_mesh:publish_accepted', publish_serialized_message, self.pub['accepted_mesh'], serialized)
        if vertices is not None: publish_preserving_cached_message(self.pub['accepted_vertices'], vertices)
        self.counts['online_shared_gauge_meshes'] += 1
        self.event('online_native_mesh_accepted', native_mesh_sequence=self.online_mesh_sequence,
            canonical_native_mesh_sha256=receipt['canonical_native_mesh_sha256'],
            vertex_count=len(message.vertices), triangle_count=len(message.triangles))

    def on_pgmo_graph(self, message):
        if self.is_online(): self.timed('online_pgmo_graph:prefix_check', self.validate_pgmo_graph_prefix, message)
        if self.graph_sent >= 0 and 0 in self.keys.nodes:
            zeros = [n for n in message.nodes if int(n.key) == 0 and int(n.robot_id) == self.a.robot_id]
            if any(n.header.stamp.to_nsec() != self.keys.nodes[0].header.stamp.to_nsec() for n in zeros):
                raise ValueError('Native PGMO key-zero stamp differs from original epoch')
            self.native_body_metadata_ack = True
            self.native_body_metadata_ack_has_optimized_key_zero |= bool(zeros)
        if self.is_online():
            # The pinned PGMO nav_msgs/Path omits individual stamps/keys. Its
            # native pose_graph carries them, so verify the full metadata there
            # rather than inventing stamp acknowledgements on the path topic.
            validate_pose_values(node.pose for node in message.nodes)
            for node in message.nodes:
                key = int(node.key)
                if int(node.robot_id) != self.a.robot_id or key not in self.keys.nodes:
                    raise ValueError('Native PGMO graph has no matching original body key')
                stamp = require_stamp(node.header.stamp, self.cutoff_ns)
                if stamp != self.keys.nodes[key].header.stamp.to_nsec():
                    raise ValueError('Native PGMO body key stamp differs from native VIO')
                self.pgmo_body_key_stamps[key] = stamp
            while self.pgmo_verified_through+1 in self.pgmo_body_key_stamps:
                self.pgmo_verified_through += 1
        self.pgmo_graph_stats = dict(nodes=len(message.nodes), edges=len(message.edges), received_wall=time.time())
        if self.transaction is not None and self.transaction['phase'] == 'await_path_echo':
            self.transaction.setdefault('query_graph_counts', []).append(dict(self.pgmo_graph_stats))
        if self.is_online() and self.online_pending_pgmo_path is not None:
            self.on_pgmo_path(self.online_pending_pgmo_path)

    def on_pgmo_path(self, message):
        if self.is_online():
            submission = self.online_submission
            if submission is None: return
            if frame(message.header.frame_id) != self.output_frame:
                raise ValueError('Native PGMO online path frame differs from configured output frame')
            stamp = require_stamp(message.header.stamp, self.cutoff_ns)
            if stamp not in {node.header.stamp.to_nsec() for node in self.keys.nodes.values()}:
                raise ValueError('Native PGMO path stamp has no original native key')
            for item in message.poses:
                if frame(item.header.frame_id) != self.output_frame:
                    raise ValueError('Native PGMO online path contains a foreign pose frame')
            validate_pose_values(item.pose for item in message.poses)
            if len(message.poses) < len(submission['metadata']['original_key_stamps_ns']):
                self.counts['online_path_observations_waiting_native_key_metadata'] += 1
                return
            if self.online_path_echo is not None:
                # Still authenticate and validate every new native path. The
                # submitted prefix metadata is immutable and already verified;
                # repeated cached paths need no N matrix inverses/residuals.
                self.counts['online_path_prior_metadata_reused'] += 1
                return
            self.online_pending_pgmo_path = message
            stamps = submission['metadata']['original_key_stamps_ns']
            if len(message.poses) < len(stamps) or any(self.pgmo_body_key_stamps.get(key) != value
                    for key, value in enumerate(stamps)):
                self.counts['online_path_observations_waiting_native_key_metadata'] += 1
                return
            error = path_error(message, submission['path'])
            if self.online_path_echo is None:
                identity = None
                if self.a.mode == 'joint':
                    try: identity = self.observe_pgmo_process_identity()
                    except NativeProcessIdentityUnavailable: return
                self.online_path_echo = dict(received_wall=time.time(), verified_native_key_count=len(stamps),
                    exact_pose_echo=False, verification='keys_stamps_and_finite_native_poses', **error)
                if self.a.mode == 'joint' and self.shared_online_proof(submission['metadata']['proof']):
                    epoch = self.online_gauge_epoch()
                    if epoch != submission['metadata']['gauge_epoch']:
                        self.invalidate_online_gauge('path_observation_epoch_changed')
                        raise ValueError('Native path observation crossed a shared-gauge epoch')
                    self.online_verified_submission = dict(submission=submission, echo=dict(self.online_path_echo),
                        epoch=epoch, process_identity=identity)
                    self.timed('publish_native_verified_body_path', publish_preserving_cached_message,
                        self.pub['verified_path'], submission['path'])
                # Re-trigger the latest live full mesh, without capturing or
                # freezing it. PGMO publishes mesh only from its mesh callback.
                self.remesh_after = time.time()
            return
        current = self.transaction
        if current is None: return
        if current['phase'] == 'await_native_seed':
            current['native_seed_path_keys'] = len(message.poses)
            if len(message.poses) >= current['expected_native_body_keys']:
                current['phase'] = 'await_baseline'; current['phase_started_wall'] = time.time()
                current['complete_native_metadata_verified_wall'] = time.time()
                current['waiting_full_mesh_ack'] = True; current['last_full_mesh_send_wall'] = time.time()
                publish_preserving_cached_message(self.pub['full'], current['fixed_mesh'])
            return
        if current['phase'] != 'await_path_echo': return
        error = diagnostic_path_error(message, current['path'])
        if error is None or error['max_translation_m'] > 1e-5 or error['max_rotation_rad'] > 1e-5: return
        current['echo_receipts'].append(dict(received_wall=time.time(), path_sha256=digest(message), **error))
        # Two identical native query results plus unchanged native factor counts
        # demonstrate that the no-edge readiness queries introduced no factor.
        counts = current.get('query_graph_counts', [])
        if len(current['echo_receipts']) < 2 or len(counts) < 2: return
        if (counts[-1]['nodes'], counts[-1]['edges']) != (counts[-2]['nodes'], counts[-2]['edges']): return
        current['phase'] = 'await_deformed_mesh'; current['phase_started_wall'] = time.time()
        current['path_echo_verified_wall'] = time.time()
        publish_preserving_cached_message(self.pub['full'], current['fixed_mesh'])
        current['fixed_mesh_retriggered_wall'] = time.time()
        self.event('native_path_echo_then_fixed_mesh_retrigger', path_sha256=current['metadata']['path_sha256'],
            mesh_sha256=current['fixed_mesh_sha256'], echo_receipts=current['echo_receipts'],
            no_edge_query_factor_counts=counts[-2:])

    def transaction_tick(self):
        if self.is_online(): return
        current = self.transaction
        if current is None: return
        if time.time()-current['phase_started_wall'] > self.a.mesh_phase_timeout:
            self.event('native_mesh_transaction_not_accepted', phase=current['phase'],
                elapsed_phase_s=time.time()-current['phase_started_wall'],
                reason='No native phase readiness/deformation proof before configured finite timeout; inputs resume and accepted snapshot stays')
            self.counts['mesh_transactions_not_accepted'] += 1; self.transaction = None; return
        if current['phase'] in ('await_native_seed', 'await_path_echo') and time.time()-current['last_query_wall'] > .25:
            # PGMO early-returns for an empty graph. A single existing native
            # node with NO edges reaches its path publisher without adding any
            # node/factor, because its trajectory has already been initialized.
            query = native_readiness_query(self.last_forwarded_graph, self.keys.nodes[0])
            self.pub['graph'].publish(query); current['last_query_wall'] = time.time()
            self.counts['no_edge_native_path_queries'] += 1
        if (current['phase'] in ('await_native_seed', 'await_baseline') and
                not current['waiting_full_mesh_ack'] and time.time()-current['last_full_mesh_send_wall'] > .2):
            # At most one same-input full-mesh request is outstanding. It
            # promotes native queued body values and then proves a stable
            # native baseline; it is never a free-running cached relay.
            current['waiting_full_mesh_ack'] = True; current['last_full_mesh_send_wall'] = time.time()
            publish_preserving_cached_message(self.pub['full'], current['fixed_mesh'])

    def on_optimized_mesh(self, message):
        xyz = self.timed('online_mesh:validate', validate_mesh, message, self.cutoff_ns, self.output_frame) if self.is_online() else validate_mesh(message, self.cutoff_ns, self.output_frame)
        if message.ns != str(self.a.robot_id): raise ValueError('Native PGMO output ownership differs from configured robot')
        if self.is_online():
            self.pgmo_initialized = True
            self.on_online_optimized_mesh(message, xyz)
            return
        serialized = None; serialized_sha = None
        def mesh_bytes():
            nonlocal serialized
            if serialized is None: serialized = message_bytes(message)
            return serialized
        def mesh_sha():
            nonlocal serialized_sha
            if serialized_sha is None: serialized_sha = hashlib.sha256(mesh_bytes()).hexdigest()
            return serialized_sha
        self.pgmo_initialized = True
        newly_accepted = False
        if self.a.mode == 'joint':
            current = self.transaction
            if current is not None:
                if not same_mesh_topology_and_stamps(message, current['fixed_mesh']): return
                if current['phase'] == 'await_native_seed':
                    current['waiting_full_mesh_ack'] = False
                    return
                if current['phase'] == 'await_baseline':
                    current['waiting_full_mesh_ack'] = False
                    previous = current.get('candidate_baseline_xyz')
                    if previous is not None and np.allclose(xyz, previous, atol=1e-7, rtol=0):
                        current['stable_baseline_count'] += 1
                    else:
                        current['stable_baseline_count'] = 1
                    current['candidate_baseline_xyz'] = xyz.copy()
                    if current['stable_baseline_count'] < 2: return
                    current['baseline_xyz'] = xyz.copy(); current['baseline_mesh_sha256'] = mesh_sha()
                    current['prefix'].with_name(current['prefix'].name+'_native_before.rosbin').write_bytes(mesh_bytes())
                    current['phase'] = 'await_path_echo'; current['phase_started_wall'] = time.time()
                    current['path_published_wall'] = time.time()
                    publish_preserving_cached_message(self.pub['path'], current['path'])
                    self.latest_submitted_body_path_sha256 = current['metadata']['path_sha256']
                    self.counts['submitted_native_paths'] += 1
                    self.event('native_path_submitted_after_fixed_mesh_baseline', **current['metadata'])
                    return
                if current['phase'] != 'await_deformed_mesh': return
                delta = np.linalg.norm(xyz-current['baseline_xyz'], axis=1)
                if not len(delta) or float(delta.max()) <= 1e-5:
                    self.counts['unchanged_native_mesh_not_deformation_proof'] += 1; return
                self.approved = current['metadata']
                self.eligible = dict(native_path_echo_verified=True, fixed_input_mesh_sha256=current['fixed_mesh_sha256'],
                    native_before_mesh_sha256=current['baseline_mesh_sha256'], native_after_mesh_sha256=mesh_sha(),
                    historical_vertices_moved=int(np.count_nonzero(delta > 1e-5)),
                    maximum_vertex_motion_m=float(delta.max()), rms_vertex_motion_m=float(np.sqrt(np.mean(delta*delta))),
                    path_echo_receipts=current['echo_receipts'], query_graph_counts=current.get('query_graph_counts', []),
                    fixed_mesh_retriggered_wall=current['fixed_mesh_retriggered_wall'],
                    same_native_input_mesh_topology_and_vertex_stamps=True, submitted_path_sha256=self.approved['path_sha256'],
                    frozen_native_body_graph_keys=current['expected_native_body_keys'],
                    complete_native_metadata_verified_wall=current['complete_native_metadata_verified_wall'],
                    same_input_native_stable_baseline_repetitions=current['stable_baseline_count'],
                    source_round_proof=self.approved['proof'], optimizer_round_binding_is_observational=True)
                current['prefix'].with_name(current['prefix'].name+'_native_after.rosbin').write_bytes(mesh_bytes())
                current['prefix'].with_name(current['prefix'].name+'_native_deformation_proof.json').write_text(
                    json.dumps(self.eligible, indent=2), encoding='utf-8')
                self.event('native_historical_mesh_deformation_measured', **self.eligible)
                self.accepted_joint_mesh_receipt = dict(run_id=self.a.run_id, cutoff_ns=self.cutoff_ns,
                    robot_id=self.a.robot_id, native_geometry_source='native_PGMO_same_fixed_mesh_deformation_transaction',
                    canonical_native_mesh_sha256=canonical_mesh_digest(message),
                    vertex_xyz_float32_le_sha256=hashlib.sha256(xyz.astype('<f4').tobytes()).hexdigest(),
                    mesh_header_stamp_ns=message.header.stamp.to_nsec(), mesh_frame=message.header.frame_id,
                    vertex_count=len(message.vertices), triangle_count=len(message.triangles),
                    vertex_stamps_uint64_le_sha256=hashlib.sha256(np.asarray([s.to_nsec() for s in message.vertex_stamps], dtype='<u8').tobytes()).hexdigest(),
                    triangles_uint32_le_sha256=hashlib.sha256(np.asarray([t.vertex_indices for t in message.triangles], dtype='<u4').tobytes()).hexdigest(),
                    optimizer_owner=self.approved['proof']['round']['owner'],
                    completed_round_serial=self.approved['proof']['round']['serial'],
                    final_native_weight_batch_sha256=self.approved['proof']['weights']['sha256'],
                    native_optimized_graph_sha256=self.approved['graph_sha256'],
                    submitted_body_path_sha256=self.approved['path_sha256'],
                    deformation_proof_file=current['prefix'].name+'_native_deformation_proof.json',
                    native_after_mesh_file=current['prefix'].name+'_native_after.rosbin',
                    completed_round_proof=self.approved['proof'],
                    native_deformation_proof=copy.deepcopy(self.eligible),
                    accepted_wall=time.time(), immutable_until_next_accepted_transaction=True,
                    exact_shared_optimizer_round_id_available=False)
                current['prefix'].with_name(current['prefix'].name+'_accepted_mesh_receipt.json').write_text(
                    json.dumps(self.accepted_joint_mesh_receipt, indent=2), encoding='utf-8')
                newly_accepted = True
                self.counts['proven_native_deformation_transactions'] += 1; self.transaction = None
            elif self.eligible is None:
                self.counts['unanchored_native_bootstrap_meshes'] += 1; return
        publish_preserving_cached_message(self.pub['native_mesh'], message)
        color = 0x34D8ED if self.a.robot_id == 0 else 0xFFA04B
        vertices = self.make_cloud(xyz, color, message.header.stamp, self.output_frame)
        self.pub['vertices'].publish(vertices)
        if newly_accepted:
            publish_preserving_cached_message(self.pub['accepted_mesh'], message)
            publish_preserving_cached_message(self.pub['accepted_vertices'], vertices)
        self.counts['native_pgmo_meshes'] += 1
        (self.evidence/'latest_native_mesh.rosbin').write_bytes(mesh_bytes())
        self.event('native_pgmo_mesh', sha256=mesh_sha(), vertices=len(xyz), triangles=len(message.triangles),
            latest_submitted_path_sha256=self.latest_submitted_body_path_sha256,
            native_deformation_proof=self.eligible,
            exact_optimizer_mesh_generation_proven=False)

    def tick(self, _):
        try:
            with self.lock:
                if self.fatal is not None or rospy.is_shutdown(): return
                self.timed('tick:drain_native_inputs', self.drain_native_inputs)
                self.timed('tick:drain_replaceable_inputs', self.drain_replaceable_inputs)
                if not self.gauge_checked and 0 in self.keys.nodes:
                    stamp = self.keys.nodes[0].header.stamp.to_nsec(); at_zero = self.history.at(stamp)
                    if at_zero is not None:
                        delta = np.linalg.inv(at_zero) @ pose_matrix(self.keys.nodes[0].pose)
                        angle = np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1., 1.))
                        if np.linalg.norm(delta[:3, 3]) > .02 or angle > .02:
                            raise ValueError('Native VIO odometry/key-zero graph gauges differ; refuse silent frame relabel')
                        self.gauge_checked = True; self.event('native_local_gauge_verified', key0_stamp_ns=stamp)
                self.timed('tick:publish_graphs', self.publish_graphs)
                self.timed('tick:publish_sensor', self.publish_sensor)
                self.timed('tick:advance_optimized_candidates', self.advance_optimized_candidates)
                self.timed('tick:approve_path', self.approve_path)
                self.timed('tick:transaction_tick', self.transaction_tick)
                if ((self.transaction is None or self.is_online()) and self.latest_full_mesh is not None and self.native_body_metadata_ack
                        and self.graph_sent >= 0 and not self.mesh_graph_queue and self.pub['full'].get_num_connections()):
                    signature = self.latest_full_mesh_sha
                    forced = self.remesh_after is not None and time.time() >= self.remesh_after
                    if signature != getattr(self, 'last_full_sha', None) or forced:
                        if self.is_online():
                            self.timed('publish_native_full_mesh', publish_serialized_message, self.pub['full'], self.latest_full_mesh_bytes)
                        else:
                            self.timed('publish_native_full_mesh', publish_preserving_cached_message, self.pub['full'], self.latest_full_mesh)
                        self.last_full_sha = signature
                        self.remesh_after = None; self.counts['full_meshes_forwarded'] += 1
                if self.is_online() and time.time()-self.last_status_wall < self.a.status_period: return
                self.last_status_wall = time.time()
                with self.timing_lock: timings = copy.deepcopy(self.timings)
                report = dict(run_id=self.a.run_id, cutoff_ns=self.cutoff_ns, robot_id=self.a.robot_id, mode=self.a.mode,
                    execution_mode=getattr(self.a, 'execution_mode', 'diagnostic'),
                    legacy_vertex_displays_enabled=self.legacy_vertices_enabled(),
                    persistence_enabled=not getattr(self.a, 'no_persist', False),
                    wall=time.time(), native_local_gauge_verified=self.gauge_checked, local_keyframes=len(self.keys.nodes),
                    input_mode='sha256_pinned_recorded_native_VIO_and_sensors' if self.a.recorded_native_bag else 'live_native_VIO_and_sensors',
                    recorded_native_bag_sha256=self.a.recorded_native_bag_sha256,
                    native_vio_maximum_interpolated_gap_s=self.history.maximum_interpolated_gap_ns*1e-9,
                    native_vio_maximum_rejected_gap_s=self.history.maximum_rejected_gap_ns*1e-9,
                    maximum_permitted_native_vio_gap_s=self.a.max_odom_gap,
                    sensor_pose_interpolation=('actual gyro curve constrained by unchanged native VIO orientation endpoints; linear translation'
                        if self.gyros is not None else 'native VIO shortest-arc quaternion and linear translation; explicit SLERP comparison mode'),
                    gyro_sample_convention=self.gyros.sample_convention if self.gyros is not None else None,
                    gyro_maximum_sample_gap_s=self.gyros.maximum_sample_gap_ns*1e-9 if self.gyros is not None else None,
                    gyro_maximum_used_gap_s=self.gyros.maximum_used_gap_ns*1e-9 if self.gyros is not None else None,
                    gyro_maximum_permitted_gap_s=self.gyros.maximum_gap_ns*1e-9 if self.gyros is not None else None,
                    gyro_maximum_endpoint_residual_rad=self.gyros.maximum_endpoint_residual_rad if self.gyros is not None else None,
                    graph_forwarded_through_key=self.graph_sent, counts=dict(self.counts), fatal_error=self.fatal,
                    native_pgmo_verified_through_key=self.pgmo_verified_through,
                    native_pgmo_unacknowledged_original_keys=max(0, self.graph_sent-self.pgmo_verified_through),
                    original_graphs_waiting_forward=len(self.graph_queue), mesh_graphs_waiting_forward=len(self.mesh_graph_queue),
                    original_graph_forwarding_policy=('online synchronous publisher; at most 32 messages per tick; no factor replay'
                        if self.is_online() else 'compatibility diagnostic native graph forwarding; no factor replay'),
                    timings=timings,
                    native_original_priority_queue_length=len(self.native_inputs),
                    native_original_priority_queue_max_length=self.native_input_max_queue,
                    native_original_max_queue_delay_s=self.native_input_max_delay,
                    native_first_body_metadata_acknowledged=self.native_body_metadata_ack,
                    native_body_metadata_ack_optimized_key_zero_seen=self.native_body_metadata_ack_has_optimized_key_zero,
                    native_body_metadata_ack_basis='fresh native pose_graph callback completion after original key0 input; optimized key0 stamp checked whenever present',
                    native_control_max_queue_delay_s=self.native_control_max_delay,
                    actual_gyro_max_queue_delay_s=self.native_imu_max_delay,
                    replaceable_full_mesh_overwrites=dict(self.replaceable_overwrites),
                    optimized_candidates_waiting=len(self.optimized_candidates),
                    optimized_candidate_wait_limit_s=self.optimized_candidate_wait,
                    optimized_candidate_missing_original_counts=[len(self.keys.missing_originals(c['nodes'])) for c in self.optimized_candidates],
                    approved_path=self.approved, local_frame=self.local_frame, output_frame=self.output_frame,
                    raw_local_vertices_topic=self.base+'/native_dense/raw_local_vertices',
                    raw_local_vertices_frame=self.local_frame,
                    raw_local_vertices_source='native_mesh_frontend/full_mesh; exact XYZ, no PGMO deformation',
                    local_tsdf_continues_during_pgmo_transaction=True,
                    sensor_stream_in_independent_process=self.a.external_sensor_stream,
                    native_deformation_eligible=(self.eligible is not None if self.a.mode == 'joint' else None)
                        if not self.is_online() else None,
                    native_deformation_proof=self.eligible,
                    accepted_joint_mesh_receipt=self.accepted_joint_mesh_receipt,
                    shared_gauge_id=getattr(self.a, 'shared_gauge_id', self.a.run_id),
                    gauge_id_is_configuration_identity_only=True,
                    shared_gauge_ready=self.accepted_joint_mesh_receipt is not None if self.is_online() else self.eligible is not None,
                    latest_submitted_path_echo_observed=self.online_path_echo is not None if self.is_online() else None,
                    last_verified_body_path_sha256=(self.online_verified_submission['submission']['metadata']['path_sha256']
                        if self.online_verified_submission is not None else None),
                    newer_path_pending=(self.online_verified_submission is not None and self.online_submission is not None
                        and self.online_verified_submission['submission']['metadata']['path_sha256'] != self.online_submission['metadata']['path_sha256'])
                        if self.is_online() else None,
                    native_pgmo_process_identity=self.online_pgmo_process_identity,
                    native_pgmo_process_probe=self.pgmo_process_probe.snapshot() if self.pgmo_process_probe else None,
                    native_pgmo_process_identity_error=self.pgmo_process_identity_error,
                    latest_native_mesh_gauge=getattr(self, 'online_latest_mesh_gauge', None) if self.is_online() else None,
                    native_optimized_mesh_topic=self.base+'/native_dense/native_optimized_mesh',
                    optimization_evidence=self.approved['proof'] if self.approved else None,
                    native_publish_iterate=self.approved.get('native_publish_iterate') if self.approved else None,
                    exact_shared_optimizer_round_id_available=False,
                    pgmo_inputs_frozen=self.transaction is not None and not self.is_online(),
                    remote_bridge_caller=getattr(self.a, 'remote_bridge_caller', None),
                    remote_controls_waiting_for_provenance=len(self.peer_controls.pending) if self.peer_controls else 0,
                    remote_control_provenance_overdue=self.peer_controls.overdue if self.peer_controls else False,
                    remote_controls_rejected_previous_boot=self.peer_controls.discarded_previous_boot_controls if self.peer_controls else 0,
                    remote_control_transport=getattr(self.a, 'remote_control_transport', 'atomic'),
                    remote_control_provenance=self.peer_controls.diagnostic(time.monotonic()) if self.peer_controls else None,
                    last_rejected_control_envelope=self.last_rejected_control_envelope,
                    latest_submitted_body_path_sha256=self.latest_submitted_body_path_sha256,
                    ordinary_native_mesh_stream_has_immutable_generation_receipt=False,
                    mesh_transaction_phase=self.transaction['phase'] if self.transaction else None,
                    mesh_transaction_phase_timeout_s=self.a.mesh_phase_timeout,
                    native_map_producers=['voxblox_ros/tsdf_server', 'kimera_pgmo_ros/voxblox_mesh_frontend_node', 'kimera_pgmo_ros/kimera_pgmo_node'],
                    adapter_not_estimator=True, consumes_ground_truth=False, consumes_lidar_or_custom_mapper=False,
                    mesh_owner_workaround='local frontend default0 -> configured robot; no geometry/key changes',
                    exact_optimizer_mesh_generation_proven=False,
                    acceptance=('Online native PGMO output with observational completion/path evidence; geometry accuracy requires separate evaluation'
                        if self.is_online() else 'Native mesh deformation must be separately measured before claiming S3/S4 complete'))
                self.pub['status'].publish(String(data=json.dumps(report, allow_nan=False)))
                self.persist_json(self.evidence/'status.json', report)
        except Exception as error: self.fail(error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', required=True, type=int, choices=(0, 1))
    parser.add_argument('--robot-name', required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--mode', choices=('local', 'joint'), default='local')
    parser.add_argument('--execution-mode', choices=('diagnostic', 'online'), default='diagnostic',
                        help='Explicit online production or compatibility fixed-mesh deformation diagnostic')
    parser.add_argument('--no-persist', action='store_true',
                        help='Online only: retain ROS/log events, never write native graphs, meshes or receipt archives')
    parser.add_argument('--shared-gauge-id',
                        help='Configuration identity only; shared-gauge acceptance still requires native cross-robot evidence')
    parser.add_argument('--remote-bridge-caller',
                        help='Exact trusted local peer proxy; remote controls additionally require matching verified ingress receipts')
    parser.add_argument('--remote-control-transport', choices=('atomic', 'receipt'), default='atomic',
                        help='Atomic coupled native bytes/provenance in production; receipt is explicit compatibility diagnostics')
    parser.add_argument('--status-period', type=float, default=1., help='Online diagnostic status period, seconds')
    parser.add_argument('--online-legacy-vertices', action='store_true',
                        help='Also produce legacy display/accepted/raw PointClouds online; diagnostic always retains them')
    parser.add_argument('--global-frame', default='kimera_world')
    parser.add_argument('--cloud-rate', type=float, default=2.)
    parser.add_argument('--pixel-stride', type=int, default=3)
    parser.add_argument('--max-depth', type=float, default=8.)
    parser.add_argument('--max-odom-gap', type=float, default=.6,
                        help='Pinned native odometry is keyframe-rate (~2Hz); measured 60s fixture max gap .53256s')
    parser.add_argument('--sensor-rotation-interpolation', choices=('gyro_endpoint', 'slerp'), default='gyro_endpoint',
                        help='Actual gyro curve between native keyframes; slerp is only an explicit comparison mode')
    parser.add_argument('--gyro-sample-convention', choices=('left_endpoint_interval_equivalent',),
                        default='left_endpoint_interval_equivalent',
                        help='Actual body IMU interval contract, matching the simulator and native Kimera integration')
    parser.add_argument('--max-gyro-gap', type=float, default=.05,
                        help='Omit RGB-D frames crossing larger actual IMU gaps; never silently use slerp')
    parser.add_argument('--weight-updates', type=int, default=25)
    parser.add_argument('--mesh-phase-timeout', type=float, default=30.,
                        help='Finite timeout PER native readiness/baseline/echo/deformation phase, not a total fixed freeze timeout')
    parser.add_argument('--require-cross', action='store_true')
    parser.add_argument('--external-sensor-stream', action='store_true',
                        help='Backend-only role: do not subscribe raw RGB-D/IMU or publish TSDF point clouds')
    parser.add_argument('--recorded-native-bag', type=Path,
                        help='Explicit offline native-input fixture; live source authentication remains default')
    parser.add_argument('--recorded-native-bag-sha256')
    parser.add_argument('--replay-caller', default='/native_dense_replay')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.shared_gauge_id is None: args.shared_gauge_id = args.run_id
    if args.robot_name != 'sim_%d' % args.robot_id: parser.error('Expected matching sim_N name')
    if args.cutoff <= 0 or args.cloud_rate <= 0 or args.pixel_stride < 1: parser.error('Invalid epoch/sampling')
    if args.mesh_phase_timeout <= 0 or args.max_odom_gap <= 0 or args.max_gyro_gap <= 0 or args.weight_updates <= 0:
        parser.error('Native phase timeout, odometry bracket limit and weight-update count must be positive')
    if not np.isfinite(args.status_period) or args.status_period <= 0: parser.error('Status period must be finite and positive')
    if args.no_persist and args.execution_mode != 'online': parser.error('--no-persist requires --execution-mode online')
    if args.remote_bridge_caller and not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*', args.remote_bridge_caller):
        parser.error('Remote bridge caller must be one absolute ROS node name')
    if args.recorded_native_bag:
        if args.mode != 'local': parser.error('Recorded native-input mode is only for isolated local S2 acceptance')
        if not args.replay_caller.startswith('/native_dense_replay'):
            parser.error('Explicit recorded input requires a dedicated /native_dense_replay caller')
        if not args.recorded_native_bag_sha256: parser.error('Recorded native bag SHA256 is required')
        checksum = hashlib.sha256()
        with args.recorded_native_bag.open('rb') as source:
            for block in iter(lambda: source.read(2**20), b''): checksum.update(block)
        if checksum.hexdigest() != args.recorded_native_bag_sha256:
            parser.error('Recorded native-input bag checksum differs from declared fixture')
    elif args.recorded_native_bag_sha256: parser.error('Bag path required with a recorded-input checksum')
    rospy.init_node('native_dense_bridge', anonymous=False)
    node = NativeDenseBridge(args); rospy.spin()
    if node.fatal: raise SystemExit(2)


if __name__ == '__main__':
    import rospy
    import tf2_ros
    from dpgo_ros.msg import Command, RelativeMeasurementWeights
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from kimera_pgmo_msgs.msg import KimeraPgmoMesh
    from nav_msgs.msg import Odometry, Path as RosPath
    from pose_graph_tools_msgs.msg import PoseGraph
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
    from std_msgs.msg import Header, String, UInt8MultiArray
    main()
