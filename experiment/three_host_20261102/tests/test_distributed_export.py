import copy
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'distributed'))
import export as distributed_export
import result_wire as wire


RUN = 'export_test'
GAUGE = RUN + '.robot0-initial'
CUTOFF = 99 * 10 ** 9
BOOTS = {0: 'result_boot_0', 1: 'result_boot_1'}


def ros_header(stamp=101 * 10 ** 9, seq=23):
    frame = b'kimera_world'
    return struct.pack('<IIII', seq, stamp // 10 ** 9, stamp % 10 ** 9, len(frame)) + frame


def native_mesh(robot, stamp):
    xyz = np.array([[robot * 3 + .1234567890123, 0., 0.],
                    [robot * 3 + 1., .000000000001, 0.],
                    [robot * 3, 1., 0.]], dtype='<f8')
    faces = np.array([[2, 0, 1]], dtype='<u4')
    rgba = np.array([[.1234567, .25, .75, 1.], [.5, .3, .4, 1.], [1., .6, .7, .8]], dtype='<f4')
    stamp_words = np.array([[stamp // 10 ** 9, stamp % 10 ** 9]] * len(xyz), dtype='<u4')
    namespace = str(robot).encode()
    output = io.BytesIO(); output.write(ros_header(stamp))
    output.write(struct.pack('<I', len(namespace)) + namespace)
    for array in (faces, xyz, stamp_words, np.arange(len(xyz), dtype='<i4'), rgba):
        output.write(struct.pack('<I', len(array))); output.write(array.tobytes())
    return output.getvalue(), xyz, faces, rgba


def result_meta(robot, stream, raw, receipt=None):
    meta = wire.contract(robot, stream)
    meta.update(protocol=1, run_id=RUN, robot_id=robot, cutoff_ns=CUTOFF,
        boot_id=BOOTS[robot], stream=stream, sequence=1, stamp_basis='header',
        stamp_ns=wire.read_header(raw)[0], frame_id='kimera_world',
        mode='full', base_sha256=None, codec='none', raw_bytes=len(raw),
        encoded_raw_bytes=len(raw), result_sha256=wire.sha256(raw),
        content_sha256=wire.content_hash(raw, stream), gauge_id=GAUGE,
        ros_md5=distributed_export.MESSAGE_MD5 if stream == 'accepted_mesh' else distributed_export.PATH_MD5,
        native_receipt=receipt, shared_gauge=receipt is not None)
    meta['snapshot_id'] = wire.snapshot_identity(meta)
    wire.validate_metadata(meta)
    return meta


def archive_record(root, meta, raw):
    directory = root / RUN / ('sim_%d' % meta['robot_id']) / meta['boot_id'] / meta['stream']
    directory.mkdir(parents=True, exist_ok=True)
    stem = '%09d_%s' % (meta['sequence'], meta['snapshot_id'])
    metadata = directory / (stem + '.json')
    metadata.write_bytes(wire.json_bytes(meta))
    native = directory / (stem + '.rosbin'); native.write_bytes(raw)
    pointer = dict(snapshot_id=meta['snapshot_id'], metadata=metadata.name,
                   original_rosbin=native.name, ply=None)
    (directory / 'latest.json').write_bytes(wire.json_bytes(pointer))
    return directory, metadata, native


def fixture(root):
    records = []
    for robot in (0, 1):
        stamp = (101 + robot) * 10 ** 9
        raw, xyz, faces, rgba = native_mesh(robot, stamp)
        path_raw = ros_header(105 * 10 ** 9) + struct.pack('<I', 1) + ros_header(stamp) + struct.pack('<7d', 1., 2., 3., 0., 0., 0., 1.)
        path_meta = result_meta(robot, 'body_path', path_raw)
        archive_record(root, path_meta, path_raw)
        rows = [[0, 3, 1, 4, 1. - robot * .25, False]]
        weight_hash = wire.sha256(json.dumps(rows, separators=(',', ':')).encode())
        serial = 16 + robot * 7  # Independent observed completion counters.
        receipt = dict(run_id=RUN, robot_id=robot, cutoff_ns=CUTOFF,
            canonical_native_mesh_sha256=wire.content_hash(raw, 'accepted_mesh'),
            mesh_frame='kimera_world', mesh_header_stamp_ns=stamp,
            vertex_count=len(xyz), triangle_count=len(faces), shared_gauge_id=GAUGE,
            shared_gauge_ready=True, native_path_echo_verified=True, native_publish_iterate=False,
            vertex_xyz_float32_le_sha256=wire.sha256(xyz.astype('<f4').tobytes()),
            triangles_uint32_le_sha256=wire.sha256(faces.tobytes()),
            vertex_stamps_uint64_le_sha256=wire.sha256(np.full(len(xyz), stamp, dtype='<u8').tobytes()),
            submitted_body_path_sha256=distributed_export.canonical_header_hash(path_raw),
            optimizer_owner=0, completed_round_serial=serial, accepted_wall=110. + robot,
            final_native_weight_batch_sha256=weight_hash,
            completed_round_proof=dict(accepted_cross_constraints=1,
                round=dict(owner=0, serial=serial, active_robots=[0, 1],
                    request_wall=100., terminate_wall=104. + robot),
                weights=dict(owner=0, received_wall=103., rows=rows, sha256=weight_hash),
                upstream_dpgo_path=dict(received_wall=106. + robot, round_serial=serial)))
        meta = result_meta(robot, 'accepted_mesh', raw, receipt)
        directory, metadata_path, raw_path = archive_record(root, meta, raw)
        records.append(dict(meta=meta, raw=raw, xyz=xyz, faces=faces, rgba=rgba,
                            directory=directory, metadata_path=metadata_path, raw_path=raw_path))
    return records


def ply_arrays(path):
    raw = path.read_bytes(); end = raw.index(b'end_header\n') + len(b'end_header\n')
    header = raw[:end].decode()
    vertices = int(next(line.split()[-1] for line in header.splitlines() if line.startswith('element vertex')))
    faces = int(next(line.split()[-1] for line in header.splitlines() if line.startswith('element face')))
    dtype = [('x', '<f8'), ('y', '<f8'), ('z', '<f8'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'), ('alpha', 'u1'),
        ('native_red', '<f4'), ('native_green', '<f4'), ('native_blue', '<f4'), ('native_alpha', '<f4'),
        ('native_color_available', 'u1'), ('robot_id', 'u1')]
    points = np.frombuffer(raw, dtype=dtype, count=vertices, offset=end)
    triangles = np.frombuffer(raw, dtype=[('count', 'u1'), ('indices', '<u4', (3,))],
                             count=faces, offset=end + points.nbytes)
    return points, triangles


class DistributedExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.base = Path(self.temp.name)
        self.archive = self.base / 'observer'; self.archive.mkdir()
        self.records = fixture(self.archive); self.output = self.base / 'final'

    def tearDown(self):
        self.temp.cleanup()

    def export(self, **kwargs):
        return distributed_export.export(self.archive, RUN, GAUGE, self.output,
                                          boots=kwargs.pop('boots', BOOTS), **kwargs)

    def test_native_coordinates_float_colors_faces_and_original_bytes_are_preserved(self):
        result = self.export()
        points, faces = ply_arrays(self.output / 'global_native_mesh.ply')
        np.testing.assert_array_equal(np.column_stack([points[k] for k in ('x', 'y', 'z')]),
                                      np.vstack([r['xyz'] for r in self.records]))
        native_rgba = np.column_stack([points['native_' + name] for name in ('red', 'green', 'blue', 'alpha')])
        np.testing.assert_array_equal(native_rgba, np.vstack([r['rgba'] for r in self.records]))
        np.testing.assert_array_equal(faces['indices'], [[2, 0, 1], [5, 3, 4]])
        np.testing.assert_array_equal(points['robot_id'], [0, 0, 0, 1, 1, 1])
        for robot, record in enumerate(self.records):
            source = self.output / 'source' / ('robot_%d' % robot)
            self.assertEqual((source / 'native_mesh.rosbin').read_bytes(), record['raw'])
            self.assertEqual(json.loads((source / 'native_receipt.json').read_bytes()), record['meta']['native_receipt'])
        self.assertEqual(result['points'], 6)
        self.assertEqual(result['triangles'], 2)

    def test_independent_completion_windows_are_not_falsely_called_atomic(self):
        result = self.export()
        self.assertEqual(result['robots']['0']['native_receipt']['completed_round_serial'], 16)
        self.assertEqual(result['robots']['1']['native_receipt']['completed_round_serial'], 23)
        self.assertTrue(result['asynchronous_observational_native_evidence'])
        self.assertFalse(result['atomic_multi_robot_snapshot'])
        self.assertFalse(result['exact_shared_optimizer_round_id_available'])

    def test_expected_sender_boots_are_required_and_old_boot_is_not_a_fallback(self):
        with self.assertRaises(ValueError): self.export(boots=None)
        with self.assertRaises(OSError): self.export(boots={0: BOOTS[0], 1: 'new_unproven_boot'})
        self.assertFalse(self.output.exists())

    def test_observer_status_selects_current_boots_and_checks_explicit_conflicts(self):
        state = self.base / 'observer_status.json'
        state.write_bytes(wire.json_bytes(dict(run_id=RUN, gauge_id=GAUGE, cutoff_ns=CUTOFF,
            robots={str(r): dict(current_boot_id=BOOTS[r]) for r in (0, 1)})))
        result = self.export(boots=None, observer_status=state)
        self.assertEqual(result['boot_selection'], 'observer current_boot_id report')
        with self.assertRaises(ValueError):
            self.export(boots={0: 'wrong', 1: BOOTS[1]}, observer_status=state)

    def test_corrupt_native_mesh_hash_is_rejected_before_any_export(self):
        self.records[1]['raw_path'].write_bytes(self.records[1]['raw'] + b'corrupt')
        with self.assertRaises(ValueError): self.export()
        self.assertFalse(self.output.exists())

    def test_metadata_and_receipt_cannot_lie_about_native_vertex_count(self):
        record = self.records[0]
        meta = copy.deepcopy(record['meta']); meta['native_receipt']['vertex_count'] = 300
        meta['snapshot_id'] = wire.snapshot_identity(meta)
        archive_record(self.archive, meta, record['raw'])
        with self.assertRaisesRegex(ValueError, 'vertex_count'):
            self.export()

    def test_receipted_body_path_must_exist_and_have_exact_content(self):
        path_dir = self.records[0]['directory'].parent / 'body_path'
        for path in path_dir.glob('*.rosbin'):
            path.write_bytes(path.read_bytes()[:-1] + b'changed')
        with self.assertRaisesRegex(ValueError, 'body path'):
            self.export()

    def retain_verified_path(self, robot):
        directory = self.records[robot]['directory'].parent / 'body_path'
        pointer = json.loads((directory / 'latest.json').read_bytes())
        raw = (directory / pointer['original_rosbin']).read_bytes()
        meta = result_meta(robot, 'verified_body_path', raw)
        archive_record(self.archive, meta, raw)
        for path in directory.iterdir(): path.unlink()
        directory.rmdir()
        return raw

    def test_export_compatible_with_mixed_legacy_and_verified_path_archives(self):
        raw = self.retain_verified_path(1)
        result = self.export()
        self.assertEqual((self.output / 'source/robot_1/submitted_body_path.rosbin').read_bytes(), raw)
        self.assertEqual(result['points'], 6)
        self.assertEqual(json.loads((self.output / 'source/robot_0/body_path_metadata.json').read_bytes())['stream'], 'body_path')
        self.assertEqual(json.loads((self.output / 'source/robot_1/body_path_metadata.json').read_bytes())['stream'], 'verified_body_path')

    def test_new_pending_approved_paths_cannot_replace_receipted_verified_bytes(self):
        expected = {robot:self.retain_verified_path(robot) for robot in (0,1)}
        for robot,raw in expected.items():
            pending = raw[:-56] + struct.pack('<d', 999.) + raw[-48:]
            archive_record(self.archive, result_meta(robot, 'body_path', pending), pending)
        self.export()
        for robot,raw in expected.items():
            self.assertEqual((self.output / ('source/robot_%d/submitted_body_path.rosbin' % robot)).read_bytes(), raw)

    def test_wrong_verified_path_hash_cannot_authorize_export(self):
        raw = self.retain_verified_path(0)
        verified = self.records[0]['directory'].parent / 'verified_body_path'
        for path in verified.glob('*.rosbin'):
            path.write_bytes(raw[:-56] + struct.pack('<d', 888.) + raw[-48:])
        with self.assertRaisesRegex(ValueError, 'body path'):
            self.export()
        self.assertFalse(self.output.exists())

    def test_pointer_path_traversal_is_rejected(self):
        path = self.records[0]['directory'] / 'latest.json'
        pointer = json.loads(path.read_bytes()); pointer['original_rosbin'] = '../outside.rosbin'
        path.write_bytes(wire.json_bytes(pointer))
        with self.assertRaisesRegex(ValueError, 'contained filename'):
            self.export()

    def test_idempotent_export_checks_all_artifacts_and_never_overwrites_corruption(self):
        first, second = self.export(), self.export()
        self.assertEqual(first['export_identity'], second['export_identity'])
        path = self.output / 'global_native_mesh.ply'; path.write_bytes(b'broken')
        with self.assertRaises(FileExistsError): self.export()
        self.assertEqual(path.read_bytes(), b'broken')

    def test_invalid_receipt_weight_digest_is_rejected_even_after_metadata_rehash(self):
        record = self.records[0]
        meta = copy.deepcopy(record['meta'])
        meta['native_receipt']['completed_round_proof']['weights']['rows'][0][4] = .75
        meta['snapshot_id'] = wire.snapshot_identity(meta)
        archive_record(self.archive, meta, record['raw'])
        with self.assertRaisesRegex(ValueError, 'weight proof'):
            self.export()


if __name__ == '__main__':
    unittest.main()
