#!/usr/bin/env python3
"""Export a verified archived native mesh pair without ROS or new estimation.

The global cloud uses robot ownership colors, not camera RGB. Original native
meshes, float64 positions, float32 RGBA, topology, stamps and receipts are kept.
The native mesh schema is locked to kimera_pgmo_msgs/KimeraPgmoMesh MD5 below.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import tempfile

import numpy as np

MESSAGE_MD5 = 'ef8f486d0a6aeedc32da3a3f5eaf8912'
SOURCE_COLORS = (0x34D8ED, 0xFFA04B)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def child(parent, name):
    if not isinstance(name, str) or Path(name).name != name or name in ('', '.', '..'):
        raise ValueError('Expected one safe input filename: ' + repr(name))
    path = parent / name
    if path.resolve().parent != parent.resolve():
        raise ValueError('Input symlink escapes its directory')
    return path


class Reader:
    def __init__(self, raw):
        self.raw, self.offset = raw, 0

    def take(self, size):
        if size < 0 or self.offset + size > len(self.raw):
            raise ValueError('Truncated native mesh')
        data = memoryview(self.raw)[self.offset:self.offset + size]
        self.offset += size
        return data

    def uint(self):
        return struct.unpack('<I', self.take(4))[0]

    def string(self):
        return bytes(self.take(self.uint())).decode('utf-8')

    def array(self, dtype, width=1):
        count = self.uint()
        dtype = np.dtype(dtype)
        data = np.frombuffer(self.take(count * width * dtype.itemsize), dtype=dtype).copy()
        return data.reshape(count, width) if width != 1 else data


def decode_mesh(raw, cutoff_ns, robot):
    """Decode the exact locked ROS1 schema, preserving native array ordering."""
    reader = Reader(raw)
    sequence, seconds, nanos = reader.uint(), reader.uint(), reader.uint()
    frame, namespace = reader.string(), reader.string()
    triangles = reader.array('<u4', 3)
    xyz = reader.array('<f8', 3)
    stamp_words = reader.array('<u4', 2)
    graph_indices = reader.array('<i4')
    rgba = reader.array('<f4', 4)
    if reader.offset != len(raw):
        raise ValueError('Unexpected trailing native mesh bytes / schema mismatch')
    if nanos >= 10**9 or frame != 'kimera_world' or namespace != str(robot):
        raise ValueError('Native mesh header/frame/robot mismatch')
    count = len(xyz)
    if not count or not len(triangles) or not np.isfinite(xyz).all():
        raise ValueError('Native mesh must have finite nonempty vertices and faces')
    if triangles.max() >= count:
        raise ValueError('Native triangle index outside vertex array')
    if len(stamp_words) != count or np.any(stamp_words[:, 1] >= 10**9):
        raise ValueError('Native vertex timestamp layout mismatch')
    stamps = stamp_words[:, 0].astype('<u8') * 10**9 + stamp_words[:, 1]
    if np.any(stamps < cutoff_ns):
        raise ValueError('Native vertex predates the run epoch')
    if len(graph_indices) not in (0, count):
        raise ValueError('Native graph correspondence layout mismatch')
    if len(rgba) not in (0, count) or not np.isfinite(rgba).all():
        raise ValueError('Native RGBA array is incomplete/nonfinite')
    # ROS colors are floats in [0,1]; exact floats remain in the arrays archive.
    if rgba.size and (rgba.min() < -1e-6 or rgba.max() > 1 + 1e-6):
        raise ValueError('Native RGBA is outside the declared normalized range')
    return dict(sequence=sequence, stamp_ns=seconds * 10**9 + nanos,
                frame=frame, namespace=namespace, xyz=xyz, triangles=triangles,
                vertex_stamps=stamps, graph_indices=graph_indices, rgba=rgba,
                canonical_sha256=sha(b'\0\0\0\0' + raw[4:]))


def check_equal(actual, expected, name):
    if actual != expected:
        raise ValueError(name + ' differs from archived receipt')


def load_pair(run, path):
    report_raw = path.read_bytes()
    report = json.loads(report_raw)
    if report.get('fusion_ready') is not True or report.get('frame') != 'kimera_world':
        raise ValueError('Archive is not a verified global pair')
    cutoff = report.get('cutoff_ns')
    if not isinstance(cutoff, int) or cutoff <= 0 or not report.get('run_id'):
        raise ValueError('Invalid run epoch')
    receipts = report.get('native_receipts')
    if not isinstance(receipts, list) or len(receipts) != 2:
        raise ValueError('Exactly two native receipts are required')
    npz_path = child(path.parent, report.get('archive_npz', path.with_suffix('.npz').name))
    npz_raw = npz_path.read_bytes()
    with np.load(npz_path, allow_pickle=False) as arrays:
        xyz = arrays['xyz'].copy(); colors = arrays['packed_colors'].copy(); robots = arrays['robot_ids'].copy()
    if xyz.dtype != np.dtype('<f4') or xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all():
        raise ValueError('Archived cloud XYZ is not finite nonempty float32 Nx3')
    if colors.dtype != np.dtype('<u4') or colors.shape != (len(xyz),) or robots.dtype != np.uint8 or robots.shape != (len(xyz),):
        raise ValueError('Archived cloud source metadata layout mismatch')
    check_equal(sha(xyz.astype('<f4', copy=False).tobytes()), report.get('ordered_xyz_float32_le_sha256'), 'Cloud XYZ hash')
    check_equal(sha(colors.astype('<u4', copy=False).tobytes()), report.get('ordered_colors_uint32_le_sha256'), 'Cloud ownership-color hash')
    check_equal(len(xyz), report.get('points'), 'Cloud point count')
    identity_inputs = [report['run_id'], cutoff, report.get('exact_native_mesh_hashes'),
                       report.get('optimizer_owner'), report.get('completed_round_serial'),
                       report.get('final_native_weight_batch_sha256')]
    check_equal(sha(json.dumps(identity_inputs, separators=(',', ':')).encode()),
                report.get('snapshot_identity'), 'Native snapshot identity')
    meshes, sources = [], []
    for robot, receipt in enumerate(receipts):
        for key, expected in [('run_id', report['run_id']), ('cutoff_ns', cutoff), ('robot_id', robot), ('mesh_frame', report['frame'])]:
            check_equal(receipt.get(key), expected, 'Receipt ' + key)
        for key in ('optimizer_owner', 'completed_round_serial', 'final_native_weight_batch_sha256'):
            check_equal(receipt.get(key), report.get(key), 'Joint ' + key)
        if receipt.get('immutable_until_next_accepted_transaction') is not True:
            raise ValueError('Native receipt is not immutable')
        if not finite_number(receipt.get('accepted_wall')):
            raise ValueError('Native receipt has no finite accepted time')
        proof = receipt.get('completed_round_proof') or {}
        if proof.get('accepted_cross_constraints', 0) <= 0:
            raise ValueError('No accepted cross-robot constraint')
        weights = proof.get('weights') or {}
        weight_raw = json.dumps(weights.get('rows'), separators=(',', ':'), allow_nan=False).encode()
        check_equal(sha(weight_raw), report['final_native_weight_batch_sha256'], 'Native weight batch hash')
        directory = run / ('dense' + str(robot)) / 'bridge'
        mesh_path = child(directory, receipt.get('native_after_mesh_file'))
        deformation_path = child(directory, receipt.get('deformation_proof_file'))
        mesh_raw, deformation_raw = mesh_path.read_bytes(), deformation_path.read_bytes()
        deformation = json.loads(deformation_raw)
        check_equal(deformation, receipt.get('native_deformation_proof'), 'Native deformation proof')
        if deformation.get('native_path_echo_verified') is not True or deformation.get('historical_vertices_moved', 0) <= 0:
            raise ValueError('Native deformation was not verified')
        mesh = decode_mesh(mesh_raw, cutoff, robot)
        checks = [('canonical_sha256', 'canonical_native_mesh_sha256'), ('stamp_ns', 'mesh_header_stamp_ns')]
        for actual, expected in checks:check_equal(mesh[actual], receipt.get(expected), 'Native mesh ' + actual)
        check_equal(len(mesh['xyz']), receipt.get('vertex_count'), 'Native vertex count')
        check_equal(len(mesh['triangles']), receipt.get('triangle_count'), 'Native face count')
        for actual, key in [(mesh['xyz'].astype('<f4').tobytes(), 'vertex_xyz_float32_le_sha256'),
                            (mesh['vertex_stamps'].astype('<u8').tobytes(), 'vertex_stamps_uint64_le_sha256'),
                            (mesh['triangles'].astype('<u4').tobytes(), 'triangles_uint32_le_sha256')]:
            check_equal(sha(actual), receipt.get(key), key)
        check_equal(mesh['canonical_sha256'], report['exact_native_mesh_hashes'][robot], 'Paired native mesh hash')
        meshes.append(mesh)
        sources.append(dict(robot=robot, receipt=receipt, mesh_raw=mesh_raw,
                            deformation_raw=deformation_raw, mesh_path=mesh_path, deformation_path=deformation_path))
    native_xyz = np.concatenate([m['xyz'] for m in meshes])
    ids = np.concatenate([np.full(len(m['xyz']), robot, np.uint8) for robot, m in enumerate(meshes)])
    source_colors = np.concatenate([np.full(len(m['xyz']), SOURCE_COLORS[robot], '<u4') for robot, m in enumerate(meshes)])
    if not np.array_equal(xyz, native_xyz.astype('<f4')) or not np.array_equal(robots, ids) or not np.array_equal(colors, source_colors):
        raise ValueError('Global archive changes native vertex order/XYZ or ownership colors')
    return dict(report=report, report_raw=report_raw, npz_raw=npz_raw, path=path,
                xyz=xyz, colors=colors, robots=robots, meshes=meshes, sources=sources)


def select_pair(run, cutoff_wall=None):
    epoch_path = run / 'epoch_config.json'
    expected_run = json.loads(epoch_path.read_text()).get('run_id') if epoch_path.exists() else None
    inventory_path = run / 'experiment_metrics/global_cloud_inventory.json'
    inventory = json.loads(inventory_path.read_text()).get('clouds', []) if inventory_path.exists() else None
    candidates, skipped = [], []
    for path in (run / 'global_display').glob('native_pair_*.json'):
        try:
            report = json.loads(path.read_text())
            if expected_run and report.get('run_id') != expected_run:raise ValueError('Different run ID')
            walls = [r['accepted_wall'] for r in report['native_receipts']]
            if len(walls) != 2 or not all(finite_number(w) for w in walls):raise ValueError('Invalid accepted times')
            wall = max(walls)
            if cutoff_wall is not None and (wall > cutoff_wall or report['cloud_stamp_ns'] > cutoff_wall * 1e9):
                raise ValueError('Native pair is after cutoff')
            recorded = None
            if cutoff_wall is not None and inventory is not None:
                matched = [c['record_wall'] for c in inventory if c.get('point_count') == report['points']
                           and c.get('header_stamp_ns') == report['cloud_stamp_ns']
                           and finite_number(c.get('record_wall')) and c['record_wall'] <= cutoff_wall]
                if not matched:raise ValueError('No matching nonempty cloud recorded before cutoff')
                recorded = min(matched)
            candidates.append((wall, path.name, path, recorded))
        except (ValueError, TypeError, KeyError, OSError) as error:
            skipped.append(dict(archive=path.name, reason=str(error)))
    for wall, _, path, recorded in sorted(candidates, reverse=True):
        try:
            pair = load_pair(run, path)
            pair['selection'] = dict(native_pair_accepted_wall=wall, cutoff_wall=cutoff_wall,
                matching_recorded_cloud_wall=recorded,
                cutoff_evidence='recorded_cloud_and_native_receipt_times' if recorded is not None else 'native_receipt_and_source_stamp_times',
                skipped_candidates=skipped)
            return pair
        except (ValueError, TypeError, KeyError, OSError) as error:
            skipped.append(dict(archive=path.name, reason=str(error)))
    raise ValueError('No complete verified native pair at requested cutoff: ' + json.dumps(skipped))


def ownership_rgba(colors):
    return np.column_stack(((colors >> 16) & 255, (colors >> 8) & 255, colors & 255,
                            np.full(len(colors), 255, np.uint32))).astype('u1')


def camera_rgba(mesh):
    if not len(mesh['rgba']):return np.full((len(mesh['xyz']), 4), 255, 'u1')
    return np.rint(np.clip(mesh['rgba'], 0, 1) * 255).astype('u1')


def write_ply(path, xyz, rgba, robot_ids, triangles=None):
    """Positions and vertex/face ordering are preserved; only color is 8-bit."""
    xyz = np.asarray(xyz)
    scalar = 'double' if xyz.dtype.itemsize == 8 else 'float'
    dtype = [('x', '<f8' if scalar == 'double' else '<f4'), ('y', '<f8' if scalar == 'double' else '<f4'),
             ('z', '<f8' if scalar == 'double' else '<f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
             ('alpha', 'u1'), ('robot_id', 'u1')]
    vertices = np.empty(len(xyz), dtype=dtype)
    for i, name in enumerate(('x', 'y', 'z')):vertices[name] = xyz[:, i]
    for i, name in enumerate(('red', 'green', 'blue', 'alpha')):vertices[name] = rgba[:, i]
    vertices['robot_id'] = robot_ids
    header = ['ply', 'format binary_little_endian 1.0', 'comment native geometry; no optimization, welding or resampling',
              'element vertex ' + str(len(xyz))]
    header += ['property ' + scalar + ' ' + name for name in ('x', 'y', 'z')]
    header += ['property uchar ' + name for name in ('red', 'green', 'blue', 'alpha', 'robot_id')]
    faces = None
    if triangles is not None:
        header += ['element face ' + str(len(triangles)), 'property list uchar uint vertex_indices']
        faces = np.empty(len(triangles), dtype=[('count', 'u1'), ('indices', '<u4', (3,))])
        faces['count'] = 3; faces['indices'] = triangles
    header += ['end_header', '']
    with path.open('wb') as f:
        f.write('\n'.join(header).encode());f.write(vertices.tobytes())
        if faces is not None:f.write(faces.tobytes())


def write_pcd(path, xyz, colors, robots):
    data = np.empty(len(xyz), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4'),('robot_id','u1')])
    for i, name in enumerate(('x','y','z')):data[name] = xyz[:,i]
    data['rgb'], data['robot_id'] = colors, robots
    header = '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb robot_id\nSIZE 4 4 4 4 1\nTYPE F F F U U\nCOUNT 1 1 1 1 1\nWIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {0}\nDATA binary\n'.format(len(xyz))
    path.write_bytes(header.encode() + data.tobytes())


def verify_existing(output, identity):
    manifest_path = output / 'export_manifest.json'
    if not manifest_path.is_file():raise FileExistsError('Existing output is not a verified export; choose a new directory')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('snapshot_identity') != identity:
        raise FileExistsError('Output contains a different native pair; choose a new directory')
    for name, record in manifest['files'].items():
        path = output / name
        if output.resolve() not in path.resolve().parents or not path.is_file() or sha(path.read_bytes()) != record['sha256']:
            raise FileExistsError('Existing export changed: ' + name)
    return manifest


def export(run_directory, output, cutoff_wall=None):
    run, output = Path(run_directory).resolve(), Path(output).resolve()
    if not run.is_dir() or run == output:raise ValueError('Use an existing run directory and a separate output directory')
    if cutoff_wall is not None and (not finite_number(cutoff_wall) or cutoff_wall <= 0):raise ValueError('Cutoff must be finite and positive')
    pair = select_pair(run, cutoff_wall);report = pair['report'];identity = report.get('snapshot_identity')
    if not isinstance(identity, str) or len(identity) != 64:raise ValueError('Missing native snapshot identity')
    if output.exists() and any(output.iterdir()):return verify_existing(output, identity)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.' + output.name + '-', dir=str(output.parent)) as temp:
        stage = Path(temp) / 'export';stage.mkdir();source = stage/'source';source.mkdir()
        (source/'native_pair.json').write_bytes(pair['report_raw']);(source/'native_pair.npz').write_bytes(pair['npz_raw'])
        for name in ('epoch_config.json', 'sensor_calibration_snapshot.json'):
            path = run/name
            if path.is_file():(source/name).write_bytes(path.read_bytes())
        native_xyz, native_rgba, native_ids, triangles = [], [], [], []
        offset = 0
        for robot, (mesh, original) in enumerate(zip(pair['meshes'], pair['sources'])):
            folder = source/('robot_' + str(robot));folder.mkdir()
            (folder/'native_mesh.rosbin').write_bytes(original['mesh_raw'])
            (folder/'deformation_proof.json').write_bytes(original['deformation_raw'])
            (folder/'accepted_receipt.json').write_bytes(json_bytes(original['receipt']))
            np.savez_compressed(folder/'native_arrays.npz', xyz=mesh['xyz'], triangles=mesh['triangles'],
                rgba=mesh['rgba'], vertex_stamps_ns=mesh['vertex_stamps'], graph_indices=mesh['graph_indices'])
            ids = np.full(len(mesh['xyz']), robot, 'u1');rgba=camera_rgba(mesh)
            write_ply(stage/('robot_' + str(robot) + '_native_mesh.ply'),mesh['xyz'],rgba,ids,mesh['triangles'])
            native_xyz.append(mesh['xyz']);native_rgba.append(rgba);native_ids.append(ids)
            shifted=mesh['triangles'].astype('<u8') + offset
            if shifted.max() > np.iinfo(np.uint32).max:raise ValueError('Merged PLY index exceeds uint32')
            triangles.append(shifted.astype('<u4'));offset += len(mesh['xyz'])
        merged_xyz=np.concatenate(native_xyz);merged_triangles=np.concatenate(triangles)
        write_ply(stage/'global_native_mesh.ply',merged_xyz,np.concatenate(native_rgba),np.concatenate(native_ids),merged_triangles)
        write_ply(stage/'global_cloud.ply',pair['xyz'],ownership_rgba(pair['colors']),pair['robots'])
        write_pcd(stage/'global_cloud.pcd',pair['xyz'],pair['colors'],pair['robots'])
        mission_files=sorted(run.glob('*_mission.json'));mission={}
        if mission_files:
            m=json.loads(mission_files[0].read_text());mission={k:m.get(k) for k in ('phase','final','completed_laps','laps_requested','failure_latch')}
        manifest=dict(format_version=1, run_id=report['run_id'], frame='kimera_world', units='metres',
            exported_at_utc=datetime.now(timezone.utc).isoformat(), snapshot_identity=identity,
            source_archive=pair['path'].name, selection=pair['selection'], native_message_type='kimera_pgmo_msgs/KimeraPgmoMesh',
            native_message_md5=MESSAGE_MD5, points=len(pair['xyz']), triangles=len(merged_triangles),
            robot_vertices=[len(m['xyz']) for m in pair['meshes']],robot_triangles=[len(m['triangles']) for m in pair['meshes']],
            native_mesh_hashes=report['exact_native_mesh_hashes'], completed_round_serial=report['completed_round_serial'],
            optimizer_binding=report.get('binding'),exact_shared_optimizer_round_id_available=False,
            geometry_operations='ordered native concatenation; triangle indices offset only; no estimation/welding/filtering',
            cloud_colors='robot ownership colors (cyan/orange), not camera RGB',
            mesh_colors='native camera RGBA rounded to 8-bit for PLY; exact original RGBA preserved in native_arrays.npz and rosbin',
            native_xyz='exact float64 coordinates in native arrays/native PLY; archived global cloud is original float32 representation',
            mission_at_export=mission, files={})
        readme='''This export is a verified archived native mesh pair, not a new map estimate.
Frame: kimera_world; units: metres. Source metadata and hashes are in export_manifest.json.
The mission outcome is recorded separately; a partial/aborted route is not a completed survey.

global_cloud.ply / .pcd: original archived float32 XYZ, robot_id, artificial robot colors.
global_native_mesh.ply: robot 0 then robot 1 native float64 vertices; native faces with index offset only.
robot_N_native_mesh.ply: native triangle meshes, original ordering and coordinates.
PLY mesh colors are 8-bit display conversions. Exact native float RGBA/XYZ/topology/stamps
remain in source/robot_N/native_arrays.npz and the original native_mesh.rosbin.
No ICP, resampling, vertex welding, optimization or replacement trajectory was performed.
Pair receipts preserve the original observational optimizer binding; no atomic round ID is claimed.
'''
        (stage/'README.txt').write_text(readme)
        for path in sorted(stage.rglob('*')):
            if path.is_file():manifest['files'][str(path.relative_to(stage))]={'sha256':sha(path.read_bytes()),'bytes':path.stat().st_size}
        (stage/'export_manifest.json').write_bytes(json_bytes(manifest))
        if output.exists():output.rmdir()  # Only an empty explicitly requested output directory.
        stage.rename(output)
    return manifest


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cutoff-wall',type=float)
    args=parser.parse_args(argv)
    try:result=export(args.run_directory,args.output,args.cutoff_wall)
    except (ValueError,OSError,KeyError,TypeError) as error:parser.exit(2,str(error)+'\n')
    print(json.dumps({'output':str(args.output),'source_archive':result['source_archive'],
                      'points':result['points'],'triangles':result['triangles'],
                      'snapshot_identity':result['snapshot_identity']},indent=2))


if __name__=='__main__':main()
