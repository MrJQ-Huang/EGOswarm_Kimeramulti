#!/usr/bin/env python3
"""Export two independently accepted native maps in their shared coordinates.

This is ordered concatenation of asynchronous observational native evidence,
not an atomic multi-robot snapshot, a new optimization, ICP or mesh welding.
The observer's latest pointers and expected sender boots select the inputs;
the exact original meshes, receipts and receipted body paths remain attached.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np

_HARNESS = Path(__file__).resolve().parents[2] / 'harness' / 'multimodal'
if str(_HARNESS) not in sys.path:
    sys.path.insert(0, str(_HARNESS))
from export_final_global_map import MESSAGE_MD5, decode_mesh
from observer import canonical_header_hash, verified_shared_body_path
from result_wire import (IDENTIFIER, MAX_HEADER_BYTES, MAX_RAW_BYTES, content_hash,
    identity, json_bytes, parse_json, read_header, receipt_proves_shared,
    sha256, validate_metadata)

PATH_MD5 = '6227e2b7e9cce15051f669a5e197bbf7'


class Sources:
    """Retain exact selected source bytes and detect writes during export."""
    def __init__(self):
        self.records = {}

    def read(self, path, maximum=MAX_RAW_BYTES):
        path = Path(path).resolve()
        with path.open('rb') as stream:
            raw = stream.read(maximum + 1)
        if len(raw) > maximum:
            raise ValueError('Source exceeds declared bound: ' + str(path))
        previous = self.records.get(path)
        if previous is not None and previous != raw:
            raise ValueError('Source changed during selection: ' + str(path))
        self.records[path] = raw
        return raw

    def remember(self, path, raw):
        path = Path(path).resolve()
        if path in self.records and self.records[path] != raw:
            raise ValueError('Source changed during selection')
        self.records[path] = raw

    def verify_unchanged(self):
        for path, raw in self.records.items():
            if path.stat().st_size != len(raw) or file_hash(path) != sha256(raw):
                raise ValueError('Source changed before export completion: ' + str(path))


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def child(directory, name):
    if not isinstance(name, str) or Path(name).name != name or name in ('', '.', '..'):
        raise ValueError('Archive pointer must name one contained filename')
    path = directory / name
    if path.resolve().parent != directory.resolve():
        raise ValueError('Archive symlink escapes selected directory')
    return path


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def verify_receipt(receipt, mesh, metadata):
    if not receipt_proves_shared(receipt, metadata['content_sha256'],
            metadata['frame_id'], metadata['gauge_id'], metadata['run_id'],
            metadata['robot_id'], metadata['cutoff_ns']):
        raise ValueError('No valid independent native shared-gauge evidence')
    if receipt.get('shared_gauge_id') != metadata['gauge_id']:
        raise ValueError('Native receipt does not explicitly name the shared gauge')
    checks = {'vertex_count': len(mesh['xyz']), 'triangle_count': len(mesh['triangles']),
        'mesh_header_stamp_ns': mesh['stamp_ns'],
        'canonical_native_mesh_sha256': mesh['canonical_sha256'],
        'vertex_xyz_float32_le_sha256': sha256(mesh['xyz'].astype('<f4').tobytes()),
        'triangles_uint32_le_sha256': sha256(mesh['triangles'].astype('<u4').tobytes()),
        'vertex_stamps_uint64_le_sha256': sha256(mesh['vertex_stamps'].astype('<u8').tobytes())}
    for key, expected in checks.items():
        if receipt.get(key) != expected:
            raise ValueError('Native receipt/mesh mismatch: ' + key)
    proof = receipt.get('optimization_evidence') or receipt.get('completed_round_proof')
    completed, weights = proof.get('round') or proof.get('completed_round'), proof.get('weights')
    if not isinstance(completed, dict) or not isinstance(weights, dict):
        raise ValueError('Missing observed completion or native weight batch')
    request, termination = completed.get('request_wall'), completed.get('terminate_wall')
    accepted, weight_wall = receipt.get('accepted_wall'), weights.get('received_wall')
    if (not all(finite(t) and t > 0 for t in (request, termination, accepted, weight_wall))
            or not request < termination <= accepted or not request <= weight_wall <= accepted):
        raise ValueError('Invalid observed native request/termination/weight window')
    owner, serial = completed.get('owner'), completed.get('serial')
    if (type(owner) is not int or owner not in (0, 1) or type(serial) is not int or serial < 1
            or receipt.get('optimizer_owner') != owner or receipt.get('completed_round_serial') != serial
            or weights.get('owner') != owner):
        raise ValueError('Inconsistent observed native optimization owner/counter')
    rows = weights.get('rows')
    if not isinstance(rows, list):
        raise ValueError('Missing immutable native weight rows')
    accepted_cross = 0
    for row in rows:
        if (not isinstance(row, list) or len(row) != 6 or
                any(type(row[k]) is not int or row[k] < 0 for k in (0, 1, 2, 3)) or
                row[0] not in (0, 1) or row[2] not in (0, 1) or
                not finite(row[4]) or not 0 <= row[4] <= 1 or type(row[5]) is not bool):
            raise ValueError('Malformed native loop-closure weights')
        accepted_cross += int(row[0] != row[2] and row[4] >= .5)
    weight_hash = sha256(json.dumps(rows, separators=(',', ':'), allow_nan=False).encode())
    if (weights.get('sha256') != weight_hash or
            receipt.get('final_native_weight_batch_sha256') != weight_hash or
            proof.get('accepted_cross_constraints') != accepted_cross or accepted_cross <= 0):
        raise ValueError('Native cross-robot weight proof is inconsistent')
    path = proof.get('upstream_dpgo_path')
    if (not isinstance(path, dict) or not finite(path.get('received_wall')) or
            not termination < path['received_wall'] <= accepted or path.get('round_serial') != serial):
        raise ValueError('Missing subsequent observed native completed path')


def load_snapshot(directory, robot, run_id, boot_id, gauge_id, sources, cutoff_ns=None):
    pointer_raw = sources.read(directory / 'latest.json', maximum=4096)
    pointer = parse_json(pointer_raw)
    if not isinstance(pointer, dict):
        raise ValueError('Invalid observer latest pointer')
    metadata_path, raw_path = child(directory, pointer.get('metadata')), child(directory, pointer.get('original_rosbin'))
    meta_raw = sources.read(metadata_path, maximum=MAX_HEADER_BYTES)
    metadata = parse_json(meta_raw)
    expected = dict(run_id=run_id, robot_id=robot, boot_id=boot_id,
                    gauge_id=gauge_id, stream='accepted_mesh', channel='map')
    if cutoff_ns is not None:
        expected['cutoff_ns'] = cutoff_ns
    validate_metadata(metadata, expected)
    stem = '%09d_%s' % (metadata['sequence'], metadata['snapshot_id'])
    if (pointer.get('snapshot_id') != metadata['snapshot_id'] or
            metadata_path.name != stem + '.json' or raw_path.name != stem + '.rosbin'):
        raise ValueError('Latest pointer does not select the declared immutable snapshot')
    if (metadata['shared_gauge'] is not True or metadata['frame_id'] != 'kimera_world'
            or metadata['ros_md5'] != MESSAGE_MD5):
        raise ValueError('Expected a proven native mesh with the pinned ROS schema')
    raw = sources.read(raw_path)
    if (len(raw) != metadata['raw_bytes'] or sha256(raw) != metadata['result_sha256']
            or content_hash(raw, 'accepted_mesh') != metadata['content_sha256']):
        raise ValueError('Archived native mesh bytes differ from their metadata')
    if read_header(raw) != (metadata['stamp_ns'], metadata['frame_id']):
        raise ValueError('Archived native mesh ROS Header differs')
    mesh = decode_mesh(raw, metadata['cutoff_ns'], robot)
    if mesh['rgba'].size and (np.any(mesh['rgba'] < 0) or np.any(mesh['rgba'] > 1)):
        raise ValueError('Native RGBA outside normalized source range')
    verify_receipt(metadata['native_receipt'], mesh, metadata)
    return dict(meta=metadata, raw=raw, mesh=mesh, metadata_raw=meta_raw,
                pointer_raw=pointer_raw, raw_path=raw_path, metadata_path=metadata_path)


def associated_body_path(boot_directory, mesh_record, sources):
    expected = mesh_record['meta']['native_receipt'].get('canonical_submitted_body_path_sha256')
    if expected is None:
        expected = mesh_record['meta']['native_receipt'].get('submitted_body_path_sha256')
    if not isinstance(expected, str) or len(expected) != 64:
        raise ValueError('Native receipt lacks its exact submitted body-path hash')
    # A fresh observer can receive a retained verified path alongside a newer
    # pending approved path. Old runs only have body_path archives; both source
    # contracts remain valid, and only the exact receipt hash selects content.
    candidates = [(boot_directory / stream, stream, path)
                  for stream in ('verified_body_path', 'body_path')
                  for path in sorted((boot_directory / stream).glob('*.json'), reverse=True)]
    for directory, path_stream, meta_path in candidates:
        if meta_path.name == 'latest.json':
            continue
        meta_path = child(directory, meta_path.name)
        with meta_path.open('rb') as stream:
            meta_raw = stream.read(MAX_HEADER_BYTES + 1)
        if len(meta_raw) > MAX_HEADER_BYTES:
            raise ValueError('Oversized body-path metadata')
        metadata = parse_json(meta_raw)
        expected_meta = {k: mesh_record['meta'][k] for k in
                         ('run_id', 'cutoff_ns', 'robot_id', 'boot_id', 'gauge_id')}
        expected_meta.update(stream=path_stream, channel='map')
        validate_metadata(metadata, expected_meta)
        if metadata['ros_md5'] != PATH_MD5:
            raise ValueError('Unexpected native body-path ROS schema')
        stem = '%09d_%s' % (metadata['sequence'], metadata['snapshot_id'])
        if meta_path.name != stem + '.json':
            raise ValueError('Body-path archive filename differs from snapshot')
        raw_path = child(directory, stem + '.rosbin')
        with raw_path.open('rb') as stream:
            raw = stream.read(MAX_RAW_BYTES + 1)
        if len(raw) > MAX_RAW_BYTES:
            raise ValueError('Oversized archived body path')
        if canonical_header_hash(raw) != expected:
            continue
        path_record = dict(meta=metadata, raw=raw)
        if not verified_shared_body_path(mesh_record, path_record):
            raise ValueError('Submitted native body path does not match its mesh receipt')
        sources.remember(meta_path, meta_raw)
        sources.remember(raw_path, raw)
        return dict(path_record, metadata_raw=meta_raw, metadata_path=meta_path, raw_path=raw_path)
    raise ValueError('Exact receipted native body path is not present in the selected boot archive')


def choose_boots(run_id, gauge_id, boots=None, observer_status=None, sources=None):
    selected = dict(boots or {})
    cutoff = None
    if observer_status is not None:
        state = parse_json(sources.read(observer_status, maximum=MAX_HEADER_BYTES))
        if state.get('run_id') != run_id or state.get('gauge_id') != gauge_id:
            raise ValueError('Observer state belongs to a different run/gauge')
        cutoff = state.get('cutoff_ns')
        from_state = {r: state.get('robots', {}).get(str(r), {}).get('current_boot_id') for r in (0, 1)}
        if selected and selected != from_state:
            raise ValueError('Explicit expected boots differ from current observer state')
        selected = from_state
    if set(selected) != {0, 1}:
        raise ValueError('Pin both current result-sender boots using --boot or --observer-status')
    if not isinstance(gauge_id, str) or not IDENTIFIER.fullmatch(gauge_id):
        raise ValueError('Invalid expected shared gauge ID')
    for robot, boot in selected.items():
        identity(robot, run_id, cutoff or 1, boot)
        if boot is None:
            raise ValueError('Missing current result sender boot')
    return selected, cutoff


def write_combined_ply(path, meshes):
    counts = [len(m['xyz']) for m in meshes]
    total = sum(counts)
    if total > np.iinfo(np.uint32).max:
        raise ValueError('Combined face indices exceed uint32')
    dtype = [('x', '<f8'), ('y', '<f8'), ('z', '<f8'),
             ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'), ('alpha', 'u1'),
             ('native_red', '<f4'), ('native_green', '<f4'), ('native_blue', '<f4'), ('native_alpha', '<f4'),
             ('native_color_available', 'u1'), ('robot_id', 'u1')]
    vertices = np.empty(total, dtype=dtype)
    face_parts, offset = [], 0
    for robot, mesh in enumerate(meshes):
        count = counts[robot]
        part = vertices[offset:offset + count]
        for axis, name in enumerate(('x', 'y', 'z')):
            part[name] = mesh['xyz'][:, axis]
        has_color = bool(len(mesh['rgba']))
        rgba = mesh['rgba'] if has_color else np.zeros((count, 4), dtype='<f4')
        display = np.rint(np.clip(rgba, 0, 1) * 255).astype('u1') if has_color else np.full((count, 4), 255, 'u1')
        for channel, name in enumerate(('red', 'green', 'blue', 'alpha')):
            part[name], part['native_' + name] = display[:, channel], rgba[:, channel]
        part['native_color_available'], part['robot_id'] = int(has_color), robot
        face_parts.append((mesh['triangles'].astype('<u8') + offset).astype('<u4'))
        offset += count
    triangles = np.concatenate(face_parts)
    faces = np.empty(len(triangles), dtype=[('count', 'u1'), ('indices', '<u4', (3,))])
    faces['count'], faces['indices'] = 3, triangles
    header = ['ply', 'format binary_little_endian 1.0',
        'comment asynchronous independently accepted native submaps; no atomic joint snapshot',
        'comment original coordinates and topology; triangle index offset only',
        'element vertex ' + str(total)]
    header += ['property double ' + n for n in ('x', 'y', 'z')]
    header += ['property uchar ' + n for n in ('red', 'green', 'blue', 'alpha')]
    header += ['property float native_' + n for n in ('red', 'green', 'blue', 'alpha')]
    header += ['property uchar native_color_available', 'property uchar robot_id',
        'element face ' + str(len(faces)), 'property list uchar uint vertex_indices', 'end_header', '']
    with Path(path).open('wb') as stream:
        stream.write('\n'.join(header).encode('ascii'))
        stream.write(vertices.tobytes())
        stream.write(faces.tobytes())
    return total, len(faces)


def verify_existing(output, export_identity):
    manifest_path = output / 'export_manifest.json'
    if not manifest_path.is_file():
        raise FileExistsError('Existing output is not a verified distributed export')
    manifest = parse_json(manifest_path.read_bytes())
    if manifest.get('export_identity') != export_identity:
        raise FileExistsError('Existing output contains a different source selection')
    expected = set(manifest['files']) | {'export_manifest.json'}
    observed = {str(p.relative_to(output)) for p in output.rglob('*') if p.is_file()}
    if observed != expected:
        raise FileExistsError('Existing export file set differs')
    for name, record in manifest['files'].items():
        path = output / name
        if output.resolve() not in path.resolve().parents or file_hash(path) != record['sha256'] or path.stat().st_size != record['bytes']:
            raise FileExistsError('Existing export has changed: ' + name)
    return manifest


def export(observer_directory, run_id, gauge_id, output_directory, *, boots=None,
           observer_status=None, cutoff_ns=None):
    root, output = Path(observer_directory).resolve(), Path(output_directory).resolve()
    if not root.is_dir() or root == output:
        raise ValueError('Use an existing observer archive and a separate export directory')
    sources = Sources()
    boots, observed_cutoff = choose_boots(run_id, gauge_id, boots, observer_status, sources)
    if cutoff_ns is not None and observed_cutoff is not None and cutoff_ns != observed_cutoff:
        raise ValueError('Expected cutoff differs from observer state')
    cutoff_ns = cutoff_ns if cutoff_ns is not None else observed_cutoff
    records, paths = [], []
    for robot in (0, 1):
        directory = root / run_id / ('sim_%d' % robot) / boots[robot]
        record = load_snapshot(directory / 'accepted_mesh', robot, run_id,
                               boots[robot], gauge_id, sources, cutoff_ns)
        if cutoff_ns is None:
            cutoff_ns = record['meta']['cutoff_ns']
        records.append(record)
        paths.append(associated_body_path(directory, record, sources))
    identity_data = dict(run_id=run_id, cutoff_ns=cutoff_ns, gauge_id=gauge_id,
        sources=[{k: r['meta'][k] for k in ('robot_id', 'boot_id', 'snapshot_id', 'result_sha256')} for r in records])
    export_identity = sha256(json_bytes(identity_data))
    sources.verify_unchanged()
    if output.exists() and any(output.iterdir()):
        return verify_existing(output, export_identity)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.' + output.name + '-', dir=str(output.parent)) as temporary:
        stage = Path(temporary) / 'export'; stage.mkdir()
        for robot, (record, path) in enumerate(zip(records, paths)):
            folder = stage / 'source' / ('robot_%d' % robot); folder.mkdir(parents=True)
            (folder / 'native_mesh.rosbin').write_bytes(record['raw'])
            (folder / 'result_metadata.json').write_bytes(record['metadata_raw'])
            (folder / 'observer_latest_pointer.json').write_bytes(record['pointer_raw'])
            (folder / 'native_receipt.json').write_bytes(json_bytes(record['meta']['native_receipt']) + b'\n')
            (folder / 'submitted_body_path.rosbin').write_bytes(path['raw'])
            (folder / 'body_path_metadata.json').write_bytes(path['metadata_raw'])
        if observer_status is not None:
            (stage / 'source' / 'observer_status.json').write_bytes(sources.records[Path(observer_status).resolve()])
        vertices, triangles = write_combined_ply(stage / 'global_native_mesh.ply', [r['mesh'] for r in records])
        manifest = dict(format_version=1, run_id=run_id, cutoff_ns=cutoff_ns,
            gauge_id=gauge_id, frame='kimera_world', units='metres',
            exported_at_utc=datetime.now(timezone.utc).isoformat(), export_identity=export_identity,
            source_selection=identity_data, result_sender_boots={str(r): boots[r] for r in (0, 1)},
            geometry_operations='robot 0 then robot 1 native vertex/color concatenation; face index offset only',
            native_xyz='unchanged float64; no ICP, trajectory fit, scale, filtering, welding or resampling',
            native_colors='source float32 RGBA preserved in native_* PLY fields and rosbin; uchar channels are display derivatives',
            missing_color_policy='native_color_available=0, native_* zeros and white display only if source has no colors',
            points=vertices, triangles=triangles, native_message_type='kimera_pgmo_msgs/KimeraPgmoMesh',
            native_message_md5=MESSAGE_MD5, asynchronous_observational_native_evidence=True,
            atomic_multi_robot_snapshot=False, exact_shared_optimizer_round_id_available=False,
            binding='independent observed native completed paths/weights and PGMO meshes; not a shared atomic optimization commit',
            boot_selection='observer current_boot_id report' if observer_status is not None else 'explicit caller-pinned expected result sender boots',
            input_selection_stable_through_export=True,
            robots={str(robot): dict(vertices=len(record['mesh']['xyz']),
                triangles=len(record['mesh']['triangles']), vertex_offset=sum(len(r['mesh']['xyz']) for r in records[:robot]),
                mesh_header_stamp_ns=record['mesh']['stamp_ns'],
                snapshot_id=record['meta']['snapshot_id'], native_receipt=record['meta']['native_receipt'],
                original_rosbin_sha256=sha256(record['raw']), canonical_native_mesh_sha256=record['mesh']['canonical_sha256'],
                native_rgba_float32_sha256=sha256(record['mesh']['rgba'].astype('<f4').tobytes()),
                associated_body_path_snapshot_id=paths[robot]['meta']['snapshot_id']) for robot, record in enumerate(records)},
            source_inputs=[dict(path=str(path), bytes=len(raw), sha256=sha256(raw)) for path, raw in sorted(sources.records.items())],
            files={})
        (stage / 'README.txt').write_text(
            'Two independently accepted native submaps in kimera_world.\n'
            'This is asynchronous observational native evidence, not an atomic joint snapshot.\n'
            'Only concatenation and triangle-index offsets were performed; native positions/colors/topology remain attached.\n'
            'Each robot may refer to a different observed completion window. No equal native optimization generation is claimed.\n'
            'The PLY native_* float fields preserve source RGBA; standard uchar colors are display conversions.\n'
            'Exact ROS mesh/body-path bytes, independent receipts and hashes are retained in source/.\n')
        for path in sorted(stage.rglob('*')):
            if path.is_file():
                manifest['files'][str(path.relative_to(stage))] = dict(bytes=path.stat().st_size, sha256=file_hash(path))
        (stage / 'export_manifest.json').write_bytes(json_bytes(manifest) + b'\n')
        sources.verify_unchanged()
        if output.exists():
            output.rmdir()  # Only an empty explicitly selected output directory.
        stage.rename(output)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observer-directory', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--gauge-id', required=True)
    parser.add_argument('--boot', action='append', help='ROBOT=expected current result-sender boot')
    parser.add_argument('--observer-status', type=Path, help='Pinned observer report with current_boot_id for both robots')
    parser.add_argument('--cutoff-ns', type=int)
    parser.add_argument('--output-directory', type=Path, required=True)
    args = parser.parse_args(argv)
    boots = {}
    try:
        for value in args.boot or []:
            key, boot = value.split('=', 1); robot = int(key)
            if robot not in (0, 1) or robot in boots:
                raise ValueError('Duplicate or unknown boot owner')
            boots[robot] = boot
        result = export(args.observer_directory, args.run_id, args.gauge_id,
            args.output_directory, boots=boots or None,
            observer_status=args.observer_status, cutoff_ns=args.cutoff_ns)
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(2, str(error) + '\n')
    print(json.dumps(dict(output=str(args.output_directory), points=result['points'],
        triangles=result['triangles'], export_identity=result['export_identity'],
        atomic_multi_robot_snapshot=False), indent=2))


if __name__ == '__main__':
    main()
