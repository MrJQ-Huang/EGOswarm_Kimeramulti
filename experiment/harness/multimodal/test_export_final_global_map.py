#!/usr/bin/env python3
"""Native message decoding, output geometry and cutoff/integrity contracts."""
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np

import export_final_global_map as export_map


def native_bytes(xyz, faces, stamps, rgba, robot=0):
    buf = io.BytesIO()
    def uint(value):buf.write(struct.pack('<I', value))
    def string(value):
        data=value.encode();uint(len(data));buf.write(data)
    uint(57);uint(101);uint(0);string('kimera_world');string(str(robot))
    for values,dtype in [(faces,'<u4'),(xyz,'<f8'),(stamps,'<u4')]:
        values=np.asarray(values,dtype=dtype);uint(len(values));buf.write(values.tobytes())
    uint(len(xyz));buf.write(np.arange(len(xyz),dtype='<i4').tobytes())
    rgba=np.asarray(rgba,dtype='<f4');uint(len(rgba));buf.write(rgba.tobytes())
    return buf.getvalue()


def fixture(run, serial=1, accepted_wall=200.):
    run.mkdir(exist_ok=True);folder=run/'global_display';folder.mkdir(exist_ok=True)
    (run/'epoch_config.json').write_text(json.dumps({'run_id':'test_run'}))
    (run/'sensor_calibration_snapshot.json').write_text(json.dumps({'test_calibration':True}))
    meshes=[];receipts=[];all_xyz=[];all_colors=[];all_ids=[]
    weights=[[0,0,1,0,1.,0.]];weight_sha=export_map.sha(json.dumps(weights,separators=(',',':')).encode())
    for robot in (0,1):
        directory=run/('dense'+str(robot))/'bridge';directory.mkdir(parents=True,exist_ok=True)
        xyz=np.asarray([[robot+serial+.123456789,0,0],[robot+serial+1,0,0],[robot+serial,1,0]],dtype='<f8')
        stamps=np.array([[101,0]]*3,dtype='<u4');faces=np.array([[0,1,2]],dtype='<u4')
        rgba=np.array([[.1,.2,.3,1.]]*3,dtype='<f4');raw=native_bytes(xyz,faces,stamps,rgba,robot)
        mesh_name='native_transaction_%04d_native_after.rosbin'%serial
        proof_name='native_transaction_%04d_native_deformation_proof.json'%serial
        (directory/mesh_name).write_bytes(raw)
        proof={'native_path_echo_verified':True,'historical_vertices_moved':1}
        (directory/proof_name).write_text(json.dumps(proof))
        canonical=export_map.sha(b'\0\0\0\0'+raw[4:])
        receipt=dict(run_id='test_run',cutoff_ns=10**9,robot_id=robot,mesh_frame='kimera_world',
            optimizer_owner=0,completed_round_serial=serial,final_native_weight_batch_sha256=weight_sha,
            immutable_until_next_accepted_transaction=True,accepted_wall=accepted_wall,
            completed_round_proof={'accepted_cross_constraints':1,'weights':{'rows':weights}},
            native_after_mesh_file=mesh_name,deformation_proof_file=proof_name,native_deformation_proof=proof,
            canonical_native_mesh_sha256=canonical,mesh_header_stamp_ns=101*10**9,
            vertex_count=3,triangle_count=1,
            vertex_xyz_float32_le_sha256=export_map.sha(xyz.astype('<f4').tobytes()),
            vertex_stamps_uint64_le_sha256=export_map.sha(np.full(3,101*10**9,dtype='<u8').tobytes()),
            triangles_uint32_le_sha256=export_map.sha(faces.tobytes()))
        receipts.append(receipt);meshes.append((xyz,faces,rgba,raw))
        all_xyz.append(xyz.astype('<f4'));all_colors.append(np.full(3,export_map.SOURCE_COLORS[robot],'<u4'));all_ids.append(np.full(3,robot,'u1'))
    xyz=np.concatenate(all_xyz);colors=np.concatenate(all_colors);ids=np.concatenate(all_ids)
    npz_name='native_pair_%04d.npz'%serial;np.savez_compressed(folder/npz_name,xyz=xyz,packed_colors=colors,robot_ids=ids)
    hashes=[r['canonical_native_mesh_sha256'] for r in receipts]
    identity=export_map.sha(json.dumps(['test_run',10**9,hashes,0,serial,weight_sha],separators=(',',':')).encode())
    report=dict(run_id='test_run',cutoff_ns=10**9,fusion_ready=True,frame='kimera_world',points=6,
        archive_npz=npz_name,native_receipts=receipts,exact_native_mesh_hashes=hashes,
        optimizer_owner=0,completed_round_serial=serial,final_native_weight_batch_sha256=weight_sha,
        snapshot_identity=identity,cloud_stamp_ns=101*10**9,
        ordered_xyz_float32_le_sha256=export_map.sha(xyz.tobytes()),
        ordered_colors_uint32_le_sha256=export_map.sha(colors.tobytes()),
        binding='matched_native_observations_no_shared_atomic_round_id')
    path=folder/('native_pair_%04d.json'%serial);path.write_text(json.dumps(report))
    return path,report,meshes


def read_ply(path):
    raw=path.read_bytes();boundary=raw.index(b'end_header\n')+len(b'end_header\n')
    header=raw[:boundary].decode();count=int(next(s.split()[-1] for s in header.splitlines() if s.startswith('element vertex')))
    wide='property double x' in header;scalar='<f8' if wide else '<f4'
    dtype=[('x',scalar),('y',scalar),('z',scalar),('red','u1'),('green','u1'),('blue','u1'),('alpha','u1'),('robot_id','u1')]
    vertices=np.frombuffer(raw,dtype=dtype,count=count,offset=boundary)
    face_line=next((s for s in header.splitlines() if s.startswith('element face')),None)
    faces=None
    if face_line:
        faces=np.frombuffer(raw,dtype=[('count','u1'),('indices','<u4',(3,))],count=int(face_line.split()[-1]),offset=boundary+vertices.nbytes)
    return vertices,faces


class ExportContracts(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.run=self.root/'run'
    def tearDown(self):self.temp.cleanup()

    def test_native_geometry_original_colors_and_merged_index_offset(self):
        path,report,meshes=fixture(self.run)
        result=export_map.export(self.run,self.root/'export')
        self.assertEqual(result['points'],6);self.assertEqual(result['triangles'],2)
        vertices,faces=read_ply(self.root/'export/global_native_mesh.ply')
        xyz=np.column_stack([vertices[k] for k in ('x','y','z')])
        np.testing.assert_array_equal(xyz,np.concatenate([m[0] for m in meshes]))
        np.testing.assert_array_equal(faces['indices'],[[0,1,2],[3,4,5]])
        np.testing.assert_array_equal(vertices['robot_id'],[0,0,0,1,1,1])
        for robot,mesh in enumerate(meshes):
            folder=self.root/'export/source'/('robot_'+str(robot))
            self.assertEqual((folder/'native_mesh.rosbin').read_bytes(),mesh[3])
            with np.load(folder/'native_arrays.npz') as arrays:
                np.testing.assert_array_equal(arrays['xyz'],mesh[0]);np.testing.assert_array_equal(arrays['rgba'],mesh[2])
        cloud,_=read_ply(self.root/'export/global_cloud.ply')
        self.assertNotEqual(int(cloud['red'][0]),int(vertices['red'][0]))
        pcd=(self.root/'export/global_cloud.pcd').read_bytes();offset=pcd.index(b'DATA binary\n')+len(b'DATA binary\n')
        self.assertEqual(len(pcd)-offset,6*17)
        self.assertEqual(result['exact_shared_optimizer_round_id_available'],False)

    def test_cutoff_uses_recorded_publication_evidence(self):
        fixture(self.run,1,200);fixture(self.run,2,300)
        directory=self.run/'experiment_metrics';directory.mkdir()
        (directory/'global_cloud_inventory.json').write_text(json.dumps({'clouds':[{'record_wall':201,'point_count':6,'header_stamp_ns':101*10**9}]}))
        pair=export_map.select_pair(self.run,250)
        self.assertEqual(pair['path'].name,'native_pair_0001.json')
        self.assertEqual(pair['selection']['matching_recorded_cloud_wall'],201)
        with self.assertRaises(ValueError):export_map.select_pair(self.run,199)

    def test_newest_damaged_archive_is_skipped_explicitly(self):
        fixture(self.run,1,200);path,report,_=fixture(self.run,2,300)
        (self.run/'dense1/bridge'/report['native_receipts'][1]['native_after_mesh_file']).write_bytes(b'truncated')
        pair=export_map.select_pair(self.run)
        self.assertEqual(pair['path'].name,'native_pair_0001.json')
        self.assertTrue(any(x['archive']==path.name for x in pair['selection']['skipped_candidates']))

    def test_corrupt_input_is_rejected_without_output(self):
        path,report,_=fixture(self.run)
        report['snapshot_identity']='f'*64;path.write_text(json.dumps(report))
        with self.assertRaises(ValueError):export_map.export(self.run,self.root/'export')
        self.assertFalse((self.root/'export').exists())

    def test_native_parser_rejects_nonfinite_and_bad_triangles(self):
        xyz=np.array([[0,0,0],[1,0,0],[0,1,0]],'<f8');stamps=[[101,0]]*3;rgba=[[1,1,1,1]]*3
        for data in [native_bytes(xyz,[[0,1,3]],stamps,rgba),native_bytes([[float('nan'),0,0],[1,0,0],[0,1,0]],[[0,1,2]],stamps,rgba),b'bad']:
            with self.subTest(length=len(data)):
                with self.assertRaises(ValueError):export_map.decode_mesh(data,10**9,0)

    def test_existing_output_is_idempotent_and_never_replaced_by_new_pair(self):
        fixture(self.run,1,200);output=self.root/'export'
        export_map.export(self.run,output);before=(output/'export_manifest.json').read_bytes()
        export_map.export(self.run,output);self.assertEqual((output/'export_manifest.json').read_bytes(),before)
        fixture(self.run,2,300)
        with self.assertRaises(FileExistsError):export_map.export(self.run,output)
        self.assertEqual((output/'export_manifest.json').read_bytes(),before)
        (output/'global_cloud.ply').write_bytes(b'changed')
        with self.assertRaises(FileExistsError):export_map.export(self.run,output,250)


if __name__=='__main__':unittest.main()
