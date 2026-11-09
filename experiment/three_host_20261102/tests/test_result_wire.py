import json
import os
from pathlib import Path
import socket
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'distributed'))
import result_wire as wire
from result_stream import LatestResults


def header(stamp=100000000000, frame='sim_0/vio_odom', seq=1):
    encoded = frame.encode()
    return struct.pack('<IIII',seq,stamp//10**9,stamp%10**9,len(encoded))+encoded


def source(robot, stream):
    spec = wire.contract(robot,stream)
    return dict(callerid=spec['source_caller'],type=spec['ros_type'],md5sum='a'*32)


def packet_fixture(stream='local_mesh', raw=None):
    raw = raw or header()+b'original vertices, original RGBA, triangles, native metadata'
    latest = LatestResults(0,'test_run',99000000000,'test_gauge','boot_a',now=lambda:101000000000)
    latest.ingest(stream,raw,source(0,stream))
    entries,status = latest.entries(wire.STREAMS[stream][0])
    meta = latest.metadata(stream,dict(entries)[stream],status)
    return meta,raw


def receipt(canonical_hash):
    return dict(run_id='test_run',robot_id=0,cutoff_ns=99000000000,
        canonical_native_mesh_sha256=canonical_hash,mesh_frame='kimera_world',
        shared_gauge_id='test_gauge',shared_gauge_ready=True,native_path_echo_verified=True,
        native_publish_iterate=False,completed_round_proof=dict(accepted_cross_constraints=1,
            round=dict(active_robots=[0,1],terminate_wall=100.)))


class ResultWireTests(unittest.TestCase):
    def test_verified_path_is_an_additive_v1_stream_with_original_source_contract(self):
        self.assertEqual(wire.VERSION, 1)
        self.assertEqual(wire.STREAMS['body_path'], ('map', 'nav_msgs/Path',
            '/sim_{r}/native_dense/approved_body_path', '/sim_{r}/native_dense_bridge'))
        raw = header(frame='kimera_world') + b'unchanged complete native path'
        meta, raw = packet_fixture('verified_body_path', raw)
        self.assertEqual(meta['source_topic'], '/sim_0/native_dense/verified_body_path')
        self.assertEqual(meta['source_caller'], '/sim_0/native_dense_bridge')
        decoded, original = wire.decode_packet(wire.encode_packet(meta, raw))
        self.assertEqual(decoded['stream'], 'verified_body_path')
        self.assertEqual(original, raw)
        latest = LatestResults(0, 'test_run', 99000000000, 'test_gauge', 'boot_a', now=lambda:101000000000)
        wrong = source(0, 'verified_body_path'); wrong['callerid'] = '/sim_0_peer_bridge'
        with self.assertRaises(wire.WireError): latest.ingest('verified_body_path', raw, wrong)

    def test_complete_original_payload_survives_compression_and_full_reconnect(self):
        meta,raw = packet_fixture(raw=header()+bytes(range(256))*400)
        packet = wire.encode_packet(meta,raw)
        decoded,original = wire.decode_packet(packet,dict(run_id='test_run',robot_id=0))
        self.assertEqual(original,raw)
        self.assertEqual(decoded['mode'],'full'); self.assertIsNone(decoded['base_sha256'])
        # No append-only assumption: historical bytes and topology can change.
        changed = raw[:80]+b'HISTORICAL VERTEX AND FACE REPLACEMENT'+raw[900:]
        newer,changed = packet_fixture(raw=changed)
        self.assertEqual(wire.decode_packet(wire.encode_packet(newer,changed))[1],changed)

    def test_wrong_epoch_source_identity_hash_and_trailing_data_rejected(self):
        meta,raw = packet_fixture(); packet=wire.encode_packet(meta,raw)
        for expected in ({'run_id':'other'},{'robot_id':1},{'boot_id':'other'}):
            with self.assertRaises(wire.WireError): wire.decode_packet(packet,expected)
        bad=dict(meta,source_caller='/relay')
        with self.assertRaises(wire.WireError): wire.encode_packet(bad,raw)
        with self.assertRaises(wire.WireError): wire.decode_packet(packet+b'extra')
        with self.assertRaises(wire.WireError): wire.decode_packet(packet[:-1])
        damaged=bytearray(packet); damaged[-1]^=1
        with self.assertRaises(wire.WireError): wire.decode_packet(bytes(damaged))

    def test_size_is_rejected_before_socket_body_read(self):
        class Connection:
            def __init__(self): self.calls=0
            def recv(self,size):
                self.calls+=1
                if self.calls>1: raise AssertionError('Read oversized body')
                return wire.PREFIX.pack(wire.MAGIC,10,wire.MAX_WIRE_BYTES+1)
        conn=Connection()
        with self.assertRaises(wire.WireError): wire.receive_packet(conn)
        self.assertEqual(conn.calls,1)

    def test_gauge_label_alone_is_not_proof(self):
        meta,raw=packet_fixture('mesh',header(frame='kimera_world')+b'native mesh')
        forged=dict(meta,shared_gauge=True)
        forged['snapshot_id']=wire.snapshot_identity(forged)
        with self.assertRaises(wire.WireError): wire.encode_packet(forged,raw)
        good=dict(meta,shared_gauge=True,native_receipt=receipt(meta['content_sha256']))
        good['snapshot_id']=wire.snapshot_identity(good)
        self.assertTrue(wire.decode_packet(wire.encode_packet(good,raw))[0]['shared_gauge'])

    def test_ros_header_and_duplicate_json_keys(self):
        self.assertEqual(wire.read_header(header()),(100000000000,'sim_0/vio_odom'))
        with self.assertRaises(wire.WireError): wire.parse_json(b'{"run":1,"run":2}')
        with self.assertRaises(wire.WireError): wire.read_header(header()[:17])

    def test_byte_delta_replaces_old_vertices_and_topology_and_requires_exact_base(self):
        base=header()+os.urandom(wire.BLOCK_SIZE*8)
        # Change a historical block and an arbitrary topology block, truncate
        # the old tail, and append a different face representation.
        changed=bytearray(base[:-wire.BLOCK_SIZE]);changed[200:220]=b'old vertices changed'
        changed[wire.BLOCK_SIZE*3+80:wire.BLOCK_SIZE*3+100]=b'old triangle changed'
        changed=bytes(changed)+b'new topology tail'
        meta,raw=packet_fixture(raw=changed)
        packet=wire.encode_packet(meta,raw,1,base)
        decoded,original=wire.decode_packet(packet,base=base)
        self.assertEqual(decoded['mode'],'blocks');self.assertEqual(decoded['base_sha256'],wire.sha256(base))
        self.assertEqual(original,changed)
        with self.assertRaises(wire.MissingBase):wire.decode_packet(packet)
        with self.assertRaises(wire.MissingBase):wire.decode_packet(packet,base=base+b'wrong base')
        self.assertEqual(wire.decode_packet(wire.encode_packet(meta,raw))[1],changed)

    def test_invalid_overlapping_and_trailing_delta_blocks_rejected(self):
        bad=struct.pack('!QI',10,2)+struct.pack('!QI',0,5)+b'12345'+struct.pack('!QI',3,2)+b'ab'
        with self.assertRaises(wire.WireError):wire.apply_delta(b'old content',bad,10)
        patch=wire.make_delta(b'abc',b'def')
        with self.assertRaises(wire.WireError):wire.apply_delta(b'abc',patch+b'extra',3)

    def test_bounded_decompression_rejects_expansion_beyond_declared_size(self):
        meta,raw=packet_fixture(raw=header()+b'x'*100000)
        packet=wire.encode_packet(meta,raw)
        magic,hsize,bsize=wire.PREFIX.unpack_from(packet)
        forged=wire.parse_json(packet[wire.PREFIX.size:wire.PREFIX.size+hsize])
        forged.update(raw_bytes=1,encoded_raw_bytes=1)
        forged['snapshot_id']=wire.snapshot_identity(forged)
        encoded=wire.json_bytes(forged)
        attack=wire.PREFIX.pack(magic,len(encoded),bsize)+encoded+packet[wire.PREFIX.size+hsize:]
        with self.assertRaises(wire.WireError):wire.decode_packet(attack)


if __name__ == '__main__': unittest.main()
