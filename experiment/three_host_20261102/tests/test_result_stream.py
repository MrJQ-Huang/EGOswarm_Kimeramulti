from pathlib import Path
import os
import socket
import struct
import sys
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'distributed'))
import result_wire as wire
from result_stream import LatestResults,ResultServer
from test_result_wire import header,source,receipt


class ResultStreamTests(unittest.TestCase):
    def latest(self): return LatestResults(0,'test_run',99000000000,'test_gauge','boot_a',now=lambda:101000000000)

    def test_source_validation_latest_bound_and_no_edge_recording(self):
        latest=self.latest()
        with patch('builtins.open',side_effect=AssertionError('Edge data recording')):
            for i in range(100):latest.ingest('actual',header(seq=i)+struct.pack('<I',i),source(0,'actual'))
            self.assertEqual(len(latest.latest),1)
            self.assertEqual(latest.latest['actual']['sequence'],100)
            with self.assertRaises(wire.WireError):latest.ingest('actual',header(stamp=1),source(0,'actual'))
            with self.assertRaises(wire.WireError):latest.ingest('actual',header(),dict(source(0,'actual'),callerid='/other'))

    def test_mesh_transport_sequence_alone_does_not_republish_geometry(self):
        latest=self.latest(); raw=header()+b'full original mesh'
        self.assertTrue(latest.ingest('mesh',raw,source(0,'mesh')))
        self.assertFalse(latest.ingest('mesh',header(seq=900)+raw[len(header()):],source(0,'mesh')))
        self.assertEqual(latest.latest['mesh']['raw'],raw)

    def test_zero_header_graph_uses_verified_native_node_times_without_rewriting_bytes(self):
        latest=self.latest();raw=header(stamp=0,frame='')+b'original graph nodes'
        with self.assertRaises(wire.WireError):latest.ingest('optimized_graph',raw,source(0,'optimized_graph'))
        with self.assertRaises(wire.WireError):latest.ingest('optimized_graph',raw,source(0,'optimized_graph'),[(1,100000000000)])
        self.assertTrue(latest.ingest('optimized_graph',raw,source(0,'optimized_graph'),[(0,100000000000)]))
        entry=dict(latest.entries('map')[0])['optimized_graph'];meta=latest.metadata('optimized_graph',entry)
        decoded,original=wire.decode_packet(wire.encode_packet(meta,raw))
        self.assertEqual(decoded['stamp_basis'],'latest_graph_node');self.assertEqual(original,raw)
        self.assertEqual(wire.read_header(original)[0],0)

    def test_async_receipt_enriches_exact_mesh_and_is_retained(self):
        latest=self.latest(); raw=header(frame='kimera_world')+b'original mesh'
        latest.ingest('mesh',raw,source(0,'mesh'))
        def status(value):
            data=wire.json_bytes(dict(run_id='test_run',robot_id=0,cutoff_ns=99000000000,
                shared_gauge_ready=True,shared_gauge_id='test_gauge',accepted_joint_mesh_receipt=value))
            latest.ingest('status',struct.pack('<I',len(data))+data,source(0,'status'))
        status(receipt(wire.content_hash(raw,'mesh')))
        entries,current=latest.entries('map')
        self.assertTrue(latest.metadata('mesh',dict(entries)['mesh'],current)['shared_gauge'])
        status(receipt('b'*64))
        entries,current=latest.entries('map')
        self.assertTrue(latest.metadata('mesh',dict(entries)['mesh'],current)['shared_gauge'])

    def test_map_and_telemetry_have_independent_socket_workers(self):
        latest=self.latest(); stop=threading.Event()
        servers=[ResultServer(latest,c,'127.0.0.1',0,r,stop,socket_timeout=.4)
                 for c,r in (('map',2.),('telemetry',30.))]
        threads=[threading.Thread(target=s.serve,daemon=True) for s in servers]
        for thread in threads:thread.start()
        for server in servers:self.assertTrue(server.ready.wait(2.));self.assertIsNone(server.error)
        try:
            # An idle map client never completes its subscribe handshake.
            with socket.create_connection(('127.0.0.1',servers[0].bound_port)) as blocked:
                with socket.create_connection(('127.0.0.1',servers[1].bound_port)) as telemetry:
                    telemetry.settimeout(1.)
                    wire.send_control(telemetry,dict(op='subscribe',protocol=1,robot_id=0,run_id='test_run',cutoff_ns=99000000000,channel='telemetry'))
                    self.assertEqual(wire.receive_control(telemetry)['boot_id'],'boot_a')
                    latest.ingest('actual',header()+b'fresh actual body',source(0,'actual'))
                    for _ in range(3):
                        meta,raw=wire.receive_packet(telemetry)
                        if meta['stream']=='actual':break
                    self.assertEqual(meta['stream'],'actual');self.assertTrue(raw.endswith(b'fresh actual body'))
        finally:
            stop.set()
            for thread in threads:thread.join(2.)

    def test_blocked_socket_never_holds_latest_replacement_lock(self):
        latest=self.latest();latest.ingest('local_mesh',header()+b'native',source(0,'local_mesh'))
        stop=threading.Event();blocked=threading.Event();release=threading.Event()
        class SlowConnection:
            def sendall(self,data):
                blocked.set();release.wait(2.)
        server=ResultServer(latest,'map','127.0.0.1',0,2.,stop)
        request=dict(op='subscribe',protocol=1,robot_id=0,run_id='test_run',cutoff_ns=99000000000,channel='map')
        with patch('result_stream.receive_control',return_value=request),patch('result_stream.send_control'):
            thread=threading.Thread(target=server.stream,args=(SlowConnection(),),daemon=True);thread.start()
            try:
                self.assertTrue(blocked.wait(1.))
                self.assertTrue(latest.ingest('actual',header()+b'continues while map blocked',source(0,'actual')))
                self.assertEqual(len(latest.entries('telemetry')[0]),1)
            finally:stop.set();release.set();thread.join(2.)

    def test_delta_missing_base_requests_exact_full_and_reconnect_starts_full(self):
        latest=self.latest();base=header()+os.urandom(wire.BLOCK_SIZE*6)
        latest.ingest('local_mesh',base,source(0,'local_mesh'))
        stop=threading.Event();server=ResultServer(latest,'map','127.0.0.1',0,100.,stop,socket_timeout=1.,delta=True)
        thread=threading.Thread(target=server.serve,daemon=True);thread.start();self.assertTrue(server.ready.wait(2.))
        def connect():
            connection=socket.create_connection(('127.0.0.1',server.bound_port));connection.settimeout(2.)
            wire.send_control(connection,dict(op='subscribe',protocol=1,robot_id=0,run_id='test_run',cutoff_ns=99000000000,channel='map',accept_delta=True))
            self.assertTrue(wire.receive_control(connection)['delta_enabled']);return connection
        def ack(connection,meta):
            wire.send_control(connection,dict(op='ack',stream=meta['stream'],snapshot_id=meta['snapshot_id'],result_sha256=meta['result_sha256']))
        try:
            with connect() as connection:
                meta,raw=wire.receive_packet(connection);self.assertEqual(meta['mode'],'full');self.assertEqual(raw,base);ack(connection,meta)
                changed=bytearray(base);changed[200:210]=b'correction';changed=bytes(changed)
                latest.ingest('local_mesh',changed,source(0,'local_mesh'))
                while True:
                    try:
                        meta,raw=wire.receive_packet(connection)
                        self.assertEqual(meta['stream'],'heartbeat')
                    except wire.MissingBase as missing:
                        wire.send_control(connection,dict(op='full_required',stream=missing.meta['stream'],snapshot_id=missing.meta['snapshot_id']))
                        meta,raw=wire.receive_packet(connection);self.assertEqual(meta['mode'],'full');self.assertEqual(raw,changed)
                        self.assertEqual(meta['snapshot_id'],missing.meta['snapshot_id']);ack(connection,meta);break
            with connect() as connection:
                meta,raw=wire.receive_packet(connection);self.assertEqual(meta['mode'],'full');self.assertEqual(raw,changed);ack(connection,meta)
            self.assertEqual(server.counts['missing_base_full_resends'],1)
        finally:stop.set();thread.join(2.)


if __name__=='__main__':unittest.main()
