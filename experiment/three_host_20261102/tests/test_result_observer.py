import copy
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace as S
import types
import unittest
from unittest.mock import patch,Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'distributed'))
import result_wire as wire
from observer import (ObserverState,ArchiveWriter,export_ply,canonical_header_hash,
    verified_shared_body_path,channel_configuration,RosTelemetryObserver,RosObserver,argument_parser)
from test_result_wire import packet_fixture,receipt
from test_result_wire import header,source
from result_stream import LatestResults


class ResultObserverTests(unittest.TestCase):
    def test_fresh_mesh_with_old_vertex_header_is_not_stale(self):
        now=[0.]
        state=ObserverState('test_run',99000000000,'test_gauge',clock=lambda:now[0],
                            wall_clock=lambda:200000000000)
        state.connected(0,'map','boot_a')
        for stream in wire.MESH_STREAMS:
            meta,raw=packet_fixture(stream,header(stamp=100000000000)+b'freshly deformed mesh')
            state.ingest(meta,raw)
            report=state.report()['robots']['0']['streams'][stream]
            self.assertFalse(report['stale'])
            self.assertEqual(report['last_receive_age_s'],0.)
            self.assertEqual(report['source_age_s'],100.)
            self.assertFalse(report['source_age_used_for_freshness'])
            self.assertIn('vertex-stamp-derived',report['source_stamp_semantics'])
            self.assertIn('not publish time',report['source_stamp_semantics'])

    def test_mesh_without_new_snapshot_or_with_disconnected_link_is_stale(self):
        now=[0.]
        state=ObserverState('test_run',99000000000,'test_gauge',clock=lambda:now[0],
                            wall_clock=lambda:200000000000)
        meta,raw=packet_fixture();state.connected(0,'map','boot_a');state.ingest(meta,raw)
        now[0]=6.
        self.assertTrue(state.report()['robots']['0']['streams']['local_mesh']['stale'])
        # Link activity does not make an unchanged old snapshot newly received.
        state.connected(0,'map','boot_a')
        self.assertTrue(state.report()['robots']['0']['streams']['local_mesh']['stale'])
        now[0]=0.;state.disconnected(0,'map','boot_a','lost')
        self.assertTrue(state.report()['robots']['0']['streams']['local_mesh']['stale'])
        state.connected(0,'map','boot_b')
        self.assertTrue(state.report()['robots']['0']['streams']['local_mesh']['stale'])

    def test_odometry_and_paths_still_use_old_source_header_age(self):
        state=ObserverState('test_run',99000000000,'test_gauge',clock=lambda:0.,
                            wall_clock=lambda:200000000000)
        state.connected(0,'telemetry','boot_a');state.connected(0,'map','boot_a')
        for stream in ('actual','vio','body_path','verified_body_path'):
            meta,raw=packet_fixture(stream,header(stamp=100000000000)+b'old stamped source')
            state.ingest(meta,raw)
            report=state.report()['robots']['0']['streams'][stream]
            self.assertTrue(report['stale'])
            self.assertTrue(report['source_age_used_for_freshness'])
            self.assertEqual(report['last_receive_age_s'],0.)

    def test_disconnect_keeps_last_known_and_boot_change_does_not_mix(self):
        now=[0.];state=ObserverState('test_run',99000000000,'test_gauge',clock=lambda:now[0])
        meta,raw=packet_fixture();state.connected(0,'map','boot_a');self.assertTrue(state.ingest(meta,raw))
        state.disconnected(0,'map','boot_a','test link loss')
        self.assertTrue(state.report()['robots']['0']['streams']['local_mesh']['stale'])
        self.assertEqual(state.records[0,'local_mesh']['raw'],raw)
        state.connected(0,'map','boot_b')
        with self.assertRaises(wire.WireError):state.ingest(meta,raw)
        with self.assertRaises(wire.WireError):state.connected(0,'telemetry','boot_a')

    def test_duplicate_replay_and_same_sequence_geometry_mutation(self):
        state=ObserverState('test_run',99000000000,'test_gauge');meta,raw=packet_fixture()
        state.connected(0,'map','boot_a');self.assertTrue(state.ingest(meta,raw));self.assertFalse(state.ingest(meta,raw))
        bad=dict(meta,result_sha256='b'*64);bad['snapshot_id']=wire.snapshot_identity(bad)
        with self.assertRaises(wire.WireError):state.ingest(bad,b'other bytes')

    def test_hub_archive_preserves_native_bytes(self):
        meta,raw=packet_fixture()
        with tempfile.TemporaryDirectory() as directory:
            ArchiveWriter(directory).archive(meta,raw)
            files=list(Path(directory).rglob('*.rosbin'))
            self.assertEqual(len(files),1);self.assertEqual(files[0].read_bytes(),raw)

    def test_ply_retains_full_xyz_native_rgba_and_faces(self):
        mesh=S(vertices=[S(x=1.000000000001,y=2.,z=3.),S(x=4.,y=5.,z=6.),S(x=7.,y=8.,z=9.)],
            vertex_colors=[S(r=.123456789,g=.25,b=.5,a=1.) for _ in range(3)],triangles=[S(vertex_indices=[2,0,1])])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'native.ply';export_ply(mesh,path);data=path.read_bytes()
            header,body=data.split(b'end_header\n',1)
            self.assertIn(b'property double native_red',header)
            self.assertEqual(struct.unpack_from('<ddd',body),(1.000000000001,2.,3.))
            self.assertEqual(struct.unpack_from('<4d',body,28),(.123456789,.25,.5,1.))
            self.assertEqual(struct.unpack_from('<BIII',body,3*60),(3,2,0,1))

    def test_shared_overlay_requires_both_proven_receipts_but_not_equal_local_serials(self):
        state=ObserverState('test_run',99000000000,'test_gauge')
        for robot in (0,1):
            latest=LatestResults(robot,'test_run',99000000000,'test_gauge','boot_'+str(robot),now=lambda:101000000000)
            latest.ingest('mesh',header(frame='kimera_world')+b'native mesh',source(robot,'mesh'))
            entry=dict(latest.entries('map')[0])['mesh'];meta=latest.metadata('mesh',entry,{})
            state.connected(robot,'map',latest.boot_id)
            if robot==0:
                state.ingest(meta,entry['raw']);self.assertIsNone(state.shared_pair())
            proof=receipt(meta['content_sha256']);proof['robot_id']=robot
            proof['completed_round_proof']['round']['serial']=10+robot
            meta.update(native_receipt=proof,shared_gauge=True);meta['snapshot_id']=wire.snapshot_identity(meta)
            state.ingest(meta,entry['raw'])
        self.assertIsNotNone(state.shared_pair())
        self.assertFalse(state.report()['shared_display_is_atomic_native_round'])
        state.connected(0,'map','restarted_boot')
        self.assertIsNone(state.shared_pair())

    def body_path_records(self,empty=False):
        pose=header(frame='kimera_world')+struct.pack('<7d',1.,2.,3.,0.,0.,0.,1.)
        raw=header(frame='kimera_world')+struct.pack('<I',0 if empty else 1)+(b'' if empty else pose)
        path_meta,path_raw=packet_fixture('body_path',raw)
        mesh_meta,mesh_raw=packet_fixture('mesh',header(frame='kimera_world')+b'native mesh')
        native=receipt(mesh_meta['content_sha256']);native['submitted_body_path_sha256']=canonical_header_hash(raw)
        mesh_meta.update(native_receipt=native,shared_gauge=True);mesh_meta['snapshot_id']=wire.snapshot_identity(mesh_meta)
        return dict(meta=mesh_meta,raw=mesh_raw),dict(meta=path_meta,raw=path_raw)

    def test_shared_body_path_requires_nonempty_exact_receipted_content(self):
        mesh,path=self.body_path_records()
        self.assertTrue(verified_shared_body_path(mesh,path))
        self.assertFalse(verified_shared_body_path(None,path));self.assertFalse(verified_shared_body_path(mesh,None))
        empty_mesh,empty_path=self.body_path_records(empty=True)
        self.assertFalse(verified_shared_body_path(empty_mesh,empty_path))
        wrong=copy.deepcopy(mesh);wrong['meta']['native_receipt']['submitted_body_path_sha256']='b'*64
        wrong['meta']['snapshot_id']=wire.snapshot_identity(wrong['meta'])
        self.assertFalse(verified_shared_body_path(wrong,path))
        missing=copy.deepcopy(mesh);missing['meta']['native_receipt'].pop('submitted_body_path_sha256')
        missing['meta']['snapshot_id']=wire.snapshot_identity(missing['meta'])
        self.assertFalse(verified_shared_body_path(missing,path))

    def test_shared_body_path_allows_only_top_header_sequence_change(self):
        mesh,path=self.body_path_records()
        changed=struct.pack('<I',900)+path['raw'][4:]
        meta,raw=packet_fixture('body_path',changed)
        self.assertTrue(verified_shared_body_path(mesh,dict(meta=meta,raw=raw)))
        wrong=bytearray(changed);wrong[-1]^=1
        meta,raw=packet_fixture('body_path',bytes(wrong))
        self.assertFalse(verified_shared_body_path(mesh,dict(meta=meta,raw=raw)))
        other_boot=copy.deepcopy(path);other_boot['meta']['boot_id']='different_boot'
        other_boot['meta']['snapshot_id']=wire.snapshot_identity(other_boot['meta'])
        self.assertFalse(verified_shared_body_path(mesh,other_boot))

    def newer_body_path(self,path,sequence):
        raw=path['raw'][:-56]+struct.pack('<d',float(sequence))+path['raw'][-48:]
        meta,raw=packet_fixture('body_path',raw)
        meta['sequence']=sequence;meta['snapshot_id']=wire.snapshot_identity(meta)
        return dict(meta=meta,raw=raw)

    def test_late_mesh_selects_matching_historical_path_after_newer_path_arrives(self):
        state=ObserverState('test_run',99000000000,'test_gauge')
        mesh,path_a=self.body_path_records();path_b=self.newer_body_path(path_a,2)
        state.connected(0,'map','boot_a')
        state.ingest(path_a['meta'],path_a['raw']);state.ingest(path_b['meta'],path_b['raw'])
        self.assertEqual(state.records[0,'body_path']['raw'],path_b['raw'])
        state.ingest(mesh['meta'],mesh['raw'])
        selected=state.shared_body_path(0,mesh)
        self.assertIsNotNone(selected);self.assertEqual(selected['raw'],path_a['raw'])
        self.assertNotEqual(selected['raw'],state.records[0,'body_path']['raw'])
        missing=copy.deepcopy(mesh);missing['meta']['native_receipt'].pop('submitted_body_path_sha256')
        missing['meta']['snapshot_id']=wire.snapshot_identity(missing['meta'])
        self.assertIsNone(state.shared_body_path(0,missing))

    def test_fresh_observer_receives_old_verified_path_during_newer_pending_approval(self):
        state=ObserverState('test_run',99000000000,'test_gauge',body_cache_versions=2)
        mesh,old=self.body_path_records();pending=self.newer_body_path(old,2)
        meta,raw=packet_fixture('verified_body_path',old['raw'])
        verified=dict(meta=meta,raw=raw)
        mesh['meta']['native_receipt'].update(newer_path_pending=True,
            exact_optimizer_mesh_generation_proven=False,
            gauge_evidence_basis='last_verified_path_continuity')
        mesh['meta']['snapshot_id']=wire.snapshot_identity(mesh['meta'])
        state.connected(0,'map','boot_a')
        state.ingest(pending['meta'],pending['raw'])
        state.ingest(mesh['meta'],mesh['raw'])
        self.assertIsNone(state.shared_body_path(0,mesh))
        state.ingest(verified['meta'],verified['raw'])
        self.assertTrue(verified_shared_body_path(mesh,verified))
        selected=state.shared_body_path(0,mesh)
        self.assertEqual(selected['meta']['stream'],'verified_body_path')
        self.assertEqual(selected['raw'],old['raw'])
        self.assertFalse(verified_shared_body_path(mesh,pending))
        # Pending approvals cannot evict the retained verified-stream record.
        for sequence in range(3,9):
            path=self.newer_body_path(old,sequence)
            state.ingest(path['meta'],path['raw'])
        self.assertNotIn(canonical_header_hash(old['raw']),state.body_paths[0])
        self.assertEqual(state.shared_body_path(0,mesh)['raw'],old['raw'])
        state.disconnected(0,'map','boot_a','link interruption')
        state.connected(0,'map','boot_a')
        self.assertEqual(state.shared_body_path(0,mesh)['raw'],old['raw'])
        state.connected(0,'map','new_boot')
        self.assertIsNone(state.shared_body_path(0,mesh))

    def test_verified_stream_name_never_substitutes_for_exact_receipt_hash(self):
        mesh,path=self.body_path_records()
        wrong=self.newer_body_path(path,2)
        meta,raw=packet_fixture('verified_body_path',wrong['raw'])
        candidate=dict(meta=meta,raw=raw)
        self.assertFalse(verified_shared_body_path(mesh,candidate))
        state=ObserverState('test_run',99000000000,'test_gauge')
        state.connected(0,'map','boot_a');state.ingest(meta,raw)
        self.assertIsNone(state.shared_body_path(0,mesh))

    def test_body_path_cache_is_bounded_and_cleared_on_boot_change(self):
        state=ObserverState('test_run',99000000000,'test_gauge',body_cache_versions=2)
        mesh,path_a=self.body_path_records();path_b=self.newer_body_path(path_a,2);path_c=self.newer_body_path(path_a,3)
        state.connected(0,'map','boot_a')
        for path in (path_a,path_b,path_c):state.ingest(path['meta'],path['raw'])
        self.assertEqual(len(state.body_paths[0]),2)
        self.assertIsNone(state.shared_body_path(0,mesh))
        proof_b=copy.deepcopy(mesh);proof_b['meta']['native_receipt']['submitted_body_path_sha256']=canonical_header_hash(path_b['raw'])
        proof_b['meta']['snapshot_id']=wire.snapshot_identity(proof_b['meta'])
        self.assertEqual(state.shared_body_path(0,proof_b)['raw'],path_b['raw'])
        foreign=copy.deepcopy(path_c);foreign['meta']['gauge_id']='another_gauge'
        foreign['meta']['snapshot_id']=wire.snapshot_identity(foreign['meta'])
        with self.assertRaises(wire.WireError):state.ingest(foreign['meta'],foreign['raw'])
        state.connected(0,'map','new_boot')
        self.assertEqual(len(state.body_paths[0]),0);self.assertEqual(state.body_cache_bytes[0],0)
        self.assertIsNone(state.shared_body_path(0,proof_b))

    def test_map_and_telemetry_modes_have_distinct_nodes_and_external_status(self):
        self.assertEqual(channel_configuration('map'),(('map',),'distributed_result_observer'))
        self.assertEqual(channel_configuration('telemetry'),(('telemetry',),'distributed_telemetry_observer'))
        parsed=argument_parser().parse_args(['--run-id','r','--cutoff','1','--gauge-id','g','--output-directory','/tmp/unused'])
        self.assertEqual(parsed.channels,'map')
        state=ObserverState('test_run',99000000000,'test_gauge',channels=('map',))
        report=state.report()
        self.assertEqual(report['managed_channels'],['map'])
        self.assertFalse(report['robots']['0']['channels']['telemetry']['status_available_here'])
        self.assertIn('independent',report['telemetry_handler'])

    def test_telemetry_sink_cannot_publish_mesh_markers_status_or_paths(self):
        published={}
        class Publisher:
            def __init__(self,topic,*args,**kwargs):self.topic=topic;published[topic]=[]
            def publish(self,message):published[self.topic].append(message)
        class AnyMsg:
            def deserialize(self,raw):self.raw=raw;return self
        fake_rospy=types.ModuleType('rospy');fake_rospy.Publisher=Publisher;fake_rospy.AnyMsg=AnyMsg
        fake_message=types.ModuleType('roslib.message')
        fake_message.get_message_class=lambda kind:type('Odometry',(),{'_md5sum':'a'*32}) if kind=='nav_msgs/Odometry' else None
        fake_roslib=types.ModuleType('roslib');fake_roslib.__path__=[];fake_roslib.message=fake_message
        with patch.dict(sys.modules,{'rospy':fake_rospy,'roslib':fake_roslib,'roslib.message':fake_message}):
            sink=RosTelemetryObserver()
            meta,raw=packet_fixture('actual',header()+b'original odometry')
            sink.receive(meta,raw)
            self.assertEqual(published['/observer/sim_0/actual'][0].raw,raw)
            map_meta,_=packet_fixture()
            with self.assertRaises(wire.WireError):sink.validate(map_meta)
        self.assertEqual(set(published),{'/observer/sim_%d/%s'%(r,k) for r in (0,1) for k in ('actual','vio')})
        self.assertFalse(hasattr(sink,'refresh'))

    def performance_sink(self):
        class Publisher:
            def __init__(self):self.messages=[]
            def publish(self,message):self.messages.append(message)
        class AnyMsg:
            def deserialize(self,raw):self.raw=raw;return self
        class Marker:
            ADD=0;DELETE=2;TRIANGLE_LIST=11;TEXT_VIEW_FACING=9
            def __init__(self):
                self.header=S(seq=0,stamp=0,frame_id='kimera_world');self.ns='';self.id=0;self.action=0
                self.pose=S(orientation=S(w=0.),position=S(x=0.,y=0.,z=0.));self.scale=S(x=0.,y=0.,z=0.)
                self.color=S(r=0.,g=0.,b=0.,a=0.);self.points=[];self.colors=[]
        class PathMessage:
            def __init__(self):self.header=S(frame_id='');self.poses=[]
        class NativeMesh:
            decoded=0
            def deserialize(self,raw):
                NativeMesh.decoded+=1
                stamp,frame=wire.read_header(raw);self.header=S(seq=0,stamp=stamp,frame_id=frame)
                self.vertices=[S(x=0.,y=0.,z=0.),S(x=1.,y=0.,z=0.),S(x=0.,y=1.,z=0.)]
                self.vertex_colors=[S(r=.125,g=.25,b=.5,a=1.) for _ in range(3)]
                self.triangles=[S(vertex_indices=[0,1,2])];return self
        sink=RosObserver.__new__(RosObserver);sink.Marker=Marker;sink.String=lambda data:S(data=data)
        sink.rospy=S(AnyMsg=AnyMsg,Time=S(now=lambda:0));sink.lock=threading.Lock()
        sink.snapshot_ply=False;sink.all_mesh_faces=False;sink.mesh_markers={}
        sink.shared_last={};sink.shared_path_last={};sink.face_last={}
        sink.kinds={key:NativeMesh for key in wire.MESH_STREAMS};sink.kinds['body_path']=PathMessage
        sink.publishers={(r,key):Publisher() for r in (0,1) for key in wire.MESH_STREAMS}
        sink.markers={(r,'local_mesh'):Publisher() for r in (0,1)}
        sink.shared={r:Publisher() for r in (0,1)};sink.labels={r:Publisher() for r in (0,1)}
        sink.shared_labels={r:Publisher() for r in (0,1)};sink.shared_paths={r:Publisher() for r in (0,1)}
        sink.status=Publisher()
        def marker(message,robot,key,components=None):
            value=Marker();value.header=copy.copy(message.header)
            value.points=[message.vertices[0]]*100000  # Model the expensive serialization payload.
            return value
        sink.marker=Mock(side_effect=marker)
        return sink,NativeMesh

    def performance_state(self):
        mesh,path=self.body_path_records()
        meshes=[];paths=[]
        for robot in (0,1):
            m=copy.deepcopy(mesh);p=copy.deepcopy(path)
            for record in (m,p):
                meta=record['meta'];meta['robot_id']=robot;meta['boot_id']='boot_'+str(robot)
                spec=wire.contract(robot,meta['stream'])
                for key in ('source_topic','source_caller','ros_type'):meta[key]=spec[key]
                if meta['native_receipt'] is not None:meta['native_receipt']['robot_id']=robot
                meta['snapshot_id']=wire.snapshot_identity(meta)
            meshes.append(m);paths.append(p)
        class State:
            def __init__(self):self.pair=meshes;self.paths=paths;self.boots={0:'boot_0',1:'boot_1'}
            def shared_pair(self):return self.pair
            def shared_body_path(self,robot,mesh):return self.paths[robot] if verified_shared_body_path(mesh,self.paths[robot]) else None
            def report(self):
                return dict(gauge_id='test_gauge',robots={str(r):dict(current_boot_id=self.boots[r],streams={
                    key:dict(stale=False,last_receive_age_s=1.,source_age_s=1.) for key in ('local_mesh','mesh')}) for r in (0,1)})
        return State()

    def test_unchanged_large_shared_marker_is_published_once_while_status_stays_live(self):
        sink,_=self.performance_sink();state=self.performance_state()
        for _ in range(5):sink.refresh(state)
        self.assertEqual(sink.marker.call_count,2)
        self.assertEqual([len(sink.shared[r].messages) for r in (0,1)],[1,1])
        self.assertEqual(len(sink.shared[0].messages[0].points),100000)
        self.assertEqual([len(sink.shared_paths[r].messages) for r in (0,1)],[1,1])
        self.assertEqual(len(sink.status.messages),5)
        self.assertEqual([len(sink.labels[r].messages) for r in (0,1)],[5,5])
        state.pair=None
        sink.refresh(state);sink.refresh(state)
        self.assertEqual([len(sink.shared[r].messages) for r in (0,1)],[2,2])
        self.assertEqual(sink.shared[0].messages[-1].action,sink.Marker.DELETE)
        self.assertEqual([len(sink.shared_paths[r].messages) for r in (0,1)],[2,2])
        state.boots[0]='next_boot';sink.refresh(state);sink.refresh(state)
        self.assertEqual([len(sink.shared[r].messages) for r in (0,1)],[3,2])

    def test_mesh_labels_use_snapshot_receive_age_not_vertex_header_age(self):
        sink,_=self.performance_sink();state=self.performance_state()
        original_report=state.report
        def report():
            value=original_report()
            for robot in value['robots'].values():
                for stream in robot['streams'].values():
                    stream.update(source_age_s=900.,last_receive_age_s=1.2)
            return value
        state.report=report;sink.refresh(state)
        for robot in (0,1):
            for label in (sink.labels[robot].messages[-1],sink.shared_labels[robot].messages[-1]):
                self.assertIn('snapshot received 1.2s ago',label.text)
                self.assertIn('LIVE',label.text)
                self.assertNotIn('900',label.text)
                self.assertNotIn('native map',label.text)

    def test_native_and_accepted_geometry_share_one_validation_and_render_with_bounded_cache(self):
        sink,kind=self.performance_sink();meta,raw=packet_fixture('mesh',header(frame='kimera_world')+b'native')
        accepted=dict(meta,stream='accepted_mesh',source_topic=wire.contract(0,'accepted_mesh')['source_topic'])
        accepted['snapshot_id']=wire.snapshot_identity(accepted)
        sink.receive(meta,raw);sink.receive(accepted,raw)
        self.assertEqual(kind.decoded,1);self.assertEqual(sink.marker.call_count,0)
        first=sink.geometry(meta,raw);second=sink.geometry(accepted,raw)
        self.assertIs(first,second);self.assertEqual(sink.marker.call_count,1)
        for index in range(10):
            key='mesh' if index%2 else 'accepted_mesh'
            changed,data=packet_fixture(key,header(frame='kimera_world')+str(index).encode())
            sink.geometry(changed,data)
        self.assertEqual(len(sink.mesh_markers),2)
        current=kind.decoded
        new_boot=dict(meta,boot_id='next_boot');new_boot['snapshot_id']=wire.snapshot_identity(new_boot)
        sink.geometry(new_boot,raw);self.assertEqual(kind.decoded,current+1)

    def test_no_snapshot_ply_still_archives_complete_original_ros_and_metadata(self):
        meta,raw=packet_fixture()
        with tempfile.TemporaryDirectory() as directory:
            writer=ArchiveWriter(directory,snapshot_ply=False)
            with patch('observer.export_ply',side_effect=AssertionError('Unexpected per-snapshot PLY')):
                writer.archive(meta,raw,mesh=S(vertices='not needed for raw archive'))
            files=list(Path(directory).rglob('*.rosbin'));self.assertEqual(len(files),1)
            self.assertEqual(files[0].read_bytes(),raw)
            self.assertEqual(json.loads(files[0].with_suffix('.json').read_text()),meta)
            self.assertIsNone(json.loads((files[0].parent/'latest.json').read_text())['ply'])
            self.assertEqual(list(Path(directory).rglob('*.ply')),[])
            self.assertTrue(ArchiveWriter(directory).snapshot_ply)  # Preserve the direct class API.

    def test_production_defaults_skip_unused_faces_but_validate_every_new_native_content(self):
        args=argument_parser().parse_args(['--run-id','r','--cutoff','1','--gauge-id','g','--output-directory','/tmp/unused'])
        self.assertFalse(args.snapshot_ply);self.assertFalse(args.all_mesh_faces)
        sink,kind=self.performance_sink();meta,raw=packet_fixture('mesh',header(frame='kimera_world')+b'native')
        self.assertIsNone(sink.receive(meta,raw))
        self.assertEqual(kind.decoded,1);self.assertEqual(sink.marker.call_count,0)
        self.assertEqual(sink.publishers[0,'mesh'].messages[0].raw,raw)
        local,data=packet_fixture('local_mesh')
        sink.receive(local,data);sink.receive(local,data)
        self.assertEqual(sink.marker.call_count,1)
        self.assertEqual(len(sink.markers[0,'local_mesh'].messages),1)
        # An unrendered source still undergoes full topology/color validation.
        class BadMesh(kind):
            def deserialize(self,data):
                super().deserialize(data);self.triangles[0].vertex_indices=[0,1,99];return self
        sink.kinds['mesh']=BadMesh
        changed,data=packet_fixture('mesh',header(frame='kimera_world')+b'invalid topology')
        with self.assertRaises(wire.WireError):sink.receive(changed,data)


if __name__=='__main__':unittest.main()
