#!/usr/bin/env python3
"""Lossless single-ingress transport, bounded queues and independent IMU lane."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.util
import json
import struct
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch
import zlib

import sensor_wire as wire
from sensor_ingress import validate_messages
from sensor_uplink import SourceStampSampler, arguments as uplink_arguments


ROBOT, RID, CUTOFF, STAMP = 0, 'transport_contract_test', 1_000_000_000, 2_000_000_000


def parts(lane='rgbd'):
    # Include binary zeros, non-UTF8 bytes and a floating-point NaN bit pattern.
    return {key: (key.encode()+b'\0\xff'+struct.pack('<I', 0x7fc00001))*31
            for key in wire.lane_keys(lane)}


def packet(lane='rgbd', seq=1, stamp=STAMP):
    return wire.encode_packet(ROBOT, RID, CUTOFF, lane, seq, 0 if lane == 'ready' else stamp, parts(lane))


def rewrite(frame, edit, payload=None):
    offset = len(wire.MAGIC)+4; n = struct.unpack_from('<I', frame, len(wire.MAGIC))[0]
    meta = json.loads(frame[offset:offset+n]); edit(meta)
    header = json.dumps(meta, separators=(',', ':')).encode()
    raw = wire.MAGIC+struct.pack('<I', len(header))+header+(frame[offset+n:-32] if payload is None else payload)
    return raw+hashlib.sha256(raw).digest()


class SensorWireContracts(unittest.TestCase):
    def test_original_binary_messages_round_trip_for_all_lanes(self):
        for lane in wire.LANES:
            for level in (0, 1, 9):
                with self.subTest(lane=lane, level=level):
                    original = parts(lane)
                    encoded = wire.encode_packet(ROBOT, RID, CUTOFF, lane, 7,
                        0 if lane == 'ready' else STAMP, original, level)
                    meta, decoded = wire.decode_packet(encoded, ROBOT, RID, CUTOFF, lane)
                    self.assertEqual(decoded, original)
                    self.assertEqual(meta['sequence'], 7)
                    if lane in ('imu', 'truth', 'ready'): self.assertEqual(meta['codec'], 'none')
                    if lane == 'rgbd': self.assertEqual(tuple(decoded), wire.RGBD_KEYS)

    def test_robot_run_cutoff_topic_and_source_are_bound(self):
        valid = packet()
        for identity in ((1, RID, CUTOFF), (0, 'different_run', CUTOFF), (0, RID, CUTOFF+1)):
            with self.subTest(identity=identity), self.assertRaises(wire.WireError):
                wire.decode_packet(valid, *identity)
        with self.assertRaisesRegex(wire.WireError, 'lane differs'):
            wire.decode_packet(valid, ROBOT, RID, CUTOFF, 'imu')
        for key, value in (('source_caller', '/another_sensor'), ('robot_id', True),
                           ('sequence', 0), ('stamp_ns', CUTOFF-1), ('raw_bytes', True),
                           ('version', 1)):
            with self.subTest(key=key), self.assertRaises(wire.WireError):
                wire.decode_packet(rewrite(valid, lambda m: m.update({key: value})), ROBOT, RID, CUTOFF)

    def test_corruption_truncation_unknown_schema_and_raw_bounds_are_rejected(self):
        valid = packet()
        for frame in (valid[:-1], b'wrong_magic'+valid[11:], valid[:80]+bytes([valid[80]^1])+valid[81:]):
            with self.assertRaises(wire.WireError): wire.decode_packet(frame, ROBOT, RID, CUTOFF)
        for edit in (lambda m:m.update(extra=True), lambda m:m.update(raw_bytes=wire.MAX_RAW_BYTES+1),
                     lambda m:m.update(codec='lossy'), lambda m:m['streams'][0].update(type='sensor_msgs/Imu'),
                     lambda m:m['streams'][0].update(bytes=10**10)):
            with self.assertRaises(wire.WireError):
                wire.decode_packet(rewrite(valid, edit), ROBOT, RID, CUTOFF)
        raw=b'a'*100_000
        inflated=rewrite(valid, lambda m: (m.update(raw_bytes=5), m.update(streams=[
            dict(key=k,type=wire.STREAMS[k]['type'],bytes=1) for k in wire.RGBD_KEYS])), zlib.compress(raw))
        with self.assertRaisesRegex(wire.WireError, 'wrong length'):
            wire.decode_packet(inflated, ROBOT, RID, CUTOFF)
        with self.assertRaises(wire.WireError):
            wire.decode_packet(rewrite(valid, lambda m: None, valid[len(wire.MAGIC)+4+struct.unpack_from('<I',valid,len(wire.MAGIC))[0]:-32]+b'junk'), ROBOT, RID, CUTOFF)

    def test_no_incomplete_or_cross_lane_rgbd_payload_can_be_encoded(self):
        invalid = parts(); invalid.pop('depth')
        with self.assertRaises(wire.WireError): wire.encode_packet(ROBOT,RID,CUTOFF,'rgbd',1,STAMP,invalid)
        with self.assertRaises(wire.WireError): wire.encode_packet(ROBOT,RID,CUTOFF,'imu',1,STAMP,parts())
        with self.assertRaises(wire.WireError): wire.encode_packet(ROBOT,RID,CUTOFF,'ready',1,STAMP,parts('ready'))

    def test_truth_provenance_and_input_are_the_actual_body_not_the_renderer(self):
        for robot in (0,1):
            self.assertEqual(wire.input_topic(robot,'truth'),'/drone_%d_visual_slam/odom'%robot)
            self.assertEqual(wire.input_caller(robot,'truth'),'/drone_%d_continuous_body'%robot)
            for key in ('rgb','depth','imu','lidar'):
                self.assertEqual(wire.input_topic(robot,key),wire.edge_topic(robot,wire.STREAMS[key]['suffix']))
                self.assertEqual(wire.input_caller(robot,key),wire.source_caller(robot))
        encoded=packet('truth');meta,decoded=wire.decode_packet(encoded,ROBOT,RID,CUTOFF,'truth')
        self.assertEqual(meta['version'],2)
        self.assertEqual(meta['source_caller'],wire.body_caller(ROBOT))
        self.assertEqual(decoded,parts('truth'))
        with self.assertRaisesRegex(wire.WireError,'source'):
            wire.decode_packet(rewrite(encoded,lambda m:m.update(source_caller=wire.source_caller(ROBOT))),
                               ROBOT,RID,CUTOFF,'truth')

    def test_actual_telemetry_100_to_30_hz_sampling_keeps_source_stamps_and_omissions(self):
        sampler=SourceStampSampler(30.)
        original=[STAMP+i*10_000_000 for i in range(1000)]
        selected=[stamp for stamp in original if sampler.allow(stamp)]
        self.assertEqual(len(selected),300)
        self.assertTrue(set(selected)<=set(original))
        self.assertEqual(sampler.snapshot()['counts']['source_samples_omitted_configured_rate'],700)
        self.assertEqual(sampler.snapshot()['counts']['source_messages_observed'],1000)
        self.assertFalse(sampler.allow(selected[-1]))
        self.assertTrue(sampler.allow(STAMP+20_000_000_000))
        self.assertFalse(sampler.allow(STAMP+20_000_000_001))
        for rate in (0.,float('nan'),float('inf'),101.):
            with self.assertRaises(ValueError):SourceStampSampler(rate)
        self.assertEqual(uplink_arguments(['--robot-id','0','--run-id',RID,'--cutoff','1']).telemetry_rate,30.)

    def test_receiver_rejects_stale_duplicate_regression_and_foreign_caller(self):
        receiver=wire.Receiver(ROBOT,RID,CUTOFF,max_age_ns=100)
        caller=wire.uplink_caller(ROBOT)
        with self.assertRaisesRegex(wire.WireError, 'publisher'):
            receiver.receive('imu',packet('imu'),'/untrusted',STAMP)
        with self.assertRaisesRegex(wire.WireError, 'stale'):
            receiver.receive('imu',packet('imu'),caller,STAMP+101)
        receiver.receive('imu',packet('imu'),caller,STAMP+10)
        with self.assertRaisesRegex(wire.WireError, 'sequence'):
            receiver.receive('imu',packet('imu'),caller,STAMP+10)
        with self.assertRaisesRegex(wire.WireError, 'stamp'):
            receiver.receive('imu',packet('imu',seq=2,stamp=STAMP-1),caller,STAMP+10)
        receiver.receive('imu',packet('imu',seq=4,stamp=STAMP+1),caller,STAMP+10)
        self.assertEqual(receiver.snapshot()['imu']['sequence_gaps'],2)
        self.assertEqual(receiver.snapshot()['imu']['last_source_stamp_ns'],STAMP+1)
        self.assertEqual(receiver.snapshot()['imu']['last_receive_wall_ns'],STAMP+10)
        # Another lane has its own sequence and clock history.
        receiver.receive('lidar',packet('lidar'),caller,STAMP+10)
        with self.assertRaisesRegex(wire.WireError, 'ahead'):
            receiver.receive('truth',packet('truth',stamp=STAMP+600_000_000),caller,STAMP)

    def test_slow_rgbd_decode_cannot_hold_the_imu_receiver(self):
        receiver=wire.Receiver(ROBOT,RID,CUTOFF)
        entered,release=threading.Event(),threading.Event();original=wire.decode_packet;errors=[]
        def slow(*args,**kw):
            if args[-1]=='rgbd': entered.set(); self.assertTrue(release.wait(2))
            return original(*args,**kw)
        def rgbd():
            try:receiver.receive('rgbd',packet(),wire.uplink_caller(ROBOT),STAMP)
            except Exception as error:errors.append(error)
        with patch.object(wire,'decode_packet',side_effect=slow):
            worker=threading.Thread(target=rgbd);worker.start();self.assertTrue(entered.wait(1))
            started=time.monotonic()
            try:
                receiver.receive('imu',packet('imu'),wire.uplink_caller(ROBOT),STAMP)
                receiver.receive('truth',packet('truth'),wire.uplink_caller(ROBOT),STAMP)
            finally:release.set();worker.join(2)
            self.assertLess(time.monotonic()-started,.5)
        self.assertFalse(errors);self.assertFalse(worker.is_alive())

    def test_bounded_frame_pairing_never_mix_stamps_and_discards_late_or_stale_parts(self):
        cache=wire.FrameBuffer(CUTOFF,max_pending=2,max_age_ns=100)
        cache.put('rgb',STAMP,b'old rgb')
        for key,value in parts().items():cache.put(key,STAMP+1,value)
        stamp,decoded=cache.take(STAMP+2)
        self.assertEqual(stamp,STAMP+1);self.assertEqual(decoded,parts())
        self.assertEqual(cache.snapshot()['counts']['incomplete_older_frames_omitted'],1)
        self.assertFalse(cache.put('depth',STAMP,b'late depth'))
        for s in (STAMP+2,STAMP+3,STAMP+4):cache.put('rgb',s,b'rgb')
        self.assertEqual(cache.snapshot()['pending_rgbd'],2)
        self.assertEqual(cache.snapshot()['counts']['frames_omitted_cache_limit'],1)
        self.assertIsNone(cache.take(STAMP+200));self.assertEqual(cache.snapshot()['pending_rgbd'],0)
        self.assertFalse(cache.put('rgb',CUTOFF-1,b'pre-epoch'))

    def test_parallel_frame_callbacks_keep_complete_binary_parts(self):
        cache=wire.FrameBuffer(CUTOFF,max_pending=16)
        work=[(key,STAMP+i,b'%d/'%i+value) for i in range(12) for key,value in parts().items()]
        with ThreadPoolExecutor(max_workers=8) as pool:
            list(pool.map(lambda x:cache.put(*x),reversed(work)))
        received=[]
        while True:
            item=cache.take(STAMP+100)
            if item is None:break
            stamp,values=item;received.append(stamp)
            for key,value in values.items():self.assertEqual(value,b'%d/'%(stamp-STAMP)+parts()[key])
        self.assertEqual(received,[STAMP+i for i in range(12)])

    def test_same_stamp_conflicting_part_is_visible(self):
        cache=wire.FrameBuffer(CUTOFF);self.assertTrue(cache.put('rgb',STAMP,b'first'))
        self.assertFalse(cache.put('rgb',STAMP,b'first'))
        with self.assertRaisesRegex(wire.WireError,'changed'):cache.put('rgb',STAMP,b'changed')

    def test_standard_remaps_cover_every_edge_sensor_output(self):
        self.assertEqual(len(wire.sensor_remaps(0)),9)
        self.assertEqual(set(wire.sensor_remaps(1)),{
            '/sim_1/'+s['suffix']+':=/edge/sim_1/'+s['suffix'] for s in wire.STREAMS.values()})
        self.assertEqual(wire.ingress_caller(0),'/sim_0_multimodal_sensors')
        self.assertNotEqual(wire.ingress_caller(0),wire.source_caller(0))

    def test_real_sensor_cli_accepts_ros_remaps_without_enabling_data_recording_or_planner_bypass(self):
        import sim_sensors
        args=sim_sensors.arguments(['--drone-id','1','--body-imu',
                                   '__name:=sim_1_edge_sensors']+wire.sensor_remaps(1))
        self.assertEqual(args.drone_id,1)
        self.assertEqual(args.body_imu,'auto')
        self.assertEqual((args.width,args.height),(320,240))
        self.assertEqual((args.camera_rate,args.lidar_rate),(10.,10.))
        self.assertIsNone(args.output_directory)
        self.assertFalse(args.publish_planner_cloud)


class OriginalMessageValidation(unittest.TestCase):
    def test_wrong_original_stamp_is_rejected_before_publication(self):
        message=S(header=S(stamp=S(to_nsec=lambda:STAMP+1),frame_id='sim_0/imu'))
        with self.assertRaisesRegex(wire.WireError,'Header'):
            validate_messages(0,dict(stamp_ns=STAMP,lane='imu'),{'imu':message})

    @unittest.skipUnless(importlib.util.find_spec('sensor_msgs') and importlib.util.find_spec('genpy'),
                         'Native ROS1 sensor messages are tested inside Noetic')
    def test_real_ros_serializations_retain_headers_covariance_and_float_depth_bits(self):
        from sensor_msgs.msg import Image,CameraInfo,Imu
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header,UInt8MultiArray
        import genpy
        stamp=genpy.Time(2,0);header=Header(seq=73,stamp=stamp,frame_id='sim_0/camera_optical')
        images={key:Image(header=header,height=2,width=2,encoding=enc,is_bigendian=False,
                    step=2*bpp,data=data) for key,enc,bpp,data in (
                        ('rgb','rgb8',3,bytes(range(12))),
                        ('depth','32FC1',4,struct.pack('<4I',0,0x7fc00001,0x7f800000,0x3fa00000)),
                        ('mask','mono8',1,b'\0\xff\0\xff'))}
        images.update(info=CameraInfo(header=header,height=2,width=2),depth_info=CameraInfo(header=header,height=2,width=2))
        raw={key:wire.serialized(value) for key,value in images.items()}
        encoded=wire.encode_packet(0,RID,CUTOFF,'rgbd',1,STAMP,raw)
        carrier=UInt8MultiArray().deserialize(wire.serialized(UInt8MultiArray(data=encoded)))
        meta,result=wire.decode_packet(carrier.data,0,RID,CUTOFF)
        recovered={key:(CameraInfo() if key.endswith('info') else Image()).deserialize(value) for key,value in result.items()}
        validate_messages(0,meta,recovered)
        self.assertEqual({k:wire.serialized(v) for k,v in recovered.items()},raw)
        self.assertEqual(recovered['depth'].data,images['depth'].data)
        # Exercise the actual rospy publishing serializer. Re-publishing a
        # decoded typed Image would rewrite Header.seq, hiding dropped frames.
        import io,rospy
        from rospy.msg import args_kwds_to_message,serialize_message
        for key,original in raw.items():
            kind=CameraInfo if key.endswith('info') else Image
            forwarded=args_kwds_to_message(kind,(rospy.AnyMsg().deserialize(original),),{})
            buffer=io.BytesIO();serialize_message(buffer,999,forwarded)
            self.assertEqual(buffer.getvalue()[4:],original)
        m=Imu(header=Header(seq=9,stamp=stamp,frame_id='sim_0/imu'));m.orientation_covariance[0]=-1.;m.angular_velocity.x=.125
        original=wire.serialized(m);meta,values=wire.decode_packet(wire.encode_packet(0,RID,CUTOFF,'imu',1,STAMP,{'imu':original}),0,RID,CUTOFF)
        restored=Imu().deserialize(values['imu']);validate_messages(0,meta,{'imu':restored})
        self.assertEqual(wire.serialized(restored),original)
        actual=Odometry(header=Header(seq=194,stamp=stamp,frame_id='world'),child_frame_id='sim_0/body')
        actual.pose.pose.orientation.w=1.;actual.pose.pose.position.x=3.5
        actual.twist.twist.linear.y=.75;original=wire.serialized(actual)
        meta,values=wire.decode_packet(wire.encode_packet(0,RID,CUTOFF,'truth',1,STAMP,{'truth':original}),0,RID,CUTOFF,'truth')
        restored=Odometry().deserialize(values['truth']);validate_messages(0,meta,{'truth':restored})
        self.assertEqual(meta['source_caller'],wire.body_caller(0))
        self.assertEqual(wire.serialized(restored),original)
        self.assertEqual(restored.header.seq,194)
        restored.twist.twist.linear.x=float('nan')
        with self.assertRaisesRegex(wire.WireError,'Nonfinite'):
            validate_messages(0,meta,{'truth':restored})


if __name__=='__main__':unittest.main()
