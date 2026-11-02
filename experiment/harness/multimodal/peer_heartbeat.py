#!/usr/bin/env python3
"""Advertise backend connectivity only while this host receives peer heartbeats."""
import argparse
import time
import rospy
from std_msgs.msg import Header, UInt16MultiArray
p=argparse.ArgumentParser();p.add_argument('--robot-id',type=int,required=True);a=p.parse_args()
rospy.init_node('sim_%d_peer_heartbeat'%a.robot_id)
peer=1-a.robot_id;last=[-100.]
pub=rospy.Publisher('/sim_%d/heartbeat'%a.robot_id,Header,queue_size=1)
connected=rospy.Publisher('/sim_%d/connected_peer_ids'%a.robot_id,UInt16MultiArray,queue_size=1,latch=True)
def cb(_):last[0]=time.monotonic()
sub=rospy.Subscriber('/sim_%d/heartbeat'%peer,Header,cb,queue_size=1,tcp_nodelay=True)
rate=rospy.Rate(2);seq=0
while not rospy.is_shutdown():
 seq+=1;pub.publish(Header(seq=seq,stamp=rospy.Time.now(),frame_id='sim_%d'%a.robot_id))
 connected.publish(UInt16MultiArray(data=[peer] if time.monotonic()-last[0]<3 else []))
 rate.sleep()
