#!/usr/bin/env python3
"""Per-host ideal 5 m omnidirectional point-cloud sensor; no occlusion or noise."""
import argparse
import threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

p=argparse.ArgumentParser();p.add_argument('--drone-id',type=int,required=True);a=p.parse_args()
rospy.init_node('drone_%d_ideal_range_sensor'%a.drone_id)
lock=threading.Lock();world=None;pose=None;stamp=None
pub=rospy.Publisher('/drone_%d_pcl_render_node/cloud'%a.drone_id,PointCloud2,queue_size=1)

def on_world(msg):
 global world
 values=np.array(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)),dtype=np.float32)
 with lock:world=values.reshape((-1,3))
 rospy.loginfo('Ideal range sensor %d loaded %d world points',a.drone_id,len(values))

def on_odom(msg):
 global pose,stamp
 q=msg.pose.pose.position
 with lock:pose=np.array([q.x,q.y,q.z],dtype=np.float32);stamp=msg.header.stamp

def tick(_):
 with lock:xyz=world;q=pose;t=stamp
 if xyz is None or q is None:return
 delta=xyz-q
 local=np.ascontiguousarray(xyz[np.einsum('ij,ij->i',delta,delta)<=25.0],dtype=np.float32)
 msg=PointCloud2(header=Header(stamp=t,frame_id='world'),height=1,width=len(local),
     fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))],
     is_bigendian=False,point_step=12,row_step=len(local)*12,is_dense=True,data=local.tobytes())
 pub.publish(msg)

subs=[rospy.Subscriber('/map_generator/global_cloud',PointCloud2,on_world,queue_size=1),
      rospy.Subscriber('/drone_%d_visual_slam/odom'%a.drone_id,Odometry,on_odom,queue_size=1,tcp_nodelay=True)]
timer=rospy.Timer(rospy.Duration(.1),tick)
rospy.spin()
