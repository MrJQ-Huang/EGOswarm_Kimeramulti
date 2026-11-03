#!/usr/bin/env python3
"""Oracle solid occupancy for EGO navigation and static visualization.

The planning cloud is an explicitly ideal, unoccluded baseline, not a SLAM map.
RGB-D/LiDAR sensor ray casting stays surface-based and must not consume this
filled planning cloud; Kimera receives only its separate rendered observations.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray
from planning_world import planning_occupancy_points
from multimodal.telemetry_contract import actual_odometry_topic, actual_odometry_caller



base=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,default=base/'complex_scene.json');p.add_argument('--path-capacity',type=int,default=12000);p.add_argument('--central-telemetry',action='store_true',help='Display local ingress telemetry; each edge owns its planning map');args=p.parse_args(rospy.myargv()[1:])
scene=json.loads(args.scene.read_text())
rospy.init_node('two_pc_complex_world_and_view')
world=(rospy.Publisher('/map_generator/global_cloud',PointCloud2,queue_size=1,latch=True) if not args.central_telemetry else None)
planning_metadata=(rospy.Publisher('/two_pc/planning_world_metadata',String,queue_size=1,latch=True) if not args.central_telemetry else None)
scene_pub=rospy.Publisher('/two_pc/complex_environment',MarkerArray,queue_size=1,latch=True)
vehicle_pub=rospy.Publisher('/two_pc/vehicles',MarkerArray,queue_size=1)
paths=[rospy.Publisher('/two_pc/drone_%d/path'%i,RosPath,queue_size=1,latch=True) for i in (0,1)]
latest=[None,None];tracks=[RosPath(),RosPath()];last_path=[0.,0.]
state={'scenario':'oppose','phase':'preparing','cycle':1};lock=threading.RLock()
colors=[(.08,.82,1.),(1.,.42,.08)]

def marker(ns,idx,kind,pos,scale,color):
 m=Marker(header=Header(frame_id='world'),ns=ns,id=idx,type=kind,action=Marker.ADD)
 m.pose.orientation.w=1;m.pose.position.x,m.pose.position.y,m.pose.position.z=pos
 m.scale.x,m.scale.y,m.scale.z=scale;m.color.r,m.color.g,m.color.b,m.color.a=color
 return m

environment=MarkerArray()
environment.markers.append(Marker(action=Marker.DELETEALL))
b=scene['bounds'];environment.markers.append(marker('floor',0,Marker.CUBE,((b['x'][0]+b['x'][1])/2,(b['y'][0]+b['y'][1])/2,-.04),(b['x'][1]-b['x'][0],b['y'][1]-b['y'][0],.08),(.13,.17,.22,1)))
for idx,o in enumerate(scene['obstacles']):
 if o['type']=='box':pos=o['center'];size=o['size'];kind=Marker.CUBE
 else:
  pos=[o['center'][0],o['center'][1],o.get('base_z',0)+o['height']/2]
  size=[o['radius']*2,o['radius']*2,o['height']];kind=Marker.CYLINDER
 color=(.34,.43,.55,.8) if o['type']=='box' else (.69,.46,.24,.95)
 environment.markers.append(marker('obstacles',idx,kind,pos,size,color))
if not args.central_telemetry:
 planning_step=scene.get('planning_cloud_step_m',.12);sample_started=time.perf_counter()
 xyz=planning_occupancy_points(scene,step=planning_step)
 planning_metadata.publish(String(data=json.dumps(dict(source='oracle static solid occupancy baseline for EGO planning only',
     planning_only=True,slam_map=False,kimera_input=False,occlusion_model='none; ideal 5m radius filter is applied by range_sensor.py',
     cloud_topic='/map_generator/global_cloud',scene_sha256=hashlib.sha256(args.scene.read_bytes()).hexdigest(),
     obstacle_sampling='box and finite-cylinder interiors plus surfaces',floor_sampling='surface only',
     planning_cloud_step_m=planning_step,point_count=len(xyz),generation_seconds=time.perf_counter()-sample_started,
     sensor_rendering='Separate RGB-D/LiDAR raycast surface renderer unchanged; never use this oracle cloud as a SLAM observation'))))
 world.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='world'),xyz))
 rospy.loginfo('Oracle EGO occupancy baseline: %d obstacles, %d solid/surface points; not a SLAM map',len(scene['obstacles']),len(xyz))
else:
 rospy.loginfo('Central display only: edge-local EGO planning maps are independent of this node')
scene_pub.publish(environment)

def on_odom(msg,i):
 if args.central_telemetry and (getattr(msg,'_connection_header',None) or {}).get('callerid')!=actual_odometry_caller(i,True):
  rospy.logwarn_throttle(5.,'World display rejected actual telemetry from a foreign publisher');return
 with lock:
  previous=latest[i];latest[i]=msg;now=rospy.get_time()
  if previous:
   p=previous.pose.pose.position;q=msg.pose.pose.position
   if (p.x-q.x)**2+(p.y-q.y)**2+(p.z-q.z)**2>4:tracks[i]=RosPath()
  if now-last_path[i]>=.1:
   pose=PoseStamped(header=msg.header,pose=msg.pose.pose)
   tracks[i].header=msg.header;tracks[i].poses.append(pose);tracks[i].poses=tracks[i].poses[-args.path_capacity:]
   paths[i].publish(tracks[i]);last_path[i]=now

def on_state(msg):
 global state,tracks
 try:d=json.loads(msg.data)
 except ValueError:return
 with lock:
  if d.get('run_id')!=state.get('run_id'):
   tracks=[RosPath(header=Header(frame_id='world')),RosPath(header=Header(frame_id='world'))]
   for i in (0,1):paths[i].publish(tracks[i])
  state=d

def text(ns,idx,pos,label,color,size=.32):
 m=marker(ns,idx,Marker.TEXT_VIEW_FACING,pos,(0,0,size),color);m.text=label;return m

def tick(_):
 with lock:poses=copy.deepcopy(latest);info=dict(state)
 arr=MarkerArray()
 for i,msg in enumerate(poses):
  c=colors[i]+(1.,)
  if msg is not None:
   p=msg.pose.pose.position;q=msg.pose.pose.orientation
   yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
   body=marker('body',i,Marker.CUBE,(p.x,p.y,p.z),(.33,.27,.14),c)
   body.pose.orientation=q;arr.markers.append(body)
   for arm in (0,1):
    m=marker('arms',i*2+arm,Marker.CUBE,(p.x,p.y,p.z),(.85,.06,.045),(.85,.9,.96,1))
    angle=yaw+math.pi/4+arm*math.pi/2;m.pose.orientation.z=math.sin(angle/2);m.pose.orientation.w=math.cos(angle/2)
    arr.markers.append(m)
   for j,(dx,dy) in enumerate(((-.27,-.27),(-.27,.27),(.27,-.27),(.27,.27))):
    x=p.x+dx*math.cos(yaw)-dy*math.sin(yaw);y=p.y+dx*math.sin(yaw)+dy*math.cos(yaw)
    arr.markers.append(marker('rotors',i*4+j,Marker.CYLINDER,(x,y,p.z+.04),(.25,.25,.035),c))
   arr.markers.append(text('hosts',i,(p.x,p.y,p.z+.8),('AMOV / drone 0' if i==0 else 'UROS / drone 1'),c,.3))
 mode=scene['scenarios'].get(info.get('scenario'),next(iter(scene['scenarios'].values())))
 goals=info.get('goals',mode['goals'])
 for i in (0,1):
  c=colors[i]+(.9,);g=goals[i];s=mode['starts'][i]
  arr.markers.append(marker('goals',i,Marker.CYLINDER,(g[0],g[1],.03),(.9,.9,.06),c))
  arr.markers.append(text('goal_labels',i,(g[0],g[1],.5),'GOAL '+str(i),c,.3))
  arr.markers.append(marker('starts',i,Marker.CYLINDER,(s[0],s[1],.025),(.7,.7,.05),(.3,.9,.4,.65)))
 title='A / OPPOSING CROSSING' if info.get('scenario')=='oppose' else 'B / SAME-DIRECTION TRAVERSAL'
 phase=info.get('phase','preparing').upper().replace('_',' ')
 arr.markers.append(text('hud',0,(0,5.7,1.0),title,(1,1,1,1),.52))
 arr.markers.append(text('hud',1,(0,5.15,.6),'Cycle %s | %s'%(info.get('cycle',1),phase),(.65,.9,.75,1),.32))
 errors=info.get('goal_errors_m')
 detail='Each computer plans its own virtual drone | direct LAN trajectory exchange'
 if errors:detail='Goal errors: AMOV %.2f m | UROS %.2f m'%(errors[0],errors[1])
 arr.markers.append(text('hud',2,(0,-5.3,.4),detail,(.8,.86,.94,1),.28))
 vehicle_pub.publish(arr)

subs=[rospy.Subscriber(actual_odometry_topic(i,args.central_telemetry),Odometry,on_odom,callback_args=i,
                       queue_size=1 if args.central_telemetry else 20,tcp_nodelay=True) for i in (0,1)]
subs.append(rospy.Subscriber('/two_pc/scene_state',String,on_state,queue_size=1))
timer=rospy.Timer(rospy.Duration(.1),tick)
rospy.spin()
