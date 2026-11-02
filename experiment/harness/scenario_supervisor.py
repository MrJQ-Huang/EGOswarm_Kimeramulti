#!/usr/bin/env python3
"""Restart only this experiment's virtual robot at a named scenario's spawn."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client
import rosgraph
import rospy
from std_msgs.msg import String

p=argparse.ArgumentParser();p.add_argument('--drone-id',type=int,required=True);p.add_argument('--root',required=True);p.add_argument('--scene',type=Path);a=p.parse_args()
root=Path(a.root);scene_path=a.scene or root/'harness/complex_scene.json';scene=json.loads(scene_path.read_text());children=[]
pending=None;current=None;lock=threading.Lock()
rospy.init_node('drone_%d_complex_sim_supervisor'%a.drone_id,disable_signals=True)
pub=rospy.Publisher('/two_pc/drone_%d/supervisor_status'%a.drone_id,String,queue_size=1,latch=True)

def publish(state,**extra):
 data=dict(drone_id=a.drone_id,run_id=current,state=state,host=os.uname().nodename,wall_time=time.time(),**extra)
 pub.publish(String(data=json.dumps(data)));print(json.dumps(data),flush=True)

def receive(msg):
 global pending
 try:
  d=json.loads(msg.data)
  if d.get('scenario') not in scene['scenarios'] or not isinstance(d.get('run_id'),str):return
  if d.get('command')!='reset':return
  with lock:pending=d
 except (ValueError,TypeError):pass

def stop_children():
 for proc,stream in children:
  if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 for proc,stream in children:
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
  stream.close()
 children.clear()

def start_child(name,args):
 path=root/'logs'/(current+'_drone%d_'%a.drone_id+name+'.log')
 stream=path.open('w');proc=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 children.append((proc,stream))

class ShortXMLRPC(xmlrpc.client.Transport):
 def make_connection(self,host):
  connection=super().make_connection(host);connection.timeout=1.;return connection

def replace_fake_body(start):
 """Stop only the known local fake-body node before the new publisher starts."""
 name='/drone_%d_poscmd_2_odom'%a.drone_id;master=rosgraph.Master(rospy.get_name())
 deadline=time.monotonic()+15;pid=None;uri=None
 while time.monotonic()<deadline and not rospy.is_shutdown():
  try:
   uri=master.lookupNode(name)
   with xmlrpc.client.ServerProxy(uri,transport=ShortXMLRPC()) as api:
    code,message,value=api.getPid(rospy.get_name())
   if code!=1:raise RuntimeError(message)
   candidate=int(value);raw=Path('/proc',str(candidate),'cmdline').read_bytes()
   if b'poscmd_2_odom' not in raw or ('__name:='+name.lstrip('/')).encode() not in raw:
    raise RuntimeError('Fake-body node PID does not belong to this local drone: '+name)
   pid=candidate;break
  except (rosgraph.MasterException,OSError,xmlrpc.client.Error):time.sleep(.1)
 if pid is None:raise RuntimeError('Original fake-body node did not become reachable: '+name)
 publish('replacing_fake_body',node=name,pid=pid,continuous_body=True)
 try:
  with xmlrpc.client.ServerProxy(uri,transport=ShortXMLRPC()) as api:
   code,message,_=api.shutdown(rospy.get_name(),'Switch to continuous actual-state simulation')
  if code!=1:raise RuntimeError('Fake-body shutdown rejected: '+message)
 except OSError:
  # Shutdown can close its XMLRPC socket before replying. Require the exact
  # verified process to have exited below before any replacement is started.
  pass
 deadline=time.monotonic()+8
 while time.monotonic()<deadline and not rospy.is_shutdown():
  try:alive=Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
  except FileNotFoundError:alive=False
  if not alive:break
  time.sleep(.05)
 else:raise RuntimeError('Original fake-body process did not terminate; refusing duplicate odometry publishers')
 start_child('continuous_body',['python3',str(root/'harness/multimodal/continuous_body.py'),
                              '--drone-id',str(a.drone_id),'--initial-position',*[str(v) for v in start]])

sub=rospy.Subscriber('/two_pc/scenario_control',String,receive,queue_size=1)
signal.signal(signal.SIGTERM,lambda *_:rospy.signal_shutdown('supervisor stopped'))
signal.signal(signal.SIGINT,lambda *_:rospy.signal_shutdown('supervisor stopped'))
try:
 while not rospy.is_shutdown():
  with lock:d=pending;pending=None
  if d and d['run_id']!=current:
   current=d['run_id'];publish('resetting');stop_children()
   mode=scene['scenarios'][d['scenario']];start=mode['starts'][a.drone_id];goal=mode['goals'][a.drone_id]
   args=['roslaunch','--screen',str(root/'harness/complex_drone.launch'),'drone_id:='+str(a.drone_id)]
   args+=['init_%s:=%s'%(axis,value) for axis,value in zip('xyz',start)]
   args+=['target_%s:=%s'%(axis,value) for axis,value in zip('xyz',goal)]
   args+=['map_size_%s:=%s'%(axis,value) for axis,value in zip('xyz',scene['map_size'])]
   args+=['max_vel:='+str(scene.get('nominal_planning_speed_m_s',1.0))]
   start_child('planner',args)
   if d.get('continuous_body') is True:
    try:replace_fake_body(start)
    except Exception as error:
     publish('continuous_body_failed',error=str(error),continuous_body=True);stop_children();continue
   start_child('sensor',['python3',str(root/'harness/range_sensor.py'),'--drone-id',str(a.drone_id)])
   publish('launched',scenario=d['scenario'],starts=start,goals=goal,pids=[p.pid for p,_ in children],continuous_body=d.get('continuous_body') is True)
  if children and any(p.poll() is not None for p,_ in children):
   publish('child_exited',codes=[p.poll() for p,_ in children]);stop_children()
  time.sleep(.1)
finally:stop_children()
