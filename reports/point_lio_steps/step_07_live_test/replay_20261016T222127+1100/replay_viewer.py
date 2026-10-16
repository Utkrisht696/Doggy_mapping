"""Isolated playback of saved outputs; no estimator or robot control nodes."""
from pathlib import Path
import os,json,subprocess,time,signal
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as PathMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
root=Path('/home/jain/Doggy');run=Path(__file__).resolve().parent;bag=root/'reports/point_lio_steps/step_07_live_test/20261016T220926+1100/bag'
assert os.environ['ROS_DOMAIN_ID']=='197' and os.environ['ROS_LOCALHOST_ONLY']=='1'
rclpy.init();node=rclpy.create_node('doggy_saved_drive_viewer');path=PathMsg();path.header.frame_id='map';counts={'poses':0,'registered_clouds':0,'clocks':0,'tf_messages':0};frames=set()
pub=node.create_publisher(PathMsg,'/point_lio/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
def odom(m):
 p=PoseStamped();p.header=m.header;p.pose=m.pose.pose;path.header=m.header;path.poses.append(p);pub.publish(path);counts['poses']+=1

def cloud(m):counts['registered_clouds']+=1;frames.add(m.header.frame_id)
def clock(m):counts['clocks']+=1
def tf(m):counts['tf_messages']+=1
subs=[node.create_subscription(Odometry,'/point_lio/odometry',odom,100),node.create_subscription(PointCloud2,'/point_lio/cloud_registered',cloud,100),node.create_subscription(Clock,'/doggy/robot_clock',clock,100),node.create_subscription(TFMessage,'/tf',tf,1000)]
rvizlog=(run/'rviz.log').open('w');playlog=(run/'playback.log').open('w');rviz=None;player=None
manifest={'status':'starting','bag':str(bag),'domain':197,'localhost_only':True,'rate':1.0,'estimator_started':False,'robot_commands_sent':False,'clock':'Recorded /doggy/robot_clock; RViz remaps /clock to that topic','view':'Recorded registered scans, recorded canonical TF, recorded developer robot model, reconstructed path from recorded odometry'}
try:
 rviz=subprocess.Popen(['rviz2','-d',str(run/'replay.rviz'),'--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock'],stdout=rvizlog,stderr=subprocess.STDOUT)
 until=time.monotonic()+3
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
 if rviz.poll() is not None:raise RuntimeError('RViz exited at startup')
 topics=['/point_lio/cloud_registered','/point_lio/odometry','/tf','/tf_static','/robot_description','/joint_states','/doggy/robot_clock']
 player=subprocess.Popen(['ros2','bag','play',str(bag),'--rate','1.0','--delay','3','--read-ahead-queue-size','3000','--disable-keyboard-controls','--qos-profile-overrides-path',str(run/'qos.yaml'),'--topics',*topics],stdout=playlog,stderr=subprocess.STDOUT)
 manifest.update(status='playing',rviz_pid=rviz.pid,player_pid=player.pid,topics=topics);(run/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest),flush=True);last=0;completed=False
 while rviz.poll() is None:
  rclpy.spin_once(node,timeout_sec=.05)
  if (run/'stop_requested').exists():break
  if time.monotonic()-last>5:
   status={'counts':counts,'frames':sorted(frames),'rviz_running':True,'player_running':player.poll() is None,'playback_returncode':player.returncode}
   (run/'progress.json').write_text(json.dumps(status,indent=2));print(json.dumps(status),flush=True);last=time.monotonic()
  if player.poll() is not None and not completed:
   completed=True;manifest.update(status='playback_finished_viewer_open',player_returncode=player.returncode);(run/'manifest.json').write_text(json.dumps(manifest,indent=2));print('Playback complete; keeping RViz and accumulated map visible.',flush=True)
finally:
 for p in [player,rviz]:
  if p is not None and p.poll() is None:
   p.send_signal(signal.SIGINT)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=3)
 node.destroy_node();rclpy.shutdown();rvizlog.close();playlog.close();manifest.update(status='stopped',final_counts=counts);(run/'manifest.json').write_text(json.dumps(manifest,indent=2))
