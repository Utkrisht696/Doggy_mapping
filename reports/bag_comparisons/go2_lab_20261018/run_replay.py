"""Isolated diagnostic playback; uses existing estimator and approved calibration."""
from pathlib import Path
import os,json,time,signal,subprocess,hashlib,traceback
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
assert os.environ.get('ROS_DOMAIN_ID')=='209' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
root=Path('/home/jain/Doggy');run=Path(__file__).resolve().parent;bag=run/'replay_stationary_start'
source=np.load(run/'source_decoded.npz');expected=np.load(run/'expected_replay_stamps.npz');origin=int(source['raw_stamp_ns'][0])
topics=['/utlidar/cloud','/doggy/imu/body_calibrated','/utlidar/robot_odom','/doggy/robot_clock','/tf','/tf_static']
files=['scripts/publish_body_imu.py','config/point_lio/body_imu_calibration.json','config/point_lio/go2_body_imu.yaml','config/point_lio/sensor_geometry.json','scripts/publish_point_lio_tf.py','launch/point_lio_go2.launch.py']
hashes={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
manifest={'source_bag':'/home/jain/go2_bags/go2_20261018_173219','derived_replay':str(bag),'domain':209,'localhost_only':True,'robot_commands_sent':False,'playback_rate':.5,'topics_replayed':topics,'clock':'Source-header-derived /doggy/robot_clock, no player-generated /clock','source_time_origin_ns':origin,'application_hashes_before':hashes,'diagnostic_override':{'init_map_size':4000},'excluded_initial_receipt_s':1.5,'reason_for_initial_exclusion':'Initial body motion; estimator starts in subsequent quiet pause, with later gaps preserved','status':'starting'}
(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
poses=[];clouds=[];cloudstamps=[];status=[];parentsets={};tf_stamps=[];input_cloud_stamps=[];input_imu_stamps=[]
launch=None;player=None;node=None;reason='';launchlog=(run/'mapping.log').open('w');playerlog=(run/'playback.log').open('w')
def ns(stamp):return int(stamp.sec)*10**9+int(stamp.nanosec)
def odom(m):
 p,q=m.pose.pose.position,m.pose.pose.orientation
 poses.append((ns(m.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w,time.monotonic()))
def cloud(m):
 endian='>' if m.is_bigendian else '<';fields={f.name:f.offset for f in m.fields}
 dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step})
 a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);a=a[::max(1,(len(a)+999)//1000)]
 p=np.column_stack([a[k] for k in ['x','y','z']]);p=p[np.isfinite(p).all(axis=1)];clouds.append(p);cloudstamps.append(ns(m.header.stamp))
def tf(m):
 for t in m.transforms:
  parentsets.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
  if t.header.frame_id=='map' and t.child_frame_id=='odom':tf_stamps.append(ns(t.header.stamp))
def current():
 return {'source_elapsed_s':float((input_cloud_stamps[-1]-origin)/1e9) if input_cloud_stamps else None,'odometry_messages':len(poses),'registered_clouds':len(clouds),'map_to_odom_transforms':len(tf_stamps),'last_bridge_status':status[-1] if status else None,'player_running':player is not None and player.poll() is None,'mapping_running':launch is not None and launch.poll() is None}
try:
 rclpy.init();node=rclpy.create_node('go2_lab_bag_observer')
 subs=[node.create_subscription(Odometry,'/point_lio/odometry',odom,1000),node.create_subscription(PointCloud2,'/point_lio/cloud_registered',cloud,1000),node.create_subscription(TFMessage,'/tf',tf,1000),node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(String,'/point_lio/tf_status',lambda m:status.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(PointCloud2,'/utlidar/cloud',lambda m:input_cloud_stamps.append(ns(m.header.stamp)),QoSProfile(depth=1000)),node.create_subscription(Imu,'/doggy/imu/body_calibrated',lambda m:input_imu_stamps.append(ns(m.header.stamp)),QoSProfile(depth=10000))]
 launch=subprocess.Popen(['ros2','launch',str(run/'offline.launch.py'),'start_mapping:=true','rviz:=false'],stdout=launchlog,stderr=subprocess.STDOUT,start_new_session=True)
 manifest['launch_pid']=launch.pid
 ready_end=time.monotonic()+4
 while time.monotonic()<ready_end:rclpy.spin_once(node,timeout_sec=.05)
 if launch.poll() is not None:raise RuntimeError('Mapping launch exited before playback; see mapping.log')
 player=subprocess.Popen(['ros2','bag','play',str(bag),'--rate','0.5','--read-ahead-queue-size','3000','--delay','2','--disable-keyboard-controls','--qos-profile-overrides-path',str(run/'replay_qos.yaml'),'--topics',*topics],stdout=playerlog,stderr=subprocess.STDOUT,start_new_session=True)
 manifest.update(player_pid=player.pid,status='running');(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print(json.dumps({'event':'started','directory':str(run),'launch_pid':launch.pid,'player_pid':player.pid}),flush=True)
 started=time.monotonic();last_progress=0;finished_at=None
 while time.monotonic()-started<530:
  rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()
  if launch.poll() is not None:reason='mapping_launch_exited';break
  if player.poll() is not None:
   if finished_at is None:finished_at=now
   if now-finished_at>4:reason='playback_complete' if player.returncode==0 else 'playback_error';break
  if poses and not np.isfinite(poses[-1][1:8]).all():reason='nonfinite_estimator_pose';break
  if now-last_progress>=15:
   result=current();(run/'progress.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True);last_progress=now
   if result['source_elapsed_s'] is not None and result['source_elapsed_s']>15 and not poses:reason='no_estimator_output';break
   if any(len(parents)>1 for parents in parentsets.values()):reason='conflicting_tf_parents';break
 if not reason:reason='wall_time_limit'
except BaseException as error:
 reason='exception: '+repr(error);(run/'exception.txt').write_text(traceback.format_exc());print(reason,flush=True)
finally:
 for proc in [player,launch]:
  if proc is not None and proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 final=current()
 if node is not None:node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 launchlog.close();playerlog.close();a=np.asarray(poses,dtype=float).reshape((-1,9));lengths=np.array([len(c) for c in clouds],dtype=np.int64)
 np.savez_compressed(run/'outputs.npz',odom_stamp_ns=np.array([p[0] for p in poses],dtype=np.int64),odom_values=a[:,1:8],odom_receipt_monotonic=a[:,8],cloud_stamp_ns=np.array(cloudstamps,dtype=np.int64),cloud_points=np.concatenate(clouds) if clouds else np.empty((0,3),dtype=np.float32),cloud_offsets=np.r_[0,np.cumsum(lengths)],map_odom_stamp_ns=np.array(tf_stamps,dtype=np.int64),input_cloud_stamp_ns=np.array(input_cloud_stamps,dtype=np.int64),input_imu_stamp_ns=np.array(input_imu_stamps,dtype=np.int64))
 final.update(stop_reason=reason,tf_parents={k:sorted(v) for k,v in parentsets.items()},multiple_tf_parents={k:sorted(v) for k,v in parentsets.items() if len(v)>1},input_cloud_messages=len(input_cloud_stamps),input_imu_messages=len(input_imu_stamps),launch_returncode=launch.returncode if launch else None,player_returncode=player.returncode if player else None,application_hashes_after={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files},robot_commands_sent=False)
 final['application_unchanged']=final['application_hashes_after']==hashes
 final['cloud_delivery_exact']=np.array_equal(expected['raw_stamp_ns'],np.asarray(input_cloud_stamps,dtype=np.int64));final['imu_delivery_exact']=np.array_equal(expected['imu_stamp_ns'],np.asarray(input_imu_stamps,dtype=np.int64))
 (run/'result.json').write_text(json.dumps(final,indent=2)+'\n');manifest['status']='stopped_saved';manifest['stop_reason']=reason;(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(run/'bridge_status.json').write_text(json.dumps(status,indent=2)+'\n')
 print(json.dumps({'event':'finished',**final}),flush=True)
