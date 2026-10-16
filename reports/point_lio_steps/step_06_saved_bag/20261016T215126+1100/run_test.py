"""Approved diagnostic replay, isolated from the robot; no active app edits."""
from pathlib import Path
import os,json,time,signal,subprocess,hashlib,traceback
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosbag2_interfaces.srv import SetRate
assert os.environ.get('ROS_DOMAIN_ID')=='197' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
root=Path.cwd();base=root/'reports/point_lio_steps/step_06_saved_bag';run=root/json.loads((base/'current_run.json').read_text())['directory']
bag=root/'reports/point_lio_steps/step_04_timing/temporal_motion/20261016T205503+1100/bag'
origin=int(np.load(bag.parent/'decoded_samples.npz')['cloud_stamp_ns'][0])
topics=['/utlidar/cloud','/doggy/imu/body_calibrated','/doggy/robot_clock','/tf','/tf_static','/robot_description','/joint_states']
files=['scripts/publish_body_imu.py','config/point_lio/go2_body_imu.yaml','config/point_lio/sensor_geometry.json','scripts/publish_point_lio_tf.py','launch/point_lio_go2.launch.py']
hashes={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
manifest={'bag':str(bag),'domain':197,'localhost_only':True,'robot_commands_sent':False,'start_rate':.5,'topics_replayed':topics,'clock':'Recorded /doggy/robot_clock only; no player-generated /clock','source_time_origin_ns':origin,'application_hashes_before':hashes,'status':'starting'}
(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
poses=[];clouds=[];cloudstamps=[];status=[];parentsets={};tf_stamps=[];input_cloud_stamps=[];input_imu_stamps=[]
launch=None;player=None;node=None;reason='';rate_future=None;rate_changed=False
launchlog=(run/'mapping.log').open('w');playerlog=(run/'playback.log').open('w')
def ns(stamp):return stamp.sec*10**9+stamp.nanosec
def odom(m):
 p,q=m.pose.pose.position,m.pose.pose.orientation
 poses.append((ns(m.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w,time.monotonic()))
def cloud(m):
 endian='>' if m.is_bigendian else '<';fields={f.name:f.offset for f in m.fields}
 dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':m.point_step})
 a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
 stride=max(1,(len(a)+999)//1000);a=a[::stride]
 points=np.column_stack([a[k] for k in ['x','y','z']]);points=points[np.isfinite(points).all(axis=1)]
 clouds.append(points);cloudstamps.append(ns(m.header.stamp))
def tf(m):
 for t in m.transforms:
  parentsets.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
  if t.header.frame_id=='map' and t.child_frame_id=='odom':tf_stamps.append(ns(t.header.stamp))
def current():
 elapsed=(input_cloud_stamps[-1]-origin)/1e9 if input_cloud_stamps else None
 result={'source_elapsed_s':elapsed,'odometry_messages':len(poses),'registered_clouds':len(clouds),'map_to_odom_transforms':len(tf_stamps),'last_bridge_status':status[-1] if status else None,'player_running':player is not None and player.poll() is None,'mapping_running':launch is not None and launch.poll() is None,'playback_rate':1.0 if rate_changed else .5}
 if len(poses)>2:
  a=np.asarray(poses);quiet=a[(a[:,0]-origin>=5e9)&(a[:,0]-origin<=55e9)]
  if len(quiet)>2:
   p=quiet[:,1:4];reference=np.median(p[:min(30,len(p))],axis=0);q=quiet[:,4:8];q/=np.linalg.norm(q,axis=1)[:,None]
   result['stationary_max_displacement_m']=float(np.linalg.norm(p-reference,axis=1).max())
   result['stationary_orientation_range_deg']=float(np.max(2*np.arccos(np.clip(np.abs(q @ q[0]),0,1))*180/np.pi))
 return result
try:
 rclpy.init();node=rclpy.create_node('point_lio_saved_bag_observer')
 subscriptions=[node.create_subscription(Odometry,'/point_lio/odometry',odom,100),node.create_subscription(PointCloud2,'/point_lio/cloud_registered',cloud,100),node.create_subscription(TFMessage,'/tf',tf,100),node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(String,'/point_lio/tf_status',lambda m:status.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(PointCloud2,'/utlidar/cloud',lambda m:input_cloud_stamps.append(ns(m.header.stamp)),qos_profile_sensor_data),node.create_subscription(Imu,'/doggy/imu/body_calibrated',lambda m:input_imu_stamps.append(ns(m.header.stamp)),qos_profile_sensor_data)]
 rate_client=node.create_client(SetRate,'/rosbag2_player/set_rate')
 launch=subprocess.Popen(['ros2','launch',str(run/'denser_seed.launch.py'),'start_mapping:=true','rviz:=true'],stdout=launchlog,stderr=subprocess.STDOUT)
 manifest['launch_pid']=launch.pid
 ready_end=time.monotonic()+4
 while time.monotonic()<ready_end:rclpy.spin_once(node,timeout_sec=.05)
 if launch.poll() is not None:raise RuntimeError('Mapping/RViz launch exited before playback; see mapping.log')
 player=subprocess.Popen(['ros2','bag','play',str(bag),'--rate','0.5','--read-ahead-queue-size','3000','--delay','2','--disable-keyboard-controls','--qos-profile-overrides-path',str(bag.parent/'record_qos.yaml'),'--topics',*topics],stdout=playerlog,stderr=subprocess.STDOUT)
 manifest.update(player_pid=player.pid,status='running');(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print(json.dumps({'event':'started','directory':str(run),'launch_pid':launch.pid,'player_pid':player.pid}),flush=True)
 started=time.monotonic();last_progress=0;finished_at=None
 while time.monotonic()-started<620:
  rclpy.spin_once(node,timeout_sec=.02)
  now=time.monotonic()
  if launch.poll() is not None:reason='mapping_launch_exited';break
  if player.poll() is not None:
   if finished_at is None:finished_at=now
   if now-finished_at>3:reason='playback_complete' if player.returncode==0 else 'playback_error';break
  if len(poses)>0 and not np.isfinite(np.asarray(poses)[-1,1:8]).all():reason='nonfinite_estimator_pose';break
  if now-last_progress>=10:
   result=current();(run/'progress.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True);last_progress=now
   if result.get('source_elapsed_s',0) is not None and result['source_elapsed_s']>10 and result['source_elapsed_s']<60 and (result.get('stationary_max_displacement_m',0)>.5 or result.get('stationary_orientation_range_deg',0)>15):reason='stationary_drift_gate_failed';break
   if result.get('source_elapsed_s',0) is not None and result['source_elapsed_s']>10 and not poses:reason='no_estimator_output';break
   if result.get('source_elapsed_s',0) is not None and result['source_elapsed_s']>=30 and not rate_changed and rate_future is None and rate_client.service_is_ready():
    req=SetRate.Request();req.rate=1.0;rate_future=rate_client.call_async(req)
  if rate_future is not None and rate_future.done():
   response=rate_future.result();rate_changed=bool(response.success);print(json.dumps({'event':'rate_change','success':rate_changed}),flush=True)
   rate_future=None
  if len(poses)>0 and np.linalg.norm(np.asarray(poses)[-1,1:4])>100:reason='pose_magnitude_gate_failed';break
 if not reason:reason='wall_time_limit'
except BaseException as error:
 reason='exception: '+repr(error);(run/'exception.txt').write_text(traceback.format_exc());print(reason,flush=True)
finally:
 # Stop test-owned processes only. Preserve all live sensor/IMU processes.
 for proc in [player,launch]:
  if proc is not None and proc.poll() is None:
   proc.send_signal(signal.SIGINT)
   try:proc.wait(timeout=10)
   except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 if node is not None:
  final=current();node.destroy_node()
 else:final={}
 if rclpy.ok():rclpy.shutdown()
 launchlog.close();playerlog.close()
 a=np.asarray(poses,dtype=np.float64).reshape((-1,9));lengths=np.array([len(c) for c in clouds],dtype=np.int64)
 np.savez_compressed(run/'outputs.npz',odom_stamp_ns=np.array([p[0] for p in poses],dtype=np.int64),odom_values=a[:,1:8],odom_receipt_monotonic=a[:,8],cloud_stamp_ns=np.array(cloudstamps,dtype=np.int64),cloud_points=np.concatenate(clouds) if clouds else np.empty((0,3),dtype=np.float32),cloud_offsets=np.r_[0,np.cumsum(lengths)],map_odom_stamp_ns=np.array(tf_stamps,dtype=np.int64),input_cloud_stamp_ns=np.array(input_cloud_stamps,dtype=np.int64),input_imu_stamp_ns=np.array(input_imu_stamps,dtype=np.int64))
 final.update(stop_reason=reason,tf_parents={k:sorted(v) for k,v in parentsets.items()},multiple_tf_parents={k:sorted(v) for k,v in parentsets.items() if len(v)>1},input_cloud_messages=len(input_cloud_stamps),input_imu_messages=len(input_imu_stamps),launch_returncode=launch.returncode if launch else None,player_returncode=player.returncode if player else None,application_hashes_after={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files},robot_commands_sent=False)
 final['application_unchanged']=final['application_hashes_after']==hashes
 (run/'result.json').write_text(json.dumps(final,indent=2)+'\n');manifest['status']='stopped_saved';manifest['stop_reason']=reason;(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print(json.dumps({'event':'finished',**final}),flush=True)
