"""User-approved live mapping observation; never sends robot control commands."""
from pathlib import Path
import os,json,time,signal,subprocess,traceback,hashlib
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,DurabilityPolicy
assert os.environ.get('ROS_DOMAIN_ID')=='0' and os.environ.get('ROS_LOCALHOST_ONLY')=='0'
root=Path.cwd();base=root/'reports/point_lio_steps/step_07_live_test';run=root/json.loads((base/'current_run.json').read_text())['directory'];adapter_pid=int((run/'body_imu_adapter.pid').read_text())
files=['scripts/publish_body_imu.py','config/point_lio/body_imu_calibration.json','config/point_lio/go2_body_imu.yaml','config/point_lio/sensor_geometry.json','scripts/publish_point_lio_tf.py','launch/point_lio_go2.launch.py'];hashes={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
topics=['/utlidar/cloud','/utlidar/cloud_base','/utlidar/imu','/doggy/imu/body_calibrated','/sportmodestate','/utlidar/robot_odom','/utlidar/lidar_state','/doggy/robot_clock','/tf','/tf_static','/odom','/joint_states','/robot_description','/point_lio/odometry','/point_lio/cloud_registered','/point_lio/tf_status','/point_lio/raw_tf','/point_lio/initial_map']
def network():return Path('/proc/net/snmp').read_text()
def ns(stamp):return stamp.sec*10**9+stamp.nanosec
manifest={'status':'starting','domain':0,'live_robot_ethernet':True,'robot_commands_sent':False,'init_map_size':4000,'gyro_variance':.1,'provisional_time_lag':0.0,'adapter_pid':adapter_pid,'topics_recorded':topics,'application_hashes_before':hashes,'network_before':network(),'automatic_wall_time_limit_s':1800,'user_readiness':'User requested start; asked to keep still until cue','user_request':'Start rosbag and mapping for vertical and odometry accuracy test; user drives manually'}
(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
poses=[];body=[];raw=[];clouds=[];cloudstamps=[];tfstamps=[];status=[];parents={};last={};errors=[];origin=None
node=None;recorder=None;launch=None;stop_reason='';first_output_wall=None;started=time.monotonic();last_progress=0
record_log=(run/'record.log').open('w');mapping_log=(run/'mapping.log').open('w')
def body_cb(m):
 last['imu']=time.monotonic();body.append((ns(m.header.stamp),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z))
def odom_cb(m):
 global first_output_wall,origin
 p,q=m.pose.pose.position,m.pose.pose.orientation;st=ns(m.header.stamp)
 if first_output_wall is None:first_output_wall=time.monotonic();origin=st
 poses.append((st,p.x,p.y,p.z,q.x,q.y,q.z,q.w));last['odom']=time.monotonic()
def point_data(m,include_time=False):
 fields={f.name:f.offset for f in m.fields};names=['x','y','z']+(['time'] if include_time else []);endian='>' if m.is_bigendian else '<';dtype=np.dtype({'names':names,'formats':[endian+'f4']*len(names),'offsets':[fields[k] for k in names],'itemsize':m.point_step});return np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
def raw_cb(m):
 last['cloud']=time.monotonic();a=point_data(m,True);r=a['x']**2+a['y']**2+a['z']**2;valid=np.isfinite(r)&np.isfinite(a['time'])&(r>.25);v=a['time'][valid];raw.append((ns(m.header.stamp),float(v.min()) if len(v) else 0,float(v.max()) if len(v) else 0,len(a),int(valid.sum())))
def cloud_cb(m):
 a=point_data(m);a=a[::max(1,(len(a)+999)//1000)];p=np.column_stack([a[k] for k in ['x','y','z']]);clouds.append(p[np.isfinite(p).all(axis=1)]);cloudstamps.append(ns(m.header.stamp))
def tf_cb(m):
 for t in m.transforms:
  parents.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
  if t.header.frame_id=='map' and t.child_frame_id=='odom':tfstamps.append(ns(t.header.stamp))
def progress():
 now=time.monotonic();s={'wall_elapsed_s':now-started,'phase':'motion' if (run/'motion_cue.json').exists() else 'stationary_startup','raw_clouds':len(raw),'calibrated_imu_samples':len(body),'lio_poses':len(poses),'map_to_odom_transforms':len(tfstamps),'last_bridge_status':status[-1] if status else None,'input_age_s':{k:now-v for k,v in last.items()},'adapter_alive':Path('/proc/'+str(adapter_pid)).exists()}
 if len(poses)>2:
  a=np.asarray(poses);rel=(a[:,0]-origin)/1e9;mask=rel<=30;p=a[mask,1:4];q=a[mask,4:8];q=q/np.linalg.norm(q,axis=1)[:,None];ref=np.median(p[:min(30,len(p))],axis=0);s['startup_position_excursion_m']=float(np.linalg.norm(p-ref,axis=1).max());s['startup_orientation_change_deg']=float((2*np.arccos(np.clip(abs(q@q[0]),0,1))*180/np.pi).max());s['latest_pose_xyz']=a[-1,1:4].tolist();s['output_source_span_s']=float(rel[-1])
 return s
try:
 rclpy.init();node=rclpy.create_node('doggy_live_mapping_observer');subscriptions=[node.create_subscription(Imu,'/doggy/imu/body_calibrated',body_cb,QoSProfile(depth=10000)),node.create_subscription(PointCloud2,'/utlidar/cloud',raw_cb,QoSProfile(depth=1000)),node.create_subscription(Odometry,'/point_lio/odometry',odom_cb,100),node.create_subscription(PointCloud2,'/point_lio/cloud_registered',cloud_cb,100),node.create_subscription(TFMessage,'/tf',tf_cb,1000),node.create_subscription(TFMessage,'/tf_static',tf_cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(String,'/point_lio/tf_status',lambda m:status.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
 if node.count_publishers('/point_lio/odometry')>0:raise RuntimeError('An estimator publisher already exists; refusing a duplicate')
 recorder=subprocess.Popen(['ros2','bag','record','-o',str(run/'bag'),'--qos-profile-overrides-path',str(run/'record_qos.yaml'),*topics],stdout=record_log,stderr=subprocess.STDOUT);manifest['recorder_pid']=recorder.pid
 until=time.monotonic()+4
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
 if recorder.poll() is not None:raise RuntimeError('Recorder exited at startup')
 required=['/utlidar/cloud','/doggy/imu/body_calibrated','/tf','/tf_static','/doggy/robot_clock']
 log=(run/'record.log').read_text();missing=[name for name in required if 'Subscribed to topic '+"'"+name+"'" not in log]
 if missing:raise RuntimeError('Required recorder subscriptions not confirmed: '+repr(missing))
 if not body or not raw:raise RuntimeError('No fresh calibrated IMU/raw clouds before mapper startup')
 launch=subprocess.Popen(['ros2','launch',str(run/'live_mapping.launch.py'),'start_mapping:=true','rviz:=true'],stdout=mapping_log,stderr=subprocess.STDOUT);manifest.update(launch_pid=launch.pid,status='running');(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps({'event':'live_started','directory':str(run),'launch_pid':launch.pid,'recorder_pid':recorder.pid}),flush=True)
 while time.monotonic()-started<1800:
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if (run/'stop_requested').exists():stop_reason='user_requested_stop';break
  if launch.poll() is not None:stop_reason='mapper_or_rviz_launch_exited';break
  if recorder.poll() is not None:stop_reason='recorder_exited';break
  if not Path('/proc/'+str(adapter_pid)).exists():stop_reason='body_imu_adapter_exited';break
  if any(len(v)>1 for v in parents.values()):stop_reason='multiple_tf_parents';break
  if any(now-last.get(k,started)>2 for k in ['imu','cloud']):stop_reason='stale_sensor_input';break
  if now-started>15 and (not poses or now-last.get('odom',started)>3):stop_reason='no_fresh_lio_output';break
  if poses and not np.isfinite(poses[-1][1:]).all():stop_reason='invalid_lio_pose';break
  if now-last_progress>=5:
   s=progress();(run/'progress.json').write_text(json.dumps(s,indent=2)+'\n');print(json.dumps(s),flush=True);last_progress=now
   # User confirmed real manual movement; pose travel is not a stop condition.
   if first_output_wall is not None and now-first_output_wall>=30 and not (run/'startup_ready.json').exists():(run/'startup_ready.json').write_text(json.dumps(s,indent=2)+'\n')
 if not stop_reason:stop_reason='automatic_wall_time_limit'
except BaseException as error:
 stop_reason='exception: '+repr(error);(run/'exception.txt').write_text(traceback.format_exc());print(stop_reason,flush=True)
finally:
 # Only stop this test's mapper/RViz/recorder. The unchanged IMU adapter stays up.
 for proc in [launch,recorder]:
  if proc is not None and proc.poll() is None:
   proc.send_signal(signal.SIGINT)
   try:proc.wait(timeout=15)
   except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 if node is not None:
  final=progress();node.destroy_node()
 else:final={}
 if rclpy.ok():rclpy.shutdown()
 record_log.close();mapping_log.close()
 lengths=np.array([len(c) for c in clouds],dtype=np.int64);np.savez_compressed(run/'observed_outputs.npz',odom_stamp_ns=np.array([p[0] for p in poses],dtype=np.int64),odom_values=np.array([p[1:] for p in poses],dtype=float).reshape((-1,7)),imu_stamp_ns=np.array([p[0] for p in body],dtype=np.int64),imu_values=np.array([p[1:] for p in body],dtype=float).reshape((-1,6)),raw_cloud_stamp_ns=np.array([p[0] for p in raw],dtype=np.int64),raw_cloud_metadata=np.array([p[1:] for p in raw],dtype=float).reshape((-1,4)),cloud_stamp_ns=np.array(cloudstamps,dtype=np.int64),cloud_points=np.concatenate(clouds) if clouds else np.empty((0,3),dtype=np.float32),cloud_offsets=np.r_[0,np.cumsum(lengths)],map_odom_stamp_ns=np.array(tfstamps,dtype=np.int64))
 final.update(stop_reason=stop_reason,robot_commands_sent=False,recorder_returncode=recorder.returncode if recorder else None,launch_returncode=launch.returncode if launch else None,application_hashes_after={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files},network_after=network(),tf_parents={k:sorted(v) for k,v in parents.items()});final['application_unchanged']=final['application_hashes_after']==hashes;(run/'result.json').write_text(json.dumps(final,indent=2)+'\n');manifest.update(status='stopped_saved',stop_reason=stop_reason);(run/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps({'event':'live_test_stopped',**final}),flush=True)
