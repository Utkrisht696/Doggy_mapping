"""Original real-go2 optional LIO and original guarded bridge, isolated from hardware."""
from pathlib import Path
import os,json,time,signal,subprocess,hashlib,numpy as np,yaml
import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,PointCloud2
r=Path(__file__).resolve().parent;out=r/'results';repo=r/'unitree_go2_ros2';assert os.environ['ROS_DOMAIN_ID']=='210' and os.environ['ROS_LOCALHOST_ONLY']=='1';origin=int(np.load(r.parent/'source_decoded.npz')['raw_stamp_ns'][0]);expected=np.load(out/'expected_inputs.npz');root=Path('/home/jain/Doggy');files=[root/'scripts/publish_body_imu.py',root/'config/point_lio/go2_body_imu.yaml',root/'config/point_lio/body_imu_calibration.json',root/'config/point_lio/sensor_geometry.json',repo/'unitree_go2_real/scripts/real_tf_bridge.py',repo/'unitree_go2_real/scripts/lio_input.py',repo/'unitree_go2_real/config/point_lio_go2.yaml',repo/'third_party/point_lio_unilidar/src/laserMapping.cpp'];before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};poses={'lio':[],'hybrid':[],'raw':[]};inputs={'cloud':[],'imu':[],'vendor':[]};subs=[];processes=[];logs=[];reason='';node=None
ns=lambda st:int(st.sec)*10**9+int(st.nanosec)
def on_pose(key,msg):
 p,q=msg.pose.pose.position,msg.pose.pose.orientation;poses[key].append((ns(msg.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w,time.monotonic()))
def progress():return {'source_elapsed_s':float((inputs['vendor'][-1]-origin)/1e9) if inputs['vendor'] else None,'lio_poses':len(poses['lio']),'hybrid_poses':len(poses['hybrid']),'raw_bridge_poses':len(poses['raw']),'received_inputs':{k:len(v) for k,v in inputs.items()}}
def spawn(cmd,label):
 log=(out/(label+'.log')).open('w');logs.append(log);p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
qos={topic:{'reliability':'reliable','durability':'volatile','history':'keep_last','depth':depth} for topic,depth in [('/go2/lio_cloud',1000),('/go2/lio_imu',10000),('/utlidar/robot_odom',10000),('/doggy/robot_clock',1000)]};(out/'replay_qos.yaml').write_text(yaml.safe_dump(qos));manifest={'repo_commit':'c3158fcef41532fc5dc754e5838b0dfdecb5506a','bag':str(out/'lio_replay_inputs'),'domain':210,'localhost_only':True,'rate':.5,'mapping_config':str(repo/'unitree_go2_real/config/point_lio_go2.yaml'),'mapping_parameters_overridden':{'use_sim_time':True},'robot_commands_replayed':False,'hashes_before':before};(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
try:
 rclpy.init();node=rclpy.create_node('repo_lio_test_observer')
 for key,topic in [('lio','/go2/lio_odom'),('hybrid','/go2/odom'),('raw','/go2/odom_raw')]:subs.append(node.create_subscription(Odometry,topic,lambda m,k=key:on_pose(k,m),QoSProfile(depth=10000)))
 for key,topic,typ in [('vendor','/utlidar/robot_odom',Odometry),('imu','/go2/lio_imu',Imu),('cloud','/go2/lio_cloud',PointCloud2)]:subs.append(node.create_subscription(typ,topic,lambda m,k=key:inputs[k].append(ns(m.header.stamp)),QoSProfile(depth=10000)))
 exe=r/'isolated_ws/install/point_lio_unilidar/lib/point_lio_unilidar/pointlio_mapping';mapper=spawn([str(exe),'--ros-args','--params-file',str(repo/'unitree_go2_real/config/point_lio_go2.yaml'),'-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock','-r','/aft_mapped_to_init:=/go2/lio_odom','-r','/cloud_registered:=/go2/lio_cloud_registered','-r','/tf:=/repo/pointlio_tf','-r','/tf_static:=/repo/pointlio_tf_static'],'point_lio')
 bridge=spawn(['python3','-u',str(r/'instrument_bridge.py'),'--ros-args','--params-file',str(repo/'unitree_go2_real/config/real_bridge.yaml'),'-p','pose_source:=lio','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock','-r','/tf:=/repo/bridge_tf','-r','/tf_static:=/repo/bridge_static','-r','/utlidar/cloud_deskewed:=/repo/unused_cloud'],'bridge')
 end=time.monotonic()+4
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
 if any(p.poll() is not None for p in processes):raise RuntimeError('Mapper or bridge failed before playback')
 player=spawn(['ros2','bag','play',str(out/'lio_replay_inputs'),'--rate','0.5','--delay','2','--read-ahead-queue-size','3000','--disable-keyboard-controls','--qos-profile-overrides-path',str(out/'replay_qos.yaml')],'playback');start=time.monotonic();last=0;finished=None;print(json.dumps({'event':'started','pids':[p.pid for p in processes]}),flush=True)
 while time.monotonic()-start<530:
  rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()
  if mapper.poll() is not None or bridge.poll() is not None:reason='estimator_or_bridge_exited';break
  if player.poll() is not None:
   if finished is None:finished=now
   if now-finished>4:reason='playback_complete' if player.returncode==0 else 'playback_error';break
  if now-last>15:
   pp=progress();(out/'progress.json').write_text(json.dumps(pp,indent=2)+'\n');print(json.dumps(pp),flush=True);last=now
   if pp['source_elapsed_s'] is not None and pp['source_elapsed_s']>25 and not poses['lio']:reason='no_lio_output';break
  if poses['lio'] and not np.isfinite(poses['lio'][-1][1:8]).all():reason='nonfinite_lio_pose';break
 if not reason:reason='wall_time_limit'
except BaseException as error:reason='exception: '+repr(error);print(reason,flush=True)
finally:
 for process in reversed(processes):
  if process.poll() is None:
   os.killpg(process.pid,signal.SIGINT)
   try:process.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
 for log in logs:log.close()
 if node is not None:node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 arrays={}
 for key,values in poses.items():
  aa=np.asarray(values,dtype=float).reshape((-1,9));arrays[key+'_stamp_ns']=np.array([x[0] for x in values],dtype='int64');arrays[key+'_values']=aa[:,1:8];arrays[key+'_receipt_monotonic']=aa[:,8]
 for key,values in inputs.items():arrays['input_'+key+'_stamp_ns']=np.array(values,dtype='int64')
 np.savez_compressed(out/'lio_outputs.npz',**arrays);after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};result=progress();result.update(stop_reason=reason,hashes_after=after,source_and_application_files_unchanged=before==after,returncodes=[p.returncode for p in processes],input_delivery_exact={key:bool(np.array_equal(expected[key+'_stamp_ns'],arrays['input_'+key+'_stamp_ns'])) for key in inputs});(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'event':'finished',**result}),flush=True)
