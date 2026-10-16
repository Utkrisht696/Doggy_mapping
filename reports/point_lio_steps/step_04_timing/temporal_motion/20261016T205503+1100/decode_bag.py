"""Offline-only bag decoding; run only after recorder has stopped."""
from pathlib import Path
import json,sqlite3,sys,time
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path('/home/jain/Doggy');sys.path.insert(0,str(ROOT/'scripts'))
from lidar_cloud_utils import cloud_arrays
INFO=json.loads((ROOT/'reports/point_lio_steps/step_04_timing/temporal_motion/current_recording.json').read_text());DEST=ROOT/INFO['directory'];assert INFO['status']=='stopped_saved'
con=sqlite3.connect(f"file:{DEST/'bag/bag_0.db3'}?mode=ro&immutable=1",uri=True)
topics={r[0]:(r[1],r[2]) for r in con.execute('select id,name,type from topics')}
selected={'/doggy/imu/body_calibrated':'body','/sportmodestate':'sport','/utlidar/imu':'lidar_imu','/utlidar/robot_odom':'odom','/odom':'relay','/doggy/robot_clock':'clock','/utlidar/cloud':'cloud','/utlidar/lidar_state':'status'}
rows={name:[] for name in selected.values()};points=[];point_times=[];cloud_offsets=[0];errors=[]
start=time.monotonic()
def ns(t):return int(t.sec)*10**9+int(t.nanosec)
classes={tid:get_message(typ) for tid,(topic,typ) in topics.items() if topic in selected}
for tid,received,data in con.execute('select topic_id,timestamp,data from messages order by timestamp'):
 topic,_=topics[tid]
 if topic not in selected:continue
 try:
  m=deserialize_message(data,classes[tid]);name=selected[topic]
  if name in ('body','lidar_imu'):
   a,g,q=m.linear_acceleration,m.angular_velocity,m.orientation
   rows[name].append((ns(m.header.stamp),received,[a.x,a.y,a.z,g.x,g.y,g.z,q.w,q.x,q.y,q.z]))
  elif name=='sport':
   st=m.imu_state
   rows[name].append((ns(m.stamp),received,[*st.accelerometer,*st.gyroscope,*st.quaternion,*st.rpy]))
  elif name in ('odom','relay'):
   p,q=m.pose.pose.position,m.pose.pose.orientation;w=m.twist.twist.angular
   rows[name].append((ns(m.header.stamp),received,[p.x,p.y,p.z,q.w,q.x,q.y,q.z,w.x,w.y,w.z]))
  elif name=='clock':rows[name].append((ns(m.clock),received,[]))
  elif name=='status':rows[name].append((int(round(m.stamp*1e9)),received,[m.cloud_frequency,m.cloud_packet_loss_rate,m.imu_frequency,m.imu_packet_loss_rate,m.error_state]))
  elif name=='cloud':
   xyz,records,valid=cloud_arrays(m);f={v.name:v for v in m.fields};dtype=np.dtype({'names':['time'],'formats':[('>' if m.is_bigendian else '<')+'f4'],'offsets':[f['time'].offset],'itemsize':m.point_step})
   t=np.ascontiguousarray(records).view(dtype).reshape(-1)['time'];keep=valid&(np.sum(xyz*xyz,axis=1)>.25)&np.isfinite(t)
   tr=t[keep];points.append(xyz[keep]);point_times.append(tr);cloud_offsets.append(cloud_offsets[-1]+len(tr))
   rows[name].append((ns(m.header.stamp),received,[float(tr.min()) if len(tr) else float('nan'),float(tr.max()) if len(tr) else float('nan'),len(t),len(tr),int((~np.isfinite(t)).sum()),int((np.diff(t)<0).sum())]))
 except Exception as e:errors.append({'topic':topic,'error':str(e)})
con.close();arrays={}
for name,rr in rows.items():
 arrays[name+'_stamp_ns']=np.array([r[0] for r in rr],dtype=np.int64);arrays[name+'_receipt_ns']=np.array([r[1] for r in rr],dtype=np.int64);arrays[name+'_values']=np.array([r[2] for r in rr],dtype=float)
arrays.update(cloud_offsets=np.array(cloud_offsets,dtype=np.int64),cloud_points=np.vstack(points),cloud_point_times=np.concatenate(point_times))
np.savez_compressed(DEST/'decoded_samples.npz',**arrays)
summary={'bag':INFO['bag'],'decoded_counts':{k:len(v) for k,v in rows.items()},'errors':errors,'source_bag_unchanged':True,'decoding_elapsed_s':time.monotonic()-start,'cue_host_ns':INFO['cue_host_ns'],'imu_columns':['ax','ay','az','gx','gy','gz','qw','qx','qy','qz'],'sport_columns':['ax','ay','az','gx','gy','gz','qw','qx','qy','qz','roll','pitch','yaw'],'odom_columns':['x','y','z','qw','qx','qy','qz','wx','wy','wz'],'cloud_columns':['kept_time_min_s','kept_time_max_s','raw_points','kept_points','nonfinite_times','negative_time_steps']}
(DEST/'decoded_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
