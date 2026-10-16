"""Passive timing capture: no publications, robot commands or estimators."""
from pathlib import Path
import time,json,sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from unitree_go.msg import SportModeState
ROOT=Path('/home/jain/Doggy');sys.path.insert(0,str(ROOT/'scripts'))
from lidar_cloud_utils import cloud_arrays
DEST=Path(__file__).resolve().parent
rclpy.init();node=rclpy.create_node('doggy_passive_timing_audit',enable_rosout=False,start_parameter_services=False)
rows={k:[] for k in ('body','sport','lidar_imu','odom','relay','clock','cloud')}; errors=[]; field_specs=[]; subs=[]
def receipt():return time.time_ns(),time.monotonic_ns()
def ns(st):return int(st.sec)*10**9+int(st.nanosec)
def imucb(m,name):
 h,mono=receipt();a=m.linear_acceleration;g=m.angular_velocity;q=m.orientation
 rows[name].append((ns(m.header.stamp),h,mono,[a.x,a.y,a.z,g.x,g.y,g.z,q.w,q.x,q.y,q.z]))
def sportcb(m):
 h,mono=receipt();s=m.imu_state
 rows['sport'].append((ns(m.stamp),h,mono,list(s.accelerometer)+list(s.gyroscope)+list(s.quaternion)))
def odomcb(m,name='odom'):
 h,mono=receipt();q=m.pose.pose.orientation
 rows[name].append((ns(m.header.stamp),h,mono,[q.w,q.x,q.y,q.z]))
def clockcb(m):
 h,mono=receipt();rows['clock'].append((ns(m.clock),h,mono,[]))
def cloudcb(m):
 h,mono=receipt()
 try:
  xyz,records,valid=cloud_arrays(m);fields={f.name:f for f in m.fields};f=fields['time'];endian='>' if m.is_bigendian else '<'
  if f.datatype!=7:raise ValueError('time is not float32')
  dtype=np.dtype({'names':['time','ring'],'formats':[endian+'f4',endian+'u2'],'offsets':[f.offset,fields['ring'].offset],'itemsize':m.point_step})
  data=np.ascontiguousarray(records).view(dtype).reshape(-1);t=data['time'];retained=(np.sum(xyz*xyz,axis=1)>.25)
  tr=t[retained];tf=t[np.isfinite(t)]
  vals=[float(tf.min()),float(tf.max()),float(tr.min()) if len(tr) else float('nan'),float(tr.max()) if len(tr) else float('nan'),len(t),int(retained.sum()),int((~np.isfinite(t)).sum()),int((np.diff(t)<0).sum()),int(data['ring'].min()),int(data['ring'].max())]
  rows['cloud'].append((ns(m.header.stamp),h,mono,vals))
  if not field_specs:field_specs.append({'frame':m.header.frame_id,'fields':[{'name':v.name,'offset':v.offset,'type':v.datatype,'count':v.count} for v in m.fields]})
 except Exception as e:errors.append(str(e))
subs.append(node.create_subscription(Imu,'/doggy/imu/body_calibrated',lambda m:imucb(m,'body'),QoSProfile(depth=20000)))
subs.append(node.create_subscription(Imu,'/utlidar/imu',lambda m:imucb(m,'lidar_imu'),QoSProfile(depth=20000)))
subs.append(node.create_subscription(SportModeState,'/sportmodestate',sportcb,QoSProfile(depth=20000)))
subs.append(node.create_subscription(Odometry,'/utlidar/robot_odom',odomcb,QoSProfile(depth=20000)))
subs.append(node.create_subscription(Odometry,'/odom',lambda m:odomcb(m,'relay'),QoSProfile(depth=20000)))
subs.append(node.create_subscription(Clock,'/doggy/robot_clock',clockcb,QoSProfile(depth=100)))
subs.append(node.create_subscription(PointCloud2,'/utlidar/cloud',cloudcb,QoSProfile(depth=20000)))
start=time.monotonic();snapshots=[]
while time.monotonic()-start<45:
 rclpy.spin_once(node,timeout_sec=.02)
 if (not snapshots and time.monotonic()-start>3):
  snapshots.append({t:[{'node':e.node_name,'namespace':e.node_namespace,'reliability':str(e.qos_profile.reliability),'durability':str(e.qos_profile.durability)} for e in node.get_publishers_info_by_topic(t)] for t in ('/clock','/doggy/robot_clock','/utlidar/cloud','/doggy/imu/body_calibrated','/sportmodestate','/utlidar/imu')})
snapshots.append({t:[{'node':e.node_name,'namespace':e.node_namespace,'reliability':str(e.qos_profile.reliability)} for e in node.get_publishers_info_by_topic(t)] for t in ('/clock','/doggy/robot_clock','/utlidar/cloud','/doggy/imu/body_calibrated')})
arrays={}
for name,data in rows.items():
 arrays[name+'_stamp_ns']=np.array([r[0] for r in data],dtype=np.int64)
 arrays[name+'_host_ns']=np.array([r[1] for r in data],dtype=np.int64)
 arrays[name+'_mono_ns']=np.array([r[2] for r in data],dtype=np.int64)
 arrays[name+'_values']=np.array([r[3] for r in data],dtype=float)
np.savez_compressed(DEST/'live_timing_capture.npz',**arrays)
summary={'duration_s':time.monotonic()-start,'counts':{k:len(v) for k,v in rows.items()},'errors':errors,'cloud_field_specs':field_specs,'publisher_snapshots':snapshots,'cloud_columns':['raw_time_min_s','raw_time_max_s','kept_time_min_s','kept_time_max_s','raw_points','kept_points','nonfinite_times','negative_point_time_steps','ring_min','ring_max'],'imu_columns':['ax','ay','az','gx','gy','gz','qw','qx','qy','qz'],'clock_changes_or_robot_commands':False}
(DEST/'live_timing_capture.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({'counts':summary['counts'],'errors':errors,'duration_s':summary['duration_s']}))
node.destroy_node();rclpy.shutdown()
