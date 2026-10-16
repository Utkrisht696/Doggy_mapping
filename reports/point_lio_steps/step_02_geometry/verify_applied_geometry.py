"""Passive verification of the approved sensor geometry; no publications."""
import json, sys, time, hashlib
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from lidar_cloud_utils import cloud_arrays
DEST=Path(__file__).resolve().parent
profile=json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())
rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock'])
n=rclpy.create_node('verify_approved_geometry',enable_rosout=False)
buf=Buffer(node=n); listener=TransformListener(buf,n)
clouds={}; pairs=[]; parents=defaultdict(set); imu=[]; raw=[]; body=[]; subscriptions=[]
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def cloud(m,topic):
 key=stamp(m); clouds.setdefault(key,{})[topic]=m
 if len(clouds)>80:clouds.pop(next(iter(clouds)))
for topic in ('raw','body'):
 name='/utlidar/cloud' if topic=='raw' else '/utlidar/cloud_base'
 subscriptions.append(n.create_subscription(PointCloud2,name,lambda m,t=topic:cloud(m,t),qos_profile_sensor_data))
def tf(m):
 for t in m.transforms:parents[t.child_frame_id].add(t.header.frame_id)
subscriptions.append(n.create_subscription(TFMessage,'/tf',tf,100))
subscriptions.append(n.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
def imucb(m):
 imu.append({'frame':m.header.frame_id,'stamp_ns':stamp(m),'acc_norm':float(np.linalg.norm([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]))})
subscriptions.append(n.create_subscription(Imu,'/doggy/imu/body_calibrated',imucb,qos_profile_sensor_data))
def pose(t):
 q=t.transform.rotation; p=t.transform.translation
 return Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.array([p.x,p.y,p.z])
def keyed(m):
 xyz,records,valid=cloud_arrays(m); f={v.name:v for v in m.fields}; endian='>' if m.is_bigendian else '<'
 dtype=np.dtype({'names':['ring','time','intensity'],'formats':[endian+'u2',endian+'u4',endian+'u4'],'offsets':[f[v].offset for v in ('ring','time','intensity')],'itemsize':m.point_step})
 keys=np.ascontiguousarray(records).view(dtype).reshape(-1)
 tuples=[tuple(map(int,(k['ring'],k['time'],k['intensity']))) for k in keys]
 counts=Counter(k for k,v in zip(tuples,valid) if v)
 return {k:xyz[i].astype(float) for i,k in enumerate(tuples) if valid[i] and counts[k]==1}
end=time.monotonic()+16; next_check=0.; done=set(); errors=[]
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.02)
 if time.monotonic()<next_check:continue
 next_check=time.monotonic()+.3
 for key, msgs in list(clouds.items()):
  if key in done or set(msgs)!= {'raw','body'}:continue
  a,b=msgs['raw'],msgs['body']
  try:
   # Lookup the live TF, rather than reusing the stored matrix being tested.
   tr=buf.lookup_transform('base_link','utlidar_lidar',Time.from_msg(a.header.stamp))
   R,t=pose(tr); ka,kb=keyed(a),keyed(b); keys=ka.keys() & kb.keys()
   if not keys:continue
   x=np.array([ka[k] for k in keys]); y=np.array([kb[k] for k in keys]); err=np.linalg.norm(x@R.T+t-y,axis=1)
   odom=buf.lookup_transform('odom','utlidar_lidar',Time.from_msg(a.header.stamp))
   pairs.append({'stamp_ns':key,'count':len(keys),'raw_frame':a.header.frame_id,'body_frame':b.header.frame_id,'max_residual_m':float(err.max()),'odom_tf_at_source_stamp':True})
   raw.append(x);body.append(y);done.add(key)
  except Exception as exc:errors.append(str(exc))
  break
 if len(pairs)>=15 and len(imu)>=1000:break
extrinsic={}
try:
 R,t=pose(buf.lookup_transform('imu','utlidar_lidar',Time()))
 extrinsic={'rotation':R.tolist(),'translation_m':t.tolist(),'max_rotation_error':float(np.max(np.abs(R-np.array(profile['point_lio_body_imu_extrinsics']['mapping.extrinsic_R']).reshape(3,3)))),'max_translation_error_m':float(np.max(np.abs(t-profile['point_lio_body_imu_extrinsics']['mapping.extrinsic_T'])))}
except Exception as exc:errors.append(str(exc))
endpoints={topic:[e.node_name for e in n.get_publishers_info_by_topic(topic)] for topic in ('/tf','/tf_static','/doggy/imu/body_calibrated','/doggy/robot_clock')}
tree=yaml.safe_load(buf.all_frames_as_yaml()) or {}
checks={'fresh_pairs':len(pairs)>=10,'live_tf_matches_vendor_body_cloud':bool(pairs) and max(p['max_residual_m'] for p in pairs)<1e-5,'expected_cloud_frames':all(p['raw_frame']=='utlidar_lidar' and p['body_frame']=='base_link' for p in pairs),'body_imu_extrinsics_match_tf':bool(extrinsic) and extrinsic['max_rotation_error']<1e-12 and extrinsic['max_translation_error_m']<1e-12,'unique_observed_parents':bool(parents) and all(len(v)==1 for v in parents.values()),'measurement_parent':parents.get('utlidar_lidar')=={'radar'},'no_legacy_body_alias':'base' not in tree,'all_45_frames_present':len(tree)+1==45,'expected_tf_endpoints':sorted(endpoints['/tf'])==['doggy_go2_state','go2_robot_state_publisher'] and endpoints['/tf_static']==['go2_robot_state_publisher'],'body_imu_live_and_unique':len(imu)>=1000 and endpoints['/doggy/imu/body_calibrated']==['doggy_body_imu'] and all(m['frame']=='imu' for m in imu),'one_robot_clock':endpoints['/doggy/robot_clock']==['doggy_go2_state']}
result={'passed':all(checks.values()),'checks':checks,'pairs':pairs,'matched_points':sum(p['count'] for p in pairs),'max_residual_m':max((p['max_residual_m'] for p in pairs),default=None),'body_imu_samples':len(imu),'body_imu_acc_norm_mean':float(np.mean([m['acc_norm'] for m in imu])) if imu else None,'extrinsic_tf':extrinsic,'publisher_endpoints':endpoints,'observed_parents':{k:sorted(v) for k,v in parents.items()},'frame_count':len(tree)+1,'transient_lookup_errors':errors[-5:],'ownership_method':'Observed parent uniqueness plus publisher endpoints and reviewed source ownership; this Humble API does not provide per-message publisher GID.','limitation':'Stream-coordinate agreement, not surveyed physical placement accuracy.'}
(DEST/'applied_geometry_validation.json').write_text(json.dumps(result,indent=2)+'\n')
if raw:np.savez_compressed(DEST/'applied_geometry_points.npz',raw=np.vstack(raw),body=np.vstack(body))
print(json.dumps({k:v for k,v in result.items() if k not in ('pairs','observed_parents')},indent=2))
n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
