"""Source clouds in the fixed IMU axes; no odometry used for alignment."""
from pathlib import Path
import sqlite3,json,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
r=Path(__file__).resolve().parent;s=np.load(r/'source_decoded.npz');origin=int(s['raw_stamp_ns'][0]);g=json.loads(Path('/home/jain/Doggy/config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics'];R=np.array(g['mapping.extrinsic_R']).reshape(3,3);T=np.array(g['mapping.extrinsic_T']);windows={'start':[2.6,3.6],'finish':[210.,216.]};clouds={k:[] for k in windows};stamps={k:[] for k in windows};db=next((r/'replay_inputs').glob('*.db3'));c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
for data, in c.execute("select data from messages where topic_id=(select id from topics where name='/utlidar/cloud') order by timestamp,id"):
 m=deserialize_message(data,PointCloud2);stamp=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec);t=(stamp-origin)/1e9
 for name,(start,end) in windows.items():
  if not start<=t<=end:continue
  f={x.name:x.offset for x in m.fields};endian='>' if m.is_bigendian else '<';dt=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[f[k] for k in ['x','y','z']],'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);p=np.column_stack([a[k] for k in ['x','y','z']]).astype(float);norm=np.linalg.norm(p,axis=1);p=p[np.isfinite(p).all(axis=1)&(norm>.5)&(norm<8)];p=p@R.T+T;clouds[name].append(p);stamps[name].append(t)
out={}
for name in clouds:
 out[name+'_points']=np.concatenate(clouds[name]);out[name+'_offsets']=np.r_[0,np.cumsum([len(x) for x in clouds[name]])];out[name+'_time']=np.array(stamps[name])
np.savez_compressed(r/'return_source_clouds.npz',**out);print({k:{'frames':len(v),'points':len(out[k+'_points'])} for k,v in clouds.items()})
