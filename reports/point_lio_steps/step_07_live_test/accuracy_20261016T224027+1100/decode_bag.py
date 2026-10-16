"""Offline read-only bag audit; writes diagnostics only."""
from pathlib import Path
import sqlite3,json,sys
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,PointCloud2
from unitree_go.msg import SportModeState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
base=Path(__file__).resolve().parent.parent
ns=lambda s:int(s.sec)*10**9+int(s.nanosec)
for label,runid in [('analysis','accuracy_20261016T224027+1100')]:
 run=base/runid;dest=Path(__file__).resolve().parent/label;dest.mkdir(exist_ok=True)
 rows={k:[] for k in ['lio','vendor','imu','sport','raw','tfmap']};statuses=[];parents={};frames={};topiccounts={}
 classes={'/point_lio/odometry':Odometry,'/utlidar/robot_odom':Odometry,'/doggy/imu/body_calibrated':Imu,'/sportmodestate':SportModeState,'/utlidar/cloud':PointCloud2,'/point_lio/tf_status':String,'/tf':TFMessage,'/tf_static':TFMessage}
 db=next((run/'bag').glob('*.db3'));c=sqlite3.connect(f'file:{db}?mode=ro',uri=True);topics={i:n for i,n in c.execute('select id,name from topics')};topiccounts={topics[i]:n for i,n in c.execute('select topic_id,count(*) from messages group by topic_id')}
 for topic,receipt,data in c.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
  name=topics[topic]
  if name not in classes:continue
  m=deserialize_message(data,classes[name])
  if hasattr(m,'header'):frames.setdefault(name,set()).add(m.header.frame_id)
  if name in ['/point_lio/odometry','/utlidar/robot_odom']:
   p,q=m.pose.pose.position,m.pose.pose.orientation;v,w=m.twist.twist.linear,m.twist.twist.angular
   rows['lio' if name.startswith('/point_lio') else 'vendor'].append((ns(m.header.stamp),receipt,p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z))
  elif name=='/doggy/imu/body_calibrated':
   a,g,q=m.linear_acceleration,m.angular_velocity,m.orientation;rows['imu'].append((ns(m.header.stamp),receipt,a.x,a.y,a.z,g.x,g.y,g.z,q.x,q.y,q.z,q.w))
  elif name=='/sportmodestate':rows['sport'].append((ns(m.stamp),receipt,*m.position,*m.velocity,m.yaw_speed,m.mode,m.gait_type,*m.imu_state.quaternion))
  elif name=='/utlidar/cloud':
   f={f.name:f for f in m.fields};dt=np.dtype({'names':['x','y','z','time'],'formats':[('>' if m.is_bigendian else '<')+'f4']*4,'offsets':[f[k].offset for k in ['x','y','z','time']],'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);rad=a['x']**2+a['y']**2+a['z']**2;good=np.isfinite(rad)&(rad>.25)&np.isfinite(a['time']);t=a['time'][good];rows['raw'].append((ns(m.header.stamp),receipt,len(a),len(t),float(t.min()) if len(t) else 0,float(t.max()) if len(t) else 0))
  elif name=='/point_lio/tf_status':statuses.append(json.loads(m.data))
  else:
   for tr in m.transforms:
    parents.setdefault(tr.child_frame_id,set()).add(tr.header.frame_id)
    if tr.header.frame_id=='map' and tr.child_frame_id=='odom':
     p,q=tr.transform.translation,tr.transform.rotation;rows['tfmap'].append((ns(tr.header.stamp),receipt,p.x,p.y,p.z,q.x,q.y,q.z,q.w))
 c.close();out={}
 for name,v in rows.items():
  out[name+'_stamp_ns']=np.array([r[0] for r in v],dtype=np.int64);out[name+'_receipt_ns']=np.array([r[1] for r in v],dtype=np.int64);out[name+'_values']=np.array([r[2:] for r in v],dtype=float)
 np.savez_compressed(dest/'decoded.npz',**out)
 d={'source_run':str(run),'topic_counts':topiccounts,'frames':{k:sorted(v) for k,v in frames.items()},'tf_parents':{k:sorted(v) for k,v in parents.items()},'bridge_status_samples':statuses};(dest/'bag_audit.json').write_text(json.dumps(d,indent=2)+'\n');print(label,{k:len(v) for k,v in rows.items()},flush=True)
