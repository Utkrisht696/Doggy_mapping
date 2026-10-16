"""Reconstruct recorded canonical TF exactly at each LIO stamp; no ROS node."""
from pathlib import Path
import sqlite3,json
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
base=Path(__file__).resolve().parent
for label,runid in [('drive','20261016T220926+1100'),('restart','20261016T221648+1100')]:
 run=base.parent/runid;a=np.load(base/label/'decoded.npz');buf=Buffer(cache_time=Duration(seconds=200));db=next((run/'bag').glob('*.db3'));c=sqlite3.connect(f'file:{db}?mode=ro',uri=True);topics={i:n for i,n in c.execute("select id,name from topics where name in ('/tf','/tf_static')")};n=0
 for tid,data in c.execute('select topic_id,data from messages where topic_id in (%s) order by timestamp,id'%','.join(map(str,topics))):
  m=deserialize_message(data,TFMessage)
  for tr in m.transforms:
   if topics[tid]=='/tf_static':buf.set_transform_static(tr,'recorded_bag')
   else:buf.set_transform(tr,'recorded_bag')
   n+=1
 c.close();errors=[];pos=[];ang=[]
 for stamp,p in zip(a['lio_stamp_ns'],a['lio_values']):
  try:
   tr=buf.lookup_transform('map','imu',Time(nanoseconds=int(stamp)));v,q=tr.transform.translation,tr.transform.rotation;pos.append(np.linalg.norm(np.array([v.x,v.y,v.z])-p[:3]));ang.append(np.rad2deg((Rotation.from_quat([q.x,q.y,q.z,q.w]).inv()*Rotation.from_quat(p[3:7])).magnitude()))
  except Exception as e:errors.append({'stamp_ns':int(stamp),'error':str(e)})
 r={'recorded_tf_transforms_loaded':n,'poses_tested':len(a['lio_stamp_ns']),'successful_lookups':len(pos),'lookup_errors':errors,'position_difference_max_m':float(max(pos)) if pos else None,'rotation_difference_max_deg':float(max(ang)) if ang else None,'method':'Recorded global TF chain map->odom->base_footprint->base_link->imu at each original LIO timestamp; no private direct map->imu TF.'};(base/label/'tf_check.json').write_text(json.dumps(r,indent=2)+'\n');print(label,json.dumps(r),flush=True)
