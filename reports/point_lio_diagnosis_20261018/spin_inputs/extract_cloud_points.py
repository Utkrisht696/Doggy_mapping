from pathlib import Path
import sys,sqlite3,numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
r=Path(sys.argv[1]);c=sqlite3.connect('file:'+str(next((r/'replay_inputs').glob('*.db3')))+'?mode=ro',uri=True);points=[];times=[];stamps=[];offsets=[0]
for data, in c.execute("select data from messages where topic_id=(select id from topics where name='/utlidar/cloud') order by timestamp,id"):
 m=deserialize_message(data,PointCloud2);f={v.name:v.offset for v in m.fields};dt=np.dtype({'names':['x','y','z','time'],'formats':['<f4']*4,'offsets':[f[k] for k in ['x','y','z','time']],'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);x=np.column_stack([a[k] for k in ['x','y','z']]);t=a['time'];rr=np.sum(x*x,axis=1);mask=np.isfinite(x).all(axis=1)&np.isfinite(t)&(rr>.25)&(rr<100);points.append(x[mask]);times.append(t[mask]);stamps.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec);offsets.append(offsets[-1]+int(mask.sum()))
c.close();np.savez_compressed(r/'raw_points.npz',points=np.concatenate(points),point_times=np.concatenate(times),offsets=np.array(offsets,dtype=np.int64),stamp_ns=np.array(stamps,dtype=np.int64));print(r.name,len(stamps),offsets[-1])
