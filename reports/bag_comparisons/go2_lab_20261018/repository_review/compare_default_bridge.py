"""Call the pinned repository's original odometry callback offline, without ROS nodes."""
from pathlib import Path
from types import SimpleNamespace
import collections,importlib.util,sqlite3,json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import yaml
r=Path(__file__).resolve().parent;repo=r/'unitree_go2_ros2';path=repo/'unitree_go2_real/scripts/real_tf_bridge.py';spec=importlib.util.spec_from_file_location('review_real_tf_bridge',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);out=r/'results';out.mkdir(exist_ok=True);origin=int(np.load(r.parent/'source_decoded.npz')['raw_stamp_ns'][0]);config=yaml.safe_load((repo/'unitree_go2_real/config/real_bridge.yaml').read_text())['real_tf_bridge']['ros__parameters'];body_mount=np.array([-.02557,0,.04232])
class Sink:
 def __init__(self,collect=False):self.rows=[];self.collect=collect
 def publish(self,msg):
  if not self.collect:return
  p,q=msg.pose.pose.position,msg.pose.pose.orientation;self.rows.append([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
 def sendTransform(self,msg):pass
class Clock:
 def __init__(self):self.ns=0
 def now(self):return SimpleNamespace(nanoseconds=self.ns)
class Logger:
 def info(self,*args,**kwargs):pass
 def warn(self,*args,**kwargs):pass
class Harness:
 update_offset=m.RealTfBridge.update_offset
 on_odom=m.RealTfBridge.on_odom
 def __init__(self,k):
  self.window_ns=5*m.NS;self.k=k;self.lio=False;self.rob_t=collections.deque(maxlen=m.HISTORY);self.rob_p=collections.deque(maxlen=m.HISTORY);self.lio_anchor=None;self.lio_M=(0.,0.,0.);self.offset_ns=None;self.recent=collections.deque();self.prev=None;self.cor=None;self.hist_t=collections.deque(maxlen=m.HISTORY);self.hist_T=collections.deque(maxlen=m.HISTORY);self.odom_pub=Sink(True);self.odom_raw_pub=Sink();self.tf=Sink();self.clock=Clock()
 def get_clock(self):return self.clock
 def get_logger(self):return Logger()
db=Path('/home/jain/go2_bags/go2_20261018_173219/go2_20261018_173219_0.db3');c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);rows=list(c.execute("select timestamp,data from messages where topic_id=(select id from topics where name='/utlidar/robot_odom') order by timestamp,id"));results={};arrays={}
for name,k in [('identity',1.),('repo_default',float(config['yaw_scale']))]:
 obj=Harness(k);stamps=[];raw=[]
 for receipt,data in rows:
  msg=deserialize_message(data,Odometry);stamps.append(m.stamp_ns(msg.header.stamp));obj.clock.ns=receipt;p,q=msg.pose.pose.position,msg.pose.pose.orientation;raw.append([p.x,p.y,p.z,q.x,q.y,q.z,q.w]);obj.on_odom(msg)
 values=np.asarray(obj.odom_pub.rows);stamps=np.asarray(stamps,dtype='int64');time=(stamps-origin)/1e9;rot=Rotation.from_quat(values[:,3:7]);pos=values[:,:3]+rot.apply(body_mount);holds={}
 for label,(start,end) in {'start':[2.6,3.6],'finish':[210.,216.]}.items():
  mask=(time>=start)&(time<=end);holds[label]={'median_position_imu_m':np.median(pos[mask],axis=0).tolist(),'orientation_xyzw':rot[mask].mean().as_quat().tolist(),'p95_position_radius_m':float(np.percentile(np.linalg.norm(pos[mask]-np.median(pos[mask],axis=0),axis=1),95))}
 delta=np.array(holds['finish']['median_position_imu_m'])-holds['start']['median_position_imu_m'];rb=Rotation.from_quat(holds['start']['orientation_xyzw']);re=Rotation.from_quat(holds['finish']['orientation_xyzw']);yaw=np.rad2deg(m.wrap(re.as_euler('xyz')[2]-rb.as_euler('xyz')[2]));heading0=rb.as_euler('xyz')[2];plotted=Rotation.from_euler('z',-heading0).apply(pos-holds['start']['median_position_imu_m']);result={'yaw_scale':k,'samples':len(values),'horizontal_return_offset_m':float(np.linalg.norm(delta[:2])),'height_return_change_m':float(delta[2]),'heading_return_change_deg':float(yaw),'full_orientation_change_deg':float((rb.inv()*re).magnitude()*180/np.pi),'holds':holds}
 if k==1.:result['identity_callback_matches_original_pose_max_absolute_difference']=float(np.max(abs(values-np.asarray(raw))));assert np.max(abs(values-np.asarray(raw)))<1e-10
 results[name]=result;arrays[name+'_values']=values;arrays[name+'_initial_heading_xyz']=plotted
arrays['source_stamp_ns']=stamps;np.savez_compressed(out/'corrected_vendor.npz',**arrays);report={'repository':'https://github.com/Faheemibrahim/unitree_go2_ros2/tree/real-go2','commit':'c3158fcef41532fc5dc754e5838b0dfdecb5506a','application_file':str(path),'callback_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'method':'Original RealTfBridge.on_odom called for every source message in bag receipt order. Mock sinks/clock replace only ROS side effects; yaw correction and offset calculations unchanged. No fitting to this bag. Metrics use the same IMU lever arm and quiet holds as the existing lab comparison.','source_seconds_origin_ns':origin,'results':results,'source_gap_policy':'Preserved; bridge integrates pose steps across gaps using recorded endpoint poses and yaw, without inventing intermediate sensor data.'};(out/'default_bridge_comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(results,indent=2))
