"""Run original real-go2 LioInput callbacks without ROS, preserving receipt order."""
from pathlib import Path
import collections,importlib.util,sqlite3,json,hashlib,numpy as np
from types import SimpleNamespace
import rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,PointCloud2
from rosgraph_msgs.msg import Clock
r=Path(__file__).resolve().parent;path=r/'unitree_go2_ros2/unitree_go2_real/scripts/lio_input.py';spec=importlib.util.spec_from_file_location('repo_lio_input',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);out=r/'results';bag=out/'lio_replay_inputs'
if bag.exists():raise RuntimeError('Refusing overwrite')
w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={'/utlidar/robot_odom':'nav_msgs/msg/Odometry','/go2/lio_cloud':'sensor_msgs/msg/PointCloud2','/go2/lio_imu':'sensor_msgs/msg/Imu','/doggy/robot_clock':'rosgraph_msgs/msg/Clock'}
for topic,typ in types.items():w.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typ,serialization_format='cdr'))
counts=collections.Counter();observed={k:[] for k in ['vendor','imu','cloud']};logs=[];lastclock=None;current_receipt=None
class Logger:
 def info(self,msg,*args,**kwargs):logs.append({'level':'info','message':msg,'receipt_ns':current_receipt})
 def warn(self,msg,*args,**kwargs):logs.append({'level':'warn','message':msg,'receipt_ns':current_receipt})
class Sink:
 def __init__(self,topic,key):self.topic=topic;self.key=key
 def publish(self,msg):
  observed[self.key].append(m.stamp_ns(msg.header.stamp));w.write(self.topic,serialize_message(msg),current_receipt);counts[self.topic]+=1
class Harness:
 on_odom=m.LioInput.on_odom;on_cloud=m.LioInput.on_cloud;on_imu=m.LioInput.on_imu;moving_now=m.LioInput.moving_now;measure_bias=m.LioInput.measure_bias
 def __init__(self):
  self.box=[-.45,.55,-.55,.5,-.4,.2];self.max_gap_ns=20_000_000;self.odom_t=collections.deque(maxlen=900);self.odom_T=collections.deque(maxlen=900);self.dropped=0;self.bias_mode='startup';self.still_ns=1_500_000_000;self.timeout_ns=20_000_000_000;self.bias=None;self.streak=[];self.first_imu_ns=None;self.cloud_pub=Sink('/go2/lio_cloud','cloud');self.imu_pub=Sink('/go2/lio_imu','imu')
 def get_logger(self):return Logger()
obj=Harness();db=Path('/home/jain/go2_bags/go2_20261018_173219/go2_20261018_173219_0.db3');c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);ids={i:name for i,name in c.execute('select id,name from topics')};selected=[i for i,name in ids.items() if name in ['/utlidar/robot_odom','/utlidar/cloud_deskewed','/utlidar/imu']];query='select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in selected)+') order by timestamp,id'
for topic,receipt,data in c.execute(query,selected):
 current_receipt=int(receipt);name=ids[topic]
 if name=='/utlidar/robot_odom':
  msg=deserialize_message(data,Odometry);stamp=m.stamp_ns(msg.header.stamp);obj.on_odom(msg);observed['vendor'].append(stamp);w.write(name,data,receipt);counts[name]+=1
  if lastclock is None or stamp-lastclock>=20_000_000:w.write('/doggy/robot_clock',serialize_message(Clock(clock=msg.header.stamp)),receipt);counts['/doggy/robot_clock']+=1;lastclock=stamp
 elif name=='/utlidar/cloud_deskewed':obj.on_cloud(deserialize_message(data,PointCloud2))
 else:obj.on_imu(deserialize_message(data,Imu))
del w;c.close();np.savez_compressed(out/'expected_inputs.npz',**{k+'_stamp_ns':np.array(v,dtype='int64') for k,v in observed.items()});origin=int(np.load(r.parent/'source_decoded.npz')['raw_stamp_ns'][0]);report={'repo_commit':'c3158fcef41532fc5dc754e5838b0dfdecb5506a','callback_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'method':'Original LioInput callbacks, full original bag in receipt order, no source patches. Mock publishers write a separate passive replay bag. Defaults copied from original constructor.','counts':dict(counts),'gyro_bias_raw_frame_rad_s':obj.bias.tolist() if obj.bias is not None else None,'clouds_dropped_for_missing_nearby_pose':obj.dropped,'first_cloud_source_s':(observed['cloud'][0]-origin)/1e9 if observed['cloud'] else None,'first_imu_source_s':(observed['imu'][0]-origin)/1e9 if observed['imu'] else None,'logs':logs,'header_stamps_changed':False,'sensor_gaps_filled':False};(out/'input_adapter_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
