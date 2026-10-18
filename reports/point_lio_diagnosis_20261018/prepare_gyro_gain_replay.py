from pathlib import Path
import argparse,json,sqlite3,hashlib,shutil,numpy as np
from collections import Counter
import rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import Imu
p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');p.add_argument('--gain',type=float,required=True);a=p.parse_args();src=Path(a.source).resolve();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False);assert .9<a.gain<1.1
c=sqlite3.connect('file:'+str(next((src/'replay_stationary_start').glob('*.db3')))+'?mode=ro',uri=True);writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(out/'replay_inputs'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'));topics={}
for identifier,name,typ,serialization in c.execute('select id,name,type,serialization_format from topics'):
 topics[identifier]=name;writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format=serialization))
counts=Counter();before={n:hashlib.sha256() for n in topics.values()};after={n:hashlib.sha256() for n in topics.values()}
for identifier,stamp,data in c.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
 name=topics[identifier];before[name].update(data)
 if name=='/doggy/imu/body_calibrated':
  msg=deserialize_message(data,Imu);msg.angular_velocity.z*=a.gain;data=serialize_message(msg)
 after[name].update(data);writer.write(name,data,stamp);counts[name]+=1
c.close();del writer
(out/'replay_stationary_start').symlink_to('replay_inputs',target_is_directory=True)
x=np.load(src/'source_decoded.npz');arrays={k:x[k].copy() for k in x.files};arrays['imu_values'][:,5]*=a.gain;np.savez_compressed(out/'source_decoded.npz',**arrays)
for name in ['candidate_holds.json','source_timing.json','stationary_start.json','expected_replay_stamps.npz']:(out/name).symlink_to(src/name)
d=json.loads((src/'input_audit.json').read_text());d['derived_gyro_calibration']={'gyro_z_gain':a.gain,'source_prepared_inputs':str(src),'calibration_training_bag':'running_mode_spins','validation_bag':'spinny','stamps_unchanged':True,'acceleration_and_orientation_unchanged':True,'all_non_imu_topic_bytes_identical':all(before[n].digest()==after[n].digest() for n in topics.values() if n!='/doggy/imu/body_calibrated'),'counts':dict(counts),'changes':'Multiply only body gyro z after the previously approved fixed bias correction; original bag and active adapter are untouched.'};assert d['derived_gyro_calibration']['all_non_imu_topic_bytes_identical'];(out/'input_audit.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d['derived_gyro_calibration'],indent=2))
