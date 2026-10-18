"""Read-only Jetson MCAP audit and isolated replay input preparation. No commands."""
from pathlib import Path
import sys,json,yaml,struct,zlib,hashlib,math,time
from collections import Counter
import numpy as np
sys.path.insert(0,'/tmp/doggy_mcap_integrity_tools')
ROOT=Path('/home/jain/Doggy');sys.path.insert(0,str(ROOT/'scripts'))
from mcap.stream_reader import StreamReader,breakup_chunk
from mcap.records import Chunk,Schema,Channel,Message,Statistics,Footer,DataEnd
from mcap_ros2.decoder import DecoderFactory
from publish_body_imu import BodyImuConverter,load_calibration
from go2_frames import split_body_pose
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
import rosbag2_py
from scipy.spatial.transform import Rotation
out=Path(__file__).resolve().parent;bag=Path('/home/jain/go2_bags/default');meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];source=bag/meta['relative_file_paths'][0];st=source.stat();counts=Counter();schemas={};channels={};factory=DecoderFactory();decoders={};cal=load_calibration(ROOT/'config/point_lio/body_imu_calibration.json');converter=BodyImuConverter(cal);geometry=json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text());rows={k:[] for k in ['vendor','imu','sport','raw']};errors=[];stats=None;footer=None;chunkcount=0;crcchunks=0;lasttf=None;written=Counter();first_log=None;last_log=None;static_written=False;pending={};geometry_errors=[];geometry_pairs=0;cloud_n=0;start_wall=time.monotonic();lastprint=start_wall
replay=out/'replay_inputs'
if replay.exists():raise RuntimeError('Replay output already exists; refusing overwrite')
writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(replay),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
types={'/utlidar/cloud':'sensor_msgs/msg/PointCloud2','/doggy/imu/body_calibrated':'sensor_msgs/msg/Imu','/utlidar/robot_odom':'nav_msgs/msg/Odometry','/doggy/robot_clock':'rosgraph_msgs/msg/Clock','/tf':'tf2_msgs/msg/TFMessage','/tf_static':'tf2_msgs/msg/TFMessage'}
for name,typ in types.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
def put(name,data,stamp):writer.write(name,data,int(stamp));written[name]+=1
def ns(t):return int(t.sec)*10**9+int(t.nanosec)
def transform(parent,child,pos,q,stamp=None):
 m=TransformStamped();m.header.frame_id=parent;m.child_frame_id=child
 if stamp is not None:m.header.stamp=stamp
 m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=map(float,pos);m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=map(float,q);return m
static=[]
for child in ['imu','radar']:
 g=geometry['unchanged_developer_mounts'][child];static.append(transform('base_link',child,g['xyz'],Rotation.from_euler('xyz',g['rpy']).as_quat()))
g=geometry['measurement_tf'];static.append(transform(g['parent'],g['child'],g['translation_m'],g['rotation_xyzw']));static_msg=serialize_message(TFMessage(transforms=static))
R=np.array(geometry['point_lio_body_imu_extrinsics']['mapping.extrinsic_R']).reshape(3,3);T=np.array(geometry['point_lio_body_imu_extrinsics']['mapping.extrinsic_T'])+np.array(geometry['unchanged_developer_mounts']['imu']['xyz'])
def keyed(m):
 fields={f.name:f.offset for f in m.fields};names=['x','y','z','ring','timebits','intensitybits'];formats=['<f4']*3+['<u2','<u4','<u4'];offset=[fields[k] for k in ['x','y','z','ring','time','intensity']];dt=np.dtype({'names':names,'formats':formats,'offsets':offset,'itemsize':m.point_step});a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);keys=[(int(v['ring']),int(v['timebits']),int(v['intensitybits'])) for v in a];cnt=Counter(keys);return {k:np.array([v['x'],v['y'],v['z']],dtype=float) for k,v in zip(keys,a) if cnt[k]==1 and np.isfinite([v['x'],v['y'],v['z']]).all() and float(v['x'])**2+float(v['y'])**2+float(v['z'])**2>.25}
def consume(rec):
 global stats,footer,first_log,last_log,static_written,lasttf,geometry_pairs,cloud_n
 if isinstance(rec,Schema):schemas[rec.id]=rec;return
 if isinstance(rec,Channel):channels[rec.id]=rec;return
 if isinstance(rec,Statistics):stats=rec;return
 if isinstance(rec,Footer):footer=rec;return
 if not isinstance(rec,Message):return
 name=channels[rec.channel_id].topic;counts[name]+=1;first_log=rec.log_time if first_log is None else min(first_log,rec.log_time);last_log=rec.log_time if last_log is None else max(last_log,rec.log_time)
 if not static_written:put('/tf_static',static_msg,rec.log_time);static_written=True
 if name not in ['/utlidar/cloud','/utlidar/cloud_base','/sportmodestate','/utlidar/robot_odom']:return
 ch=channels[rec.channel_id]
 if name=='/sportmodestate':
  if rec.channel_id not in decoders:decoders[rec.channel_id]=factory.decoder_for(ch.message_encoding,schemas[ch.schema_id])
  m=decoders[rec.channel_id](rec.data);stamp=ns(m.stamp);imu=converter.convert(m);rows['sport'].append((stamp,rec.log_time,*m.position,*m.velocity,m.yaw_speed,m.mode,m.gait_type,*m.imu_state.quaternion))
  if imu is not None:
   q=imu.orientation;v=imu.linear_acceleration;w=imu.angular_velocity;rows['imu'].append((stamp,rec.log_time,v.x,v.y,v.z,w.x,w.y,w.z,q.x,q.y,q.z,q.w));put('/doggy/imu/body_calibrated',serialize_message(imu),rec.log_time)
 elif name=='/utlidar/robot_odom':
  m=deserialize_message(rec.data,Odometry);p,q=m.pose.pose.position,m.pose.pose.orientation;v,w=m.twist.twist.linear,m.twist.twist.angular;stamp=ns(m.header.stamp)
  if m.header.frame_id!='odom' or m.child_frame_id!='base_link':raise ValueError('Unexpected vendor odometry frames')
  rows['vendor'].append((stamp,rec.log_time,p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z));put(name,rec.data,rec.log_time)
  if lasttf is None or stamp-lasttf>=20_000_000:
   fp,fq,bp,bq=split_body_pose([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],0.0);trs=[transform('odom','base_footprint',fp,fq,m.header.stamp),transform('base_footprint','base_link',bp,bq,m.header.stamp)];put('/tf',serialize_message(TFMessage(transforms=trs)),rec.log_time);put('/doggy/robot_clock',serialize_message(Clock(clock=m.header.stamp)),rec.log_time);lasttf=stamp
 else:
  m=deserialize_message(rec.data,PointCloud2);stamp=ns(m.header.stamp)
  if len(m.data)!=m.row_step*m.height:raise ValueError('Point cloud layout invalid')
  if name=='/utlidar/cloud':
   cloud_n+=1;put(name,rec.data,rec.log_time);f={f.name:f.offset for f in m.fields};dt=np.dtype({'names':['x','y','z','time'],'formats':['<f4']*4,'offsets':[f[k] for k in ['x','y','z','time']],'itemsize':m.point_step});arr=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);rad=arr['x']**2+arr['y']**2+arr['z']**2;valid=np.isfinite(rad)&np.isfinite(arr['time'])&(rad>.25);pt=arr['time'][valid];rows['raw'].append((stamp,rec.log_time,len(arr),int(valid.sum()),float(pt.min()) if len(pt) else 0,float(pt.max()) if len(pt) else 0))
  # Match by stamp and unique ring/time/intensity signatures; sparse audit across route.
  if (name=='/utlidar/cloud' and cloud_n%100==1) or (name=='/utlidar/cloud_base' and stamp in pending):
   pending.setdefault(stamp,{})[name]=m
  if stamp in pending and len(pending[stamp])==2:
   pair=pending.pop(stamp);raw=keyed(pair['/utlidar/cloud']);body=keyed(pair['/utlidar/cloud_base']);keys=raw.keys()&body.keys()
   if keys:
    x=np.array([raw[k] for k in keys]);y=np.array([body[k] for k in keys]);err=np.linalg.norm(x@R.T+T-y,axis=1);geometry_errors.extend(err.tolist());geometry_pairs+=1
  if len(pending)>20:pending.pop(next(iter(pending)))
with source.open('rb') as f:
 for record in StreamReader(f,emit_chunks=True,validate_crcs=True,record_size_limit=256*1024*1024).records:
  if isinstance(record,Chunk):
   chunkcount+=1;crcchunks+=bool(record.uncompressed_crc)
   for inner in breakup_chunk(record,validate_crc=True):consume(inner)
  else:consume(record)
  if time.monotonic()-lastprint>10:
   print(json.dumps({'messages_checked':sum(counts.values()),'clouds':len(rows['raw']),'calibrated_imu':len(rows['imu'])}),flush=True);lastprint=time.monotonic()
 assert f.tell()==st.st_size
# Destroy writer to flush its Humble-compatible metadata.
del writer
with source.open('rb') as f:
 f.seek(footer.summary_start or st.st_size-37);summarycrc=zlib.crc32(f.read(st.st_size-12-f.tell()))
expected={t['topic_metadata']['name']:t['message_count'] for t in meta['topics_with_message_count']};mismatch={k:[expected.get(k,0),counts.get(k,0)] for k in expected.keys()|counts.keys() if expected.get(k,0)!=counts.get(k,0)}
assert not mismatch and sum(counts.values())==meta['message_count']==stats.message_count
assert not footer.summary_crc or summarycrc==footer.summary_crc
assert st.st_size==source.stat().st_size and st.st_mtime_ns==source.stat().st_mtime_ns
arrays={}
for k,values in rows.items():arrays[k+'_stamp_ns']=np.array([r[0] for r in values],dtype=np.int64);arrays[k+'_receipt_ns']=np.array([r[1] for r in values],dtype=np.int64);arrays[k+'_values']=np.array([r[2:] for r in values],dtype=float)
np.savez_compressed(out/'source_decoded.npz',**arrays);origin=int(arrays['raw_stamp_ns'][0]);vt=(arrays['vendor_stamp_ns']-origin)/1e9;initial=vt<=2;vendor=arrays['vendor_values'];it=(arrays['imu_stamp_ns']-origin)/1e9;initialimu=arrays['imu_values'][it<=2];stat={'source_bag':str(source),'duration_s':(last_log-first_log)/1e9,'messages':sum(counts.values()),'metadata_counts_match':not mismatch,'chunks':chunkcount,'chunks_with_crc':crcchunks,'summary_crc_present':bool(footer.summary_crc),'summary_crc_matches':summarycrc==footer.summary_crc,'counts':dict(counts),'replay_topics':dict(written),'raw_frames':['utlidar_lidar'],'calibrated_imu_duplicates_dropped':counts['/sportmodestate']-len(rows['imu']),'body_converter_errors':0,'geometry_pairs':geometry_pairs,'geometry_matches':len(geometry_errors),'geometry_max_residual_m':max(geometry_errors) if geometry_errors else None,'initial_2s_vendor_position_excursion_m':float(np.linalg.norm(vendor[initial,:3]-vendor[initial,:3][0],axis=1).max()),'initial_2s_calibrated_acc_norm_mean':float(np.mean(np.linalg.norm(initialimu[:,:3],axis=1))),'initial_2s_calibrated_gyro_norm_max':float(np.linalg.norm(initialimu[:,3:6],axis=1).max()),'robot_commands_replayed':False,'metadata_compatibility':'Original Jazzy MCAP left unchanged; separate Humble sqlite replay contains only raw cloud, calibrated body IMU, vendor odometry, derived clock and canonical sensor TF.'};(out/'input_audit.json').write_text(json.dumps(stat,indent=2)+'\n');print(json.dumps(stat,indent=2),flush=True)
