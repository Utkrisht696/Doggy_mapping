"""Read-only MCAP integrity, embedded-schema decoding and metadata audit."""
from pathlib import Path
from collections import Counter,defaultdict
import sys,json,struct,zlib,hashlib,time,math
sys.path.insert(0,'/tmp/doggy_mcap_integrity_tools')
import yaml,numpy as np
from mcap.stream_reader import StreamReader,breakup_chunk
from mcap.records import Chunk,Schema,Channel,Message,Statistics,Footer,DataEnd,Header
from mcap_ros2.decoder import DecoderFactory
bag=Path('/home/jain/go2_bags/robot_20261018_055326');dest=Path(__file__).resolve().parent;metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];p=bag/metadata['relative_file_paths'][0];before=p.stat();record_counts=Counter();counts=Counter();decoded_counts=Counter();decode_error_counts=Counter();schemas={};channels={};errors=[];cloud_errors=[];sensor_nonfinite=Counter();frames=defaultdict(set);source_stamps=defaultdict(list);log_stamps=defaultdict(list);chunks=0;chunks_crc=0;data_crc=None;stats=None;footer=None;factory=DecoderFactory();decoders={};compressed=Counter();cloud_points=Counter();valid_cloud_points=Counter();last_progress=time.monotonic()

def ns(stamp):return int(stamp.sec)*10**9+int(stamp.nanosec)
def consume(rec):
 global stats,footer,data_crc
 record_counts[type(rec).__name__]+=1
 if isinstance(rec,Schema):schemas[rec.id]=rec
 elif isinstance(rec,Channel):channels[rec.id]=rec
 elif isinstance(rec,Statistics):stats=rec
 elif isinstance(rec,Footer):footer=rec
 elif isinstance(rec,DataEnd):data_crc=rec.data_section_crc
 elif isinstance(rec,Message):
  ch=channels[rec.channel_id];name=ch.topic;counts[name]+=1;log_stamps[name].append(rec.log_time)
  try:
   if rec.channel_id not in decoders:
    decoders[rec.channel_id]=factory.decoder_for(ch.message_encoding,schemas.get(ch.schema_id))
   decoder=decoders[rec.channel_id]
   if decoder is None:raise ValueError('Unsupported schema/message encoding')
   m=decoder(rec.data);decoded_counts[name]+=1
   if hasattr(m,'header') and hasattr(m.header,'frame_id') and hasattr(m.header,'stamp'):
    frames[name].add(m.header.frame_id);source_stamps[name].append(ns(m.header.stamp))
   elif name in ['/sportmodestate','/lf/sportmodestate']:source_stamps[name].append(ns(m.stamp))
   if ch.schema_id and schemas[ch.schema_id].name=='sensor_msgs/msg/PointCloud2':
    fields={v.name:v for v in m.fields};msgerrors=[]
    if len(m.data)!=m.row_step*m.height:msgerrors.append('Data length differs from row_step*height')
    if m.row_step<m.width*m.point_step:msgerrors.append('row_step smaller than width*point_step')
    if any(k not in fields for k in ['x','y','z']):msgerrors.append('Missing XYZ')
    if any(v.offset<0 or v.offset>=m.point_step for v in m.fields):msgerrors.append('Field offset outside point record')
    if msgerrors:cloud_errors.append({'topic':name,'sequence_index':counts[name],'errors':msgerrors})
    else:
     dtype=np.dtype({'names':['x','y','z'],'formats':[('>' if m.is_bigendian else '<')+'f4']*3,'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':m.point_step});arr=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1);v=np.column_stack([arr[k] for k in ['x','y','z']]);cloud_points[name]+=len(v);valid_cloud_points[name]+=int((np.isfinite(v).all(axis=1)&(np.linalg.norm(v,axis=1)>.001)).sum())
   if name in ['/sportmodestate','/lf/sportmodestate']:
    v=[*m.imu_state.accelerometer,*m.imu_state.gyroscope,*m.imu_state.quaternion];sensor_nonfinite[name]+=sum(not math.isfinite(x) for x in v)
   elif name=='/utlidar/imu':
    v=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w];sensor_nonfinite[name]+=sum(not math.isfinite(x) for x in v)
  except Exception as e:
   decode_error_counts[name]+=1
   if len(errors)<30:errors.append({'topic':name,'sequence_index':counts[name],'error':repr(e)})

with p.open('rb') as f:
 stream=StreamReader(f,emit_chunks=True,validate_crcs=True,record_size_limit=256*1024*1024)
 for rec in stream.records:
  if isinstance(rec,Chunk):
   chunks+=1;chunks_crc+=bool(rec.uncompressed_crc);compressed[rec.compression or 'none']+=1
   for inner in breakup_chunk(rec,validate_crc=True):consume(inner)
  else:consume(rec)
  if time.monotonic()-last_progress>10:
   print(json.dumps({'messages_read':sum(counts.values()),'chunks_checked':chunks}),flush=True);last_progress=time.monotonic()
 assert f.tell()==before.st_size,'Unread trailing bytes after MCAP closing magic'
assert footer is not None,'Missing footer'
# Independently verify the MCAP summary/footer checksum, including the footer prefix.
with p.open('rb') as f:
 footer_offset=before.st_size-37;f.seek(footer.summary_start or footer_offset);summary_and_footer_prefix=f.read(before.st_size-12-f.tell());calculated_summary_crc=zlib.crc32(summary_and_footer_prefix)
summary_crc_match=footer.summary_crc==0 or calculated_summary_crc==footer.summary_crc
expected={x['topic_metadata']['name']:x['message_count'] for x in metadata['topics_with_message_count']};count_mismatch={k:{'metadata':expected.get(k,0),'actual':counts.get(k,0)} for k in expected.keys()|counts.keys() if expected.get(k,0)!=counts.get(k,0)}
summary_counts=Counter()
for channel,count in stats.channel_message_counts.items():summary_counts[channels[channel].topic]+=count
actual_total=sum(counts.values());log_min=min(min(v) for v in log_stamps.values());log_max=max(max(v) for v in log_stamps.values());timing={}
for name in ['/utlidar/cloud','/sportmodestate','/utlidar/robot_odom','/utlidar/imu']:
 stamps=np.array(source_stamps[name],dtype=np.int64);dt=np.diff(stamps)/1e9;positive=dt[dt>0];timing[name]={'messages':len(stamps),'first_source_stamp_ns':int(stamps[0]) if len(stamps) else None,'last_source_stamp_ns':int(stamps[-1]) if len(stamps) else None,'backward_source_steps':int((dt<0).sum()),'duplicate_source_stamps':int((dt==0).sum()),'median_positive_dt_ms':float(np.median(positive)*1000) if len(positive) else None,'max_positive_dt_ms':float(positive.max()*1000) if len(positive) else None}
sha=hashlib.sha256()
with p.open('rb') as f:
 for block in iter(lambda:f.read(8*1024*1024),b''):sha.update(block)
after=p.stat();stable=(before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
checks={'source_file_unchanged_during_read':stable,'complete_header_footer_and_trailing_magic':True,'all_chunks_decompressed':True,'present_chunk_crcs_verified':True,'summary_crc_passed':summary_crc_match,'metadata_total_matches':actual_total==metadata['message_count'],'per_topic_metadata_counts_match':not count_mismatch,'summary_total_matches':actual_total==stats.message_count,'summary_channel_counts_match':dict(counts)==dict(summary_counts),'all_messages_decoded':sum(decoded_counts.values())==actual_total and not errors,'pointcloud_layout_checks_passed':not cloud_errors,'recorded_start_matches_metadata':log_min==metadata['starting_time']['nanoseconds_since_epoch'],'recorded_duration_matches_metadata':log_max-log_min==metadata['duration']['nanoseconds']}
report={'bag_directory':str(bag),'file':str(p),'bytes':before.st_size,'sha256':sha.hexdigest(),'ros_distro':metadata.get('ros_distro'),'metadata_version':metadata['version'],'storage':'mcap','checks':checks,'no_corruption_detected':all(checks.values()),'messages_read':actual_total,'messages_decoded':sum(decoded_counts.values()),'duration_s':(log_max-log_min)/1e9,'listed_topics':len(expected),'nonempty_topics':len(counts),'chunks':chunks,'chunks_with_crc':chunks_crc,'data_section_crc_present':bool(data_crc),'summary_crc_present':bool(footer.summary_crc),'compression':dict(compressed),'topic_counts':dict(sorted(counts.items())),'decoded_topic_counts':dict(sorted(decoded_counts.items())),'decode_error_counts':dict(decode_error_counts),'count_mismatches':count_mismatch,'decoding_errors':errors,'pointcloud_layout_errors':cloud_errors,'point_counts':dict(cloud_points),'nonzero_finite_point_counts':dict(valid_cloud_points),'nonfinite_selected_imu_fields':dict(sensor_nonfinite),'frames':{k:sorted(v) for k,v in frames.items()},'sensor_timestamp_checks':timing,'humble_rosbag_info':'Fails on Jazzy version-9 metadata/QoS representation; raw MCAP independently audited without modifying metadata','inspection_only_no_playback_or_publish':True,'libraries':{'mcap':'1.5.0','mcap-ros2-support':'0.5.7','installation':'Temporary /tmp/doggy_mcap_integrity_tools'}}
(dest/'integrity_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:report[k] for k in ['no_corruption_detected','checks','messages_read','messages_decoded','duration_s','listed_topics','nonempty_topics','chunks','chunks_with_crc','data_section_crc_present','summary_crc_present','compression','decoding_errors','count_mismatches']},indent=2))
