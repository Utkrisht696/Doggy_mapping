"""Exclude initial body motion; preserve all later data and gaps, no stamp edits."""
from pathlib import Path
import sqlite3,json,numpy as np
import rosbag2_py
r=Path(__file__).resolve().parent;db=next((r/'replay_inputs').glob('*.db3'));c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);first=int(c.execute('select min(timestamp) from messages').fetchone()[0]);cut=first+1_500_000_000;out=r/'replay_stationary_start'
if out.exists():raise RuntimeError('Refusing overwrite')
w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(out),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'));topics={}
for identifier,name,typ,serialization in c.execute('select id,name,type,serialization_format from topics'):
 topics[identifier]=name;w.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format=serialization))
static=c.execute("select data from messages where topic_id=(select id from topics where name='/tf_static') limit 1").fetchone()[0];w.write('/tf_static',static,cut);count={'/tf_static':1}
for identifier,stamp,data in c.execute("select topic_id,timestamp,data from messages where timestamp>=? and topic_id!=(select id from topics where name='/tf_static') order by timestamp,id",(cut,)):
 name=topics[identifier];w.write(name,data,stamp);count[name]=count.get(name,0)+1
del w;c.close();a=np.load(r/'source_decoded.npz');origin=int(a['raw_stamp_ns'][0]);expected={}
for key in ['raw','imu']:
 mask=a[key+'_receipt_ns']>=cut;expected[key+'_stamp_ns']=a[key+'_stamp_ns'][mask]
np.savez_compressed(r/'expected_replay_stamps.npz',**expected);report={'omitted_initial_receipt_time_s':1.5,'receipt_cut_ns':cut,'source_origin_ns':origin,'first_raw_source_s':float((expected['raw_stamp_ns'][0]-origin)/1e9),'first_imu_source_s':float((expected['imu_stamp_ns'][0]-origin)/1e9),'counts':count,'source_header_times_changed':False,'gaps_filled':False,'static_sensor_tf_reissued_at_replay_start':True};(r/'stationary_start.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
