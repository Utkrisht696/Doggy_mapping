from pathlib import Path
import sys,json,sqlite3,numpy as np
from scipy.spatial.transform import Rotation
import rosbag2_py
r=Path(sys.argv[1]);a=np.load(r/'source_decoded.npz');origin=int(a['raw_stamp_ns'][0]);it=(a['imu_stamp_ns']-origin)/1e9;vt=(a['vendor_stamp_ns']-origin)/1e9;v=a['vendor_values'];im=a['imu_values'];quiet=None
for start in np.arange(0,min(vt[-1],it[-1])-2,.1):
 mask=(vt>=start)&(vt<start+2);mi=(it>=start)&(it<start+2)
 if mask.sum()<100 or mi.sum()<200:continue
 pos=v[mask,:3];q=Rotation.from_quat(v[mask,3:7]);exc=float(np.linalg.norm(pos-pos[0],axis=1).max());angle=float(np.rad2deg((q[0].inv()*q).magnitude()).max());gyro=float(np.sqrt(np.mean(np.sum(im[mi,3:6]**2,axis=1))))
 if exc<.04 and angle<3 and gyro<.08:quiet={'start_source_s':float(start),'duration_s':2.,'position_excursion_m':exc,'orientation_excursion_deg':angle,'body_gyro_rms_rad_s':gyro};break
if quiet is None:raise RuntimeError('No quiet startup found; inspect before replay')
c=sqlite3.connect('file:'+str(next((r/'replay_inputs').glob('*.db3')))+'?mode=ro',uri=True);first=int(c.execute('select min(timestamp) from messages').fetchone()[0]);cut=first if quiet['start_source_s']==0 else int(a['raw_receipt_ns'][np.searchsorted((a['raw_stamp_ns']-origin)/1e9,quiet['start_source_s'])]);out=r/'replay_stationary_start'
assert not out.exists()
if cut==first:out.symlink_to('replay_inputs',target_is_directory=True)
else:
 w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(out),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'));topics={}
 for identifier,name,typ,serialization in c.execute('select id,name,type,serialization_format from topics'):
  topics[identifier]=name;w.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format=serialization))
 static=c.execute("select data from messages where topic_id=(select id from topics where name='/tf_static') limit 1").fetchone()[0];w.write('/tf_static',static,cut)
 for identifier,stamp,data in c.execute("select topic_id,timestamp,data from messages where timestamp>=? and topic_id!=(select id from topics where name='/tf_static') order by timestamp,id",(cut,)):w.write(topics[identifier],data,stamp)
 del w
c.close();np.savez_compressed(r/'expected_replay_stamps.npz',**{k+'_stamp_ns':a[k+'_stamp_ns'][a[k+'_receipt_ns']>=cut] for k in ['raw','imu']});report={'omitted_initial_receipt_time_s':(cut-first)/1e9,'quiet_startup_check':quiet,'receipt_cut_ns':cut,'source_origin_ns':origin,'source_header_times_changed':False,'gaps_filled':False};(r/'stationary_start.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
