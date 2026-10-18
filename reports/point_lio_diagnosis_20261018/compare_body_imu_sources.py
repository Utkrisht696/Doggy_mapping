from pathlib import Path
import sys,json,numpy as np,yaml
sys.path.insert(0,'/tmp/doggy_mcap_integrity_tools')
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
r=Path(sys.argv[1]);out=Path(sys.argv[2]);rows={'/lowstate':[],'/sportmodestate':[]};meta=yaml.safe_load((r/'metadata.yaml').read_text())['rosbag2_bagfile_information']
for fn in meta['relative_file_paths']:
 with (r/fn).open('rb') as f:
  reader=make_reader(f,decoder_factories=[DecoderFactory()])
  for schema,ch,msg,m in reader.iter_decoded_messages(topics=list(rows)):
   imu=m.imu_state
   tick=float(m.tick) if ch.topic=='/lowstate' else float(m.stamp.sec)+m.stamp.nanosec/1e9
   rows[ch.topic].append([msg.log_time,tick,*imu.gyroscope,*imu.accelerometer,*imu.quaternion])
a={};
for topic,vals in rows.items():
 k='low' if topic=='/lowstate' else 'sport';a[k+'_receipt_ns']=np.array([v[0] for v in vals],dtype=np.int64);a[k+'_values']=np.array([v[1:] for v in vals],dtype=float)
np.savez_compressed(out,**a)
l=a['low_values'];s=a['sport_values'];lt=(a['low_receipt_ns']-a['low_receipt_ns'][0])/1e9;st=(a['sport_receipt_ns']-a['low_receipt_ns'][0])/1e9;lg=np.column_stack([np.interp(st,lt,l[:,j]) for j in [1,2,3]]);sg=s[:,1:4];m=np.linalg.norm(sg,axis=1)>.2;coef=np.linalg.lstsq(np.column_stack([lg[m],np.ones(m.sum())]),sg[m],rcond=None)[0];report={'bag':str(r),'low_samples':len(l),'sport_samples':len(s),'low_tick_first_last':l[[0,-1],0].tolist(),'low_tick_delta_quantiles':np.percentile(np.diff(l[:,0]),[0,50,99,100]).tolist(),'low_vs_sport_gyro_affine_by_receipt':coef.tolist(),'residual_rms_rad_s':np.sqrt(np.mean((np.column_stack([lg[m],np.ones(m.sum())])@coef-sg[m])**2,axis=0)).tolist(),'limits':'Receipt-time interpolation only, not acquisition calibration; use to detect large source-value differences.'};out.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
