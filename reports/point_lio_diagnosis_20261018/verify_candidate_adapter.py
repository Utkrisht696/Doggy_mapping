from pathlib import Path
import sys,importlib.util,json,numpy as np,yaml
sys.path.insert(0,'/tmp/doggy_mcap_integrity_tools')
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
root=Path('/home/jain/Doggy');r=root/'reports/point_lio_diagnosis_20261018/clean_candidate';spec=importlib.util.spec_from_file_location('candidate_adapter',r/'scripts/publish_body_imu.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);cal=mod.load_calibration(r/'config/body_imu_calibration.json');converter=mod.BodyImuConverter(cal);source=Path('/home/jain/go2_bags/spinny');meta=yaml.safe_load((source/'metadata.yaml').read_text())['rosbag2_bagfile_information'];expected=np.load(root/'reports/point_lio_diagnosis_20261018/gain_inputs/spinny/source_decoded.npz');rows=[];stamps=[];duplicates=0
for fn in meta['relative_file_paths']:
 with (source/fn).open('rb') as f:
  reader=make_reader(f,decoder_factories=[DecoderFactory()])
  for schema,ch,msg,m in reader.iter_decoded_messages(topics=['/sportmodestate']):
   out=converter.convert(m)
   if out is None:duplicates+=1;continue
   assert out.header.frame_id=='imu';assert out.orientation_covariance[0]==-1.;assert list(out.angular_velocity_covariance)==[0.]*9 and list(out.linear_acceleration_covariance)==[0.]*9
   stamps.append(out.header.stamp.sec*10**9+out.header.stamp.nanosec);rows.append([out.linear_acceleration.x,out.linear_acceleration.y,out.linear_acceleration.z,out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z,out.orientation.x,out.orientation.y,out.orientation.z,out.orientation.w])
actual=np.array(rows);same=np.array_equal(actual,expected['imu_values']);stamp_same=np.array_equal(np.array(stamps,dtype=np.int64),expected['imu_stamp_ns']);assert same and stamp_same
invalid=[]
for value in [[1,1,float('nan')],[1,1,2.],[1,1],0]:
 d=dict(cal);d['gyro_scale_xyz']=value;p=r/'config/invalid_gain_check.json';p.write_text(json.dumps(d))
 try:mod.load_calibration(p)
 except ValueError:invalid.append(True)
 else:invalid.append(False)
p.unlink();assert all(invalid)
report={'recording':'spinny','source_messages':meta['message_count'],'converted_imu_samples':len(rows),'identical_duplicate_stamps_dropped':duplicates,'all_numeric_measurements_exactly_match_independent_replay_preparation':same,'all_source_timestamps_exactly_preserved':stamp_same,'frame_id':'imu','vendor_orientation_marked_unavailable':True,'gyro_and_acc_covariance_unknown_not_fabricated':True,'invalid_gain_profiles_rejected':all(invalid)};(r/'adapter_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
