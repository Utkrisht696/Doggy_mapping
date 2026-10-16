"""Offline timing/integrity checks of the saved repeat posture recording."""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[3];base=ROOT/'reports/point_lio_steps/step_04_timing';info=json.loads((base/'current_posture_recording.json').read_text());DEST=ROOT/info['directory'];d=np.load(DEST/'decoded_samples.npz');ref=int(d['body_stamp_ns'][0]);b=d['body_stamp_ns'];s=d['sport_stamp_ns'];cs=d['cloud_stamp_ns'];v=d['cloud_values'];lo=cs+np.rint(v[:,0]*1e9).astype(np.int64);hi=cs+np.rint((np.float32(v[:,1])*np.float32(1000)).astype(float)*1e6).astype(np.int64)
# Explicitly exclude recording start/end discovery boundaries for integrity rates.
interior=(lo>=b[0]+10**9)&(hi<=b[-1]-10**9);motion=(lo>=ref+90*10**9)&(hi<=ref+165*10**9)
coverage=[]
for i in np.flatnonzero(interior):
 l=int(np.searchsorted(b,lo[i],side='right')-1);r=int(np.searchsorted(b,hi[i],side='left'));covered=l>=0 and r<len(b)
 coverage.append({'cloud_index':int(i),'motion_window':bool(motion[i]),'covered':covered,'body_samples_within':int(max(0,r-l-1)),'max_bracketing_gap_ms':float(np.max(np.diff(b[l:r+1]))/1e6) if covered else None})
common,bi,si=np.intersect1d(b,s,return_indices=True);cal=json.loads((ROOT/'config/point_lio/body_imu_calibration.json').read_text());bias=np.array(cal['acc_bias_xyz_m_s2']+cal['gyro_bias_xyz_rad_s']);error=float(np.max(np.abs(d['body_values'][bi,:6]-(d['sport_values'][si,:6]-bias))))
# Distinguish gaps visible in both streams from source packets recorded without
# a corresponding calibrated output. Missing data is never interpolated/retimed.
gaps=[]
for j in np.flatnonzero(np.diff(b)>12_000_000):
 if not ref+85*10**9<=b[j]<=ref+170*10**9:continue
 gaps.append({'time_source_s':float((b[j]-ref)/1e9),'gap_ms':float((b[j+1]-b[j])/1e6),'recorded_sport_packets_inside':int(np.sum((s>b[j])&(s<b[j+1])))})
clock=d['clock_stamp_ns'];odom=d['odom_stamp_ns'];c=clock[(clock>max(clock[0],odom[0])+10**9)&(clock<min(clock[-1],odom[-1])-10**9)]
checks={'bag_saved_cleanly':info['status']=='stopped_saved' and info['metadata_message_count_matches'],'no_decoding_errors':not json.loads((DEST/'decoded_summary.json').read_text())['errors'],'selected_imu_strictly_increasing':bool(np.all(np.diff(b)>0)),'raw_cloud_headers_strictly_increasing':bool(np.all(np.diff(cs)>0)),'point_times_finite_nonnegative':bool(np.isfinite(v[:,0:2]).all() and np.all(v[:,0]>=0) and np.all(v[:,4]==0)),'every_interior_cloud_has_bracketing_imu':all(x['covered'] for x in coverage),'source_stamp_and_fixed_bias_values_preserved':len(common)>50000 and error==0,'all_clock_stamps_have_source_or_both_body_tf_evidence':json.loads((DEST/'clock_capture_crosscheck.json').read_text())['all_clock_stamps_have_source_or_both_body_tf_evidence']}
result={'input_timing_integrity_passed':all(checks.values()),'checks':checks,'recording_duration_s':(info['stored_receipt_end_ns']-info['stored_receipt_start_ns'])/1e9,'interior_scans':len(coverage),'covered_scans':sum(x['covered'] for x in coverage),'motion_window_scans':sum(x['motion_window'] for x in coverage),'matched_body_source_packets':len(common),'maximum_matched_value_error':error,'max_body_imu_gap_ms':float(np.max(np.diff(b))/1e6),'motion_gap_count_over_12_ms':len(gaps),'motion_gaps_also_present_in_sport_capture':sum(g['recorded_sport_packets_inside']==0 for g in gaps),'motion_gaps':gaps,'clock_stamps_checked':len(c),'clock_stamps_absent_from_source_recording':int(np.sum(~np.isin(c,odom))),'lidar_status_error_codes':np.unique(d['status_values'][:,-1]).tolist(),'reported_lidar_packet_loss_max':{'cloud':float(np.max(d['status_values'][:,1])),'imu':float(np.max(d['status_values'][:,3]))},'profile_sha256':hashlib.sha256((ROOT/'config/point_lio/go2_body_imu.yaml').read_bytes()).hexdigest(),'notes':['Bracketing is a coverage check, not a no-drop guarantee.','A gap in both captures may originate in robot publishing or common transport; it does not prove a hardware sampling gap.','A timestamp offset cannot recover missing samples.','Inter-sensor acquisition lag is considered separately from input transport/integrity.']}
(DEST/'input_timing_integrity.json').write_text(json.dumps(result,indent=2)+'\n');(DEST/'scan_coverage.json').write_text(json.dumps(coverage,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='motion_gaps'},indent=2))
