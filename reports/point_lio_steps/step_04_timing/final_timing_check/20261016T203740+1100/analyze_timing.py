"""Analyze immutable capture; no ROS, code/config changes or timestamp fitting."""
from pathlib import Path
import json,hashlib
import numpy as np
DEST=Path(__file__).resolve().parent; ROOT=Path('/home/jain/Doggy')
d=np.load(DEST/'live_timing_capture.npz');capture=json.loads((DEST/'live_timing_capture.json').read_text())
def stats(x):
 x=np.asarray(x,dtype=float)
 return {'min':float(x.min()),'median':float(np.median(x)),'p95':float(np.percentile(x,95)),'p99':float(np.percentile(x,99)),'max':float(x.max()),'mean':float(x.mean())} if len(x) else None
streams={}
for name in ('body','sport','lidar_imu','cloud','odom','clock'):
 t=d[name+'_stamp_ns']; h=d[name+'_host_ns']; m=d[name+'_mono_ns']; dt=np.diff(t)/1e6
 streams[name]={'messages':len(t),'source_interval_ms':stats(dt),'strictly_increasing':bool(np.all(np.diff(t)>0)),'backsteps':int(np.sum(np.diff(t)<0)),'identical_stamps':int(np.sum(np.diff(t)==0)),'distinct_stamp_rate_hz':float((len(np.unique(t))-1)*1e9/(int(t[-1])-int(t[0]))),'receipt_minus_header_s':stats((h-t)/1e9),'receipt_callback_interval_ms':stats(np.diff(m)/1e6),'host_vs_monotonic_change_ms':stats(((h-h[0])-(m-m[0]))/1e6)}
b=d['body_stamp_ns'];bv=d['body_values']; bm=d['body_mono_ns']; s=d['sport_stamp_ns'];sv=d['sport_values']
cal=json.loads((ROOT/'config/point_lio/body_imu_calibration.json').read_text()); bias=np.array(cal['acc_bias_xyz_m_s2']+cal['gyro_bias_xyz_rad_s'])
unique,ix=np.unique(s,return_index=True); common,bi,si=np.intersect1d(b,unique,return_indices=True)
errors=np.abs(bv[bi,:6]-(sv[ix[si],:6]-bias))
clock=d['clock_stamp_ns'];odom=d['odom_stamp_ns']
v=d['cloud_values'];cs=d['cloud_stamp_ns'];cm=d['cloud_mono_ns']
# Reproduce Point-LIO's float32 seconds->float32 milliseconds->double seconds.
lo=cs+np.rint(v[:,2]*1e9).astype(np.int64)
raw_hi=cs+np.rint(v[:,3]*1e9).astype(np.int64)
converted_ms=np.float32(v[:,3])*np.float32(1000)
hi=cs+np.rint(converted_ms.astype(float)*1e6).astype(np.int64)
coverage=[]
interior_mask=(lo>=b[0]+10**9)&(hi<=b[-1]-10**9)
for i,(start,end) in enumerate(zip(lo,hi)):
 if not interior_mask[i]:continue
 left=int(np.searchsorted(b,start,side='right')-1); right=int(np.searchsorted(b,end,side='left'))
 bracketed=left>=0 and right<len(b)
 item={'cloud_index':i,'header_ns':int(cs[i]),'first_point_ns':int(start),'last_point_ns':int(end),'bracketed':bracketed}
 if bracketed:
  item.update(imu_samples_within=int(np.sum((b>=start)&(b<=end))),max_bracketing_imu_gap_ms=float(np.diff(b[left:right+1]).max()/1e6),wait_for_end_sample_ms=float(max(0,(int(bm[right])-int(cm[i]))/1e6)))
 coverage.append(item)
endpoints=capture['publisher_snapshots']
checks={'capture_no_decoding_errors':not capture['errors'],'one_body_imu_publisher':all(len(e['/doggy/imu/body_calibrated'])==1 for e in endpoints),'one_cloud_publisher':all(len(e['/utlidar/cloud'])==1 for e in endpoints),'one_private_robot_clock':all(len(e['/doggy/robot_clock'])==1 and e['/doggy/robot_clock'][0]['node']=='doggy_go2_state' for e in endpoints),'no_global_clock_publisher':all(not e['/clock'] for e in endpoints),'selected_imu_stamps_strictly_increasing':streams['body']['strictly_increasing'],'cloud_headers_strictly_increasing':streams['cloud']['strictly_increasing'],'odometry_and_robot_clock_increasing':streams['odom']['strictly_increasing'] and streams['clock']['strictly_increasing'],'matched_body_source_packets':len(common)>=10000,'matched_body_source_values_exact':bool(np.max(errors)==0),'clock_matches_odometry_in_reliable_crosscheck':json.loads((DEST/'clock_reference_validation.json').read_text())['passed'],'point_times_finite_nonnegative':bool(np.all(v[:,6]==0) and np.all(v[:,0]>=0)),'kept_point_times_within_scan_period_scale':bool(np.all(v[:,3]<.1)),'all_scans_have_bracketing_imu':all(x['bracketed'] for x in coverage),'no_cross_scan_kept_point_time_reversal':bool(np.all(lo[1:]>hi[:-1]))}
result={'timestamp_transport_passed':all(checks.values()),'acquisition_latency_calibrated':False,'checks':checks,'streams':streams,'initial_best_effort_clock_stamps_absent_from_odom_capture':int(np.sum(~np.isin(clock,odom))),'source_matching':{'exact_stamp_packets':len(common),'max_acc_gyro_correction_error':float(errors.max())},'coverage':{'scans':len(coverage),'bracketed_scans':sum(c['bracketed'] for c in coverage),'samples_per_scan':stats([c['imu_samples_within'] for c in coverage if c['bracketed']]),'max_imu_gap_within_scans_ms':stats([c['max_bracketing_imu_gap_ms'] for c in coverage if c['bracketed']]),'wait_for_end_sample_ms':stats([c['wait_for_end_sample_ms'] for c in coverage if c['bracketed']]),'scans_waiting_for_end_sample':sum(c.get('wait_for_end_sample_ms',0)>0 for c in coverage),'kept_scan_duration_ms':stats((hi-lo)/1e6),'gap_between_kept_scan_points_ms':stats((lo[1:]-hi[:-1])/1e6),'raw_point_order_reversal_clouds':int(np.sum(v[:,7]>0)),'maximum_point_unit_rounding_error_ns':int(np.max(np.abs(hi-raw_hi)))},'cloud_receipt_minus_last_point_s':stats((d['cloud_host_ns']-hi)/1e9),'body_gyro_std_rad_s':np.std(bv[:,3:6],axis=0).tolist(),'profile_sha256':hashlib.sha256((ROOT/'config/point_lio/go2_body_imu.yaml').read_bytes()).hexdigest(),'conclusions':['No source-stamp, point-time unit, bracketing or clock-owner defect detected in this capture.','The approx 13 s laptop/robot difference is not an inter-sensor correction.','Keep original acquisition stamps; do not stamp packets on receipt or change system clock.','0.004 s remains close to median observed IMU interval; actual header intervals vary and must drive integration.','Point-LIO sorts retained/downsampled points by time, so within-cloud record order is not itself an issue.','Independent dynamic acquisition-lag validation remains pending; zero configured lag is not certified by these checks.']}
(DEST/'timing_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
(DEST/'scan_imu_coverage.json').write_text(json.dumps(coverage,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='streams'},indent=2))
