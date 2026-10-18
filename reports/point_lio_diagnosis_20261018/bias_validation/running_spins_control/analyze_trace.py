from pathlib import Path
import json
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

r = Path(__file__).resolve().parent
root = Path('/home/jain/Doggy')
run = r/'run_01'
out = run/'analysis'
out.mkdir(exist_ok=True)
trace_path = run/'heading_trace.csv'
c = np.genfromtxt(trace_path, delimiter=',', names=True)
a = np.load(run/'outputs.npz')
b = a
s = np.load(r/'source_decoded.npz')
origin = int(s['raw_stamp_ns'][0])
t = (a['odom_stamp_ns']-origin)/1e9
assert len(c) == len(t), (len(c),len(t))
assert np.isfinite(np.column_stack([c[n] for n in c.dtype.names])).all()
csv_q = np.column_stack([c[n] for n in ['qx','qy','qz','qw']])
csv_p = np.column_stack([c[n] for n in ['px','py','pz']])
quat_error = np.rad2deg((Rotation.from_quat(csv_q).inv()*Rotation.from_quat(a['odom_values'][:,3:])).magnitude())
assert quat_error.max() < 1e-6
assert np.max(np.abs(csv_p-a['odom_values'][:,:3])) < 1e-6
st,i,j = np.intersect1d(a['odom_stamp_ns'],b['odom_stamp_ns'],return_indices=True)
pose_exact = np.array_equal(a['odom_values'][i],b['odom_values'][j]) and len(a['odom_stamp_ns']) == len(b['odom_stamp_ns'])
pose_diff = np.linalg.norm(a['odom_values'][i,:3]-b['odom_values'][j,:3],axis=1)
orientation_diff = np.rad2deg((Rotation.from_quat(a['odom_values'][i,3:]).inv()*Rotation.from_quat(b['odom_values'][j,3:])).magnitude())
holds = json.loads((run/'analysis/comparison.json').read_text())['holds']
wins = {'0':holds[0]['analysis_window_s'],'1':holds[-1]['analysis_window_s']}
m0 = (t>=wins['0'][0])&(t<=wins['0'][1])
m1 = (t>=wins['1'][0])&(t<=wins['1'][1])
yaw = np.rad2deg(np.unwrap(c['yaw_rad']))
yaw -= np.median(yaw[m0])
it = (s['imu_stamp_ns']-origin)/1e9
body_r = Slerp(it,Rotation.from_quat(s['imu_values'][:,6:10]))(t)
body_yaw = np.rad2deg(np.unwrap(body_r.as_euler('xyz')[:,2]))
body_yaw -= np.median(body_yaw[m0])
phase = {}
for n in ['prediction','imu','lidar']:
    v = np.rad2deg(np.cumsum(c['dyaw_'+n+'_rad']))
    phase[n] = v - np.median(v[m0])
delta_sum = c['dyaw_prediction_rad']+c['dyaw_imu_rad']+c['dyaw_lidar_rad']
closure = np.arctan2(np.sin(delta_sum-c['dyaw_total_rad']),np.cos(delta_sum-c['dyaw_total_rad']))
sum_yaw = sum(phase.values())
sum_yaw -= np.median(sum_yaw[m0])
csv_time_offset = (a['odom_stamp_ns']/1e9)-c['scan_stamp_s']
selected_times = [6,20,30,40,52.5,56,70,90,110,130,140]
snapshots = []
for target in selected_times:
    k = int(np.argmin(np.abs(t-target)))
    snapshots.append({'source_s':float(t[k]),'point_lio_heading_deg':float(yaw[k]),'body_heading_deg':float(body_yaw[k]),'heading_disagreement_deg':float(yaw[k]-body_yaw[k]),'cumulative_phase_heading_deg':{n:float(v[k]) for n,v in phase.items()},'gyro_bias_rad_s':[float(c[n][k]) for n in ['bgx','bgy','bgz']],'tilt_deg':[float(np.rad2deg(c[n][k])) for n in ['roll_rad','pitch_rad']]})
protected = json.loads((r/'protected_hashes_before.json').read_text())
protected_ok = all(hashlib.sha256((root/f).read_bytes()).hexdigest()==v for f,v in protected.items())
audit = json.loads((r/'input_audit.json').read_text())
source_ok = all((Path(audit['source_bag'])/f['file']).stat().st_size == f['bytes'] and (Path(audit['source_bag'])/f['file']).stat().st_mtime_ns == f['mtime_ns'] for f in audit['source_files'])
report = {
    'trace_rows':len(c),
    'trial_vs_control':{'common_stamps':len(st),'all_pose_values_exactly_equal':bool(pose_exact),'max_position_difference_m':float(pose_diff.max()),'max_orientation_difference_deg':float(orientation_diff.max())},
    'accounting':{'max_per_scan_closure_error_deg':float(np.rad2deg(np.abs(closure).max())),'max_phase_sum_minus_heading_deg':float(np.max(np.abs(sum_yaw-yaw))),'max_csv_vs_published_orientation_difference_deg':float(quat_error.max()),'min_output_minus_scan_stamp_s':float(csv_time_offset.min()),'max_output_minus_scan_stamp_s':float(csv_time_offset.max())},
    'phase_counts':{n:int(c[n+'_count'].sum()) for n in ['prediction','imu','lidar']},
    'lidar_successful_updates':int(c['lidar_success'].sum()),
    'prediction_dt':{'negative_steps':int(c['negative_prediction_dt'].sum()),'scans_with_negative_dt':int((c['negative_prediction_dt']>0).sum()),'min_s':float(c['min_prediction_dt_s'].min()),'max_s':float(c['max_prediction_dt_s'].max()),'interpretation':'Negative dt is audited, not automatically classified as erroneous: this implementation backtracks between point-time and IMU-time predictions.'},
    'final_hold_heading_change_deg':{'point_lio':float(np.median(yaw[m1])),'body_orientation':float(np.median(body_yaw[m1])),'phase_contributions':{n:float(np.median(v[m1])) for n,v in phase.items()}},
    'gyro_bias_range_rad_s':{n:[float(c[n].min()),float(c[n].max())] for n in ['bgx','bgy','bgz']},
    'snapshots':snapshots,
    'protected_active_and_baseline_files_unchanged':protected_ok,
    'source_bag_files_unchanged':source_ok,
    'limits':['Heading components are sums of local Euler yaw changes for each filter operation, not independent physical rotations. They identify algorithmic changes without establishing which correction is physically correct.','Body orientation shares the same body IMU. No independent surveyed yaw reference.','Only final per-scan state is retained; transient bias/state changes within each scan are not captured.','Pre-update innovation values are RMS vector norms aggregated per scan, not per-axis innovations.','This private trial changes only gyro measurement variance from 0.1 to 0.001; no active source/config changes are included.'],
}
(out/'trace_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
np.savez_compressed(out/'trace_analysis.npz',time=t,heading=yaw,body_heading=body_yaw,**phase)
fig,axs = plt.subplots(3,2,figsize=(14,12),layout='constrained')
fig.suptitle('Soft-turn heading trace: prediction / IMU updates / LiDAR updates')
axs[0,0].plot(t,yaw,label='Point-LIO')
axs[0,0].plot(t,body_yaw,'--',label='Body orientation comparison')
axs[0,0].set(ylabel='Heading change (degrees)',title='Initial hold alignment only')
for n,v in phase.items(): axs[0,1].plot(t,v,label=n)
axs[0,1].set(ylabel='Cumulative yaw contribution (degrees)',title='Sum of each operation’s yaw changes')
for n in ['imu','lidar']: axs[1,0].plot(t,phase[n],label=n)
axs[1,0].plot(t,phase['prediction']-body_yaw,label='prediction minus body heading')
axs[1,0].plot(t,yaw-body_yaw,'--',label='total minus body heading')
axs[1,0].set(ylabel='Heading contribution / difference (degrees)',title='Where divergence enters the state')
for n in ['bgx','bgy','bgz']: axs[1,1].plot(t,np.rad2deg(c[n]),label=n)
axs[1,1].set(ylabel='Estimated gyro bias (degrees/s)',title='Bias estimated after adapter correction')
for n in ['roll_rad','pitch_rad']: axs[2,0].plot(t,np.rad2deg(c[n]),label=n)
axs[2,0].set(ylabel='Tilt (degrees)',title='Estimated roll and pitch')
axs[2,1].plot(t,c['gyro_innovation_rms_vector'],label='gyro RMS norm (rad/s)')
axs[2,1].plot(t,c['acc_innovation_rms_vector'],label='acceleration RMS norm (m/s²)')
axs[2,1].set(title='Pre-update innovations; different units')
for ax in axs.flat:
    ax.set_xlabel('Source time (s)')
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
fig.savefig(out/'trace_analysis.png',dpi=150)
plt.close(fig)
print(json.dumps(report,indent=2))
