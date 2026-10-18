from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path('/home/jain/Doggy')
r = Path(__file__).resolve().parent
runs = {'control': r/'control/run_01', 'initialization_fix': r/'run_01'}
labels = {'control': 'Cache-fixed control', 'initialization_fix': 'IMU batch-clearing fix'}
cases = {}
for name, run in runs.items():
    report = json.loads((run/'analysis/comparison.json').read_text())
    heading = json.loads((run/'analysis/heading_comparison.json').read_text())
    cases[name] = {
        'return': report['first_to_last_analyzed_hold']['point_lio'],
        'vendor_return': report['first_to_last_analyzed_hold']['vendor'],
        'heading': heading,
        'health': report['health'],
        'delivery': {k: report['replay_result'][k] for k in ['cloud_delivery_exact', 'imu_delivery_exact']},
        'quiet_windows': [h['analysis_window_s'] for h in report['holds']],
        'replay_result': report['replay_result'],
    }
a, b = [np.load(runs[n]/'outputs.npz') for n in ['control', 'initialization_fix']]
st, i, j = np.intersect1d(a['odom_stamp_ns'], b['odom_stamp_ns'], return_indices=True)
error = np.linalg.norm(a['odom_values'][i,:3] - b['odom_values'][j,:3], axis=1)
angle = np.rad2deg((Rotation.from_quat(a['odom_values'][i,3:]).inv()*Rotation.from_quat(b['odom_values'][j,3:])).magnitude())
protected = json.loads((r/'protected_hashes_before.json').read_text())
after = {f: hashlib.sha256((root/f).read_bytes()).hexdigest() for f in protected}
audit = json.loads((r/'input_audit.json').read_text())
source_ok = all((Path(audit['source_bag'])/f['file']).stat().st_size == f['bytes'] and (Path(audit['source_bag'])/f['file']).stat().st_mtime_ns == f['mtime_ns'] for f in audit['source_files'])
summary = {
    'only_new_estimator_change': 'Both startup meas.imu.shrink_to_fit calls replaced with clear; cache resize retained in both cases',
    'cases': cases,
    'paired_pose_comparison': {'common_stamps': len(st), 'exact_pose_equality': bool(np.array_equal(a['odom_values'][i], b['odom_values'][j])), 'max_direct_position_difference_m': float(error.max()), 'max_direct_orientation_difference_deg': float(angle.max())},
    'protected_active_and_baseline_files_unchanged': protected == after,
    'protected_hashes_after': after,
    'original_bag_files_unchanged': source_ok,
    'limits': ['Physical return is user reported, not surveyed.', 'Body quaternion and integrated gyro share the same body IMU; they are diagnostic comparisons, not independent ground truth.', 'Zero acquisition-lag correction remains uncalibrated.', 'Saved cache-fixed control is used; its full pose array was exactly reproduced in three earlier baseline/control runs.'],
}
(r/'comparison.json').write_text(json.dumps(summary, indent=2)+'\n')
fig, axs = plt.subplots(2, 2, figsize=(13,9), layout='constrained')
fig.suptitle('Soft-turn bag: isolated IMU startup-batch correction')
for name, run in runs.items():
    data = np.load(run/'analysis/comparison.npz')
    heading = np.load(run/'analysis/heading_comparison.npz')
    p, t = data['lio_position_initial_heading'], data['time']
    label = labels[name]
    axs[0,0].plot(p[:,0], p[:,1], label=label)
    axs[0,1].plot(t, p[:,2]*100, label=label)
    axs[1,0].plot(heading['time'], heading['Point_LIO'], label=label)
    axs[1,1].plot(heading['time'], heading['Point_LIO']-heading['Body_orientation'], label=label)
    if name == 'control':
        vp = data['vendor_position_initial_heading']
        axs[0,0].plot(vp[:,0], vp[:,1], '--', color='gray', label='Vendor comparison')
        axs[1,0].plot(heading['time'], heading['Body_orientation'], '--', color='gray', label='Body orientation comparison')
        axs[1,0].plot(heading['time'], heading['Integrated_calibrated_gyro'], ':', color='green', label='Integrated body gyro')
axs[0,0].scatter(0,0,marker='*',c='black',s=80)
axs[0,0].set(xlabel='Initial heading X (m)', ylabel='Y (m)', title='Full trajectory; initial alignment only')
axs[0,0].set_aspect('equal', adjustable='datalim')
axs[0,1].set(xlabel='Source time (s)', ylabel='Height relative to start (cm)', title='Vertical estimate')
axs[1,0].set(xlabel='Source time (s)', ylabel='Unwrapped heading change (degrees)', title='Heading throughout the route')
axs[1,1].set(xlabel='Source time (s)', ylabel='Heading disagreement (degrees)', title='Point-LIO minus body orientation; shared IMU')
for ax in axs.flat:
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
fig.savefig(r/'comparison.png', dpi=150)
plt.close(fig)
print(json.dumps({k: {'return': v['return'], 'heading_disagreement': v['heading']['disagreement_vs_body_orientation_during_analyzed_route']['Point-LIO'], 'health': v['health'], 'delivery': v['delivery']} for k,v in cases.items()}, indent=2))
print('Protected files unchanged:', protected == after, 'Original bags unchanged:', source_ok)
