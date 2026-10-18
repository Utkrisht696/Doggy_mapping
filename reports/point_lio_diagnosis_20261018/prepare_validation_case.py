from pathlib import Path
import argparse,shutil,json
p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--out',required=True);p.add_argument('--control');p.add_argument('--gyro-variance',type=float,default=.001);p.add_argument('--lock',action='store_true');a=p.parse_args();root=Path('/home/jain/Doggy');base=root/'reports/point_lio_diagnosis_20261018/bias_lock_soft_turn';r=Path(a.out).resolve();r.mkdir(parents=True,exist_ok=False);inp=Path(a.input).resolve()
for n in ['run_replay.py','analyze_heading.py']:
 shutil.copy2(base/n,r/n)
s=(base/'offline.launch.py').read_text().replace(str(base/'gyro_weight_override.yaml'),str(r/'gyro_weight_override.yaml')).replace("'diagnostics.hold_residual_gyro_bias_zero': True", "'diagnostics.hold_residual_gyro_bias_zero': "+str(a.lock));(r/'offline.launch.py').write_text(s)
(r/'gyro_weight_override.yaml').write_text('/**:\n  ros__parameters:\n    mapping:\n      imu_meas_omg_cov: '+str(a.gyro_variance)+'\n')
s=(r/'run_replay.py').read_text().replace("'mapping.imu_meas_omg_cov':0.001,'diagnostics.hold_residual_gyro_bias_zero':True", "'mapping.imu_meas_omg_cov':"+str(a.gyro_variance)+",'diagnostics.hold_residual_gyro_bias_zero':"+str(a.lock));(r/'run_replay.py').write_text(s)
for n in ['source_decoded.npz','candidate_holds.json','input_audit.json','source_timing.json','stationary_start.json','expected_replay_stamps.npz']:(r/n).symlink_to(inp/n)
shutil.copy2(root/'reports/bag_comparisons/wander_comeback_20261018/analyze_wander.py',r/'analyze_trial.py')
shutil.copy2(base/'protected_hashes_before.json',r/'protected_hashes_before.json')
if a.control:
 (r/'control').mkdir();(r/'control/run_01').symlink_to(Path(a.control).resolve(),target_is_directory=True)
(r/'scope.json').write_text(json.dumps({'input':str(inp),'gyro_measurement_variance':a.gyro_variance,'hold_residual_gyro_bias_zero':a.lock,'binary':str(base/'install/point_lio/lib/point_lio/pointlio_mapping'),'archive_trace_from':str(base/'src/point_lio/Log/heading_trace.csv'),'no_robot_commands':True,'no_per_bag_tuning':True},indent=2)+'\n')
print(r)
