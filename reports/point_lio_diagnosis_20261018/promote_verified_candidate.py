"""Install the independently checked candidate, saving reversible originals."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path('/home/jain/Doggy')
report = Path(__file__).resolve().parent
candidate = report/'clean_candidate'
before = json.loads((report/'final_validation/soft/protected_hashes_before.json').read_text())
files = ['third_party/point_lio_ros2/src/laserMapping.cpp', 'scripts/publish_body_imu.py',
         'config/point_lio/body_imu_calibration.json', 'config/point_lio/go2_body_imu.yaml']
preserved = ['config/point_lio/sensor_geometry.json', 'scripts/publish_point_lio_tf.py',
             'launch/point_lio_go2.launch.py']
for name in files + preserved:
    assert hashlib.sha256((root/name).read_bytes()).hexdigest() == before[name], name
for case in ['soft', 'small', 'spinny', 'measured_route_2', 'running_regression']:
    result = json.loads((report/'final_validation'/case/'run_01/result.json').read_text())
    assert result['stop_reason'] == 'playback_complete', case
    assert result['cloud_delivery_exact'] and result['imu_delivery_exact'], case
    assert result['application_unchanged'] and not result['robot_commands_sent'], case
    assert not result['multiple_tf_parents'], case

deployment = report/'deployment'
deployment.mkdir(exist_ok=False)
for name in files:
    dest = deployment/'original'/name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(root/name, dest)
shutil.copy2(candidate/'src/point_lio/src/laserMapping.cpp', root/files[0])
shutil.copy2(candidate/'scripts/publish_body_imu.py', root/files[1])
shutil.copy2(candidate/'config/body_imu_calibration.json', root/files[2])
p = root/files[3]
s = p.read_text()
header_end = s.index('/**:')
s = '''# Go2 body-IMU profile: independently validated gyro gain and fixed residual bias.
# Offline evidence: reports/point_lio_diagnosis_20261018/FINAL_REPORT.md.
# L1 model identification remains provisional. Acceleration/noise/saturation
# settings otherwise retain upstream values; this is not a full noise calibration.
# imu_time_inte is nominal; original source/per-point timestamps are preserved.
# time_lag_imu_to_lidar=0.0 remains an uncalibrated acquisition-time correction.
''' + s[header_end:]
for old, new in [
    ('      b_gyr_cov: 0.0001', '      b_gyr_cov: 0.0\n      gyro_bias_initial_variance: 0.0'),
    ('      imu_meas_omg_cov: 0.1', '      imu_meas_omg_cov: 0.0001'),
    ('      publish_odometry_without_downsample: true', '      publish_odometry_without_downsample: false'),
    ('    init_map_size: 10', '    init_map_size: 4000')]:
    assert s.count(old) == 1, old
    s = s.replace(old, new)
p.write_text(s)
patch = root/'reports/patches/point_lio_heading_20261018.patch'
patch.parent.mkdir(exist_ok=True)
shutil.copy2(candidate/'estimator.patch', patch)
after = {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in files + preserved}
assert all(after[name] == before[name] for name in preserved)
(deployment/'source_changes.json').write_text(json.dumps({
    'authorization': 'User: do everything you can until you fix the issue',
    'before': {name: before[name] for name in files + preserved}, 'after': after,
    'candidate_estimator_exact_match': (root/files[0]).read_bytes() == (candidate/'src/point_lio/src/laserMapping.cpp').read_bytes(),
    'robot_commands_sent': False, 'live_processes_restarted': False,
    'originals': str(deployment/'original')}, indent=2)+'\n')
print('Installed validated source, calibration and profile; original files saved in', deployment/'original')
