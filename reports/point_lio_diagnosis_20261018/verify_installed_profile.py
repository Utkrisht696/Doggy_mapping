"""Inspect the real installed mapper/launch in an empty localhost ROS domain."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rcl_interfaces.srv import GetParameters

root = Path('/home/jain/Doggy')
out = root/'reports/point_lio_diagnosis_20261018/deployment'
assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
assert os.environ['ROS_DOMAIN_ID'] == '170'
expected = {'mapping.imu_meas_omg_cov': .0001,
            'mapping.gyro_bias_initial_variance': 0., 'mapping.b_gyr_cov': 0.,
            'init_map_size': 4000, 'use_imu_as_input': False,
            'odometry.publish_odometry_without_downsample': False,
            'common.time_lag_imu_to_lidar': 0.}
log = (out/'installed_launch.log').open('w')
process = subprocess.Popen(['ros2', 'launch', str(root/'launch/point_lio_go2.launch.py'),
                            'start_mapping:=true', 'rviz:=false'],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
node = None
try:
    rclpy.init()
    node = rclpy.create_node('verified_profile_inspector')
    client = node.create_client(GetParameters, '/doggy_point_lio/get_parameters')
    assert client.wait_for_service(timeout_sec=20), 'Installed mapper parameter services unavailable'
    future = client.call_async(GetParameters.Request(names=list(expected)))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done(), 'Parameter query timed out'
    actual = {}
    for name, value in zip(expected, future.result().values):
        actual[name] = {1: value.bool_value, 2: value.integer_value, 3: value.double_value}.get(value.type)
    assert actual == expected, actual
    assert process.poll() is None, 'Installed launch exited unexpectedly'
    binary = root/'ros2_ws/install/point_lio/lib/point_lio/pointlio_mapping'
    result = {'parameters': actual, 'all_expected_values_match': True,
              'binary': str(binary), 'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'domain': 170, 'localhost_only': True, 'robot_inputs_replayed': False,
              'robot_commands_sent': False, 'rviz_started': False}
    (out/'installed_profile_validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
finally:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    log.close()
