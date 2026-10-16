"""Load approved YAML in a temporary ROS validator; never starts Point-LIO."""
from pathlib import Path
import re, json, hashlib, subprocess, math
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
ROOT = Path(__file__).resolve().parents[3]
REPORT = Path(__file__).resolve().parent
path = ROOT/'config/point_lio/go2_body_imu.yaml'
source = (ROOT/'third_party/point_lio_ros2/src/parameters.cpp').read_text()
declarations = {name: kind.strip() for kind, name in re.findall(r'declare_parameter<([^;]+?)>\("([^"]+)"', source)}
def flatten(data, prefix=''):
    out = {}
    for k, v in data.items():
        if isinstance(v, dict): out.update(flatten(v, prefix+k+'.'))
        else: out[prefix+k] = v
    return out
expected = flatten(yaml.safe_load(path.read_text())['/**']['ros__parameters'])
draft = flatten(yaml.safe_load((REPORT/'go2_body_imu.review.yaml').read_text())['/**']['ros__parameters'])
types = {'bool': ParameterType.PARAMETER_BOOL, 'int': ParameterType.PARAMETER_INTEGER,
         'float': ParameterType.PARAMETER_DOUBLE, 'double': ParameterType.PARAMETER_DOUBLE,
         'std::string': ParameterType.PARAMETER_STRING, 'std::vector<double>': ParameterType.PARAMETER_DOUBLE_ARRAY}
rclpy.init(args=['--ros-args', '--params-file', str(path)])
node = Node('doggy_point_lio_profile_validator', enable_rosout=False,
            start_parameter_services=False, automatically_declare_parameters_from_overrides=True)
results = {}
for name, value in expected.items():
    p = node.get_parameter(name)
    results[name] = {'cpp_type': declarations.get(name), 'ros_type': p.type_.value,
                     'type_matches': name in declarations and p.type_.value == types.get(declarations[name]),
                     'value_matches': p.value == value}
geometry = json.loads((ROOT/'config/point_lio/sensor_geometry.json').read_text())['point_lio_body_imu_extrinsics']
checks = {
    'reviewed_parameter_values_unchanged': expected == draft,
    'all_52_parameters_loaded': len(results) == 52,
    'all_names_declared_by_estimator': all(n in declarations for n in expected),
    'all_ros_types_match_cpp_declarations': all(v['type_matches'] for v in results.values()),
    'all_ros_loaded_values_match_yaml': all(v['value_matches'] for v in results.values()),
    'raw_cloud_selected': expected['common.lid_topic'] == '/utlidar/cloud',
    'approved_body_imu_selected': expected['common.imu_topic'] == '/doggy/imu/body_calibrated',
    'exact_approved_extrinsics': all(expected[k] == geometry[k] for k in ('mapping.extrinsic_R','mapping.extrinsic_T')),
    'pcd_and_body_cloud_disabled': not expected['pcd_save.pcd_save_en'] and not expected['publish.scan_bodyframe_pub_en'],
    'stationary_initialization_and_fixed_extrinsics': not expected['mapping.start_in_aggressive_motion'] and not expected['mapping.extrinsic_est_en'],
    'point_time_in_seconds': expected['preprocess.timestamp_unit'] == 0,
    'finite_numeric_values': all(math.isfinite(x) for v in expected.values() for x in (v if isinstance(v,list) else [v]) if isinstance(x,(float,int))),
}
status = subprocess.run(['git','-C',str(ROOT/'third_party/point_lio_ros2'),'status','--porcelain'],capture_output=True,text=True,check=True)
checks['upstream_point_lio_unchanged'] = not status.stdout.strip()
result = {'passed': all(checks.values()), 'checks': checks, 'parameter_count': len(results),
          'parameter_details': results, 'profile_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
          'method': 'Actual ROS parameter-file loader in a temporary validator on domain 197, CycloneDDS explicitly restricted to lo; C++ names/types compared statically. No estimator execution.',
          'limitations': ['L1 model identification provisional.', 'Noise/saturation thresholds retained from upstream, not measured sensor calibration.', 'Nominal 0.004 s interval and zero inter-sensor lag need step 4.', 'Output frames and runnable Go2 launch need step 5.']}
node.destroy_node()
rclpy.shutdown()
(REPORT/'applied_profile_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k != 'parameter_details'},indent=2))
raise SystemExit(0 if result['passed'] else 1)
