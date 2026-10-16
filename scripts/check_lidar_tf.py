#!/usr/bin/env python3
"""Check live cloud transforms at their original timestamps and the robot clock."""
import argparse
import json
import hashlib
import math
import xml.etree.ElementTree as ET
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
import yaml
from go2_frames import canonical_robot_description, BODY_FRAME, FOOTPRINT_FRAME


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout',type=float,default=20)
    parser.add_argument('--output',default='reports/tf/live_tf_check.json')
    args=parser.parse_args()
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock'])
    node=Node('doggy_check_lidar_tf',enable_rosout=False,start_parameter_services=False)
    buffer=Buffer(cache_time=Duration(seconds=10),node=node)
    listener=TransformListener(buffer,node)
    topics={'/utlidar/cloud':'utlidar_lidar','/utlidar/cloud_base':'base_link','/utlidar/cloud_deskewed':'odom'}
    urdf_path=Path(get_package_share_directory('go2_description'))/'urdf/go2_description.urdf'
    official_xml=urdf_path.read_text()
    canonical_xml=canonical_robot_description(official_xml, Path(__file__).resolve().parents[1]/'config/point_lio/sensor_geometry.json')
    model=ET.fromstring(canonical_xml)
    model_links=[link.attrib['name'] for link in model.findall('link')]
    model_joints={joint.attrib['name'] for joint in model.findall('joint') if joint.attrib['type']!='fixed'}
    descriptions=[]
    joints=[]
    node.create_subscription(String,'/robot_description',lambda m:descriptions.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append(m),10)
    resolved_links=[]
    samples={}; received={n:0 for n in topics}; successful={n:set() for n in topics}; errors={}
    clocks=[]
    poses={}
    pose_checks={}
    def odometry(msg):
        key=Time.from_msg(msg.header.stamp).nanoseconds
        poses[key]=msg.pose.pose
        if len(poses)>1000:poses.pop(next(iter(poses)))
    node.create_subscription(Odometry,'/utlidar/robot_odom',odometry,qos_profile_sensor_data)
    topology_ok=False
    pose_ok=False
    tree={}
    def receive(msg,name):
        samples[name]=msg.header
        received[name]+=1
    for name in topics:
        node.create_subscription(PointCloud2,name,lambda msg,name=name:receive(msg,name),qos_profile_sensor_data)
    node.create_subscription(Clock,'/doggy/robot_clock',lambda m:clocks.append(m.clock.sec+m.clock.nanosec/1e9),qos_profile_sensor_data)
    deadline=time.monotonic()+args.timeout
    frame_checks={}
    next_check=0.0
    next_tree_check=0.0
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.02)
        # Keep draining high-rate odometry/TF callbacks. Re-parsing the whole
        # graph after every callback can starve the very data being checked.
        now=time.monotonic()
        if now<next_check:
            continue
        next_check=now+0.1
        for name,header in list(samples.items()):
            if header.frame_id != topics[name]:
                errors[name]=f'Unexpected frame {header.frame_id}'
                continue
            stamp=Time.from_msg(header.stamp)
            try:
                transform=buffer.lookup_transform('odom',header.frame_id,stamp)
                successful[name].add(stamp.nanoseconds)
                frame_checks[name]={'source_frame':header.frame_id,'target_frame':'odom','last_cloud_stamp_ns':stamp.nanoseconds,'last_transform_stamp_ns':Time.from_msg(transform.header.stamp).nanoseconds}
                errors.pop(name,None)
            except Exception as exc:
                errors[name]=str(exc)
        if now>=next_tree_check:
            next_tree_check=now+0.5
            try:
                resolved_links=[link for link in model_links if buffer.can_transform('odom',link,Time())]
            except Exception:
                resolved_links=[]
            tree=yaml.safe_load(buffer.all_frames_as_yaml()) or {}
            topology_ok=(tree.get(BODY_FRAME,{}).get('parent')==FOOTPRINT_FRAME
                         and tree.get(FOOTPRINT_FRAME,{}).get('parent')=='odom' and 'base' not in tree)
        try:
            composed=buffer.lookup_transform('odom',BODY_FRAME,Time())
            key=Time.from_msg(composed.header.stamp).nanoseconds
            if key in poses:
                p,q=poses[key].position,poses[key].orientation
                tr,qr=composed.transform.translation,composed.transform.rotation
                actual=[qr.x,qr.y,qr.z,qr.w];expected=[q.x,q.y,q.z,q.w]
                expected=[v/math.sqrt(sum(x*x for x in expected)) for v in expected]
                position_error=math.sqrt((p.x-tr.x)**2+(p.y-tr.y)**2+(p.z-tr.z)**2)
                rotation_error=min(math.sqrt(sum((a-b)**2 for a,b in zip(actual,expected))),math.sqrt(sum((a+b)**2 for a,b in zip(actual,expected))))
                pose_checks[key]={'position_error_m':position_error,'quaternion_error':rotation_error}
        except Exception:
            pass
        pose_ok=len(pose_checks)>=3 and all(p['position_error_m']<1e-8 and p['quaternion_error']<1e-8 for p in pose_checks.values())
        model_ready=len(resolved_links)==len(model_links) and descriptions and descriptions[-1]==canonical_xml and joints and set(joints[-1].name)==model_joints
        if all(len(v)>=3 for v in successful.values()) and len(clocks)>=3 and node.get_clock().now().nanoseconds>0 and model_ready and topology_ok and pose_ok:
            break
    try:
        latest=buffer.lookup_transform('odom','base_link',Time())
        latest_tf=latest.header.stamp.sec+latest.header.stamp.nanosec/1e9
    except Exception:
        latest_tf=None
    result={'passed':bool(all(len(v)>=3 for v in successful.values()) and len(clocks)>=3 and node.get_clock().now().nanoseconds>0 and len(resolved_links)==len(model_links) and descriptions and descriptions[-1]==canonical_xml and joints and set(joints[-1].name)==model_joints and topology_ok and pose_ok),
            'urdf_source':str(urdf_path),'urdf_sha256':hashlib.sha256(official_xml.encode()).hexdigest(),
            'published_description_matches_canonical_developer_model':bool(descriptions and descriptions[-1]==canonical_xml),
            'canonical_description_sha256':hashlib.sha256(canonical_xml.encode()).hexdigest(),
            'root_link_rename':{'from':'base','to':'base_link'},'canonical_topology':topology_ok,
            'source_pose_comparisons':len(pose_checks),'max_position_error_m':max((p['position_error_m'] for p in pose_checks.values()),default=None),
            'source_poses_buffered':len(poses),
            'max_quaternion_error':max((p['quaternion_error'] for p in pose_checks.values()),default=None),
            'model_links':len(model_links),'resolved_model_links':resolved_links,
            'joint_names':joints[-1].name if joints else [],'joint_positions':list(joints[-1].position) if joints else [],
            'received_clouds':received,'distinct_timestamp_transform_checks':{n:len(v) for n,v in successful.items()},
            'frame_checks':frame_checks,'errors':errors,'robot_clock_received':len(clocks),
            'latest_robot_clock':clocks[-1] if clocks else None,'latest_dynamic_tf_stamp':latest_tf,
            'checker_ros_time_s':node.get_clock().now().nanoseconds/1e9,
            'host_unix_s':time.time(),'tf_tree':buffer.all_frames_as_yaml(),
            'method':'Published description matches developer geometry with root-name normalization; all model links and 12 joints checked; odom/footprint/body composition compared to source poses at identical stamps; all three cloud TFs checked at source stamps; approved topic-derived measurement frame appended without altering developer mounts.'}
    dest=Path(args.output);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(json.dumps(result,indent=2)+'\n')
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    print(json.dumps(result,indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
