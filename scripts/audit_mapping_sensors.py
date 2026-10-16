#!/usr/bin/env python3
"""Passive mapping prerequisites audit. Does not run SLAM or publish commands."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from unitree_go.msg import LidarState, SportModeState
from tf2_ros import Buffer, TransformListener
from lidar_cloud_utils import cloud_arrays


def xyz(msg):
    return cloud_arrays(msg)[0].astype(float)


def rotation(q):
    x,y,z,w=np.array([q.x,q.y,q.z,q.w],dtype=float)
    n=math.sqrt(x*x+y*y+z*z+w*w)
    x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def bounds(values):
    a=np.array(values,dtype=float)
    return {'min':float(np.min(a)), 'median':float(np.median(a)), 'max':float(np.max(a))} if a.size else None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--motion-condition',default='Unspecified; pose variation is not automatically classified as drift.')
    parser.add_argument('--output',type=Path,default=Path('reports/foundation/sensor_audit.json'))
    args=parser.parse_args()
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock'])
    node=rclpy.create_node('doggy_passive_foundation_audit')
    buffer=Buffer(node=node)
    listener=TransformListener(buffer,node)
    types={'/utlidar/cloud':PointCloud2,'/utlidar/cloud_base':PointCloud2,
           '/utlidar/cloud_deskewed':PointCloud2,'/utlidar/imu':Imu,
           '/utlidar/robot_odom':Odometry,'/lf/sportmodestate':SportModeState,
           '/utlidar/lidar_state':LidarState}
    data={t:[] for t in types}
    latest={}
    cloud_samples={t:[] for t in types if types[t] is PointCloud2}
    imu_values=[]
    odom_values=[]
    sport_values=[]
    errors=[]

    def receive(topic,msg):
        arrival=time.monotonic()
        stamp=(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9) if hasattr(msg,'header') else None
        data[topic].append((arrival,stamp,time.time()-stamp if stamp else None))
        latest[topic]=msg
        try:
            if isinstance(msg,PointCloud2) and (not cloud_samples[topic] or arrival-cloud_samples[topic][-1]['receipt']>=1):
                points=xyz(msg)
                finite=np.isfinite(points).all(axis=1)
                zero=np.equal(points,0).all(axis=1)
                usable=points[finite & ~zero]
                sample={'receipt':arrival,'points':len(points),'zero_xyz':int(zero.sum()),
                        'nonfinite_xyz':int((~finite).sum()),'usable_points':len(usable),
                        'is_dense':msg.is_dense,'frame':msg.header.frame_id}
                if usable.size:
                    sample['xyz_min']=usable.min(axis=0).tolist()
                    sample['xyz_max']=usable.max(axis=0).tolist()
                if topic=='/utlidar/cloud_base':
                    distance=np.linalg.norm(usable[:,:2],axis=1)
                    bins=np.floor((np.arctan2(usable[:,1],usable[:,0])+np.pi)/(np.pi/180)).astype(int)%360
                    sample['potential_1deg_angular_bins_by_min_height']={str(h):int(np.unique(bins[(usable[:,2]>=h)&(usable[:,2]<=1.5)&(distance>=0.4)&(distance<=12)]).size) for h in (-0.3,-0.15,0,0.05)}
                cloud_samples[topic].append(sample)
            elif isinstance(msg,Imu):
                a=msg.linear_acceleration;g=msg.angular_velocity;q=msg.orientation
                imu_values.append([a.x,a.y,a.z,g.x,g.y,g.z,q.x,q.y,q.z,q.w])
            elif isinstance(msg,Odometry):
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                odom_values.append([stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            elif isinstance(msg,SportModeState):
                sport_values.append(list(msg.position))
        except Exception as exc:
            if len(errors)<10:errors.append(f'{topic}: {exc}')

    subs=[node.create_subscription(mt,t,lambda msg,t=t:receive(t,msg),qos_profile_sensor_data) for t,mt in types.items()]
    started=time.monotonic()
    while time.monotonic()-started<args.seconds:
        rclpy.spin_once(node,timeout_sec=0.02)
    streams={}
    for t,samples in data.items():
        times=np.array([s[0] for s in samples]);stamps=np.array([s[1] for s in samples if s[1] is not None])
        row={'messages':len(samples),'rate_hz':float((len(samples)-1)/(times[-1]-times[0])) if len(samples)>1 else 0,
             'publisher_count':len(node.get_publishers_info_by_topic(t)),
             'receipt_interval_s':bounds(np.diff(times)), 'header_interval_s':bounds(np.diff(stamps)),
             'non_increasing_headers':int(np.sum(np.diff(stamps)<=0)),
             'laptop_minus_header_s':bounds([s[2] for s in samples if s[2] is not None])}
        msg=latest.get(t)
        if msg is not None and hasattr(msg,'header'):row['frame']=msg.header.frame_id
        streams[t]=row
    result={'checked_local':datetime.now().astimezone().isoformat(),'duration_s':args.seconds,
            'method':'Passive subscriptions; no SLAM, cloud registration optimization, service calls or robot commands.',
            'motion_condition':args.motion_condition,
            'streams':streams,'cloud_samples':cloud_samples,'errors':errors}
    if odom_values:
        a=np.array(odom_values);p=a[:,1:4];q=a[:,4:8]
        yaw=np.unwrap(np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2)))
        result['odometry']={'child_frame':latest['/utlidar/robot_odom'].child_frame_id,
            'position_start_m':p[0].tolist(),'position_end_m':p[-1].tolist(),
            'position_span_m':np.ptp(p,axis=0).tolist(),'net_translation_m':float(np.linalg.norm(p[-1]-p[0])),
            'largest_position_step_m':float(np.linalg.norm(np.diff(p,axis=0),axis=1).max()) if len(p)>1 else 0,
            'yaw_span_deg':float(np.ptp(yaw)*180/np.pi),'yaw_net_change_deg':float((yaw[-1]-yaw[0])*180/np.pi),
            'largest_yaw_step_deg':float(np.max(np.abs(np.diff(yaw)))*180/np.pi) if len(yaw)>1 else 0,
            'quaternion_norm':bounds(np.linalg.norm(q,axis=1)),
            'pose_covariance_all_zero':not any(latest['/utlidar/robot_odom'].pose.covariance),
            'twist_covariance_all_zero':not any(latest['/utlidar/robot_odom'].twist.covariance)}
    if imu_values:
        a=np.array(imu_values)
        result['imu']={'finite_values':bool(np.isfinite(a).all()),'acceleration_norm_m_s2':bounds(np.linalg.norm(a[:,:3],axis=1)),
            'angular_velocity_mean_rad_s':np.mean(a[:,3:6],axis=0).tolist(),
            'angular_velocity_std_rad_s':np.std(a[:,3:6],axis=0).tolist(),
            'quaternion_norm':bounds(np.linalg.norm(a[:,6:10],axis=1)),
            'covariances_all_zero':not any(list(latest['/utlidar/imu'].orientation_covariance)+list(latest['/utlidar/imu'].angular_velocity_covariance)+list(latest['/utlidar/imu'].linear_acceleration_covariance))}
    if sport_values:
        a=np.array(sport_values)
        result['sport_position']={'span_m':np.ptp(a,axis=0).tolist(),'net_translation_m':float(np.linalg.norm(a[-1]-a[0])),
                                 'note':'Separate vendor estimate; no frame alias or fusion assumed.'}
    if '/utlidar/lidar_state' in latest:result['lidar_state']=message_to_ordereddict(latest['/utlidar/lidar_state'])
    actual_topics=dict(node.get_topic_names_and_types())
    result['local_publishers']={t:[{'node':p.node_name,'namespace':p.node_namespace} for p in node.get_publishers_info_by_topic(t)] if t in actual_topics else [] for t in ('/tf','/tf_static','/doggy/robot_clock','/clock','/joint_states')}
    result['tf_tree']=buffer.all_frames_as_yaml()
    try:
        msg=latest['/utlidar/robot_odom']
        tf=buffer.lookup_transform('odom','base_link',Time())
        tfstamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
        closest=min(odom_values,key=lambda x:abs(x[0]-tfstamp))
        tr=tf.transform.translation
        result['tf_matches_odometry']={'stamp_distance_s':abs(closest[0]-tfstamp),
            'translation_error_m':float(np.linalg.norm(np.array([tr.x,tr.y,tr.z])-np.array(closest[1:4])))}
        desk=latest['/utlidar/cloud_deskewed'];body=latest['/utlidar/cloud_base']
        tf=buffer.lookup_transform('base_link',desk.header.frame_id,Time.from_msg(desk.header.stamp))
        cloud=xyz(desk);good=np.isfinite(cloud).all(axis=1)&~np.equal(cloud,0).all(axis=1)
        tr=tf.transform.translation
        transformed=cloud[good]@rotation(tf.transform.rotation).T+np.array([tr.x,tr.y,tr.z])
        from scipy.spatial import cKDTree
        body_points=xyz(body);body_points=body_points[np.isfinite(body_points).all(axis=1)&~np.equal(body_points,0).all(axis=1)]
        distances=cKDTree(body_points).query(transformed)[0]
        result['body_deskewed_consistency']={'nearest_neighbor_distance_m':bounds(distances),
            'points_compared':len(transformed),'note':'Existing TF only; no transform fitted. Clouds have different processing/timestamps.'}
    except Exception as exc:result['tf_consistency_error']=str(exc)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('cloud_samples','tf_tree','lidar_state')},indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
