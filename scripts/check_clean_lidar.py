#!/usr/bin/env python3
"""Verify cleaned sensor messages byte for byte against live source point records."""
import argparse
import json
from pathlib import Path
import time
from collections import defaultdict
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from tf2_msgs.msg import TFMessage
from lidar_cloud_utils import cloud_arrays


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--output',type=Path,default=Path('reports/foundation/clean_cloud_check.json'))
    args=parser.parse_args()
    rclpy.init()
    node=rclpy.create_node('doggy_check_clean_lidar')
    pairs={'/utlidar/cloud':'/doggy/lidar/cloud_raw','/utlidar/cloud_base':'/doggy/lidar/cloud_body',
           '/utlidar/cloud_deskewed':'/doggy/lidar/cloud_deskewed'}
    cache={source:{'source':{},'clean':{}} for source in pairs}
    stats={source:{'checked':0,'failed':0,'last_removed':None,'source_received':0,'clean_received':0} for source in pairs}
    delays={source:[] for source in pairs}
    arrivals={source:{'source':[],'clean':[]} for source in pairs}
    errors=[]
    authorities=defaultdict(set)
    subscriptions=[]
    odom_cache={'source':{},'standard':{}}
    odom_stats={'checked':0,'failed':0}

    def odom_receive(side,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        odom_cache[side][stamp]=msg
        if stamp in odom_cache['source'] and stamp in odom_cache['standard']:
            original=odom_cache['source'].pop(stamp)
            standard=odom_cache['standard'].pop(stamp)
            odom_stats['checked']+=1
            odom_stats['failed']+=int(serialize_message(original)!=serialize_message(standard))
        for entries in odom_cache.values():
            while len(entries)>64:entries.pop(next(iter(entries)))

    def receive(source,side,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        entries=cache[source]
        stats[source][side+'_received']+=1
        arrivals[source][side].append(time.monotonic())
        entries[side][stamp]=(msg,time.monotonic())
        if stamp in entries['source'] and stamp in entries['clean']:
            original,t0=entries['source'].pop(stamp)
            clean,t1=entries['clean'].pop(stamp)
            try:
                xyz,records,valid=cloud_arrays(original)
                _,_,valid_clean=cloud_arrays(clean)
                ok=(clean.header==original.header and clean.fields==original.fields
                    and clean.is_bigendian==original.is_bigendian and clean.point_step==original.point_step
                    and clean.height==1 and clean.width==int(valid.sum())
                    and bytes(clean.data)==records[valid].tobytes() and bool(valid_clean.all()))
                stats[source]['checked']+=1
                stats[source]['failed']+=int(not ok)
                stats[source]['last_removed']=int((~valid).sum())
                delays[source].append(t1-t0)
                if not ok and len(errors)<5:errors.append(source+': header/fields/records mismatch')
            except Exception as exc:
                stats[source]['failed']+=1
                if len(errors)<5:errors.append(source+': '+str(exc))
        for side_cache in entries.values():
            while len(side_cache)>64:side_cache.pop(next(iter(side_cache)))

    def tf_receive(msg):
        for tf in msg.transforms:authorities[tf.child_frame_id].add(tf.header.frame_id)

    for source,output in pairs.items():
        subscriptions.append(node.create_subscription(PointCloud2,source,lambda m,s=source:receive(s,'source',m),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(PointCloud2,output,lambda m,s=source:receive(s,'clean',m),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage,'/tf',tf_receive,qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',tf_receive,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(Odometry,'/utlidar/robot_odom',lambda m:odom_receive('source',m),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Odometry,'/odom',lambda m:odom_receive('standard',m),qos_profile_sensor_data))
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=0.02)
    duplicate_frames={frame:sorted(parents) for frame,parents in authorities.items() if len(parents)>1}
    for source,row in stats.items():
        if delays[source]:
            row['clean_minus_source_receipt_ms']={'median':float(np.median(delays[source])*1000),
                'p95':float(np.percentile(delays[source],95)*1000),'max':float(np.max(delays[source])*1000)}
        for side in ('source','clean'):
            ts=arrivals[source][side]
            row[side+'_receipt_rate_hz']=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0
    tf_publishers={t:[p.node_name for p in node.get_publishers_info_by_topic(t)] for t in ('/tf','/tf_static')}
    expected_publishers={'doggy_go2_state','go2_robot_state_publisher'}
    tf_publishers_ok=(len(tf_publishers['/tf'])==2 and set(tf_publishers['/tf'])==expected_publishers
                      and tf_publishers['/tf_static']==['go2_robot_state_publisher'])
    nodes=node.get_node_names()
    slam_nodes=[n for n in nodes if n in {'slam_toolbox','laserMapping','go2_cloud_to_scan'}]
    result={'duration_s':args.seconds,'streams':stats,'tf_child_frames_observed':len(authorities),
            'standard_odometry':odom_stats,
            'tf_frames_with_multiple_parents':duplicate_frames,'tf_publishers':tf_publishers,
            'tf_publishers_expected_only':tf_publishers_ok,'mapping_nodes_running':slam_nodes,
            'clock_publishers':{t:[p.node_name for p in node.get_publishers_info_by_topic(t)] for t in ('/clock','/doggy/robot_clock')},
            'errors':errors,
            'passed':all(row['checked']>=3 and row['failed']==0 for row in stats.values()) and odom_stats['checked']>=3 and odom_stats['failed']==0 and not duplicate_frames and tf_publishers_ok and not slam_nodes,
            'method':'Matched source timestamps, unchanged frame/field definitions and exact retained point bytes; no SLAM or commands.'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
