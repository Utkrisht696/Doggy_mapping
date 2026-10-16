#!/usr/bin/env python3
"""Read-only sensor and optional SLAM Toolbox output check; no robot commands."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, Imu, LaserScan
from unitree_go.msg import LowState
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--with-mapping', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-r', '/clock:=/doggy/robot_clock'])
    node = rclpy.create_node('doggy_slam_readiness_check')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    stats, latest, subscriptions, beam_counts = {}, {}, [], []
    topics = {'/utlidar/cloud': PointCloud2, '/utlidar/cloud_base': PointCloud2,
              '/utlidar/cloud_deskewed': PointCloud2, '/utlidar/imu': Imu,
              '/utlidar/robot_odom': Odometry, '/lf/lowstate': LowState}
    if args.with_mapping:
        topics.update({'/tests/slam_toolbox/scan': LaserScan,
                       '/tests/slam_toolbox/map': OccupancyGrid})

    def receive(topic, msg):
        now = time.monotonic()
        row = stats[topic]
        row['messages'] += 1
        row.setdefault('first_receipt', now)
        row['last_receipt'] = now
        latest[topic] = msg
        if isinstance(msg, LaserScan):
            beam_counts.append(sum(math.isfinite(x) and msg.range_min <= x <= msg.range_max for x in msg.ranges))

    for topic, msgtype in topics.items():
        stats[topic] = {'messages': 0}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if msgtype is OccupancyGrid else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(msgtype, topic, lambda msg, t=topic: receive(t, msg), qos))
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    for topic, row in stats.items():
        count = row['messages']
        if count:
            interval = row.pop('last_receipt') - row.pop('first_receipt')
            row['receipt_rate_hz'] = round((count - 1) / interval, 2) if interval else 0
            msg = latest[topic]
            if hasattr(msg, 'header'):
                row['frame'] = msg.header.frame_id
                row['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                row['laptop_minus_header_seconds'] = round(time.time() - row['stamp'], 3)
            if isinstance(msg, PointCloud2):
                row['points'] = msg.width * msg.height
                row['fields'] = [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype} for f in msg.fields]
            elif isinstance(msg, Imu):
                a = msg.linear_acceleration
                row['acceleration_norm_m_s2'] = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z)
            elif isinstance(msg, Odometry):
                row['child_frame'] = msg.child_frame_id
            elif isinstance(msg, LaserScan):
                row.update(beams=len(msg.ranges), finite_beams=beam_counts[-1],
                           finite_beams_min=min(beam_counts), finite_beams_max=max(beam_counts))
            elif isinstance(msg, OccupancyGrid):
                row.update(width=msg.info.width, height=msg.info.height, resolution_m=msg.info.resolution,
                           known_cells=sum(x >= 0 for x in msg.data), occupied_cells=sum(x >= 50 for x in msg.data))
            elif isinstance(msg, LowState):
                row['motor_count'] = len(msg.motor_state)
    checks = {'all_sensor_streams_received': all(stats[t]['messages'] > 0 for t in list(topics)[:6])}
    if args.with_mapping:
        scan = latest.get('/tests/slam_toolbox/scan')
        checks['scan_received'] = scan is not None
        checks['map_received'] = '/tests/slam_toolbox/map' in latest
        checks['scan_has_returns'] = bool(beam_counts and max(beam_counts) > 0)
        checks['odom_to_scan_at_source_stamp'] = bool(scan and buffer.can_transform('odom', scan.header.frame_id, Time.from_msg(scan.header.stamp)))
        checks['map_to_odom'] = bool(buffer.can_transform('map', 'odom', Time()))
        checks['official_radar_link'] = bool(buffer.can_transform('base_link', 'radar', Time()))
    result = {'checked_local': datetime.now().astimezone().isoformat(),
              'duration_s': args.seconds, 'topics': stats, 'checks': checks,
              'startup_passed': all(checks.values()),
              'scope': 'Stationary read-only startup check; does not establish walking mapping quality or drift.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['startup_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
