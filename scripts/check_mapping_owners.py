#!/usr/bin/env python3
"""Refuse a duplicate live bringup before it can publish competing TF/odometry."""
import time
import rclpy


def main():
    rclpy.init()
    node = rclpy.create_node('doggy_2d_preflight')
    end = time.monotonic()+3
    while time.monotonic() < end:
        rclpy.spin_once(node,timeout_sec=0.1)
    conflicts = {}
    for topic in ['/odom','/map','/robot_description','/doggy/robot_clock','/doggy/imu/body_calibrated',
                  '/point_lio/odometry','/point_lio/raw/odometry','/point_lio/height_status']:
        owners = [p.node_name for p in node.get_publishers_info_by_topic(topic)]
        if owners:
            conflicts[topic] = owners
    node.destroy_node()
    rclpy.shutdown()
    if conflicts:
        raise SystemExit(f'Existing mapping publishers must be stopped first: {conflicts}')


if __name__ == '__main__':
    main()
