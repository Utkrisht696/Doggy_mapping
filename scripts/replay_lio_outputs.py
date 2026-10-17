#!/usr/bin/env python3
"""Replay recorded LIO outputs into an isolated graph, never robot inputs.

The recording's estimator-local world was called map. Reuse those exact local
coordinates as odom for this pipeline; do not replay the old vendor TF/odometry.
This is output replay, not a rerun or new validation of the LIO estimator.
"""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String


class OutputReplay(Node):
    def __init__(self, bag, rate):
        super().__init__('doggy_lio_output_replay')
        if not 0.1 <= rate <= 10:
            raise ValueError('Replay rate must be between 0.1 and 10')
        self.db = sqlite3.connect(f'file:{next(Path(bag).resolve().glob("*.db3"))}?mode=ro',uri=True)
        topics = dict(self.db.execute('select name,id from topics'))
        self.cloud_id,self.odom_id = topics['/point_lio/cloud_registered'],topics['/point_lio/odometry']
        self.joint_id = topics['/joint_states']
        topic_ids = (self.cloud_id,self.odom_id,self.joint_id)
        self.rows = self.db.execute('select topic_id,timestamp,data from messages where topic_id in (?,?,?) order by timestamp,id',topic_ids)
        self.next_row = next(self.rows,None)
        if self.next_row is None:
            raise ValueError('No recorded LIO outputs')
        self.first_ns = self.next_row[1]
        self.total = self.db.execute('select count(*) from messages where topic_id in (?,?,?)',topic_ids).fetchone()[0]
        self.rate,self.start_wall = rate,time.monotonic()+3
        self.count,self.latest_clock,self.elapsed = 0,0,0.0
        self.odom = self.create_publisher(Odometry,'/point_lio/odometry',100)
        self.clouds = self.create_publisher(PointCloud2,'/point_lio/cloud_registered',20)
        self.joint_pub = self.create_publisher(JointState,'/joint_states',100)
        self.clock_pub = self.create_publisher(Clock,'/doggy/robot_clock',10)
        self.status_pub = self.create_publisher(String,'/terrain/replay_status',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wall = RosClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.01,self.tick,clock=wall)
        self.create_timer(2,self.status,clock=wall)
        self.get_logger().info(f'Replaying {self.total} recorded Point-LIO output messages at {rate}x; original vendor TF excluded')

    def tick(self):
        limit = (time.monotonic()-self.start_wall)*self.rate
        batch = 0
        while self.next_row is not None and (self.next_row[1]-self.first_ns)/1e9 <= limit and batch < 100:
            topic,receipt,data = self.next_row
            cls = {self.cloud_id:PointCloud2,self.odom_id:Odometry,self.joint_id:JointState}[topic]
            msg = deserialize_message(data,cls)
            if topic != self.joint_id:
                if msg.header.frame_id not in ('map','odom'):
                    raise ValueError('Unexpected recorded LIO world frame')
                msg.header.frame_id = 'odom'
            ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if ns > self.latest_clock:
                self.clock_pub.publish(Clock(clock=msg.header.stamp))
                self.latest_clock = ns
            {self.cloud_id:self.clouds,self.odom_id:self.odom,self.joint_id:self.joint_pub}[topic].publish(msg)
            self.count += 1
            batch += 1
            self.elapsed = (receipt-self.first_ns)/1e9
            self.next_row = next(self.rows,None)
            if self.next_row is None:
                self.get_logger().info('Replay complete. Keeping the final 2D map and RViz open; Ctrl+C closes the session.')
                self.status()

    def status(self):
        state = dict(mode='recorded Point-LIO output replay',completed=self.next_row is None,
            published=self.count,total=self.total,recorded_seconds=self.elapsed,rate=self.rate,
            vendor_odometry_replayed=False,recorded_joints_replayed=True,robot_commands_sent=False)
        self.status_pub.publish(String(data=json.dumps(state)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag',required=True)
    parser.add_argument('--rate',type=float,default=1)
    args,ros_args = parser.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Recorded-output replay requires a nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    rclpy.init(args=ros_args)
    node = OutputReplay(args.bag,args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.db.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
