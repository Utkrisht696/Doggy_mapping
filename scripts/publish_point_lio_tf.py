#!/usr/bin/env python3
"""Bridge Point-LIO map/imu odometry to one canonical map -> odom TF edge.

Requires odom -> imu at the exact estimator stamp. Never uses latest TF,
an identity fallback, arrival-time stamps or a guessed IMU/body offset.
"""
from collections import deque
import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from point_lio_tf import checked_pose, map_to_odom


class PointLioTf(Node):
    def __init__(self):
        super().__init__('doggy_point_lio_tf')
        self.wait_seconds = float(self.declare_parameter('tf_wait_seconds', 2.0).value)
        if not math.isfinite(self.wait_seconds) or not 0 < self.wait_seconds <= 10:
            raise ValueError('tf_wait_seconds must be in (0, 10]')
        self.buffer = Buffer(cache_time=Duration(seconds=30), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pending = deque()
        self.last_input_ns = None
        self.last_published_ns = None
        self.counts = dict(received=0, published=0, invalid=0, nonincreasing=0,
                           expired=0, overflow=0, lookup_failures=0)
        self.last_tf_error = ''
        self.subscription = self.create_subscription(
            Odometry, '/point_lio/odometry', self.receive, 100)
        self.status_publisher = self.create_publisher(String, '/point_lio/tf_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Wall timers keep waiting/expiry diagnostics useful even if robot time stops.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.flush, clock=self.wall_clock)
        self.create_timer(5.0, self.status, clock=self.wall_clock)
        self.get_logger().info('Waiting for /point_lio/odometry (map / imu); only map -> odom will be published.')

    def receive(self, message):
        self.counts['received'] += 1
        stamp = message.header.stamp
        ns = stamp.sec*1_000_000_000+stamp.nanosec
        pose = message.pose.pose
        try:
            if (message.header.frame_id != 'map' or message.child_frame_id != 'imu'
                    or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or ns <= 0):
                raise ValueError('Expected positive source stamp and map / imu frames')
            checked_pose((pose.position.x, pose.position.y, pose.position.z),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
        except ValueError as error:
            self.counts['invalid'] += 1
            self.get_logger().error(f'Refusing Point-LIO pose: {error}', throttle_duration_sec=5.0)
            return
        if self.last_input_ns is not None and ns <= self.last_input_ns:
            self.counts['nonincreasing'] += 1
            self.get_logger().error('Nonincreasing LIO stamp rejected; restart this launch after a bag/robot clock reset.',
                                    throttle_duration_sec=5.0)
            return
        self.last_input_ns = ns
        if len(self.pending) >= 128:
            self.pending.popleft()
            self.counts['overflow'] += 1
        self.pending.append((message, time.monotonic()))
        self.flush()

    def flush(self):
        while self.pending:
            message, arrival = self.pending[0]
            if time.monotonic()-arrival > self.wait_seconds:
                self.pending.popleft()
                self.counts['expired'] += 1
                continue
            try:
                matching = self.buffer.lookup_transform('odom', 'imu', Time.from_msg(message.header.stamp))
            except TransformException as error:
                self.counts['lookup_failures'] += 1
                self.last_tf_error = str(error)
                break  # Wait for bracketing TF; never ask for Time() / latest.
            pose, tf = message.pose.pose, matching.transform
            try:
                p, q = map_to_odom(
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                    (tf.translation.x, tf.translation.y, tf.translation.z),
                    (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w))
            except ValueError as error:
                self.pending.popleft()
                self.counts['invalid'] += 1
                self.get_logger().error(f'Refusing TF composition: {error}', throttle_duration_sec=5.0)
                continue
            output = TransformStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id, output.child_frame_id = 'map', 'odom'
            output.transform.translation.x, output.transform.translation.y, output.transform.translation.z = p
            output.transform.rotation.x, output.transform.rotation.y, output.transform.rotation.z, output.transform.rotation.w = q
            self.broadcaster.sendTransform(output)
            self.last_published_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            self.counts['published'] += 1
            self.pending.popleft()
            self.last_tf_error = ''

    def status(self):
        state = dict(self.counts, waiting=len(self.pending), last_published_stamp_ns=self.last_published_ns,
                     last_tf_error=self.last_tf_error, edge='map -> odom',
                     input='/point_lio/odometry', acquisition_lag_calibrated=False)
        payload = json.dumps(state, sort_keys=True)
        self.status_publisher.publish(String(data=payload))
        self.get_logger().info(payload)


def main():
    rclpy.init()
    node = PointLioTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
