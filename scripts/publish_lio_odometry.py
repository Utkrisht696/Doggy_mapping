#!/usr/bin/env python3
"""Sole /odom and local-body TF publisher for the live 2D mapping launch."""
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from go2_frames import split_body_pose
from lio_odometry import body_odometry


class LioOdometry(Node):
    def __init__(self):
        super().__init__('doggy_lio_odometry')
        self.ground_z = float(self.declare_parameter('footprint_ground_z', 0.0).value)
        if not math.isfinite(self.ground_z):
            raise ValueError('footprint_ground_z must be finite')
        self.buffer = Buffer(cache_time=Duration(seconds=30), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, '/odom', 20)
        self.subscription = self.create_subscription(Odometry, '/point_lio/odometry', self.receive, 100)
        self.pending = deque(maxlen=100)
        self.last_stamp = 0
        self.last_output = None
        self.mount = None
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.flush, clock=wall)
        self.create_timer(5.0, self.status, clock=wall)

    def receive(self, msg):
        ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'imu' or ns <= self.last_stamp:
            self.get_logger().error('Rejecting unexpected LIO frames or nonincreasing source time', throttle_duration_sec=5.0)
            return
        p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
        if not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z)):
            return
        self.last_stamp = ns
        self.pending.append((msg, time.monotonic()))
        self.flush()

    def flush(self):
        if not self.pending:
            return
        if self.mount is None:
            try:
                # This is an invariant URDF mount, not a dynamic pose lookup.
                tf = self.buffer.lookup_transform('base_link', 'imu', Time()).transform
                self.mount = ((tf.translation.x,tf.translation.y,tf.translation.z),
                              (tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w))
            except TransformException:
                while self.pending and time.monotonic()-self.pending[0][1] > 2:
                    self.pending.popleft()
                return
        while self.pending:
            msg, _ = self.pending.popleft()
            p, q, v, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
            try:
                bp, bq, bv, bw = body_odometry((p.x,p.y,p.z), (q.x,q.y,q.z,q.w),
                    (v.x,v.y,v.z), (w.x,w.y,w.z), *self.mount)
            except ValueError as error:
                self.get_logger().error(str(error), throttle_duration_sec=5.0)
                continue
            output = Odometry()
            output.header = msg.header
            output.child_frame_id = 'base_link'
            output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = bp
            output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = bq
            output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = bv
            output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = bw
            # The selected upstream mapping mode supplies zero/unknown covariance.
            # Do not fabricate confidence or copy covariance into different frames.
            self.publisher.publish(output)
            fp, fq, rp, rq = split_body_pose(bp, bq, self.ground_z)
            transforms = []
            for parent, child, pos, quat in [('odom','base_footprint',fp,fq), ('base_footprint','base_link',rp,rq)]:
                tf = TransformStamped()
                tf.header.stamp = msg.header.stamp
                tf.header.frame_id, tf.child_frame_id = parent, child
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quat
                transforms.append(tf)
            self.broadcaster.sendTransform(transforms)
            self.last_output = time.monotonic()

    def status(self):
        if self.last_output is None or time.monotonic()-self.last_output > 2:
            self.get_logger().warning('Waiting for fresh Point-LIO odometry and URDF IMU mount')


def main():
    rclpy.init()
    node = LioOdometry()
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
