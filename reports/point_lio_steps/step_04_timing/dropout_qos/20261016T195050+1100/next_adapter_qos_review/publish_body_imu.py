#!/usr/bin/env python3
"""Publish a calibrated body IMU from timestamped SportModeState; no robot commands.

Fixed biases are loaded from an explicitly approved calibration file. Source
TimeSpec timestamps are preserved. This node publishes no TF and does not launch
an estimator, clock, controller or calibration routine.
"""
import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState


def load_calibration(path):
    data = json.loads(Path(path).read_text())
    if data.get('approved') is not True:
        raise ValueError('Calibration is a review-only candidate; approval is required before publishing.')
    required = {'source_topic': '/sportmodestate',
                'output_topic': '/doggy/imu/body_calibrated', 'frame_id': 'imu'}
    if any(data.get(key) != value for key, value in required.items()):
        raise ValueError('Calibration source, private output topic or developer IMU frame does not match the reviewed contract.')
    for key, bound in [('acc_bias_xyz_m_s2', 1.0), ('gyro_bias_xyz_rad_s', 0.1)]:
        values = data.get(key)
        if not isinstance(values, list) or len(values) != 3 or any(
                not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > bound for v in values):
            raise ValueError(f'Invalid or out-of-profile {key}')
    return data


class BodyImuConverter:
    """Message conversion with checked source time; usable in offline validation."""
    def __init__(self, calibration):
        self.calibration = calibration
        self.last_stamp_ns = None
        self.last_values = None

    def convert(self, source):
        sec, nsec = int(source.stamp.sec), int(source.stamp.nanosec)
        if sec <= 0 or nsec < 0 or nsec >= 1_000_000_000:
            raise ValueError('Invalid source TimeSpec; refusing to invent a timestamp')
        state = source.imu_state
        values = tuple(float(v) for v in [*state.accelerometer, *state.gyroscope, *state.quaternion])
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite body IMU measurement')
        stamp_ns = sec * 1_000_000_000 + nsec
        if self.last_stamp_ns is not None:
            if stamp_ns < self.last_stamp_ns:
                raise ValueError('Source clock moved backwards; restart after resolving the clock reset')
            if stamp_ns == self.last_stamp_ns:
                if values != self.last_values:
                    raise ValueError('Different IMU measurements share a source timestamp')
                return None
        qnorm = math.sqrt(sum(v*v for v in values[6:10]))
        if not 0.95 <= qnorm <= 1.05:
            raise ValueError('Body orientation quaternion is outside the reviewed normalization range')
        out = Imu()
        out.header.stamp.sec, out.header.stamp.nanosec = sec, nsec
        out.header.frame_id = self.calibration['frame_id']
        # Unitree IMUState is WXYZ; preserve its fused estimate in ROS XYZW order.
        qw, qx, qy, qz = values[6:10]
        out.orientation.x, out.orientation.y = qx/qnorm, qy/qnorm
        out.orientation.z, out.orientation.w = qz/qnorm, qw/qnorm
        ba, bg = self.calibration['acc_bias_xyz_m_s2'], self.calibration['gyro_bias_xyz_rad_s']
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (
            values[j] - ba[j] for j in range(3))
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = (
            values[j+3] - bg[j] for j in range(3))
        # Covariances remain zero (unknown). No per-sample acceleration normalization,
        # gravity synthesis/removal, axes rotation or fabricated noise matrices.
        self.last_stamp_ns, self.last_values = stamp_ns, values
        return out


class BodyImuPublisher(Node):
    def __init__(self, calibration):
        super().__init__('doggy_body_imu')
        self.converter = BodyImuConverter(calibration)
        self.publisher = self.create_publisher(Imu, calibration['output_topic'], 100)
        self.subscription = self.create_subscription(
            SportModeState, calibration['source_topic'], self.receive,
            QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE))
        self.published = self.duplicates = 0
        self.create_timer(5.0, self.status)
        self.get_logger().info('Body IMU only: preserving source stamps and applying reviewed fixed biases.')

    def receive(self, source):
        # A violated data/time contract stops this passive node; no hidden retiming.
        message = self.converter.convert(source)
        if message is None:
            self.duplicates += 1
        else:
            self.publisher.publish(message)
            self.published += 1

    def status(self):
        self.get_logger().info(f'Published {self.published} body IMU samples; dropped {self.duplicates} identical duplicates')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-file', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    calibration = load_calibration(args.calibration_file)
    rclpy.init(args=ros_args)
    node = BodyImuPublisher(calibration)
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
