#!/usr/bin/env python3
"""Sensor-time clock and visualization joints; no vendor odometry subscription."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from unitree_go.msg import LowState

JOINT_NAMES = [f'{leg}_{joint}_joint' for leg in ('FR','FL','RR','RL') for joint in ('hip','thigh','calf')]


class SensorState(Node):
    def __init__(self):
        super().__init__('doggy_sensor_state')
        self.clock = self.create_publisher(Clock, '/doggy/robot_clock', 10)
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.latest = None
        self.ns = 0
        self.create_subscription(Imu, '/doggy/imu/body_calibrated', self.imu, qos_profile_sensor_data)
        self.create_subscription(LowState, '/lf/lowstate', self.lowstate, qos_profile_sensor_data)

    def imu(self, msg):
        ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if ns <= self.ns:
            return
        self.ns, self.latest = ns, msg.header.stamp
        self.clock.publish(Clock(clock=self.latest))

    def lowstate(self, msg):
        if self.latest is None or len(msg.motor_state) < 12:
            return
        motors = msg.motor_state[:12]
        values = [float(m.q) for m in motors]
        if not all(math.isfinite(v) for v in values):
            return
        out = JointState()
        out.header.stamp = self.latest
        out.name, out.position = JOINT_NAMES, values
        out.velocity = [float(m.dq) for m in motors]
        out.effort = [float(m.tau_est) for m in motors]
        self.joints.publish(out)


def main():
    rclpy.init()
    node = SensorState()
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
