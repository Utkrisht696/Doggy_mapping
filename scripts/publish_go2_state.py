#!/usr/bin/env python3
"""Convert read-only Go2 telemetry to joint states, body odometry TF and clock.

All robot link and sensor mounting geometry comes from the developer URDF through
robot_state_publisher, with its root normalized to base_link. This adapter
decomposes the reported body pose through a dynamic base_footprint.
"""
import fcntl
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from unitree_go.msg import LowState
from go2_frames import ODOM_FRAME, FOOTPRINT_FRAME, BODY_FRAME, split_body_pose

# Official Go2 motor order: SDK example/go2/low_level/unitree_legged_const.py.
JOINT_NAMES=[f'{leg}_{joint}_joint' for leg in ('FR','FL','RR','RL') for joint in ('hip','thigh','calf')]


class Go2State(Node):
    def __init__(self):
        super().__init__('doggy_go2_state')
        self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.clock_pub=self.create_publisher(Clock,'/doggy/robot_clock',QoSProfile(depth=1))
        self.dynamic_tf=TransformBroadcaster(self)
        self.ground_z=float(self.declare_parameter('footprint_ground_z',0.0).value)
        if not math.isfinite(self.ground_z):
            raise ValueError('footprint_ground_z must be finite')
        self.latest_stamp=None
        self.last_tf_ns=None
        self.low_state=None
        self.started=False
        self.last_receipt=None
        self.create_subscription(Odometry,'/utlidar/robot_odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(LowState,'/lf/lowstate',self.lowstate,qos_profile_sensor_data)
        self.create_timer(5.0,self.status)
        self.get_logger().info('Using Unitree URDF for robot/sensor geometry. Listening to odometry and 12 motor encoders only.')
        self.get_logger().info(f'Footprint projection reference: horizontal odom plane z={self.ground_z:.4f} m; body pose is preserved.')

    def odometry(self,msg):
        if msg.header.frame_id!=ODOM_FRAME or msg.child_frame_id!=BODY_FRAME:
            self.get_logger().error('Unexpected robot odometry frames; ignoring')
            return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        norm=math.sqrt(sum(x*x for x in values[3:]))
        stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if stamp_ns<=0 or not all(math.isfinite(x) for x in values) or norm<1e-6:
            return
        # Standard ROS odometry topic; retain the complete vendor message,
        # including timestamps, twist and the supplied covariance values.
        self.odom_pub.publish(msg)
        self.latest_stamp=msg.header.stamp
        self.last_receipt=time.monotonic()
        if self.last_tf_ns is not None and 0<=stamp_ns-self.last_tf_ns<20_000_000:
            return
        flat_position,flat_rotation,body_position,body_rotation=split_body_pose(values[:3],values[3:],self.ground_z)
        footprint=TransformStamped()
        footprint.header=msg.header
        footprint.child_frame_id=FOOTPRINT_FRAME
        body=TransformStamped()
        body.header.stamp=msg.header.stamp
        body.header.frame_id=FOOTPRINT_FRAME
        body.child_frame_id=BODY_FRAME
        for transform,position,rotation in ((footprint,flat_position,flat_rotation),(body,body_position,body_rotation)):
            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=position
            transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=rotation
        self.dynamic_tf.sendTransform([footprint,body])
        self.clock_pub.publish(Clock(clock=msg.header.stamp))
        self.last_tf_ns=stamp_ns
        if self.low_state is not None and not self.started:
            self.publish_joints(self.low_state)

    def lowstate(self,msg):
        self.low_state=msg
        if self.latest_stamp is not None:
            self.publish_joints(msg)

    def publish_joints(self,msg):
        if len(msg.motor_state)<12:
            return
        positions=[float(m.q) for m in msg.motor_state[:12]]
        if not all(math.isfinite(x) for x in positions):
            return
        state=JointState()
        # LowState has a boot tick but no ROS header/epoch time. Associate encoder
        # samples with the latest received robot odometry stamp (visualization only).
        state.header.stamp=self.latest_stamp
        state.name=JOINT_NAMES
        state.position=positions
        state.velocity=[float(m.dq) for m in msg.motor_state[:12]]
        state.effort=[float(m.tau_est) for m in msg.motor_state[:12]]
        self.joint_pub.publish(state)
        if not self.started:
            self.get_logger().info('Live 12-joint model active. TF: odom -> base_footprint -> base_link -> developer URDF links.')
            self.started=True

    def status(self):
        if self.last_receipt is None or time.monotonic()-self.last_receipt>2:
            self.get_logger().warning('Waiting for fresh robot odometry')
        elif not self.started:
            self.get_logger().warning('Waiting for /lf/lowstate motor encoders')


def main():
    root=Path(__file__).resolve().parent.parent
    (root/'reports/tf').mkdir(parents=True,exist_ok=True)
    with (root/'reports/tf/go2_state.lock').open('w') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Go2 state adapter is already running')
        rclpy.init()
        node=Go2State()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt,ExternalShutdownException):
            pass
        finally:
            import signal
            signal.signal(signal.SIGINT,signal.SIG_IGN)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
