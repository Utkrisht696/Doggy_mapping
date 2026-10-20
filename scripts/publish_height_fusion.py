#!/usr/bin/env python3
"""Canonical Point-LIO outputs: preserve XY/orientation, fuse output height."""
from collections import deque
import copy
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from height_fusion import HeightFusion


def ns(stamp):
    return int(stamp.sec)*10**9+int(stamp.nanosec)


class HeightOutput(Node):
    def __init__(self):
        super().__init__('doggy_height_fusion')
        self.frame = str(self.declare_parameter('output_frame', 'odom').value)
        if self.frame not in ('odom', 'map'):
            raise ValueError('output_frame must be odom or map')
        self.core = HeightFusion(
            std=float(self.declare_parameter('vendor_height_std', .01).value),
            process_variance_per_s=float(self.declare_parameter('process_variance_per_s', .0025).value),
            max_age=float(self.declare_parameter('max_vendor_age', .05).value))
        geometry = Path(str(self.declare_parameter('geometry_file',
            str(Path(__file__).resolve().parents[1]/'config/point_lio/sensor_geometry.json')).value))
        self.mount = np.asarray(json.loads(geometry.read_text())['unchanged_developer_mounts']['imu']['xyz'], dtype=float)
        if self.mount.shape != (3,) or not np.isfinite(self.mount).all():
            raise ValueError('Invalid developer body-to-IMU mount')
        self.path_period_ns = int(float(self.declare_parameter('path_sample_period', .2).value)*1e9)
        limit = int(self.declare_parameter('path_max_poses', 6000).value)
        if self.path_period_ns <= 0 or not 1 <= limit <= 50000:
            raise ValueError('Invalid path history limits')
        self.path = deque(maxlen=limit)
        self.path_stamp = 0
        self.path_dirty = False
        self.shifts, self.cloud_queue = {}, deque()
        self.counts = dict(raw_poses=0, fused_poses=0, raw_clouds=0, fused_clouds=0,
                           invalid_vendor=0, invalid_lio=0, cloud_expired=0, path_publications=0)
        self.last_state = None
        self.last_output_wall = None
        self.odom = self.create_publisher(Odometry, '/point_lio/odometry', 1000)
        self.clouds = self.create_publisher(PointCloud2, '/point_lio/cloud_registered', 100)
        self.path_pub = self.create_publisher(RosPath, '/point_lio/path', 1)
        self.status_pub = self.create_publisher(String, '/point_lio/height_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        vendor_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.vendor, vendor_qos)
        self.create_subscription(Odometry, '/point_lio/raw/odometry', self.lio, 1000)
        self.create_subscription(PointCloud2, '/point_lio/raw/cloud_registered', self.cloud, 100)
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.flush, clock=wall)
        self.create_timer(.5, self.publish_path, clock=wall)
        self.create_timer(5., self.status, clock=wall)
        destination = str(self.declare_parameter('diagnostics_csv', '').value)
        self.stream = None
        if destination:
            path = Path(destination); path.parent.mkdir(parents=True, exist_ok=True)
            self.stream = path.open('w')
            self.writer = csv.writer(self.stream)
            self.writer.writerow(['state_ns','measurement_ns','raw_z','fused_z','variance',
                                  'age_s','gain','innovation','mode'])
        self.get_logger().info(f'Height fusion is the canonical output in {self.frame}; raw XY/orientation preserved')

    def vendor(self, message):
        q = message.pose.pose.orientation
        quat = np.array([q.x,q.y,q.z,q.w])
        length = np.linalg.norm(quat)
        if (message.header.frame_id != 'odom' or message.child_frame_id != 'base_link'
                or not np.isfinite(quat).all() or abs(length-1) > .01):
            self.counts['invalid_vendor'] += 1
            return
        x,y,z,w = quat/length
        # Third row of R_body times the unchanged developer IMU position.
        offset_z = np.dot([2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)], self.mount)
        self.core.add(ns(message.header.stamp), message.pose.pose.position.z+float(offset_z))

    def lio(self, message):
        self.counts['raw_poses'] += 1
        stamp = ns(message.header.stamp)
        p,q = message.pose.pose.position, message.pose.pose.orientation
        if (message.header.frame_id != 'lio_local' or message.child_frame_id != 'imu'
                or stamp <= 0 or not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all()
                or abs(math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1) > .01):
            self.counts['invalid_lio'] += 1
            return
        try:
            state = self.core.step(stamp, p.z)
        except ValueError:
            self.counts['invalid_lio'] += 1
            return
        output = copy.deepcopy(message)
        output.header.frame_id = self.frame
        output.pose.pose.position.z = state['z']
        covariance = np.array(output.pose.covariance).reshape(6,6)
        if self.core.offset is not None:
            covariance[2,:] = 0.; covariance[:,2] = 0.
            covariance[2,2] = state['variance']
            output.pose.covariance = covariance.ravel().tolist()
        self.odom.publish(output)
        self.counts['fused_poses'] += 1
        self.shifts[stamp] = state['z']-p.z
        if len(self.shifts) > 2000:
            del self.shifts[next(iter(self.shifts))]
        if not self.path or stamp-self.path_stamp >= self.path_period_ns:
            pose = PoseStamped(); pose.header = copy.deepcopy(output.header)
            pose.pose = copy.deepcopy(output.pose.pose)
            self.path.append(pose); self.path_stamp = stamp; self.path_dirty = True
        self.last_state = state
        self.last_output_wall = time.monotonic()
        if self.stream is not None:
            self.writer.writerow([stamp,state['source_stamp'],p.z,state['z'],state['variance'],
                                  state['source_age'],state['gain'],state['innovation'],state['mode']])
            if self.counts['fused_poses'] % 50 == 0:
                self.stream.flush()
        self.flush()

    def cloud(self, message):
        self.counts['raw_clouds'] += 1
        if len(self.cloud_queue) >= 100:
            self.cloud_queue.popleft(); self.counts['cloud_expired'] += 1
        self.cloud_queue.append((message, time.monotonic()))
        self.flush()

    def flush(self):
        while self.cloud_queue:
            message,arrival = self.cloud_queue[0]
            stamp = ns(message.header.stamp)
            if stamp not in self.shifts:
                if time.monotonic()-arrival > 2.:
                    self.cloud_queue.popleft(); self.counts['cloud_expired'] += 1
                    continue
                break
            self.cloud_queue.popleft()
            fields = {f.name:f for f in message.fields}
            if (message.header.frame_id != 'lio_local' or 'z' not in fields
                    or fields['z'].datatype != 7 or fields['z'].count != 1
                    or fields['z'].offset+4 > message.point_step
                    or message.row_step < message.width*message.point_step
                    or len(message.data) != message.row_step*message.height):
                self.counts['cloud_expired'] += 1
                continue
            output = copy.deepcopy(message)
            z = np.ndarray((message.height,message.width), dtype='>f4' if message.is_bigendian else '<f4',
                buffer=output.data, offset=fields['z'].offset, strides=(message.row_step,message.point_step))
            z += self.shifts[stamp]
            output.header.frame_id = self.frame
            self.clouds.publish(output); self.counts['fused_clouds'] += 1

    def publish_path(self):
        if not self.path_dirty:
            return
        message = RosPath(); message.header = copy.deepcopy(self.path[-1].header)
        message.poses = list(self.path)
        self.path_pub.publish(message)
        self.path_dirty = False
        self.counts['path_publications'] += 1

    def status(self):
        state = dict(self.last_state or {})
        payload = dict(**self.counts, filter=self.core.counts, last_state=state,
            output_frame=self.frame, waiting_clouds=len(self.cloud_queue), path_poses=len(self.path),
            input_age_seconds=None if self.last_output_wall is None else time.monotonic()-self.last_output_wall,
            x_y_orientation_preserved=True, coupled_into_lio=False,
            world_height_on_real_slopes_validated=False)
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(json.dumps(payload))


def main():
    rclpy.init(); node = HeightOutput()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.stream is not None:
            node.stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
