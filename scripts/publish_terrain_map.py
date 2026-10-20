#!/usr/bin/env python3
"""Build live 2D occupancy and terrain layers from exact-stamp LIO outputs."""
from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from lidar_cloud_utils import cloud_arrays
from terrain_grid import TerrainConfig, TerrainGrid
from robot_self_filter import JointHistory, RobotSelfFilter


def stamp_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


class TerrainMapper(Node):
    def __init__(self):
        super().__init__('doggy_terrain_mapper')
        defaults = TerrainConfig()
        config = {f.name:self.declare_parameter(f.name,getattr(defaults,f.name)).value for f in fields(defaults)}
        self.grid = TerrainGrid(TerrainConfig(**config))
        self.wall_min_body_z = float(self.declare_parameter('wall_min_body_z',-0.05).value)
        if not np.isfinite(self.wall_min_body_z):
            raise ValueError('wall_min_body_z must be finite')
        self.self_padding = float(self.declare_parameter('self_filter_padding',0.025).value)
        self.joint_history = JointHistory(
            float(self.declare_parameter('self_filter_max_joint_gap',0.15).value),
            float(self.declare_parameter('self_filter_max_joint_age',0.075).value))
        self.robot_filter = None
        self.self_filtered_points = 0
        self.self_filter_dropped_scans = 0
        self.joint_sample_modes = dict(exact=0,interpolated=0,bounded_past=0)
        self.output = str(self.declare_parameter('output_map','maps/live_2d/map').value)
        period = float(self.declare_parameter('publish_period',1.0).value)
        if not 0.2 <= period <= 10:
            raise ValueError('publish_period must be between 0.2 and 10 seconds')
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publishers = {key:self.create_publisher(OccupancyGrid,topic,qos) for key,topic in
            [('occupancy','/map'),('cost','/terrain/costmap'),('traversability','/terrain/traversability'),
             ('walls','/walls/map')]}
        self.status_pub = self.create_publisher(String,'/terrain/status',qos)
        self.create_subscription(Odometry,'/point_lio/odometry',self.pose,100)
        self.create_subscription(PointCloud2,'/point_lio/cloud_registered',self.cloud,qos_profile_sensor_data)
        self.create_subscription(String,'/robot_description',self.description,qos)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_service(Trigger,'/terrain/save_map',self.save_service)
        self.poses = OrderedDict()
        self.pending = deque(maxlen=60)
        self.last_input = 0
        self.last_stamp = None
        self.last_render_stamp = None
        self.last_cloud_wall = None
        self.dropped = 0
        self.last_error = ''
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.latest = None
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05,self.flush,clock=self.wall)
        self.create_timer(period,self.publish_map,clock=self.wall)
        self.create_timer(5.0,self.status,clock=self.wall)

    def pose(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'imu':
            return
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        xyz = (p.x,p.y,p.z)
        quaternion = (q.x,q.y,q.z,q.w)
        if not np.isfinite((*xyz,*quaternion)).all():
            return
        self.poses[stamp_ns(msg.header.stamp)] = (xyz,quaternion)
        while len(self.poses) > 300:
            self.poses.popitem(last=False)
        self.flush()

    def description(self, msg):
        try:
            robot_filter = RobotSelfFilter(msg.data,self.self_padding)
        except ValueError as error:
            self.last_error = str(error)
            self.get_logger().error(f'Invalid self-filter robot model: {error}')
            return
        self.robot_filter = robot_filter
        self.flush()

    def joints(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        try:
            self.joint_history.add(stamp_ns(msg.header.stamp),dict(zip(msg.name,msg.position)))
        except ValueError as error:
            self.last_error = str(error)
            return
        self.flush()

    def cloud(self, msg):
        ns = stamp_ns(msg.header.stamp)
        if msg.header.frame_id != 'odom' or ns <= self.last_input:
            self.get_logger().error('Cloud needs odom frame and strictly increasing source stamps',throttle_duration_sec=5.0)
            return
        self.last_input = ns
        if len(self.pending) == self.pending.maxlen:
            self.dropped += 1
        self.pending.append((msg,time.monotonic()))
        self.flush()

    def flush(self):
        while self.pending:
            msg,arrival = self.pending[0]
            ns = stamp_ns(msg.header.stamp)
            pose = self.poses.get(ns)
            joint_positions,joint_mode = self.joint_history.sample(ns,allow_hold=time.monotonic()-arrival > .15)
            model_ready = self.robot_filter is not None
            joints_ready = (joint_positions is not None and model_ready
                            and self.robot_filter.required_joints.issubset(joint_positions))
            if pose is None or not joints_ready:
                if time.monotonic()-arrival > 2:
                    self.pending.popleft()
                    self.dropped += 1
                    if not joints_ready:
                        self.self_filter_dropped_scans += 1
                    continue
                return
            self.pending.popleft()
            try:
                xyz,_,valid = cloud_arrays(msg)
                xyz = xyz[valid]
                origin,quaternion = pose
                keep = self.robot_filter.keep_world(xyz,origin,quaternion,joint_positions)
                self.self_filtered_points += int(np.count_nonzero(~keep))
                self.joint_sample_modes[joint_mode] += 1
                filtered = xyz[keep]
                body = self.robot_filter.world_to_body(filtered,origin,quaternion)
                self.grid.integrate(filtered,origin)
                self.grid.integrate_walls(filtered[body[:,2] > self.wall_min_body_z],origin)
            except ValueError as error:
                self.last_error = str(error)
                self.get_logger().error(str(error),throttle_duration_sec=5.0)
                continue
            self.last_stamp = msg.header.stamp
            self.last_cloud_wall = time.monotonic()

    def copy_grid(self):
        clone = TerrainGrid(self.grid.cfg)
        clone.voxels = self.grid.voxels.copy()
        clone.seeds = self.grid.seeds.copy()
        clone.wall_hits = self.grid.wall_hits.copy()
        clone.scans = self.grid.scans
        clone.clipped_points = self.grid.clipped_points
        clone.capacity_drops = self.grid.capacity_drops
        clone.preprocessing = self.preprocessing_metadata()
        return clone

    def preprocessing_metadata(self):
        return dict(robot_self_filter='URDF primitive collision volumes at source scan time',
            padding_m=self.self_padding,removed_points=self.self_filtered_points,
            skipped_scans_missing_model_or_joints=self.self_filter_dropped_scans,
            joint_sample_modes=self.joint_sample_modes.copy(),wall_min_body_z_m=self.wall_min_body_z)

    def publish_map(self):
        if self.future is not None and self.future.done():
            try:
                data = self.future.result()
                if data is not None:
                    self.latest = data
                    for key,publisher in self.map_publishers.items():
                        msg = OccupancyGrid()
                        msg.header.frame_id,msg.header.stamp = 'odom',self.last_render_stamp
                        msg.info.map_load_time = self.last_render_stamp
                        msg.info.resolution = data['resolution']
                        msg.info.height,msg.info.width = data[key].shape
                        msg.info.origin.position.x,msg.info.origin.position.y = map(float,data['origin'])
                        msg.info.origin.orientation.w = 1.0
                        # Typed byte storage avoids Python per-element validation.
                        import array
                        msg.data = array.array('b',data[key].ravel().tobytes())
                        publisher.publish(msg)
            except Exception as error:
                self.last_error = str(error)
                self.get_logger().error(f'Terrain rendering failed: {error}')
            self.future = None
        if self.future is None and self.last_stamp is not None and self.last_stamp != self.last_render_stamp:
            clone = self.copy_grid()
            self.last_render_stamp = self.last_stamp
            self.future = self.worker.submit(clone.render)

    def status(self):
        data = self.latest
        status = dict(scans=self.grid.scans,voxels=len(self.grid.voxels),unmatched_clouds=self.dropped,
            pending_clouds=len(self.pending),clipped_points=self.grid.clipped_points,
            capacity_drops=self.grid.capacity_drops,resolution=self.grid.cfg.resolution,frame='odom',
            source='Point-LIO XY/orientation with causal vendor-height fusion',provisional_terrain_thresholds=True,last_error=self.last_error,
            input_age_seconds=None if self.last_cloud_wall is None else time.monotonic()-self.last_cloud_wall)
        status.update(self.preprocessing_metadata(),self_filter_model_ready=self.robot_filter is not None)
        if data is not None:
            status.update(width=data['occupancy'].shape[1],height=data['occupancy'].shape[0],
                free_cells=int(np.count_nonzero(data['occupancy'] == 0)),
                occupied_cells=int(np.count_nonzero(data['occupancy'] == 100)),
                terrain_blocked_cells=int(np.count_nonzero(data['cost'] == 100)))
            status['wall_cells'] = int(np.count_nonzero(data['walls'] == 100))
        self.status_pub.publish(String(data=json.dumps(status)))
        self.get_logger().info(json.dumps(status))

    def save_service(self, request, response):
        try:
            response.message = self.copy_grid().save(self.output)
            response.success = True
        except Exception as error:
            response.success,response.message = False,str(error)
        return response

    def finish(self):
        self.worker.shutdown(wait=True)
        if self.grid.scans:
            try:
                self.grid.preprocessing = self.preprocessing_metadata()
                saved = self.grid.save(self.output)
                print(f'Saved occupancy, traversability and terrain layers: {saved}',flush=True)
            except Exception as error:
                print(f'Map save failed: {error}',flush=True)


def main():
    rclpy.init()
    node = TerrainMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        import signal
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
