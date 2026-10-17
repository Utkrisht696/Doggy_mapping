#!/usr/bin/env python3
"""Offline terrain-map evaluation on existing registered scans; no ROS graph."""
import argparse
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, PointCloud2
from nav_msgs.msg import Odometry

from lidar_cloud_utils import cloud_arrays
from terrain_grid import TerrainGrid
from robot_self_filter import JointHistory, RobotSelfFilter
from go2_frames import canonical_robot_description
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--stride',type=int,default=1)
    parser.add_argument('--compare-self-filter',action='store_true')
    args = parser.parse_args()
    db = next(Path(args.bag).glob('*.db3'))
    c = sqlite3.connect(f'file:{db.resolve()}?mode=ro',uri=True)
    topics = dict(c.execute('select name,id from topics'))
    poses = {}
    for row in c.execute('select data from messages where topic_id=?',(topics['/point_lio/odometry'],)):
        m = deserialize_message(row[0],Odometry)
        p,q = m.pose.pose.position,m.pose.pose.orientation
        poses[m.header.stamp.sec*10**9+m.header.stamp.nanosec] = ((p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
    count = c.execute('select count(*) from messages where topic_id=?',(topics['/joint_states'],)).fetchone()[0]
    history = JointHistory(capacity=max(count,2048))
    for row in c.execute('select data from messages where topic_id=? order by timestamp,id',(topics['/joint_states'],)):
        m = deserialize_message(row[0],JointState)
        history.add(m.header.stamp.sec*10**9+m.header.stamp.nanosec,dict(zip(m.name,m.position)))
    root = Path(__file__).resolve().parents[1]
    share = Path(get_package_share_directory('go2_description'))
    robot_filter = RobotSelfFilter(canonical_robot_description((share/'urdf/go2_description.urdf').read_text(),
                                    root/'config/point_lio/sensor_geometry.json'))
    grid = TerrainGrid()
    baseline = TerrainGrid() if args.compare_self_filter else None
    start = time.monotonic()
    maximum_callback = 0
    missing = 0
    missing_joints = 0
    removed = 0
    modes = dict(exact=0,interpolated=0,bounded_past=0)
    first_stamp = min(poses)
    stationary = None
    for i,row in enumerate(c.execute('select data from messages where topic_id=? order by timestamp,id',
                                   (topics['/point_lio/cloud_registered'],))):
        if i%args.stride:
            continue
        m = deserialize_message(row[0],PointCloud2)
        ns = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        pose = poses.get(ns)
        if pose is None:
            missing += 1
            continue
        xyz,_,valid = cloud_arrays(m)
        positions,mode = history.sample(ns,allow_hold=True)
        if positions is None or not robot_filter.required_joints.issubset(positions):
            missing_joints += 1
            continue
        t = time.monotonic()
        if stationary is None and baseline is not None and ns-first_stamp > 27*10**9:
            stationary = dict(before=map_counts(baseline.render(),1.0),after=map_counts(grid.render(),1.0),
                              removed_points=removed,scans=grid.scans)
        xyz = xyz[valid]
        keep = robot_filter.keep_world(xyz,*pose,positions)
        removed += int(np.count_nonzero(~keep))
        modes[mode] += 1
        grid.integrate(xyz[keep],pose[0])
        filtered = xyz[keep]
        body = robot_filter.world_to_body(filtered,*pose)
        grid.integrate_walls(filtered[body[:,2] > -0.05],pose[0])
        if baseline is not None:
            baseline.integrate(xyz,pose[0])
        maximum_callback = max(maximum_callback,time.monotonic()-t)
    elapsed = time.monotonic()-start
    t = time.monotonic()
    data = grid.render()
    render_seconds = time.monotonic()-t
    grid.preprocessing = dict(robot_self_filter='URDF primitive collision volumes at source scan time',
        padding_m=robot_filter.padding,removed_points=removed,skipped_scans_missing_model_or_joints=missing_joints,
        joint_sample_modes=modes)
    saved = grid.save(args.output)
    report = dict(source_bag=str(db),source_coordinates='Recorded Point-LIO map frame reused as local odom; no estimator rerun or vendor poses',
        scans=grid.scans,voxels=len(grid.voxels),missing_matching_poses=missing,integration_seconds=elapsed,
        max_integration_callback_seconds=maximum_callback,render_seconds=render_seconds,
        width=data['occupancy'].shape[1],height=data['occupancy'].shape[0],
        free_cells=int(np.count_nonzero(data['occupancy'] == 0)),
        occupied_cells=int(np.count_nonzero(data['occupancy'] == 100)),
        terrain_blocked_cells=int(np.count_nonzero(data['cost'] == 100)),
        observed_slope_median_deg=float(np.nanmedian(data['slope_deg'])),saved=saved,
        self_filter=grid.preprocessing)
    if baseline is not None:
        report['stationary_first_27_seconds'] = stationary
        report['full_session_comparison'] = dict(before=map_counts(baseline.render()),after=map_counts(data))
        baseline.save(Path(args.output).with_name('unfiltered_reference'))
    Path(args.output).with_name('validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


def map_counts(data,radius=None):
    if data is None:
        return None
    selected = np.ones(data['occupancy'].shape,dtype=bool)
    if radius is not None:
        y,x = np.indices(selected.shape)
        xx = data['origin'][0]+(x+.5)*data['resolution']
        yy = data['origin'][1]+(y+.5)*data['resolution']
        selected = xx*xx+yy*yy < radius*radius
    return dict(occupied_cells=int(np.count_nonzero(data['occupancy'][selected] == 100)),
                terrain_blocked_cells=int(np.count_nonzero(data['cost'][selected] == 100)),
                free_cells=int(np.count_nonzero(data['occupancy'][selected] == 0)))


if __name__ == '__main__':
    main()
