"""Actual ROS message invariants and bounded visualization output."""
from array import array
import copy
from pathlib import Path
import struct
import sys
import unittest

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from publish_height_fusion import HeightOutput


class Capture:
    def __init__(self):
        self.messages = []
    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class HeightNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'path_max_poses:=8'])
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    def setUp(self):
        self.node = HeightOutput()
        self.node.odom = Capture(); self.node.clouds = Capture(); self.node.path_pub = Capture()
        self.origin = 1792322606503946542
    def tearDown(self):
        self.node.destroy_node()
    def pose(self, stamp, z=0., vendor=False):
        m = Odometry(); m.header.stamp.sec,m.header.stamp.nanosec = divmod(stamp, 10**9)
        m.header.frame_id = 'odom' if vendor else 'lio_local'
        m.child_frame_id = 'base_link' if vendor else 'imu'
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z = 1.25,-2.5,z
        m.pose.pose.orientation.z,m.pose.pose.orientation.w = .6,.8
        m.twist.twist.linear.x = .2
        return m

    def test_pose_and_raw_messages_preserved_except_height_and_world_label(self):
        for i in range(100):
            stamp = self.origin+i*65000000
            self.node.vendor(self.pose(stamp, .35, True))
            raw = self.pose(stamp, .002*i); before = copy.deepcopy(raw)
            self.node.lio(raw); out = self.node.odom.messages[-1]
            self.assertEqual(raw, before)
            self.assertEqual(out.header.stamp, raw.header.stamp)
            self.assertEqual(out.pose.pose.position.x, raw.pose.pose.position.x)
            self.assertEqual(out.pose.pose.position.y, raw.pose.pose.position.y)
            self.assertEqual(out.pose.pose.orientation, raw.pose.pose.orientation)
            self.assertEqual(out.twist, raw.twist)
            self.assertEqual(out.header.frame_id, 'odom')
        self.assertLess(abs(out.pose.pose.position.z), .005)

    def test_padded_big_endian_cloud_waits_for_exact_pose_and_preserves_other_bytes(self):
        self.node.vendor(self.pose(self.origin, .35, True))
        self.node.lio(self.pose(self.origin))
        stamp = self.origin+65000000
        self.node.vendor(self.pose(stamp, .35, True))
        m = PointCloud2(); m.header = self.pose(stamp).header
        m.height,m.width,m.point_step,m.row_step = 2,2,16,40
        m.is_bigendian = True
        m.fields = [PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,k in enumerate(['x','y','z','intensity'])]
        m.data = array('B', [127]*80)
        for row in range(2):
            for col in range(2):
                struct.pack_into('>ffff', m.data, row*40+col*16, 1.,2.,3.,4.)
        before = bytes(m.data)
        self.node.cloud(m)
        self.assertEqual(len(self.node.clouds.messages), 0)
        self.node.lio(self.pose(stamp, .1))
        out = self.node.clouds.messages[-1]
        correction = self.node.odom.messages[-1].pose.pose.position.z-.1
        ignore = set()
        for row in range(2):
            for col in range(2):
                offset = row*40+col*16+8; ignore.update(range(offset,offset+4))
                self.assertAlmostEqual(struct.unpack_from('>f',out.data,offset)[0],3.+correction,places=6)
        self.assertEqual(bytes(m.data), before)
        self.assertTrue(all(out.data[i]==before[i] for i in range(80) if i not in ignore))
        self.assertEqual(out.fields,m.fields)
        self.assertEqual(out.header.stamp,m.header.stamp)
        self.assertEqual(out.header.frame_id,'odom')

    def test_path_is_bounded_and_does_not_publish_per_odometry_sample(self):
        for i in range(600):
            self.node.lio(self.pose(self.origin+i*65000000))
        self.assertEqual(len(self.node.odom.messages),600)
        self.assertEqual(len(self.node.path),8)
        self.assertEqual(len(self.node.path_pub.messages),0)
        self.node.publish_path(); self.node.publish_path()
        self.assertEqual(len(self.node.path_pub.messages),1)
        self.assertEqual(len(self.node.path_pub.messages[0].poses),8)

    def test_feedback_frames_and_repeated_stamps_are_rejected(self):
        m = self.pose(self.origin); m.header.frame_id = 'odom'
        self.node.lio(m)
        self.assertEqual(len(self.node.odom.messages),0)
        m.header.frame_id = 'lio_local'; self.node.lio(m); self.node.lio(m)
        self.assertEqual(len(self.node.odom.messages),1)
        self.assertEqual(self.node.counts['invalid_lio'],2)
