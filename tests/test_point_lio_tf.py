"""Check full 3-D composition and exact-stamp bridge behavior without a mapper.

ROS cases run only with explicit isolated-domain/localhost settings, so fake
poses cannot enter the live robot graph. They use real tf2 interpolation.
"""
import copy
import math
import os
from pathlib import Path
import sys
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from point_lio_tf import map_to_odom


class CompositionTests(unittest.TestCase):
    def test_random_full_3d_poses_against_matrix_inverse(self):
        rng = np.random.default_rng(8026)
        for _ in range(500):
            a, b = np.eye(4), np.eye(4)
            a[:3, :3] = Rotation.random(random_state=rng).as_matrix()
            b[:3, :3] = Rotation.random(random_state=rng).as_matrix()
            a[:3, 3], b[:3, 3] = rng.uniform(-10, 10, (2, 3))
            p, q = map_to_odom(a[:3, 3], Rotation.from_matrix(a[:3, :3]).as_quat(),
                               b[:3, 3], Rotation.from_matrix(b[:3, :3]).as_quat())
            actual = np.eye(4)
            actual[:3, :3], actual[:3, 3] = Rotation.from_quat(q).as_matrix(), p
            np.testing.assert_allclose(actual, a @ np.linalg.inv(b), atol=1e-12)
            np.testing.assert_allclose(actual @ b, a, atol=1e-12)

    def test_imu_lever_arm_and_tilt_are_preserved(self):
        # Body and IMU are displaced. Renaming the IMU pose to base_link fails
        # this independent round trip, especially during roll/pitch changes.
        body = np.eye(4)
        body[:3, :3] = Rotation.from_euler('xyz', [.4, -.3, 1.2]).as_matrix()
        body[:3, 3] = [2, -3, .4]
        mount = np.eye(4)
        mount[:3, 3] = [-.02557, 0, .04232]
        odom_imu = body @ mount
        desired = np.eye(4)
        desired[:3, :3] = Rotation.from_euler('xyz', [-.2, .1, -1]).as_matrix()
        desired[:3, 3] = [-2, 1, .3]
        map_imu = desired @ odom_imu
        p, q = map_to_odom(map_imu[:3, 3], Rotation.from_matrix(map_imu[:3, :3]).as_quat(),
                           odom_imu[:3, 3], Rotation.from_matrix(odom_imu[:3, :3]).as_quat())
        result = np.eye(4)
        result[:3, :3], result[:3, 3] = Rotation.from_quat(q).as_matrix(), p
        np.testing.assert_allclose(result @ body @ mount, map_imu, atol=1e-12)

    def test_invalid_measurements_rejected(self):
        for p, q in [([float('nan'), 0, 0], [0, 0, 0, 1]),
                     ([0, 0, 0], [0, 0, 0, 0]),
                     ([0, 0, 0], [0, 0, 0, 2]),
                     ([0, 0, 0], [math.inf, 0, 0, 1])]:
            with self.assertRaises(ValueError):
                map_to_odom(p, q, [0, 0, 0], [0, 0, 0, 1])


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '197':
            raise unittest.SkipTest('ROS tests require isolated localhost/domain 197')
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def setUp(self):
        from publish_point_lio_tf import PointLioTf
        self.node = PointLioTf()
        self.sent = []
        class Capture:
            def sendTransform(_, message):
                self.sent.append(copy.deepcopy(message))
        self.node.broadcaster = Capture()

    def tearDown(self):
        self.node.destroy_node()

    def tf(self, sec, x):
        from geometry_msgs.msg import TransformStamped
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = 'odom', 'imu'
        m.header.stamp.sec = sec
        m.transform.translation.x = float(x)
        m.transform.rotation.w = 1.0
        self.node.buffer.set_transform(m, 'isolated_test')

    def odometry(self, sec=11, nanosec=123456789):
        from nav_msgs.msg import Odometry
        m = Odometry()
        m.header.frame_id, m.child_frame_id = 'map', 'imu'
        m.header.stamp.sec, m.header.stamp.nanosec = sec, nanosec
        m.pose.pose.position.x = 20.0
        m.pose.pose.orientation.w = 1.0
        return m

    def test_waits_for_exact_stamp_and_interpolates_instead_of_latest(self):
        self.tf(10, 1)
        m = self.odometry()
        self.node.receive(m)
        self.assertEqual(self.sent, [])
        self.assertEqual(len(self.node.pending), 1)
        self.tf(12, 3)
        self.node.flush()
        self.assertEqual(len(self.sent), 1)
        result = self.sent[0]
        self.assertEqual(result.header.stamp, m.header.stamp)
        self.assertEqual((result.header.frame_id, result.child_frame_id), ('map', 'odom'))
        self.assertAlmostEqual(result.transform.translation.x, 20-2.123456789, places=9)

    def test_missing_tf_expires_without_publishing_an_identity_fallback(self):
        self.node.receive(self.odometry())
        m, _ = self.node.pending[0]
        self.node.pending[0] = (m, time.monotonic()-3)
        self.node.flush()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.node.counts['expired'], 1)

    def test_rejects_wrong_frames_zero_time_invalid_pose_and_clock_reset(self):
        for kind in ('parent', 'child', 'time', 'pose'):
            m = self.odometry()
            if kind == 'parent': m.header.frame_id = 'camera_init'
            if kind == 'child': m.child_frame_id = 'base_link'
            if kind == 'time': m.header.stamp.sec = m.header.stamp.nanosec = 0
            if kind == 'pose': m.pose.pose.orientation.w = 0.0
            self.node.receive(m)
        self.assertEqual(self.node.counts['invalid'], 4)
        self.tf(10, 1)
        self.tf(12, 3)
        self.node.receive(self.odometry())
        self.node.receive(self.odometry())
        self.node.receive(self.odometry(sec=10))
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.node.counts['nonincreasing'], 2)

    def test_queue_is_bounded_and_reports_overflow(self):
        for i in range(130):
            self.node.receive(self.odometry(nanosec=i))
        self.assertEqual(len(self.node.pending), 128)
        self.assertEqual(self.node.counts['overflow'], 2)
        self.assertEqual(self.sent, [])


if __name__ == '__main__':
    unittest.main()
