"""Posed self geometry must disappear without erasing nearby world obstacles."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from robot_self_filter import JointHistory, RobotSelfFilter

MODEL = '''<robot name="test">
 <link name="base_link"><collision><geometry><box size="0.4 0.2 0.2"/></geometry></collision></link>
 <link name="imu"/>
 <joint name="mount" type="fixed"><parent link="base_link"/><child link="imu"/><origin xyz="0.1 0 0.05" rpy="0 0 1.5707963267948966"/></joint>
 <link name="leg"><collision><origin xyz="0 0 -0.2"/><geometry><cylinder radius="0.03" length="0.4"/></geometry></collision></link>
 <joint name="hip" type="revolute"><parent link="base_link"/><child link="leg"/><origin xyz="-0.2 0.12 0"/><axis xyz="0 1 0"/></joint>
 <link name="foot"><collision><geometry><sphere radius="0.03"/></geometry></collision></link>
 <joint name="ankle" type="fixed"><parent link="leg"/><child link="foot"/><origin xyz="0 0 -0.4"/></joint>
</robot>'''


class SelfFilterTests(unittest.TestCase):
    def test_body_shapes_and_nearby_obstacles_under_rotated_world_pose(self):
        model = RobotSelfFilter(MODEL,.025)
        body = np.array([[0,0,0],[-.2,.12,-.2],[-.2,.12,-.4],
                         [.23,0,0],[0,.13,0],[0,0,-.4],[.3,.1,-.2]])
        # Independently compose T_world_base, then derive T_world_imu.
        rot = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        translation = np.array([3,-2,.4])
        world = body@rot.T+translation
        imu_position = translation+rot@np.array([.1,0,.05])
        # World/base yaw 90 deg and base/imu yaw 90 deg => world/imu yaw 180.
        keep = model.keep_world(world,imu_position,(0,0,1,0),{'hip':0})
        np.testing.assert_array_equal(keep,[False,False,False,True,True,True,True])

    def test_leg_mask_follows_measured_joint_angle(self):
        model = RobotSelfFilter(MODEL,0)
        # Hip rotated 90 deg about Y: leg center moves towards negative X.
        body = np.array([[-.4,.12,0],[-.2,.12,-.2]])
        # Identity world/IMU rotation; convert base points to IMU/world explicitly.
        mount_r = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        world = (body-np.array([.1,0,.05]))@mount_r
        keep = model.keep_world(world,(0,0,0),(0,0,0,1),{'hip':math.pi/2})
        np.testing.assert_array_equal(keep,[False,True])

    def test_missing_joint_measurement_is_rejected(self):
        with self.assertRaises(ValueError):
            RobotSelfFilter(MODEL).keep_world(np.zeros((1,3)),(0,0,0),(0,0,0,1),{})

    def test_joint_interpolation_and_stale_data_rejection(self):
        history = JointHistory()
        history.add(1000000000,{'hip':0.0})
        history.add(1050000000,{'hip':1.0})
        value,mode = history.sample(1025000000)
        self.assertEqual(mode,'interpolated')
        self.assertAlmostEqual(value['hip'],.5)
        self.assertEqual(history.sample(1200000000,allow_hold=True),(None,'missing'))
        self.assertEqual(history.sample(990000000,allow_hold=True),(None,'missing'))
        self.assertEqual(history.sample(1060000000,allow_hold=True)[1],'bounded_past')
        # A large gap must not be bridged, even if measurements eventually arrive.
        history.add(1400000000,{'hip':2.0})
        self.assertEqual(history.sample(1225000000),(None,'missing'))


if __name__ == '__main__':
    unittest.main()
