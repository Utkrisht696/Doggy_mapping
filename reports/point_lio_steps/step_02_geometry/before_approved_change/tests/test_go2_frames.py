"""Verify physical-pose preservation and unchanged developer robot geometry."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from go2_frames import canonical_robot_description, split_body_pose


class FrameTests(unittest.TestCase):
    def test_pose_composition_for_full_body_motion(self):
        rng = np.random.default_rng(736)
        for _ in range(300):
            position = rng.uniform(-5, 5, 3)
            q = Rotation.random(random_state=rng).as_quat()
            ground = float(rng.uniform(-2, 2))
            fp, fq, bp, bq = split_body_pose(position, q * 1.013, ground)
            combined_position = np.array(fp) + Rotation.from_quat(fq).apply(bp)
            combined_rotation = Rotation.from_quat(fq) * Rotation.from_quat(bq)
            np.testing.assert_allclose(combined_position, position, atol=1e-12)
            np.testing.assert_allclose(combined_rotation.as_matrix(), Rotation.from_quat(q).as_matrix(), atol=1e-12)
            np.testing.assert_allclose(Rotation.from_quat(fq).as_matrix()[:, 2], [0, 0, 1], atol=1e-12)
            self.assertEqual(fp[2], ground)
            self.assertAlmostEqual(bp[2], position[2] - ground)

    def test_heading_wrap_does_not_change_the_composed_pose(self):
        for yaw in (-math.pi, math.pi, -math.pi + 1e-9, math.pi - 1e-9):
            q = Rotation.from_euler('xyz', [0.3, -0.4, yaw]).as_quat()
            _, fq, _, bq = split_body_pose([0, 0, 0.31], q, 0)
            combined = Rotation.from_quat(fq) * Rotation.from_quat(bq)
            np.testing.assert_allclose(combined.as_matrix(), Rotation.from_quat(q).as_matrix(), atol=1e-12)

    def test_only_root_link_name_changes(self):
        source = (ROOT / 'sdk/unitree_ros/robots/go2_description/urdf/go2_description.urdf').read_text()
        original = ET.fromstring(source)
        canonical = ET.fromstring(canonical_robot_description(source))
        self.assertEqual(len(canonical.findall('link')), 42)
        self.assertIsNone(canonical.find("link[@name='base']"))
        self.assertIsNotNone(canonical.find("link[@name='base_link']"))
        # Reverse only the approved symbolic rename. Every remaining element,
        # mesh URI, joint origin, inertia and numeric attribute must be identical.
        canonical.find("link[@name='base_link']").set('name', 'base')
        for joint in canonical.findall('joint'):
            for tag in ('parent', 'child'):
                endpoint = joint.find(tag)
                if endpoint.get('link') == 'base_link':
                    endpoint.set('link', 'base')
        def structure(element):
            return (element.tag, element.attrib, (element.text or '').strip(),
                    [structure(child) for child in element])
        self.assertEqual(structure(canonical), structure(original))

    def test_invalid_poses_are_rejected(self):
        for position, q, ground in (([0, 0, 0], [0, 0, 0, 0], 0),
                                     ([0, float('nan'), 0], [0, 0, 0, 1], 0),
                                     ([0, 0, 0], [0, 0, 0, 1], float('inf'))):
            with self.assertRaises(ValueError):
                split_body_pose(position, q, ground)


if __name__ == '__main__':
    unittest.main()
