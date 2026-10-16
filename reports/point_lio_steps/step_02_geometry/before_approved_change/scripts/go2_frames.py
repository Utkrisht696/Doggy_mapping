"""Canonical body-frame description and an exact planar/body pose decomposition.

The developer URDF stays on disk unchanged. Only the root link name and its
references are normalized in the robot_description supplied to ROS.
"""
import math
import xml.etree.ElementTree as ET

ODOM_FRAME = 'odom'
FOOTPRINT_FRAME = 'base_footprint'
BODY_FRAME = 'base_link'
DEVELOPER_BODY_FRAME = 'base'


def canonical_robot_description(source_xml):
    model = ET.fromstring(source_xml)
    links = {link.attrib['name'] for link in model.findall('link')}
    children = {joint.find('child').attrib['link'] for joint in model.findall('joint')}
    if links - children != {DEVELOPER_BODY_FRAME} or BODY_FRAME in links:
        raise ValueError('Expected the untouched developer URDF with root base')
    model.find("link[@name='base']").set('name', BODY_FRAME)
    for joint in model.findall('joint'):
        for tag in ('parent', 'child'):
            endpoint = joint.find(tag)
            if endpoint.get('link') == DEVELOPER_BODY_FRAME:
                endpoint.set('link', BODY_FRAME)
    return ET.tostring(model, encoding='unicode')


def split_body_pose(position, quaternion, ground_z):
    """Return odom->footprint and footprint->body; their product is the input.

    ground_z is the height of the chosen horizontal reference plane in odom.
    The footprint retains XY and yaw; the body retains height, roll and pitch.
    No nominal body height, motion filtering or pose correction is introduced.
    """
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError('Expected XYZ and quaternion XYZW')
    if not all(math.isfinite(value) for value in (*position, *quaternion, ground_z)):
        raise ValueError('Pose and reference-plane height must be finite')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-6:
        raise ValueError('Invalid body quaternion')
    x, y, z, w = (value / norm for value in quaternion)
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    # inverse(yaw quaternion) * full body quaternion.
    body_rotation = (c*x + s*y, c*y - s*x, c*z - s*w, c*w + s*z)
    return ((position[0], position[1], ground_z), (0.0, 0.0, s, c),
            (0.0, 0.0, position[2] - ground_z), body_rotation)
