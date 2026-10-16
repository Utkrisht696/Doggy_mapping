"""Canonical body-frame description and an exact planar/body pose decomposition.

The developer URDF stays on disk unchanged. Its root name is normalized and an
approved, topic-derived raw measurement frame can be appended at runtime.
"""
import math
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ODOM_FRAME = 'odom'
FOOTPRINT_FRAME = 'base_footprint'
BODY_FRAME = 'base_link'
DEVELOPER_BODY_FRAME = 'base'


def canonical_robot_description(source_xml, measurement_profile=None):
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
    if measurement_profile is not None:
        profile = json.loads(Path(measurement_profile).read_text())
        if profile.get('approved') is not True:
            raise ValueError('Measurement geometry requires explicit approval')
        if hashlib.sha256(source_xml.encode()).hexdigest() != profile.get('source_urdf_sha256'):
            raise ValueError('Developer URDF differs from the reviewed geometry source')
        edge = profile['measurement_tf']
        if edge['parent'] != 'radar' or edge['child'] != 'utlidar_lidar':
            raise ValueError('Unexpected measurement-frame contract')
        joint_name = 'radar_to_utlidar_lidar_measurement'
        if ('radar' not in links or edge['child'] in links
                or model.find(f"joint[@name='{joint_name}']") is not None):
            raise ValueError('Missing parent or duplicate measurement link/joint')
        for key in ('translation_m', 'rpy_radians'):
            values = edge[key]
            if len(values) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                raise ValueError(f'Invalid measurement origin {key}')
        ET.SubElement(model, 'link', name=edge['child'])
        joint = ET.SubElement(model, 'joint', name=joint_name, type='fixed')
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, edge['translation_m'])),
                      rpy=' '.join(map(str, edge['rpy_radians'])))
        ET.SubElement(joint, 'parent', link=edge['parent'])
        ET.SubElement(joint, 'child', link=edge['child'])
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
