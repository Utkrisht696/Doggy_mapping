"""URDF collision-volume filtering and bounded, scan-time joint interpolation.

This module has no ROS dependency. It never erases a radius around the robot:
only points inside its posed primitive collision shapes plus padding are removed.
"""
from bisect import bisect_left
import math
import xml.etree.ElementTree as ET

import numpy as np


def rotation_rpy(rpy):
    r,p,y = rpy
    sr,cr,sp,cp,sy,cy = math.sin(r),math.cos(r),math.sin(p),math.cos(p),math.sin(y),math.cos(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def quaternion_rotation(quaternion):
    q = np.asarray(quaternion,dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or not .95 <= norm <= 1.05:
        raise ValueError('Invalid source pose quaternion')
    x,y,z,w = q/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def origin_matrix(element):
    out = np.eye(4)
    if element is not None:
        out[:3,3] = list(map(float,element.get('xyz','0 0 0').split()))
        out[:3,:3] = rotation_rpy(list(map(float,element.get('rpy','0 0 0').split())))
    if not np.isfinite(out).all():
        raise ValueError('Nonfinite URDF origin')
    return out


class RobotSelfFilter:
    def __init__(self, description, padding=0.025):
        if not math.isfinite(padding) or not 0 <= padding <= .05:
            raise ValueError('Self-filter padding must be between 0 and 5 cm')
        self.padding = padding
        model = ET.fromstring(description)
        links = {link.get('name') for link in model.findall('link')}
        parents = {}
        self.joints = []
        for joint in model.findall('joint'):
            parent,child = joint.find('parent').get('link'),joint.find('child').get('link')
            if child in parents or parent not in links or child not in links:
                raise ValueError('Invalid URDF hierarchy')
            parents[child] = parent
            kind = joint.get('type')
            if kind not in ('fixed','revolute','continuous','prismatic') or joint.find('mimic') is not None:
                raise ValueError('Unsupported URDF joint')
            axis = np.array(list(map(float,joint.find('axis').get('xyz','1 0 0').split()))) if joint.find('axis') is not None else np.array([1.,0,0])
            if kind != 'fixed':
                if not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
                    raise ValueError('Invalid URDF joint axis')
                axis = axis/np.linalg.norm(axis)
            self.joints.append((joint.get('name'),kind,parent,child,origin_matrix(joint.find('origin')),axis))
        roots = links-set(parents)
        if len(roots) != 1 or next(iter(roots)) not in ('base','base_link'):
            raise ValueError('Expected a Go2 URDF rooted at base or base_link')
        self.root = next(iter(roots))
        self.required_joints = {name for name,kind,*_ in self.joints if kind != 'fixed'}
        self.collisions = []
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('Invalid collision geometry')
                shape = geometry[0]
                if shape.tag == 'box':
                    dimensions = np.array(list(map(float,shape.get('size').split())))
                    if dimensions.shape != (3,):
                        raise ValueError('Expected XYZ box dimensions')
                elif shape.tag == 'cylinder':
                    dimensions = np.array([float(shape.get('radius')),float(shape.get('length'))])
                elif shape.tag == 'sphere':
                    dimensions = np.array([float(shape.get('radius'))])
                else:
                    raise ValueError('Self-filter needs primitive URDF collision shapes')
                if not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
                    raise ValueError('Invalid collision dimensions')
                self.collisions.append((link.get('name'),origin_matrix(collision.find('origin')),shape.tag,dimensions))
        if not self.collisions:
            raise ValueError('No robot collision shapes')
        # Only the fixed developer body-to-IMU chain is used for this mount.
        static = {self.root:np.eye(4)}
        for _ in self.joints:
            for _,kind,parent,child,transform,_ in self.joints:
                if kind == 'fixed' and parent in static:
                    static[child] = static[parent]@transform
        if 'imu' not in static:
            raise ValueError('Missing fixed body-to-IMU mount')
        self.imu_mount = static['imu']

    def link_transforms(self, positions):
        if not self.required_joints.issubset(positions):
            raise ValueError('Missing measured joint positions for robot self-filter')
        poses = {self.root:np.eye(4)}
        pending = list(self.joints)
        while pending:
            progressed = False
            for joint in list(pending):
                name,kind,parent,child,fixed,axis = joint
                if parent not in poses:
                    continue
                motion = np.eye(4)
                if kind != 'fixed':
                    angle = float(positions[name])
                    if not math.isfinite(angle):
                        raise ValueError('Nonfinite joint position')
                    if kind == 'prismatic':
                        motion[:3,3] = angle*axis
                    else:
                        x,y,z = axis
                        skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                        motion[:3,:3] = np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)
                poses[child] = poses[parent]@fixed@motion
                pending.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError('URDF hierarchy contains a cycle')
        return poses

    def world_to_body(self, points, imu_position, imu_quaternion):
        p = np.asarray(points,dtype=float)
        origin = np.asarray(imu_position,dtype=float)
        if p.ndim != 2 or p.shape[1] != 3 or origin.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(origin).all():
            raise ValueError('Expected finite world points and IMU position')
        imu_points = (p-origin)@quaternion_rotation(imu_quaternion)
        return imu_points@self.imu_mount[:3,:3].T+self.imu_mount[:3,3]

    def keep_world(self, points, imu_position, imu_quaternion, positions):
        body = self.world_to_body(points,imu_position,imu_quaternion)
        transforms = self.link_transforms(positions)
        remove = np.zeros(len(body),dtype=bool)
        pad = self.padding
        for link,offset,kind,dimensions in self.collisions:
            transform = transforms[link]@offset
            local = (body-transform[:3,3])@transform[:3,:3]
            if kind == 'box':
                inside = (np.abs(local) <= dimensions/2+pad).all(axis=1)
            elif kind == 'cylinder':
                inside = ((np.linalg.norm(local[:,:2],axis=1) <= dimensions[0]+pad)
                          & (np.abs(local[:,2]) <= dimensions[1]/2+pad))
            else:
                inside = np.linalg.norm(local,axis=1) <= dimensions[0]+pad
            remove |= inside
        return ~remove


class JointHistory:
    def __init__(self, max_gap=0.15, max_age=0.075, capacity=2048):
        if not 0 < max_age <= max_gap <= .5 or capacity < 2:
            raise ValueError('Invalid joint timestamp bounds')
        self.max_gap_ns = round(max_gap*1e9)
        self.max_age_ns = round(max_age*1e9)
        self.capacity = capacity
        self.stamps,self.values = [],[]

    def add(self, stamp, positions):
        if stamp <= 0 or not positions or not all(math.isfinite(float(v)) for v in positions.values()):
            raise ValueError('Invalid timestamped joint sample')
        index = bisect_left(self.stamps,stamp)
        if index < len(self.stamps) and self.stamps[index] == stamp:
            self.values[index] = dict(positions)
        else:
            self.stamps.insert(index,stamp)
            self.values.insert(index,dict(positions))
        if len(self.stamps) > self.capacity:
            del self.stamps[:len(self.stamps)-self.capacity]
            del self.values[:len(self.values)-self.capacity]

    def sample(self, stamp, allow_hold=False):
        index = bisect_left(self.stamps,stamp)
        if index < len(self.stamps) and self.stamps[index] == stamp:
            return self.values[index], 'exact'
        if 0 < index < len(self.stamps):
            left,right = self.stamps[index-1:index+1]
            a,b = self.values[index-1:index+1]
            if (right-left <= self.max_gap_ns and stamp-left <= self.max_age_ns
                    and right-stamp <= self.max_age_ns and a.keys() == b.keys()):
                ratio = (stamp-left)/(right-left)
                return {key:a[key]+ratio*(b[key]-a[key]) for key in a}, 'interpolated'
        # At the recording tail or a brief transport delay, hold only a bounded
        # past sample. Never use future-only poses or stale/default joint angles.
        if allow_hold and index > 0 and stamp-self.stamps[index-1] <= self.max_age_ns:
            return self.values[index-1], 'bounded_past'
        return None, 'missing'
