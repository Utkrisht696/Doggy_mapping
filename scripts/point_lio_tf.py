"""Rigid-pose composition for the Go2 Point-LIO output contract (XYZ, XYZW)."""
import math


def checked_pose(position, quaternion):
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError('Expected position XYZ and quaternion XYZW')
    if not all(math.isfinite(v) for v in (*position, *quaternion)):
        raise ValueError('Nonfinite pose')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if not 0.95 <= norm <= 1.05:
        raise ValueError('Quaternion outside normalization range')
    return tuple(position), tuple(v/norm for v in quaternion)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rotate(q, p):
    v = multiply(multiply(q, (*p, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return v[:3]


def map_to_odom(map_imu_position, map_imu_quaternion,
                odom_imu_position, odom_imu_quaternion):
    """T_map_odom = T_map_imu * inverse(T_odom_imu), without dropping tilt/height."""
    mp, mq = checked_pose(map_imu_position, map_imu_quaternion)
    op, oq = checked_pose(odom_imu_position, odom_imu_quaternion)
    q = multiply(mq, (-oq[0], -oq[1], -oq[2], oq[3]))
    offset = rotate(q, op)
    return checked_pose(tuple(mp[i]-offset[i] for i in range(3)), q)
