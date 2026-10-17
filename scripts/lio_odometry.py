"""Point-LIO world/IMU pose and mixed-frame twist to ROS body odometry."""
from point_lio_tf import checked_pose, multiply, rotate


def inverse_quaternion(q):
    return (-q[0], -q[1], -q[2], q[3])


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def body_odometry(position, quaternion, world_velocity, imu_omega, mount_position, mount_quaternion):
    """Mount is T_base_imu. Point-LIO linear velocity is world expressed.

    Its estimated angular velocity is IMU expressed. Account for both the
    orientation and velocity lever arm when changing the reference origin.
    """
    p, q = checked_pose(position, quaternion)
    r, mount_q = checked_pose(mount_position, mount_quaternion)
    body_q = multiply(q, inverse_quaternion(mount_q))
    offset = rotate(body_q, r)
    body_p = tuple(p[i]-offset[i] for i in range(3))
    omega = rotate(mount_q, imu_omega)
    at_imu = rotate(inverse_quaternion(body_q), world_velocity)
    lever = cross(omega, r)
    velocity = tuple(at_imu[i]-lever[i] for i in range(3))
    return body_p, body_q, velocity, omega
