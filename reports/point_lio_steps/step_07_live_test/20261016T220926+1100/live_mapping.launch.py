"""Go2 Point-LIO output integration. Mapping is disabled unless explicitly enabled.

Use the existing sensor/IMU bringup live, or recorded TF/model/IMU in an isolated
bag environment. This launch creates no sensor adapter, clock, URDF or controls.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    root = Path('/home/jain/Doggy')
    clock_remap = ('/clock', '/doggy/robot_clock')
    mapping_enabled = LaunchConfiguration('start_mapping')
    rviz_view = PythonExpression([
        "'", str(root/'config/point_lio/go2_point_lio.rviz'), "' if '",
        mapping_enabled, "'.lower() == 'true' else '",
        str(root/'config/point_lio/go2_point_lio_preview.rviz'), "'"])
    mapper = Node(
        package='point_lio', executable='pointlio_mapping', name='doggy_point_lio',
        condition=IfCondition(LaunchConfiguration('start_mapping')), output='screen',
        parameters=[str(root/'config/point_lio/go2_body_imu.yaml'), '/home/jain/Doggy/reports/point_lio_steps/step_07_live_test/20261016T220926+1100/denser_seed_trial.yaml', {
            'use_sim_time': True,
            'odom_only': False,
            'odom_header_frame_id': 'map', 'odom_child_frame_id': 'imu',
            'odometry.publish_odometry_without_downsample': False,
        }],
        remappings=[clock_remap, ('/tf', '/point_lio/raw_tf'),
            ('/tf_static', '/point_lio/raw_tf_static'),
            ('/aft_mapped_to_init', '/point_lio/odometry'),
            ('/cloud_registered', '/point_lio/cloud_registered'),
            ('/cloud_registered_body', '/point_lio/cloud_registered_body'),
            ('/cloud_effected', '/point_lio/cloud_effected'),
            ('/Laser_map', '/point_lio/initial_map'), ('/path', '/point_lio/path')])
    bridge = ExecuteProcess(cmd=[
        'python3', '-u', str(root/'scripts/publish_point_lio_tf.py'),
        '--ros-args', '-p', 'use_sim_time:=true', '-r', '/clock:=/doggy/robot_clock'], output='screen')
    rviz = Node(package='rviz2', executable='rviz2', name='doggy_point_lio_rviz',
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
        arguments=['-d', rviz_view,
                   '-f', LaunchConfiguration('fixed_frame')],
        parameters=[{'use_sim_time': True}], remappings=[clock_remap])
    actions = [
        DeclareLaunchArgument('start_mapping', default_value='false',
            description='Explicit opt-in to start the estimator; false previews configuration only'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open the prepared RViz view'),
        DeclareLaunchArgument('fixed_frame', default_value=PythonExpression([
            "'map' if '", mapping_enabled, "'.lower() == 'true' else 'odom'"]),
            description='Automatically odom for preview and map for mapping; may be overridden'),
        bridge, mapper, rviz]
    # Closing RViz or losing either essential process stops this launch as a unit.
    for process in (bridge, mapper, rviz):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Point-LIO launch component stopped'))])))
    return LaunchDescription(actions)
