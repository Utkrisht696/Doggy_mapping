"""Isolated recorded-output replay into the live terrain mapper and 2D view."""
from datetime import datetime
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from go2_frames import canonical_robot_description


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    clock = [('/clock','/doggy/robot_clock')]
    ros_time = ['--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock']
    share = Path(get_package_share_directory('go2_description'))
    description = canonical_robot_description((share/'urdf/go2_description.urdf').read_text(),root/'config/point_lio/sensor_geometry.json')
    model = Node(package='robot_state_publisher',executable='robot_state_publisher',name='go2_replay_robot_state_publisher',
        parameters=[{'robot_description':description,'use_sim_time':True}],remappings=clock,output='screen')
    replay = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/replay_lio_outputs.py'),
        '--bag',LaunchConfiguration('bag'),'--rate',LaunchConfiguration('rate')],output='screen')
    fusion = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_height_fusion.py'),
        *ros_time,'--params-file',str(root/'config/point_lio/height_fusion.yaml'),
        '-p','output_frame:=odom'],output='screen')
    odom = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_lio_odometry.py'),*ros_time],output='screen')
    terrain = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_terrain_map.py'),
        *ros_time,'--params-file',str(root/'config/terrain_mapping.yaml'),
        '-p',['output_map:=',LaunchConfiguration('output_map')]],output='screen',sigterm_timeout='30')
    rviz = Node(package='rviz2',executable='rviz2',name='doggy_replay_2d_rviz',
        arguments=['-d',LaunchConfiguration('rviz_config')],parameters=[{'use_sim_time':True}],remappings=clock,
        condition=IfCondition(LaunchConfiguration('rviz')),output='screen')
    actions = [DeclareLaunchArgument('bag',default_value=str(root/'reports/point_lio_steps/step_07_live_test/accuracy_20261016T224027+1100/bag')),
        DeclareLaunchArgument('rate',default_value='1.0'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('rviz_config',default_value=str(root/'config/live_walls.rviz')),
        DeclareLaunchArgument('output_map',default_value=str(root/'maps'/datetime.now().strftime('replay_%Y%m%d_%H%M%S')/'map')),
        model,fusion,odom,terrain,replay,rviz]
    for process in (model,fusion,odom,terrain,replay,rviz):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='2D replay component stopped'))])))
    return LaunchDescription(actions)
