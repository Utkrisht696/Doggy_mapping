"""Sensor preparation and official robot TF only. No SLAM, RViz or controls."""
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'scripts'))
from go2_frames import canonical_robot_description


def generate_launch_description():
    root=Path(__file__).resolve().parent.parent
    share=Path(get_package_share_directory('go2_description'))
    state=ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_go2_state.py'),
        '--ros-args','-p',['footprint_ground_z:=',LaunchConfiguration('footprint_ground_z')]],output='screen')
    model=Node(package='robot_state_publisher',executable='robot_state_publisher',name='go2_robot_state_publisher',
        parameters=[{'robot_description':canonical_robot_description((share/'urdf/go2_description.urdf').read_text(), root/'config/point_lio/sensor_geometry.json'),
                     'use_sim_time':True,'publish_frequency':50.0}],
        remappings=[('/clock','/doggy/robot_clock')],output='screen')
    clouds=ExecuteProcess(cmd=['python3','-u',str(root/'scripts/prepare_lidar_clouds.py'),
                              '--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock'],output='screen')
    actions=[DeclareLaunchArgument('footprint_ground_z',default_value='0.0',description='Horizontal reference-plane Z in odom, in metres'),state,model,clouds]
    for process in (state,model,clouds):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Sensor preparation process stopped'))])))
    return LaunchDescription(actions)
