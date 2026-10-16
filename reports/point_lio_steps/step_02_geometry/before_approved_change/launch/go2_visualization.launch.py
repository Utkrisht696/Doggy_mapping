"""Official Go2 URDF + encoder telemetry + live LiDAR in RViz."""
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'scripts'))
from go2_frames import canonical_robot_description


def generate_launch_description():
    root=Path(__file__).resolve().parent.parent
    share=Path(get_package_share_directory('go2_description'))
    description=canonical_robot_description((share/'urdf/go2_description.urdf').read_text())
    # RViz launched from the VS Code Snap must use host GTK/GIO plugins.
    gui_env={key:value for key,value in os.environ.items() if key not in {
        'GTK_PATH','GTK_EXE_PREFIX','GTK_DATA_PREFIX','GTK_IM_MODULE_FILE',
        'GIO_MODULE_DIR','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH','LD_PRELOAD'}}
    state=ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_go2_state.py'),
        '--ros-args','-p',['footprint_ground_z:=',LaunchConfiguration('footprint_ground_z')]],output='screen')
    rviz=ExecuteProcess(cmd=['rviz2','-d',str(root/'config/lidar.rviz'),'--ros-args','-p','use_sim_time:=true',
                            '-r','/clock:=/doggy/robot_clock'],env=gui_env,output='screen',condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('footprint_ground_z',default_value='0.0',description='Horizontal reference-plane Z in odom, in metres'),
        state,
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='go2_robot_state_publisher',
             parameters=[{'robot_description':description,'use_sim_time':True,'publish_frequency':50.0}],
             remappings=[('/clock','/doggy/robot_clock')],output='screen'),
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz,on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=state,on_exit=[EmitEvent(event=Shutdown(reason='State adapter stopped'))])),
    ])
