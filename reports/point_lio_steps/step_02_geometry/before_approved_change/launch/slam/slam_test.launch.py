"""Mapping-only tests with official URDF, robot clock and optional Toolbox RViz."""
import math
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from go2_frames import canonical_robot_description


def start(context):
    root=Path(__file__).resolve().parents[2]
    algorithm=LaunchConfiguration('algorithm').perform(context)
    if algorithm != 'slam_toolbox':
        raise ValueError('Only slam_toolbox is supported here. See POINT_LIO_SETUP.md for standalone Point-LIO readiness.')
    show_rviz=LaunchConfiguration('rviz').perform(context).lower() in {'true','1','yes'}
    clock_remap=[('/clock','/doggy/robot_clock')]
    share=Path(get_package_share_directory('go2_description'))
    state=ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_go2_state.py'),
        '--ros-args','-p',['footprint_ground_z:=',LaunchConfiguration('footprint_ground_z')]],output='screen')
    actions=[state,
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='go2_robot_state_publisher',
             parameters=[{'robot_description':canonical_robot_description((share/'urdf/go2_description.urdf').read_text()),'use_sim_time':True}],
             remappings=clock_remap,output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=state,on_exit=[EmitEvent(event=Shutdown(reason='Robot state/clock adapter stopped'))]))]
    if algorithm=='slam_toolbox':
        projection=Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='go2_cloud_to_scan',
            parameters=[{'use_sim_time':True,'target_frame':'base_link','queue_size':5,'min_height':-0.15,'max_height':1.5,
                         'angle_min':-math.pi,'angle_max':math.pi,'angle_increment':math.pi/180,
                         'scan_time':1.0/15.5,'range_min':0.4,'range_max':12.0,'use_inf':True}],
            remappings=clock_remap+[('cloud_in','/utlidar/cloud_deskewed'),('scan','/tests/slam_toolbox/scan')],output='screen')
        mapper=Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
                    parameters=[str(root/'config/slam/slam_toolbox.yaml')],remappings=clock_remap,output='screen')
        actions.append(TimerAction(period=2.0,actions=[projection,mapper]))
    actions.append(RegisterEventHandler(OnProcessExit(target_action=mapper,on_exit=[EmitEvent(event=Shutdown(reason='Mapper stopped'))])))
    if show_rviz:
        # Use host Qt/GTK plugins when invoked from the VS Code Snap.
        gui_env={key:value for key,value in os.environ.items() if key not in {
            'GTK_PATH','GTK_EXE_PREFIX','GTK_DATA_PREFIX','GTK_IM_MODULE_FILE',
            'GIO_MODULE_DIR','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH','LD_PRELOAD'}}
        actions.append(TimerAction(period=3.0,actions=[ExecuteProcess(
            cmd=['rviz2','-d',str(root/'config/slam/slam_toolbox.rviz'),
                 '--ros-args','-p','use_sim_time:=true','-r','/clock:=/doggy/robot_clock'],
            env=gui_env,output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('algorithm',default_value='slam_toolbox'),
        DeclareLaunchArgument('footprint_ground_z',default_value='0.0',description='Horizontal reference-plane Z in odom, in metres'),
        DeclareLaunchArgument('rviz',default_value='false',description='Open the SLAM Toolbox map/scan view'),
        OpaqueFunction(function=start),
    ])
