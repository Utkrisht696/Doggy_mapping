"""Point-LIO XY/orientation plus causal vendor-height fusion and 2D mapping."""
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
    description = canonical_robot_description((share/'urdf/go2_description.urdf').read_text(),
                                               root/'config/point_lio/sensor_geometry.json')
    imu = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_body_imu.py'),
        '--calibration-file',str(root/'config/point_lio/body_imu_calibration.json')],output='screen',sigterm_timeout='20')
    state = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_sensor_state.py')],output='screen')
    model = Node(package='robot_state_publisher',executable='robot_state_publisher',
        name='go2_robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}],
        remappings=clock,output='screen')
    mapper = Node(package='point_lio',executable='pointlio_mapping',name='doggy_point_lio',
        parameters=[str(root/'config/point_lio/go2_body_imu.yaml'),{
            'use_sim_time':True,'init_map_size':4000,'odom_only':False,
            'odom_header_frame_id':'lio_local','odom_child_frame_id':'imu',
            'publish.path_en':False,
            'odometry.publish_odometry_without_downsample':False}],
        remappings=clock+[
            ('/tf','/point_lio/raw_tf'),('/tf_static','/point_lio/raw_tf_static'),
            ('/aft_mapped_to_init','/point_lio/raw/odometry'),
            ('/cloud_registered','/point_lio/raw/cloud_registered'),
            ('/cloud_registered_body','/point_lio/cloud_registered_body'),
            ('/cloud_effected','/point_lio/raw/cloud_effected'),
            ('/Laser_map','/point_lio/raw/initial_map'),('/path','/point_lio/raw/path')],output='screen')
    fusion = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_height_fusion.py'),
        *ros_time,'--params-file',str(root/'config/point_lio/height_fusion.yaml'),
        '-p','output_frame:=odom'],output='screen')
    odom = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_lio_odometry.py'),*ros_time],output='screen')
    terrain = ExecuteProcess(cmd=['python3','-u',str(root/'scripts/publish_terrain_map.py'),
        *ros_time,'--params-file',str(root/'config/terrain_mapping.yaml'),
        '-p',['output_map:=',LaunchConfiguration('output_map')]],output='screen',sigterm_timeout='30')
    rviz = Node(package='rviz2',executable='rviz2',name='doggy_live_2d_rviz',
        arguments=['-d',LaunchConfiguration('rviz_config')],parameters=[{'use_sim_time':True}],
        remappings=clock,condition=IfCondition(LaunchConfiguration('rviz')),output='screen')
    actions = [DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('rviz_config',default_value=str(root/'config/live_walls.rviz')),
        DeclareLaunchArgument('output_map',default_value=str(root/'maps'/datetime.now().strftime('terrain_%Y%m%d_%H%M%S')/'map')),
        imu,state,model,mapper,fusion,odom,terrain,rviz]
    for process in (imu,state,model,mapper,fusion,odom,terrain,rviz):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Live 2D mapping component stopped'))])))
    return LaunchDescription(actions)
