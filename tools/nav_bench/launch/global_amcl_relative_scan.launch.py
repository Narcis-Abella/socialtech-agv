"""Stationary exploratory AMCL global-localization path without absolute sensor height.

Only suitable for S1/S2 development. The laser band is swept relative to the
sensor origin; the published frame is a planar sensor-origin proxy, not a
physically calibrated base_footprint. Do not use this launch for formal scores.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOOLS=os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    map_yaml=LaunchConfiguration('map')
    height=LaunchConfiguration('sensor_height_hypothesis_m')
    input_start=LaunchConfiguration('input_start_ns')
    input_end=LaunchConfiguration('input_end_ns')
    fit_end=LaunchConfiguration('fit_end_ns')
    imu_median=LaunchConfiguration('imu_median_ns')
    return LaunchDescription([
        DeclareLaunchArgument('map',default_value=os.environ.get('GLOBAL_RELOC_MAP','/map/map.yaml')),
        DeclareLaunchArgument('sensor_height_hypothesis_m',
                              default_value=os.environ.get('SENSOR_HEIGHT_HYPOTHESIS_M','1.0')),
        DeclareLaunchArgument('input_start_ns',default_value=os.environ.get('INPUT_START_NS','0')),
        DeclareLaunchArgument('input_end_ns',default_value=os.environ.get('INPUT_END_NS','9223372036854775807')),
        DeclareLaunchArgument('fit_end_ns',default_value=os.environ.get('FIT_END_NS','9223372036854775807')),
        DeclareLaunchArgument('imu_median_ns',default_value=os.environ.get('IMU_MEDIAN_NS','0')),
        Node(package='tf2_ros',executable='static_transform_publisher',name='stationary_odom_tf',
             arguments=['0','0','0','0','0','0','odom','livox_level'],
             parameters=[{'use_sim_time':True}],output='screen'),
        ExecuteProcess(cmd=['python3',os.path.join(TOOLS,'sensor_relative_scan.py'),
                            '--ros-args','-p',['sensor_height_hypothesis_m:=',height],
                            '-p',['input_start_ns:=',input_start],'-p',['input_end_ns:=',input_end],
                            '-p',['fit_end_ns:=',fit_end],'-p',['imu_median_ns:=',imu_median]],
                       name='sensor_relative_scan',output='screen'),
        ExecuteProcess(cmd=['python3',os.path.join(TOOLS,'amcl_ros_observer.py'),
                            '--ros-args','-p','scenario:=S2',
                            '-p','output_jsonl:=/tmp/amcl-observer.jsonl'],
                       name='amcl_acceptance_observer',output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',
             parameters=[{'yaml_filename':map_yaml,'use_sim_time':True}],output='screen'),
        Node(package='nav2_amcl',executable='amcl',name='amcl',
             parameters=[os.path.join(TOOLS,'config/amcl.yaml'),
                         {'use_sim_time':True,'base_frame_id':'livox_level','odom_frame_id':'odom',
                          'global_frame_id':'map','scan_topic':'/scan','set_initial_pose':False,
                          'save_pose_rate':-1.0}],output='screen'),
        TimerAction(period=2.0,actions=[
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',
                 parameters=[{'autostart':True,'node_names':['map_server','amcl'],'use_sim_time':True}],output='screen')]),
    ])
