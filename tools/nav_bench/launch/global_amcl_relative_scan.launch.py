"""Stationary exploratory AMCL global-localization path without absolute sensor height.

Only suitable for S1/S2 development. The laser band is swept relative to the
sensor origin; the published frame is a planar sensor-origin proxy, not a
physically calibrated base_footprint. Do not use this launch for formal scores.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

TOOLS=os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    scenario=LaunchConfiguration('scenario')
    map_yaml=LaunchConfiguration('map')
    height=LaunchConfiguration('sensor_height_hypothesis_m')
    input_start=LaunchConfiguration('input_start_ns')
    input_end=LaunchConfiguration('input_end_ns')
    fit_end=LaunchConfiguration('fit_end_ns')
    imu_median=LaunchConfiguration('imu_median_ns')
    scan_fov=LaunchConfiguration('fov_deg')
    scan_center=LaunchConfiguration('fov_center_deg')
    scan_increment=LaunchConfiguration('angle_increment_deg')
    scan_min=LaunchConfiguration('range_min')
    scan_max=LaunchConfiguration('range_max')
    input_reliability=LaunchConfiguration('input_reliability')
    sensor_band=LaunchConfiguration('sensor_relative_band_m')
    obstacle_min=LaunchConfiguration('min_obstacle_height')
    obstacle_max=LaunchConfiguration('max_obstacle_height')
    particle_min=LaunchConfiguration('min_particles')
    particle_max=LaunchConfiguration('max_particles')
    max_beams=LaunchConfiguration('max_beams')
    recovery_fast=LaunchConfiguration('recovery_alpha_fast')
    recovery_slow=LaunchConfiguration('recovery_alpha_slow')
    nomotion_trigger=LaunchConfiguration('nomotion_trigger')
    prescan_relay=os.environ.get('GLOBAL_RELOC_PRESCAN_RELAY','false').lower()=='true'
    return LaunchDescription([
        DeclareLaunchArgument('scenario',default_value=os.environ.get('GLOBAL_RELOC_SCENARIO','S2')),
        DeclareLaunchArgument('map',default_value=os.environ.get('GLOBAL_RELOC_MAP','/map/map.yaml')),
        DeclareLaunchArgument('sensor_height_hypothesis_m',
                              default_value=os.environ.get('SENSOR_HEIGHT_HYPOTHESIS_M','1.0')),
        DeclareLaunchArgument('input_start_ns',default_value=os.environ.get('INPUT_START_NS','0')),
        DeclareLaunchArgument('input_end_ns',default_value=os.environ.get('INPUT_END_NS','9223372036854775807')),
        DeclareLaunchArgument('fit_end_ns',default_value=os.environ.get('FIT_END_NS','9223372036854775807')),
        DeclareLaunchArgument('imu_median_ns',default_value=os.environ.get('IMU_MEDIAN_NS','0')),
        DeclareLaunchArgument('fov_deg',default_value=os.environ.get('SCAN_FOV_DEG','360.0')),
        DeclareLaunchArgument('fov_center_deg',default_value=os.environ.get('SCAN_FOV_CENTER_DEG','0.0')),
        DeclareLaunchArgument('angle_increment_deg',default_value=os.environ.get('SCAN_ANGLE_INCREMENT_DEG','0.5')),
        DeclareLaunchArgument('range_min',default_value=os.environ.get('SCAN_RANGE_MIN','0.5')),
        DeclareLaunchArgument('range_max',default_value=os.environ.get('SCAN_RANGE_MAX','30.0')),
        DeclareLaunchArgument('input_reliability',default_value=os.environ.get('SCAN_INPUT_RELIABILITY','best_effort')),
        DeclareLaunchArgument('sensor_relative_band_m',default_value=os.environ.get('SENSOR_RELATIVE_BAND_M','none')),
        DeclareLaunchArgument('min_obstacle_height',default_value=os.environ.get('MIN_OBSTACLE_HEIGHT','0.10')),
        DeclareLaunchArgument('max_obstacle_height',default_value=os.environ.get('MAX_OBSTACLE_HEIGHT','1.80')),
        DeclareLaunchArgument('min_particles',default_value=os.environ.get('AMCL_MIN_PARTICLES','2000')),
        DeclareLaunchArgument('max_particles',default_value=os.environ.get('AMCL_MAX_PARTICLES','2000')),
        DeclareLaunchArgument('max_beams',default_value=os.environ.get('AMCL_MAX_BEAMS','180')),
        DeclareLaunchArgument('recovery_alpha_fast',default_value=os.environ.get('AMCL_RECOVERY_ALPHA_FAST','0.0')),
        DeclareLaunchArgument('recovery_alpha_slow',default_value=os.environ.get('AMCL_RECOVERY_ALPHA_SLOW','0.0')),
        DeclareLaunchArgument('nomotion_trigger',default_value=os.environ.get('NOMOTION_TRIGGER','scan_delay')),
        Node(package='tf2_ros',executable='static_transform_publisher',name='stationary_odom_tf',
             arguments=['0','0','0','0','0','0','odom','livox_level'],
             parameters=[{'use_sim_time':True}],output='screen'),
        ExecuteProcess(cmd=['python3',os.path.join(TOOLS,'sensor_relative_scan.py'),
                            '--ros-args','-p',['scenario:=',scenario],
                            '-p',['sensor_height_hypothesis_m:=',height],
                            '-p',['input_start_ns:=',input_start],'-p',['input_end_ns:=',input_end],
                            '-p',['fit_end_ns:=',fit_end],'-p',['imu_median_ns:=',imu_median],
                            '-p',['fov_deg:=',scan_fov],'-p',['fov_center_deg:=',scan_center],
                            '-p',['angle_increment_deg:=',scan_increment],
                            '-p',['range_min:=',scan_min],'-p',['range_max:=',scan_max],
                            '-p',f'output_topic:={"/candidate_scan" if prescan_relay else "/scan"}',
                            '-p',['input_reliability:=',input_reliability],
                            '-p',['sensor_relative_band_m:=',sensor_band],
                            '-p',['min_obstacle_height:=',obstacle_min],
                            '-p',['max_obstacle_height:=',obstacle_max]],
                       name='sensor_relative_scan',output='screen'),
        ExecuteProcess(cmd=['python3',os.path.join(TOOLS,'amcl_ros_observer.py'),
                            '--ros-args','-p',['scenario:=',scenario],
                            '-p',['input_start_ns:=',input_start],
                            '-p','drive_nomotion_updates:=false',
                            '-p','output_jsonl:=/tmp/amcl-observer.jsonl'],
                       name='amcl_acceptance_observer',output='screen'),
        ExecuteProcess(cmd=['python3',os.path.join(TOOLS,
                            'amcl_prescan_relay.py' if prescan_relay else 'amcl_nomotion_driver.py'),
                            *([] if prescan_relay else ['--ros-args','-p',['trigger:=',nomotion_trigger]])],
                       name='amcl_prescan_relay' if prescan_relay else 'amcl_nomotion_driver',output='screen'),
        ExecuteProcess(cmd=['python3',os.path.join(TOOLS,'amcl_topic_probe.py'),
                            '--ros-args','-p','output_jsonl:=/tmp/amcl-topic-probe.jsonl'],
                       name='amcl_topic_probe',output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',
             parameters=[{'yaml_filename':map_yaml,'use_sim_time':True}],output='screen'),
        Node(package='nav2_amcl',executable='amcl',name='amcl',
             parameters=[os.path.join(TOOLS,'config/amcl.yaml'),
                         {'use_sim_time':True,'base_frame_id':'livox_level','odom_frame_id':'odom',
                          'global_frame_id':'map','scan_topic':'/scan','set_initial_pose':False,
                          'save_pose_rate':-1.0,
                          'min_particles':ParameterValue(particle_min,value_type=int),
                          'max_particles':ParameterValue(particle_max,value_type=int),
                          'max_beams':ParameterValue(max_beams,value_type=int),
                          'recovery_alpha_fast':ParameterValue(recovery_fast,value_type=float),
                          'recovery_alpha_slow':ParameterValue(recovery_slow,value_type=float)}],output='screen'),
        TimerAction(period=2.0,actions=[
            Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',
                 parameters=[{'autostart':True,'node_names':['map_server','amcl'],'use_sim_time':True}],output='screen')]),
    ])
