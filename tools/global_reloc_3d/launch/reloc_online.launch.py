"""Boot-time relocalization stack: relocalizer node (owns FAST-LIO2 and odom->base_footprint) + scan conversion + map_server + AMCL (no initial pose).
ros2 launch reloc_online.launch.py map:=M.yaml map_xyz:=prior.xyz work_dir:=D [sim_time:=true] [mode:=s3] [sensor_height:=0.53] [overrides:=amcl_overrides.yaml] [max_attempts:=1] [extra:=...]"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.realpath(__file__))
NAV = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'nav_bench')


def generate_launch_description():
    names = ('map', 'map_xyz', 'work_dir')
    args = [DeclareLaunchArgument(n) for n in names]
    for n, d in (('sim_time', 'true'), ('mode', 's3'), ('sensor_height', '0.53'), ('max_attempts', '1'), ('overrides', os.path.join(NAV, 'config/no_overrides.yaml')),
                 ('register_binary', os.path.join(os.path.dirname(HERE), 'register_v2')), ('lib_dir', '/opt')):
        args.append(DeclareLaunchArgument(n, default_value=d))
    sim = {'use_sim_time': ParameterValue(LC('sim_time'), value_type=bool)}
    return LaunchDescription(args + [
        ExecuteProcess(cmd=['python3', '-B', os.path.join(os.path.dirname(HERE), 'reloc_node.py'), '--ros-args',
                            '-p', ['use_sim_time:=', LC('sim_time')], '-p', ['map_xyz:=', LC('map_xyz')], '-p', ['work_dir:=', LC('work_dir')],
                            '-p', ['mode:=', LC('mode')], '-p', ['sensor_height:=', LC('sensor_height')],
                            '-p', ['max_attempts:=', LC('max_attempts')], '-p', ['register_binary:=', LC('register_binary')],
                            '-p', ['lib_dir:=', LC('lib_dir')]], output='screen', name='global_reloc'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', parameters=[os.path.join(NAV, 'config/scan.yaml'), sim],
             remappings=[('cloud_in', '/cloud_registered_body'), ('scan', '/scan')]),
        Node(package='nav2_map_server', executable='map_server', parameters=[{'yaml_filename': LC('map')}, sim]),
        Node(package='nav2_amcl', executable='amcl', parameters=[os.path.join(NAV, 'config/amcl.yaml'), LC('overrides'), sim]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}, sim]),
    ])
