"""map_server + gn_loc, same arguments as nav_bench's amcl_only.launch.py (init_x/init_y/init_yaw/overrides are accepted and ignored: the pose comes from /initialpose).
ros2 launch gn_loc gn_loc.launch.py map:=M.yaml [use_sim_time:=true]"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = {"use_sim_time": ParameterValue(LC("use_sim_time"), value_type=bool)}
    args = [DeclareLaunchArgument("map"), DeclareLaunchArgument("use_sim_time", default_value="true")]
    args += [DeclareLaunchArgument(n, default_value="") for n in ("init_x", "init_y", "init_yaw", "overrides")]
    return LaunchDescription(args + [
        Node(package="nav2_map_server", executable="map_server", parameters=[{"yaml_filename": LC("map")}, sim]),
        Node(package="gn_loc", executable="gn_loc_node", name="gn_loc", output="screen", parameters=[sim]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             parameters=[{"autostart": True, "node_names": ["map_server", "gn_loc"]}, sim]),
    ])
