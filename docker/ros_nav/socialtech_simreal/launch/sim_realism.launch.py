"""The four realism nodes. Start the simulation with SIM_REALISM=1 in its environment (hospital-agv-sim 'sim-realism' branch): it then leaves /scan, /imu/data and /odom to these nodes, and routes /cmd_vel through sim_base_model.
ros2 launch socialtech_simreal sim_realism.launch.py [seed:=N] [outlier_scale:=1.0] [noise_scale:=1.0] [kick:=0.0] [deadband:=-1.0]"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SIM = {"use_sim_time": True}


def p(name, kind=float):
    return ParameterValue(LC(name), value_type=kind)


def generate_launch_description():
    args = [DeclareLaunchArgument("seed", default_value="-1"), DeclareLaunchArgument("outlier_scale", default_value="1.0"), DeclareLaunchArgument("noise_scale", default_value="1.0"),
            DeclareLaunchArgument("kick", default_value="0.0"), DeclareLaunchArgument("deadband", default_value="-1.0")]
    return LaunchDescription(args + [
        Node(package="socialtech_simreal", executable="sim_base_model", output="screen", parameters=[SIM, {"seed": p("seed", int), "kick": p("kick"), "deadband": p("deadband")}]),
        Node(package="socialtech_simreal", executable="sim_wheel_odom", output="screen", parameters=[SIM, {"seed": p("seed", int)}]),
        Node(package="socialtech_simreal", executable="sim_scan_model", output="screen", parameters=[SIM, {"seed": p("seed", int), "outlier_scale": p("outlier_scale"), "noise_scale": p("noise_scale")}]),
        Node(package="socialtech_simreal", executable="sim_imu_model", output="screen", parameters=[SIM, {"seed": p("seed", int)}]),
    ])
