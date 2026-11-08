from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
def generate_launch_description():
    bond = os.environ.get("BOND", "1") == "1"   # BOND=0: no bond on either side (gn_loc period 0, manager timeout 0)
    nodes = [Node(package="gn_loc", executable="gn_loc_node", name="gn_loc", output="screen", parameters=[{"bond_heartbeat_period": 0.1 if bond else 0.0}]),
             Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization", output="screen",
                  parameters=[{"autostart": True, "node_names": ["gn_loc"], "bond_timeout": 4.0 if bond else 0.0}])]
    if os.environ.get("SYNTH", "1") == "1":
        nodes.append(ExecuteProcess(cmd=["python3", "/g/synth.py"], output="screen"))
    return LaunchDescription(nodes)
