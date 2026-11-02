"""A node that does nothing, to price the sim clock: idle_node.py sim|wall (use_sim_time on/off). clock_sim.sh compares the CPU of the two."""
import sys

import rclpy
from rclpy.node import Node

rclpy.init()
rclpy.spin(Node(f'idle_{sys.argv[1]}', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, sys.argv[1] == 'sim')]))
