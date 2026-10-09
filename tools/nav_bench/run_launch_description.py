#!/usr/bin/env python3
"""Run a Python ROS 2 launch file directly from a research workspace."""
import sys

from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: run_launch_description.py LAUNCH_FILE')
    service=LaunchService()
    source=PythonLaunchDescriptionSource(sys.argv[1])
    service.include_launch_description(source.get_launch_description(service.context))
    return service.run()


if __name__=='__main__':
    raise SystemExit(main())
