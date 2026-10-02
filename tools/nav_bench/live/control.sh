#!/bin/bash
# Pause / resume / rate of the bag playing on the board, from the laptop.   usage: PEER=<board IP> control.sh resume | pause | toggle | rate <x>
source "$(dirname "$(readlink -f "$0")")/laptop_env.sh"
case $1 in
  resume) svc=resume; typ=rosbag2_interfaces/srv/Resume; arg="{}" ;;
  pause) svc=pause; typ=rosbag2_interfaces/srv/Pause; arg="{}" ;;
  toggle) svc=toggle_paused; typ=rosbag2_interfaces/srv/TogglePaused; arg="{}" ;;
  rate) svc=set_rate; typ=rosbag2_interfaces/srv/SetRate; arg="{rate: $2}" ;;
  *) echo "usage: $0 resume|pause|toggle|rate <x>"; exit 1 ;;
esac
ros2 service call /rosbag2_player/$svc $typ "$arg"
