#!/bin/bash
# Print the Livox Mid-360 driver config (MID360_config.json) for this robot's link to stdout. Sourced by ops/record_bag.sh and ops/nav_live.sh.
#   ops/livox_config.sh > MID360_config.json        env: LIVOX_HOST_IP (192.168.1.50)  LIVOX_LIDAR_IP (192.168.1.126)
HOST_IP=${LIVOX_HOST_IP:-192.168.1.50}
LIDAR_IP=${LIVOX_LIDAR_IP:-192.168.1.126}
cat <<EOF
{
  "lidar_summary_info": {"lidar_type": 8},
  "MID360": {
    "lidar_net_info": {"cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500},
    "host_net_info": {
      "cmd_data_ip": "$HOST_IP", "cmd_data_port": 56101,
      "push_msg_ip": "$HOST_IP", "push_msg_port": 56201,
      "point_data_ip": "$HOST_IP", "point_data_port": 56301,
      "imu_data_ip": "$HOST_IP", "imu_data_port": 56401,
      "log_data_ip": "", "log_data_port": 56501
    }
  },
  "lidar_configs": [{"ip": "$LIDAR_IP", "pcl_data_type": 1, "pattern_mode": 0, "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0}}]
}
EOF
