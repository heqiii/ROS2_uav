#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /home/he/ros2_ws/install/setup.bash ]; then
  source /home/he/ros2_ws/install/setup.bash
fi

export AMENT_PREFIX_PATH=/home/he/ros2_ws/install/imu_bridge:/home/he/ros2_ws/install/vio_to_px4:${AMENT_PREFIX_PATH}
export CMAKE_PREFIX_PATH=/home/he/ros2_ws/install/imu_bridge:/home/he/ros2_ws/install/vio_to_px4:${CMAKE_PREFIX_PATH}

alias run_imu_bridge='ros2 run imu_bridge imu_bridge_node'
alias run_vio_to_px4='ros2 run vio_to_px4 vio_to_px4_node'
