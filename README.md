# ros2_uav

ROS2 Jazzy UAV stack backup for PX4 offboard + OpenVINS integration.

## Contents
- px4_offboard/: offboard square mission script and subagent workflow
- openvins/: OpenVINS single-cam planning docs and subagent workflow
- bridges/imu_bridge/: PX4 SensorCombined -> sensor_msgs/Imu package
- bridges/vio_to_px4/: OpenVINS odometry -> PX4 VehicleVisualOdometry package
- bridges/open_vins_patches/: local OpenVINS custom config and modified upstream files
- scripts/install_deps_jazzy.sh: dependency installation for Jazzy
- docs/AGENT_workspace.md: workspace-level architecture notes
