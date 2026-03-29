# subagent_px4_offboard.md

## 目标
在 ROS2 Jazzy + PX4 uXRCE-DDS 环境中，稳定运行 offboard_square.py，完成解锁、切 Offboard、正方形轨迹飞行、返航降落，并可用于后续 Docker 化打包。

## 项目现状（自动扫描结论）
- 当前目录为脚本型工程，不是独立 ROS2 包。
- 关键文件：offboard_square.py
- 订阅话题：/fmu/out/vehicle_status_v1、/fmu/out/vehicle_odometry
- 发布话题：/fmu/in/offboard_control_mode、/fmu/in/trajectory_setpoint、/fmu/in/vehicle_command
- 依赖：rclpy、px4_msgs

## 必要依赖（宿主机 / Docker 通用）
```bash
apt-get update
apt-get install -y \
  python3-pip \
  python3-colcon-common-extensions \
  ros-jazzy-rclpy \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs
```

若未使用系统版 px4_msgs，请保持源码工作空间内已有的 px4_msgs 包。

## 启动流程
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/1.px4_offboard/offboard_square.py
```

## 运行前检查
```bash
ros2 topic list | grep /fmu
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

## 常见故障处理
1. 无法进入 Offboard：确认 offboard_control_mode 心跳持续大于 2 Hz。
2. 未解锁：检查 VehicleStatus.arming_state 是否变化，必要时重发 VEHICLE_CMD_DO_SET_MODE 与解锁指令。
3. 无里程计：先排查 DDS/Agent，确认 /fmu/out/vehicle_odometry 有数据。

## Docker 化建议
- 该目录建议作为 apps/px4_offboard/ 挂载进容器。
- 不要把 build/ install/ log/ 打进镜像层。
- 运行容器时使用 host 网络：--net=host。

## 子 Agent 执行边界
可做：
- 调整轨迹参数（高度、边长、悬停时间、阈值）
- 增加状态机日志与安全保护
- 适配不同 vehicle_status 话题名

不要做：
- 修改 PX4 SITL 世界或 SDF（应在 WSL-A）
- 直接改飞控参数（应在 QGC/PX4 流程中）
