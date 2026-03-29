# subagent_openvins.md

## 目标
在 ros2_ws 中完成单目 OpenVINS + PX4 IMU 输入链路，得到稳定的 /ov_msckf/odomimu，并可选回传 PX4（vio_to_px4）。输出应可复用于 Docker 镜像构建。

## 项目现状（自动扫描结论）
- 当前目录核心文档：plan.md、want_px4.md
- 真正代码位于：
  - ~/ros2_ws/src/open_vins（上游仓库，已有本地修改）
  - ~/ros2_ws/src/imu_bridge（已实现 imu_bridge_node.py）
  - ~/ros2_ws/src/vio_to_px4（已实现 vio_to_px4_node.py，含 ENU->NED 与 FLU->FRD 旋转）
- 已存在配置：~/ros2_ws/src/open_vins/config/px4_mono/estimator_config.yaml

## 必要依赖（已在当前机器执行过安装）
```bash
apt-get update
apt-get install -y \
  python3-rosdep \
  python3-colcon-common-extensions \
  libeigen3-dev \
  libboost-all-dev \
  libceres-dev \
  libopencv-dev \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-proc \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-sensor-msgs

rosdep init || true
rosdep update
```

## 构建命令
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select ov_core ov_init ov_msckf imu_bridge vio_to_px4
source install/setup.bash
```

## 启动顺序
```bash
# 终端1
source ~/ros2_ws/install/setup.bash
ros2 run imu_bridge imu_bridge_node

# 终端2
source ~/ros2_ws/install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(ros2 pkg prefix ov_msckf)/share/ov_msckf/config/px4_mono/estimator_config.yaml

# 终端3（可选）
source ~/ros2_ws/install/setup.bash
ros2 run vio_to_px4 vio_to_px4_node
```

## 关键验收
```bash
ros2 topic hz /imu0
ros2 topic hz /camera_left
ros2 topic hz /cam0/image_raw
ros2 topic echo /ov_msckf/odomimu --once
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

## 常见风险
1. 时钟不同步：必须确保 /clock 打通并启用 use_sim_time。
2. 图像编码不匹配：若输入是 rgb8，需用 image_proc 转 mono8。
3. 坐标系错误：PX4 需要 NED+FRD，回传时必须做 ENU/FLU 转换。
4. OpenCV 与 cv_bridge 版本不一致：会导致运行时异常。

## Docker 化建议
- 镜像只做基础依赖安装，源码通过 bind mount 注入。
- 推荐在 Dockerfile 中固定 ROS_DISTRO=jazzy 与 apt 包版本来源。
- colcon build 可放在镜像构建阶段确保一致性。

## 子 Agent 执行边界
可做：
- 调整 px4_mono 配置参数与 launch remap
- 修复 OpenVINS Jazzy 兼容问题
- 优化 imu_bridge、vio_to_px4 的协方差与时间戳处理

不要做：
- 修改 WSL-A 的 Gazebo/SDF
- 在本目录直接手工修改 install 产物
