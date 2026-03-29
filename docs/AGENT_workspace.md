# AGENT.md — ROS2 / OpenVINS 工作空间（WSL-B）

## 这个项目是什么
运行在 Ubuntu 24.04 独立 WSL2 实例中的 ROS2 Jazzy 工作空间。
负责所有感知、状态估计和 Offboard 控制逻辑。
PX4 仿真运行在**另一个 WSL 实例（WSL-A）**中，见其对应的 `AGENT.md`。

---

## 环境基本信息

| 项目 | 值 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS2 发行版 | Jazzy |
| DDS 中间件 | Fast-DDS（Jazzy 默认） |
| 跨 WSL 通信 | Micro-XRCE-DDS-Agent → UDP → PX4 WSL-A |
| 工作空间根目录 | `~/ros2_ws/` |
| 构建工具 | `colcon` |

---

## 当前进度

### 已完成
- `px4_offboard` 节点：解锁、切换 Offboard 模式、发送轨迹设定点。**已端到端验证可用。**
- DDS 桥接到 PX4 WSL-A：双向通信，localhost UDP 稳定运行。
- `px4_msgs` 包：已编译并已 source。

### 进行中
- **OpenVINS 集成**——包已克隆，尚未完整编译和配置：
  - `ov_core`、`ov_msckf`、`ov_init`、`ov_eval` 位于 `src/open_vins/`
  - 目标配置文件：`src/open_vins/config/px4_stereo/estimator_config.yaml`（尚未编写）
- **IMU 桥接节点**（`imu_bridge`）——将 `px4_msgs/SensorCombined` 转换为 `sensor_msgs/Imu` 发布到 `/imu0`。已设计，尚未实现。
- **VIO 回传节点**（`vio_to_px4`）——将 OpenVINS 输出的 `nav_msgs/Odometry` 转换为 `px4_msgs/VehicleVisualOdometry`。已设计，尚未实现。
- **相机图像接入**——图像已从 WSL-A `ros_gz_bridge` 到达，但尚未重映射到 OpenVINS 期望的话题名。

### 尚未开始
- Kalibr 标定流程（需先在 WSL-A 完成标定板 SDF）
- rviz2 可视化配置文件
- 完整飞行测试

---

## 包结构

```
~/ros2_ws/
├── src/
│   ├── px4_msgs/                    # PX4 uORB ↔ ROS2 消息定义
│   ├── px4_ros_com/                 # 辅助工具（如已安装）
│   ├── px4_offboard/                # ← 已完成：Offboard 控制节点
│   ├── open_vins/                   # ← 进行中：VIO 估计
│   │   ├── ov_core/                 # 特征跟踪、数学工具
│   │   ├── ov_msckf/                # 主估计器节点 + launch 文件
│   │   ├── ov_init/                 # 初始化器
│   │   ├── ov_eval/                 # 评估工具
│   │   └── config/
│   │       └── px4_stereo/          # ← 在此创建我们的自定义配置
│   │           └── estimator_config.yaml
│   ├── imu_bridge/                  # ← 待实现：SensorCombined → sensor_msgs/Imu
│   └── vio_to_px4/                  # ← 待实现：Odometry → VehicleVisualOdometry
└── install/                         # colcon 产物——不要手动修改
```

---

## 话题连接关系

### 从 PX4 WSL-A 接收（经 DDS）

| 话题 | 消息类型 | 消费方 |
|---|---|---|
| `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | `imu_bridge` 节点 |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | 调试 / 日志 |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | `px4_offboard` |

### 从 WSL-A gz_bridge 接收（相机图像）

| 话题 | 消息类型 | 消费方 |
|---|---|---|
| `/camera_left` | `sensor_msgs/Image` | OpenVINS cam0（重映射后） |
| `/camera_right` | `sensor_msgs/Image` | OpenVINS cam1（重映射后） |
| `/camera_left/camera_info` | `sensor_msgs/CameraInfo` | OpenVINS |
| `/camera_right/camera_info` | `sensor_msgs/CameraInfo` | OpenVINS |

> **重映射说明：** OpenVINS 默认订阅 `/cam0/image_raw` 和 `/cam1/image_raw`。
> 在 launch 文件中用 remapping 处理，**不要**修改 gz_bridge 的话题名。

### 内部 ROS2 话题

| 话题 | 消息类型 | 发布方 → 订阅方 |
|---|---|---|
| `/imu0` | `sensor_msgs/Imu` | `imu_bridge` → OpenVINS |
| `/ov_msckf/odomimu` | `nav_msgs/Odometry` | OpenVINS → `vio_to_px4` |
| `/ov_msckf/pathimu` | `nav_msgs/Path` | OpenVINS → rviz2 |

### 发送给 PX4 WSL-A（经 DDS）

| 话题 | 消息类型 | 发布方 |
|---|---|---|
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleVisualOdometry` | `vio_to_px4` |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | `px4_offboard` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | `px4_offboard` |

---

## 坐标系约定

这是跨系统对接中最容易出 bug 的地方，写转换代码前务必确认。

| 系统 | 坐标系约定 |
|---|---|
| PX4 内部 | NED（北-东-下），机体系 FRD |
| ROS2 / OpenVINS 输出 | ENU（东-北-上），机体系 FLU |
| `VehicleVisualOdometry` 期望输入 | NED + FRD |

**向 PX4 回传 VIO 时必须做的转换：**
- 位置：`(x_enu, y_enu, z_enu)` → `(x_ned, y_ned, z_ned)` = `(y, x, -z)`
- 四元数：需要额外旋转（绕 Z 轴 +90° 后翻转 Z 轴）。建议用 `tf2` 或写明确的旋转矩阵，不要凭感觉写。

---

## OpenVINS 配置说明

目标配置文件：`src/open_vins/config/px4_stereo/estimator_config.yaml`

关键参数说明：
```yaml
use_stereo: true           # 双目模式
max_cameras: 2             # 双目为 2
gravity_mag: 9.81

# IMU 话题（必须与 imu_bridge 输出一致）
topic_imu: "/imu0"

# 相机话题（通过 launch remapping 从 /camera_left 映射过来）
topic_camera0: "/cam0/image_raw"
topic_camera1: "/cam1/image_raw"

# IMU 噪声参数——以下为 PX4 SITL Gazebo 的估算值，标定后需替换
gyroscope_noise_density: 1.7e-4
gyroscope_random_walk: 2.3e-6
accelerometer_noise_density: 2.0e-3
accelerometer_random_walk: 3.0e-5
```

> 相机内参和 IMU-Camera 外参（`T_imu_cam0` 等）**不要手动填写**，必须来自 Kalibr 标定输出。

---

## 编译命令

```bash
cd ~/ros2_ws

# 编译全部包
colcon build --symlink-install

# 只编译指定包（日常迭代用）
colcon build --symlink-install \
  --packages-select ov_core ov_msckf ov_init imu_bridge vio_to_px4

# source 工作空间
source install/setup.bash

# 自动安装缺失依赖
rosdep install --from-paths src --ignore-src -r -y
```

---

## 完整启动顺序

按顺序在各自独立的终端中执行：

```bash
# 每个终端都需要先执行
source ~/ros2_ws/install/setup.bash

# 1. IMU 桥接节点（SensorCombined → /imu0）
ros2 run imu_bridge imu_bridge_node

# 2. OpenVINS 估计器
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=$(ros2 pkg prefix ov_msckf)/share/ov_msckf/config/px4_stereo/estimator_config.yaml

# 3. VIO 回传节点（odometry → VehicleVisualOdometry）
ros2 run vio_to_px4 vio_to_px4_node

# 4. Offboard 控制器（仅测试完整飞行时启动）
ros2 run px4_offboard offboard_control

# 5. 可视化
ros2 run rviz2 rviz2
```

---

## 调试检查清单

```bash
# 确认 IMU 频率（目标：≥ 200 Hz）
ros2 topic hz /imu0

# 确认相机图像到达（目标：约 20 Hz）
ros2 topic hz /camera_left
ros2 topic hz /cam0/image_raw   # 重映射后

# 确认 OpenVINS 有输出
ros2 topic echo /ov_msckf/odomimu --once

# 确认 VIO 位姿已到达 PX4
ros2 topic echo /fmu/in/vehicle_visual_odometry --once

# 排查 DDS 连通性
ros2 topic list | grep fmu
```

---

## 已知问题 / 注意事项

**1. 仿真时间同步**
仿真中必须为 OpenVINS 和桥接节点设置 `use_sim_time: true`，并确保从 WSL-A 通过 gz_bridge 转发 `/clock` 话题。否则 IMU 和相机时间戳不对齐，VIO 会立刻发散。

**2. OpenVINS 在 Jazzy 上的编译问题**
`cv_bridge` 在 Jazzy 中包名有变动。如果编译报错，检查 `CMakeLists.txt` 中 `find_package(cv_bridge REQUIRED)` 的写法是否与已安装的 `ros-jazzy-cv-bridge` 一致。

**3. 图像编码格式**
gz Harmonic 相机默认发布 RGB8，OpenVINS 期望 mono8 或 BGR8。需要在 pipeline 中加入 `image_proc` 转换节点，或在配置中开启灰度分离参数。

**4. IMU 坐标系**
`sensor_combined` 的陀螺仪和加速度计数据处于 PX4 机体系（FRD）。确保与 Kalibr 标定时使用的相机坐标系对齐，这里不一致会导致估计直接失败。

---

## 让 Agent 做什么 / 不做什么

**可以让 Agent 处理：**
- 实现 `imu_bridge` 节点（`src/imu_bridge/`）
- 实现 `vio_to_px4` 节点（`src/vio_to_px4/`），包含 NED/ENU 坐标转换
- 根据 Kalibr 输出编写 `estimator_config.yaml`
- 编写含话题重映射的 launch 文件
- 修复 OpenVINS `CMakeLists.txt` 的 Jazzy 兼容性问题
- 编写 rviz2 可视化配置

**不要让 Agent 在这个上下文中处理：**
- PX4 SDF 模型文件——在 WSL-A 中
- Gazebo 世界编辑——在 WSL-A 中
- QGroundControl 操作或飞控参数调整
