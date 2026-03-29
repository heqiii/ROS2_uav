# PLAN.md — 单目 OpenVINS 基础工程（ROS2 WSL-B）

## 目标
在现有 ROS2 Jazzy 环境中，用 PX4 SITL Gazebo 仿真的**单目相机 + IMU**，
跑通 OpenVINS 单目惯性里程计，输出位姿并在 rviz2 中可视化。
**本阶段不涉及双目、不涉及 VIO 回传 PX4、不涉及实际飞行。**

---

## 前置假设（开始前确认）

- [ ] WSL-A 中 PX4 + Gazebo 可以正常启动（`make px4_sitl gz_x500_mono_cam` 或等效）
- [ ] WSL-A 中 `ros_gz_bridge` 已将 `/camera_left` 和 `/camera_left/camera_info` 桥接到 ROS2
- [ ] WSL-B 中 `/fmu/out/sensor_combined` 话题有数据（DDS 已通）
- [ ] `src/open_vins/` 已克隆（`ov_core` `ov_msckf` `ov_init` 目录存在）

---

## 阶段划分

```
阶段 1  编译 OpenVINS          ──→  能 colcon build 成功
阶段 2  实现 imu_bridge        ──→  /imu0 有 200Hz 数据
阶段 3  打通相机话题           ──→  /cam0/image_raw 有 20Hz 图像
阶段 4  写单目配置文件         ──→  estimator_config.yaml 可用
阶段 5  编写 launch 文件       ──→  一条命令跑起全栈
阶段 6  验证与调试             ──→  rviz2 能看到轨迹
```

---

## 阶段 1 — 编译 OpenVINS

### 1.1 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-image-proc \
  ros-jazzy-rviz2 \
  libopencv-dev \
  libeigen3-dev \
  libboost-all-dev
```

### 1.2 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select ov_core ov_init ov_msckf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 1.3 常见编译错误及修复

**错误：`cv_bridge` 找不到**
```cmake
# 在 ov_core/CMakeLists.txt 和 ov_msckf/CMakeLists.txt 中确认写法
find_package(cv_bridge REQUIRED)   # Jazzy 正确写法（小写）
```

**错误：`OpenCV` 版本冲突**
```bash
# 确认系统 OpenCV 版本
pkg-config --modversion opencv4
# OpenVINS 需要 OpenCV 4.x，Ubuntu 24.04 默认满足
```

**错误：`boost` 头文件缺失**
```bash
sudo apt install -y libboost-filesystem-dev libboost-system-dev
```

### 1.4 验证

```bash
# 能列出 ov_msckf 的可执行文件即为成功
ros2 pkg executables ov_msckf
# 期望输出: ov_msckf run_subscribe_msckf
```

---

## 阶段 2 — 实现 imu_bridge 节点

**作用：** 把 `px4_msgs/SensorCombined`（PX4 FRD 机体系）转换为
`sensor_msgs/Imu`（ROS2 标准格式）发布到 `/imu0`。

### 2.1 创建包

```bash
cd ~/ros2_ws/src
ros2 pkg create imu_bridge \
  --build-type ament_python \
  --dependencies rclpy px4_msgs sensor_msgs
```

### 2.2 节点实现

文件：`src/imu_bridge/imu_bridge/imu_bridge_node.py`

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        # PX4 DDS 话题需要 BEST_EFFORT QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.cb,
            px4_qos
        )
        self.pub = self.create_publisher(Imu, '/imu0', 100)
        self.get_logger().info('imu_bridge 已启动，等待 /fmu/out/sensor_combined ...')

    def cb(self, msg: SensorCombined):
        imu = Imu()

        # 时间戳：PX4 使用微秒，ROS2 使用纳秒
        stamp_sec = msg.timestamp / 1e6
        imu.header.stamp.sec = int(stamp_sec)
        imu.header.stamp.nanosec = int((stamp_sec % 1) * 1e9)
        imu.header.frame_id = 'imu'

        # PX4 FRD 机体系 → ROS2 FLU（x前-y左-z上）
        # FRD→FLU: x不变, y取反, z取反
        imu.angular_velocity.x =  msg.gyro_rad[0]
        imu.angular_velocity.y = -msg.gyro_rad[1]
        imu.angular_velocity.z = -msg.gyro_rad[2]

        imu.linear_acceleration.x =  msg.accelerometer_m_s2[0]
        imu.linear_acceleration.y = -msg.accelerometer_m_s2[1]
        imu.linear_acceleration.z = -msg.accelerometer_m_s2[2]

        # 协方差：-1 表示未知，OpenVINS 会用配置文件里的噪声参数
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(imu)

def main():
    rclpy.init()
    node = ImuBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> **坐标系说明：** PX4 内部是 FRD（前-右-下），ROS2 约定是 FLU（前-左-上）。
> 转换规则：Y 轴和 Z 轴取反。Kalibr 标定时相机参考系也是 FLU，所以这里必须转换。

### 2.3 注册入口点

文件：`src/imu_bridge/setup.py`

```python
entry_points={
    'console_scripts': [
        'imu_bridge_node = imu_bridge.imu_bridge_node:main',
    ],
},
```

### 2.4 编译并验证

```bash
colcon build --symlink-install --packages-select imu_bridge
source install/setup.bash

# 终端 1：启动节点
ros2 run imu_bridge imu_bridge_node

# 终端 2：验证频率（目标 ≥ 200 Hz）
ros2 topic hz /imu0

# 验证数据合理性（z 轴加速度仿真静止时应接近 9.81）
ros2 topic echo /imu0 --once
```

---

## 阶段 3 — 打通相机话题

**目标：** `/cam0/image_raw` 有稳定的 20 Hz 图像数据。

### 3.1 确认 WSL-A 侧 gz_bridge 已启动

在 WSL-A 终端执行：
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera_left@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

### 3.2 在 WSL-B 确认图像到达

```bash
ros2 topic hz /camera_left          # 期望约 20 Hz
ros2 topic echo /camera_left/camera_info --once   # 确认内参不全为 0
```

### 3.3 图像编码格式转换（如需要）

gz Harmonic 默认发布 `rgb8`，OpenVINS 期望 `mono8`。
在 launch 文件中加入 `image_proc` 节点做转换（见阶段 5）。

验证当前编码：
```bash
ros2 topic echo /camera_left --field encoding --once
# 若输出 "rgb8" 则需要转换；若已是 "mono8" 则跳过此步
```

---

## 阶段 4 — 编写单目配置文件

文件路径：`~/ros2_ws/src/open_vins/config/px4_mono_sim/estimator_config.yaml`

```yaml
%YAML:1.0

# =====================
# 估计器基础设置
# =====================
use_stereo: false          # 单目模式
max_cameras: 1
gravity_mag: 9.81

# =====================
# 话题名（与 launch 文件 remapping 对应）
# =====================
topic_imu: "/imu0"
topic_camera0: "/cam0/image_raw"

# =====================
# 初始化设置
# =====================
init_window_time: 1.0      # 静止初始化等待时间（秒）
init_imu_thresh: 0.5       # 初始化 IMU 阈值

# =====================
# 状态估计设置
# =====================
dt_slam_delay: 2.0
max_clones: 11             # 滑动窗口大小
max_slam: 50               # SLAM 路标点数量
max_slam_in_update: 25
max_msckf_in_update: 40
up_msckf_sigma_px: 1.0
up_slam_sigma_px: 1.0

# =====================
# 特征跟踪设置
# =====================
use_klt: true              # 使用 KLT 光流（仿真推荐）
num_pts: 200               # 跟踪特征点数量
fast_threshold: 20
grid_x: 5
grid_y: 3
min_px_dist: 15
knn_ratio: 0.70

# =====================
# 相机内参（来自 Gazebo 仿真相机设置）
# =====================
# 仿真相机参数：640x480，FOV 90°（水平）
# fx = fy = width / (2 * tan(fov/2)) = 640 / (2 * tan(0.7854)) ≈ 320
cam0_wh: [640, 480]
cam0_intrinsics: [320.0, 320.0, 320.0, 240.0]  # fx fy cx cy
cam0_distortion_coeffs: [0.0, 0.0, 0.0, 0.0]   # 仿真无畸变
cam0_distortion_model: "radtan"

# =====================
# IMU-Camera 外参（T_ItoC：从 IMU 系到相机系的变换）
# 仿真中相机安装在机体正前方，此处为近似值
# 真实标定后必须替换
# =====================
T_imu_cam0:
  rows: 4
  cols: 4
  data: [0.0,  0.0, 1.0, 0.10,   # 相机朝前（x轴对齐机体前向）
        -1.0,  0.0, 0.0, 0.00,
         0.0, -1.0, 0.0, 0.00,
         0.0,  0.0, 0.0, 1.00]

# =====================
# IMU 噪声参数（PX4 SITL Gazebo 仿真估算值）
# =====================
gyroscope_noise_density: 1.7e-4
gyroscope_random_walk: 2.3e-6
accelerometer_noise_density: 2.0e-3
accelerometer_random_walk: 3.0e-5

# =====================
# 输出设置
# =====================
use_zahra_filter: false
up_slam_sigma_px: 1.0
```

> **注意：** `T_imu_cam0` 此处为仿真近似值。仿真阶段跑通流程后，
> 正式部署前必须用 Kalibr 标定真实值替换。

---

## 阶段 5 — 编写 Launch 文件

文件路径：`~/ros2_ws/src/open_vins/ov_msckf/launch/px4_mono_sim.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('ov_msckf'),
        'config', 'px4_mono_sim', 'estimator_config.yaml'
    )

    return LaunchDescription([

        # ── 1. 图像格式转换：rgb8 → mono8 ──────────────────────────────
        # 如果 gz_bridge 已输出 mono8，删除此节点
        Node(
            package='image_proc',
            executable='image_proc',
            name='cam0_image_proc',
            namespace='cam0_proc',
            remappings=[
                ('image',        '/camera_left'),
                ('camera_info',  '/camera_left/camera_info'),
            ],
        ),

        # ── 2. OpenVINS 主估计器 ────────────────────────────────────────
        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='ov_msckf',
            output='screen',
            parameters=[{
                'config_path': config_path,
                'verbosity': 'WARNING',
                'use_sim_time': True,     # 仿真时使用 /clock
            }],
            remappings=[
                # 把 image_proc 输出的 mono8 话题映射到 OpenVINS 期望的名字
                ('/cam0/image_raw', '/cam0_proc/image_mono'),
            ],
        ),

        # ── 3. rviz2 可视化 ─────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('ov_msckf'),
                'launch', 'display.rviz'
            )],
            parameters=[{'use_sim_time': True}],
        ),

    ])
```

### 5.1 确保配置文件被安装

在 `ov_msckf/CMakeLists.txt` 中添加：
```cmake
install(DIRECTORY config/px4_mono_sim
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

---

## 阶段 6 — 验证与调试

### 6.1 完整启动顺序

```bash
# WSL-A：终端 1 — 启动 PX4 + Gazebo
make px4_sitl gz_x500_mono_cam

# WSL-A：终端 2 — 启动 gz → ROS2 相机桥接
ros2 run ros_gz_bridge parameter_bridge \
  /camera_left@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo

# WSL-A：终端 3 — 启动 DDS Agent
MicroXRCEAgent udp4 -p 8888

# WSL-B：终端 4 — 启动 imu_bridge
source ~/ros2_ws/install/setup.bash
ros2 run imu_bridge imu_bridge_node

# WSL-B：终端 5 — 启动 OpenVINS
source ~/ros2_ws/install/setup.bash
ros2 launch ov_msckf px4_mono_sim.launch.py
```

### 6.2 逐步验证检查表

**数据流验证（先于 OpenVINS 启动前确认）：**
```bash
# IMU 频率（目标 ≥ 200 Hz）
ros2 topic hz /imu0

# 相机频率（目标 ~20 Hz）
ros2 topic hz /camera_left

# 确认图像编码已转换
ros2 topic echo /cam0_proc/image_mono --field encoding --once
# 期望: "mono8"

# 确认时钟话题存在（仿真时间同步）
ros2 topic hz /clock
```

**OpenVINS 启动后验证：**
```bash
# 确认有位姿输出（启动后约 2-5 秒开始输出，取决于初始化）
ros2 topic hz /ov_msckf/odomimu

# 查看一帧位姿数据
ros2 topic echo /ov_msckf/odomimu --once

# 查看特征跟踪图像（可视化跟踪质量）
ros2 topic hz /ov_msckf/loop_feats
```

### 6.3 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|---|---|---|
| OpenVINS 一直不初始化 | 无人机静止时间不足 | 让无人机保持静止 ≥ 2 秒再起飞 |
| 位姿输出后立刻发散 | `T_imu_cam0` 外参错误 | 检查相机安装方向，修正旋转矩阵 |
| IMU 为 0 Hz | DDS QoS 不匹配 | 确认 `imu_bridge` 使用 `BEST_EFFORT` QoS |
| 图像为 0 Hz | gz_bridge 未启动或话题名错误 | `gz topic -l` 检查 gz 侧话题名 |
| 时间戳跳变导致崩溃 | 未启用 `use_sim_time` | 所有节点加 `use_sim_time: true` |
| 特征跟踪点数量为 0 | 图像太暗或场景无纹理 | Gazebo 世界中添加纹理地面或物体 |

---

## 完成标准

本阶段完成的判断依据：

- [ ] `colcon build` 无报错
- [ ] `/imu0` 稳定在 200 Hz 以上
- [ ] `/cam0_proc/image_mono` 稳定在 20 Hz，编码为 `mono8`
- [ ] OpenVINS 在无人机起飞后 5 秒内完成初始化
- [ ] `/ov_msckf/odomimu` 有连续输出且位姿不立刻发散
- [ ] rviz2 中可以看到估计轨迹

---

## 下一阶段（本阶段完成后）

- **双目升级：** 添加 `camera_right`，改 `use_stereo: true`，精度显著提升
- **回传 PX4：** 实现 `vio_to_px4` 节点，把 VIO 位姿送给 PX4 作为位置来源（见 `AGENT.md`）
- **Kalibr 标定：** 用真实标定值替换配置文件中的近似外参
