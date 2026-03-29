#!/usr/bin/env python3
"""
ROS2 Offboard 控制测试程序 - 正方形飞行
适用环境: ROS2 Jazzy + PX4 1.16 SITL (uXRCE-DDS)
功能: 起飞 -> 飞正方形 -> 返回原点 -> 降落

话题:
  订阅: /fmu/out/vehicle_status_v1, /fmu/out/vehicle_odometry
  发布: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint,
        /fmu/in/vehicle_command
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
)

# PX4 arming_state 枚举值（来自 VehicleStatus uORB 定义）
ARMING_STATE_ARMED = 2


class OffboardSquare(Node):

    # ── 参数（可按需修改）──────────────────────────────────────────
    TAKEOFF_HEIGHT = -2.0   # 起飞高度，NED 坐标系，负值为向上 (m)
    SQUARE_SIZE = 3.0       # 正方形边长 (m)
    HOLD_TIME = 3.0         # 每个航点悬停时间 (s)
    REACH_THRESHOLD = 0.3   # 到达航点的距离阈值 (m)
    OFFBOARD_FREQ = 10      # Offboard 心跳频率 (Hz)，必须 > 2Hz
    ARM_COUNTDOWN = 30      # 解锁前等待心跳发送的次数（约 3s）
    RETRY_PERIOD = 10       # ARMING 状态下，每 N 帧重发一次模式切换+解锁
    # ──────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('offboard_square')

        # QoS 配置：与 PX4 DDS 匹配
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 发布者 ────────────────────────────────────────────────
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── 订阅者 ────────────────────────────────────────────────
        self.sub_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.cb_vehicle_status,
            qos,
        )
        self.sub_odom = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.cb_vehicle_odometry,
            qos,
        )

        # ── 状态变量 ──────────────────────────────────────────────
        self.vehicle_status = VehicleStatus()
        self.current_pos = [0.0, 0.0, 0.0]   # [x, y, z] NED
        self.heartbeat_count = 0
        self.state = 'IDLE'
        self.waypoint_index = 0
        self.hold_start_time = None
        self.home_pos = None
        self.has_odom = False
        self.has_status = False
        self.arming_retry_count = 0

        # 正方形航点：相对起飞点的偏移 [dx, dy, dz]
        self.waypoints_rel = [
            [0.0, 0.0, self.TAKEOFF_HEIGHT],                # 0: 起飞悬停点
            [self.SQUARE_SIZE, 0.0, self.TAKEOFF_HEIGHT],   # 1: 右前
            [self.SQUARE_SIZE, self.SQUARE_SIZE, self.TAKEOFF_HEIGHT],  # 2: 右后
            [0.0, self.SQUARE_SIZE, self.TAKEOFF_HEIGHT],   # 3: 左后
            [0.0, 0.0, self.TAKEOFF_HEIGHT],                # 4: 返回起飞点
        ]
        self.waypoints = []

        # ── 定时器 ────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.OFFBOARD_FREQ, self.timer_callback)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  Offboard 正方形飞行测试节点已启动')
        self.get_logger().info(f'  起飞高度: {abs(self.TAKEOFF_HEIGHT)} m')
        self.get_logger().info(f'  正方形边长: {self.SQUARE_SIZE} m')
        self.get_logger().info(f'  每点悬停: {self.HOLD_TIME} s')
        self.get_logger().info('=' * 55)

    # ── 回调 ──────────────────────────────────────────────────────

    def cb_vehicle_status(self, msg):
        self.vehicle_status = msg
        self.has_status = True

    def cb_vehicle_odometry(self, msg):
        self.has_odom = True
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]
        if self.home_pos is None and self.state != 'IDLE':
            self.home_pos = list(self.current_pos)
            self._build_waypoints()
            self.get_logger().info(
                f'记录起飞点: [{self.home_pos[0]:.2f}, '
                f'{self.home_pos[1]:.2f}, {self.home_pos[2]:.2f}]')

    # ── 工具方法 ──────────────────────────────────────────────────

    def _build_waypoints(self):
        self.waypoints = [
            [self.home_pos[0] + rel[0],
             self.home_pos[1] + rel[1],
             self.home_pos[2] + rel[2]]
            for rel in self.waypoints_rel
        ]
        names = ['起飞悬停', '右前角', '右后角', '左后角', '返回原点']
        self.get_logger().info('航点列表（绝对坐标，NED）:')
        for i, (wp, name) in enumerate(zip(self.waypoints, names)):
            self.get_logger().info(
                f'  WP{i} [{name}]: x={wp[0]:.2f} y={wp[1]:.2f} z={wp[2]:.2f}')

    def _distance_to(self, target):
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]
        dz = target[2] - self.current_pos[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _reached(self, target):
        return self._distance_to(target) < self.REACH_THRESHOLD

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_mode.publish(msg)

    def _publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        self.pub_setpoint.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info('>>> 发送解锁指令')

    def _disarm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info('>>> 发送上锁指令')

    def _set_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info('>>> 切换至 Offboard 模式')

    def _land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('>>> 发送降落指令')

    def _is_armed(self):
        return self.vehicle_status.arming_state == ARMING_STATE_ARMED

    # ── 状态机主循环 ──────────────────────────────────────────────

    def timer_callback(self):
        # 每帧都必须发送 Offboard 心跳（>2Hz）
        self._publish_offboard_mode()

        if self.state == 'IDLE':
            if not self.has_status or not self.has_odom:
                self._publish_setpoint(0.0, 0.0, self.TAKEOFF_HEIGHT)
                self.get_logger().info(
                    f'等待 PX4 数据... status={self.has_status} odom={self.has_odom}',
                    throttle_duration_sec=2.0,
                )
                return

            self.heartbeat_count += 1
            if self.waypoints:
                self._publish_setpoint(*self.waypoints[0])
            else:
                self._publish_setpoint(0.0, 0.0, self.TAKEOFF_HEIGHT)

            if self.heartbeat_count >= self.ARM_COUNTDOWN:
                self.get_logger().info(
                    f'心跳已发送 {self.heartbeat_count} 次，准备解锁...')
                self._set_offboard_mode()
                self._arm()
                self.state = 'ARMING'
                self.arming_retry_count = 0
                self.get_logger().info('[状态] IDLE -> ARMING')

        elif self.state == 'ARMING':
            if self.waypoints:
                self._publish_setpoint(*self.waypoints[0])
            else:
                self._publish_setpoint(0.0, 0.0, self.TAKEOFF_HEIGHT)

            self.arming_retry_count += 1
            if self.arming_retry_count % self.RETRY_PERIOD == 0 and not self._is_armed():
                self.get_logger().info('未解锁，重发 Offboard 模式与解锁指令...')
                self._set_offboard_mode()
                self._arm()

            if self._is_armed():
                self.get_logger().info('无人机已解锁！开始起飞...')
                self.state = 'TAKEOFF'
                self.waypoint_index = 0
                self.get_logger().info('[状态] ARMING -> TAKEOFF')
            else:
                self.get_logger().info(
                    f'等待解锁... arming_state={self.vehicle_status.arming_state}',
                    throttle_duration_sec=2.0,
                )

        elif self.state == 'TAKEOFF':
            if not self.waypoints:
                self._publish_setpoint(0.0, 0.0, self.TAKEOFF_HEIGHT)
                return

            wp = self.waypoints[0]
            self._publish_setpoint(*wp)
            dist = self._distance_to(wp)
            self.get_logger().info(
                f'[起飞] 距目标 {dist:.2f} m  当前高度 {self.current_pos[2]:.2f} m',
                throttle_duration_sec=2.0,
            )
            if self._reached(wp):
                self.get_logger().info('到达起飞高度，开始飞正方形！')
                self.waypoint_index = 1
                self.hold_start_time = None
                self.state = 'SQUARE'
                self.get_logger().info('[状态] TAKEOFF -> SQUARE')

        elif self.state == 'SQUARE':
            if self.waypoint_index >= len(self.waypoints):
                self.state = 'LANDING'
                self.get_logger().info('[状态] SQUARE -> LANDING')
                return

            wp = self.waypoints[self.waypoint_index]
            self._publish_setpoint(*wp)
            dist = self._distance_to(wp)

            if not self._reached(wp):
                self.hold_start_time = None
                self.get_logger().info(
                    f'[正方形] WP{self.waypoint_index} 距目标 {dist:.2f} m',
                    throttle_duration_sec=1.5,
                )
            else:
                if self.hold_start_time is None:
                    self.hold_start_time = self.get_clock().now()
                    self.get_logger().info(
                        f'到达 WP{self.waypoint_index}，悬停 {self.HOLD_TIME} s...')
                elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
                if elapsed >= self.HOLD_TIME:
                    self.get_logger().info(
                        f'WP{self.waypoint_index} 完成，前往下一个航点')
                    self.waypoint_index += 1
                    self.hold_start_time = None

        elif self.state == 'LANDING':
            self._land()
            self.get_logger().info('已发送降落指令，等待着陆...')
            self.state = 'DONE'
            self.get_logger().info('[状态] LANDING -> DONE')

        elif self.state == 'DONE':
            if not self._is_armed():
                self.get_logger().info('无人机已上锁，任务完成！')
                self.get_logger().info('=' * 55)
                self.get_logger().info('  正方形飞行任务完成 ✓')
                self.get_logger().info('=' * 55)
                self.timer.cancel()
            else:
                self.get_logger().info('等待降落并上锁...', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，发送上锁指令...')
        node._disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
