#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


def quat_to_rot(qx: float, qy: float, qz: float, qw: float):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rot_to_quat(r):
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


class VioToPx4Node(Node):
    def __init__(self) -> None:
        super().__init__("vio_to_px4_node")

        self.declare_parameter("input_topic", "/ov_msckf/odomimu")
        self.declare_parameter("output_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("quality", 100)
        self.declare_parameter("reset_counter", 0)
        self.declare_parameter("use_ros_time", True)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.quality = int(self.get_parameter("quality").value)
        self.reset_counter = int(self.get_parameter("reset_counter").value)
        self.use_ros_time = self.get_parameter("use_ros_time").get_parameter_value().bool_value

        self.sub = self.create_subscription(Odometry, input_topic, self.cb_odom, 50)
        self.pub = self.create_publisher(VehicleOdometry, output_topic, 50)

        self.get_logger().info(f"vio_to_px4 started: {input_topic} -> {output_topic}")

    def _timestamp_us(self, odom: Odometry) -> int:
        if self.use_ros_time:
            ns = self.get_clock().now().nanoseconds
        else:
            ns = odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec
        return int(ns // 1000)

    @staticmethod
    def _enu_to_ned_vec(x: float, y: float, z: float):
        return y, x, -z

    @staticmethod
    def _flu_to_frd_vec(x: float, y: float, z: float):
        return x, -y, -z

    @staticmethod
    def _convert_body_to_world_quat_enu_flu_to_ned_frd(qx: float, qy: float, qz: float, qw: float):
        # M maps NED<->ENU and FRD<->FLU when axis order/signs are applied.
        m = [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
        r_enu = quat_to_rot(qx, qy, qz, qw)

        # R_ned_frd = M * R_enu_flu * M
        tmp = [[0.0, 0.0, 0.0] for _ in range(3)]
        r_ned = [[0.0, 0.0, 0.0] for _ in range(3)]
        for i in range(3):
            for j in range(3):
                tmp[i][j] = sum(m[i][k] * r_enu[k][j] for k in range(3))
        for i in range(3):
            for j in range(3):
                r_ned[i][j] = sum(tmp[i][k] * m[k][j] for k in range(3))

        return rot_to_quat(r_ned)

    def cb_odom(self, odom: Odometry) -> None:
        msg = VehicleOdometry()

        ts = self._timestamp_us(odom)
        msg.timestamp = ts
        msg.timestamp_sample = ts

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD

        px = float(odom.pose.pose.position.x)
        py = float(odom.pose.pose.position.y)
        pz = float(odom.pose.pose.position.z)
        nx, ny, nz = self._enu_to_ned_vec(px, py, pz)
        msg.position = [nx, ny, nz]

        qx = float(odom.pose.pose.orientation.x)
        qy = float(odom.pose.pose.orientation.y)
        qz = float(odom.pose.pose.orientation.z)
        qw = float(odom.pose.pose.orientation.w)
        nqx, nqy, nqz, nqw = self._convert_body_to_world_quat_enu_flu_to_ned_frd(qx, qy, qz, qw)
        msg.q = [nqw, nqx, nqy, nqz]

        vx = float(odom.twist.twist.linear.x)
        vy = float(odom.twist.twist.linear.y)
        vz = float(odom.twist.twist.linear.z)
        bvx, bvy, bvz = self._flu_to_frd_vec(vx, vy, vz)
        msg.velocity = [bvx, bvy, bvz]

        wx = float(odom.twist.twist.angular.x)
        wy = float(odom.twist.twist.angular.y)
        wz = float(odom.twist.twist.angular.z)
        bwx, bwy, bwz = self._flu_to_frd_vec(wx, wy, wz)
        msg.angular_velocity = [bwx, bwy, bwz]

        # covariance in nav_msgs/Odometry pose covariance order:
        # [x, y, z, rotX, rotY, rotZ], with row-major 6x6 layout
        c = odom.pose.covariance
        var_x = float(c[0])
        var_y = float(c[7])
        var_z = float(c[14])
        var_roll = float(c[21])
        var_pitch = float(c[28])
        var_yaw = float(c[35])

        # ENU->NED swaps x/y and flips z sign (variance stays positive)
        msg.position_variance = [var_y, var_x, var_z]
        msg.orientation_variance = [var_pitch, var_roll, var_yaw]

        tc = odom.twist.covariance
        tvar_x = float(tc[0])
        tvar_y = float(tc[7])
        tvar_z = float(tc[14])
        msg.velocity_variance = [tvar_x, tvar_y, tvar_z]

        msg.reset_counter = max(0, min(255, self.reset_counter))
        msg.quality = max(-1, min(100, self.quality))

        finite = [
            *msg.position,
            *msg.q,
            *msg.velocity,
            *msg.angular_velocity,
            *msg.position_variance,
            *msg.orientation_variance,
            *msg.velocity_variance,
        ]
        if not all(math.isfinite(v) for v in finite):
            return

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = VioToPx4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
