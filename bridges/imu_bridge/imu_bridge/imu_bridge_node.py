#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node

from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu


class ImuBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_bridge_node")

        self.declare_parameter("input_topic", "/fmu/out/sensor_combined")
        self.declare_parameter("output_topic", "/imu0")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("convert_frd_to_flu", True)
        self.declare_parameter("use_ros_time", True)
        self.declare_parameter("gyro_covariance_diag", [1.0e-3, 1.0e-3, 1.0e-3])
        self.declare_parameter("accel_covariance_diag", [4.0e-3, 4.0e-3, 4.0e-3])

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.convert_frd_to_flu = self.get_parameter("convert_frd_to_flu").get_parameter_value().bool_value
        self.use_ros_time = self.get_parameter("use_ros_time").get_parameter_value().bool_value
        self.gyro_covariance_diag = self._read_diag("gyro_covariance_diag", 1.0e-3)
        self.accel_covariance_diag = self._read_diag("accel_covariance_diag", 4.0e-3)

        self.pub = self.create_publisher(Imu, output_topic, 50)
        self.sub = self.create_subscription(SensorCombined, input_topic, self.cb_sensor, 100)

        self.get_logger().info(
            f"imu_bridge started: {input_topic} -> {output_topic}, convert_frd_to_flu={self.convert_frd_to_flu}"
        )

    def _read_diag(self, name: str, default_value: float) -> List[float]:
        values = [float(v) for v in self.get_parameter(name).value]
        if len(values) != 3:
            self.get_logger().warn(f"{name} must have 3 values, fallback to defaults")
            return [default_value, default_value, default_value]
        return values

    def _stamp(self, msg: SensorCombined):
        if self.use_ros_time:
            return self.get_clock().now().to_msg()

        sec = int(msg.timestamp // 1_000_000)
        nanosec = int((msg.timestamp % 1_000_000) * 1000)
        stamp = self.get_clock().now().to_msg()
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def cb_sensor(self, msg: SensorCombined) -> None:
        imu = Imu()
        imu.header.stamp = self._stamp(msg)
        imu.header.frame_id = self.frame_id

        gx, gy, gz = [float(x) for x in msg.gyro_rad]
        ax, ay, az = [float(x) for x in msg.accelerometer_m_s2]

        if self.convert_frd_to_flu:
            gy = -gy
            gz = -gz
            ay = -ay
            az = -az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 1.0

        imu.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        imu.angular_velocity_covariance = [
            self.gyro_covariance_diag[0], 0.0, 0.0,
            0.0, self.gyro_covariance_diag[1], 0.0,
            0.0, 0.0, self.gyro_covariance_diag[2],
        ]
        imu.linear_acceleration_covariance = [
            self.accel_covariance_diag[0], 0.0, 0.0,
            0.0, self.accel_covariance_diag[1], 0.0,
            0.0, 0.0, self.accel_covariance_diag[2],
        ]

        if not math.isfinite(imu.angular_velocity.x + imu.angular_velocity.y + imu.angular_velocity.z):
            return
        if not math.isfinite(imu.linear_acceleration.x + imu.linear_acceleration.y + imu.linear_acceleration.z):
            return

        self.pub.publish(imu)


def main() -> None:
    rclpy.init()
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
