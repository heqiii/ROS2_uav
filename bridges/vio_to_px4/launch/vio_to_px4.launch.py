from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory("vio_to_px4"), "config", "vio_to_px4.yaml")
    return LaunchDescription([
        Node(
            package="vio_to_px4",
            executable="vio_to_px4_node",
            name="vio_to_px4_node",
            output="screen",
            parameters=[config],
        )
    ])
