from setuptools import find_packages, setup

package_name = "imu_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/imu_bridge.launch.py"]),
        ("share/" + package_name + "/config", ["config/imu_bridge.yaml"]),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="he",
    maintainer_email="he@local",
    description="Bridge SensorCombined to sensor_msgs/Imu",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_bridge_node = imu_bridge.imu_bridge_node:main",
        ],
    },
)
