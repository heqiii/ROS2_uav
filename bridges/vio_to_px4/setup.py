from setuptools import find_packages, setup

package_name = "vio_to_px4"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vio_to_px4.launch.py"]),
        ("share/" + package_name + "/config", ["config/vio_to_px4.yaml"]),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="he",
    maintainer_email="he@local",
    description="Bridge OpenVINS odometry to PX4 vehicle visual odometry topic",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vio_to_px4_node = vio_to_px4.vio_to_px4_node:main",
        ],
    },
)
