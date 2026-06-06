from setuptools import setup, find_packages

package_name = "ackermann_serial_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/serial_bridge.launch.py",
            "launch/cmd_vel_republisher.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/ackermann_serial_bridge.yaml",
            "config/cmd_vel_republisher.yaml",
        ]),
    ],
    install_requires=["setuptools", "pyserial"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="ROS2 serial bridge for WHEELTEC Ackermann chassis",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_bridge_node = ackermann_serial_bridge.serial_bridge_node:main",
            "cmd_vel_republisher = ackermann_serial_bridge.cmd_vel_republisher:main",
        ],
    },
)
