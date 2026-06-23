from setuptools import setup

package_name = "ackermann_serial_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/serial_bridge.launch.py",
            "launch/keyboard_teleop.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/ackermann_serial_bridge.yaml",
            "config/keyboard_teleop.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="ROS2 serial bridge for WHEELTEC Ackermann chassis",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_bridge_node = ackermann_serial_bridge.serial_bridge_node:main",
            "keyboard_teleop = ackermann_serial_bridge.keyboard_teleop:main",
        ],
    },
)
