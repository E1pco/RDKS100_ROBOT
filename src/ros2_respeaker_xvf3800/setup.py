from glob import glob
from setuptools import find_packages, setup

package_name = "respeaker_xvf3800"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyusb"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="ROS 2 driver node for the reSpeaker XVF3800 USB 4-Mic Array DOA interface.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "doa_node = respeaker_xvf3800.doa_node:main",
        ],
    },
)
