from setuptools import setup, find_packages

package_name = 'ROS2_Remote'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: ['driver/*.json'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='dev@example.com',
    description='Leader-follower teleoperation over LAN using ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_node = ROS2_Remote.leader_node:main',
            'follower_node = ROS2_Remote.follower_node:main',
        ],
    },
)
