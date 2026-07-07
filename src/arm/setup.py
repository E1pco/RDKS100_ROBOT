from setuptools import find_packages, setup
from glob import glob

package_name = 'arm'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    package_data={
        package_name: ['driver/*.json'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.com',
    description='Arm servo driver, IK solver and follower teleoperation node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follower_node = arm.follower_node:main',
            'camera_node = arm.camera_node:main',
            'dosod_filter_node = arm.dosod_filter_node:main',
            'feature_manager_node = arm.feature_manager_node:main',
            'rgb_img_republisher_node = arm.rgb_img_republisher_node:main',
            'mvs_jpeg_republisher_node = arm.mvs_jpeg_republisher_node:main',
            'livox_raw_cloud_node = arm.livox_raw_cloud_node:main',
        ],
    },
)
