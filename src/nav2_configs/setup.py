from setuptools import setup
import os
from glob import glob

package_name = 'nav2_configs'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aurora ROS2 Team',
    maintainer_email='aurora@ros2team.com',
    description='Nav2配置包 - 专为Aurora激光雷达 + Acamana阿克曼底盘设计',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_cmd_vel_bridge.py = nav2_configs.ackermann_cmd_vel_bridge:main',
        ],
    },
) 