from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hardware_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = hardware_interface.serial_bridge_node:main',
            'twist_to_ackermann_node = hardware_interface.twist_to_ackermann_node:main',
            'typec_serial_bridge_node = hardware_interface.typec_serial_bridge_node:main',
            'tf_bridge_node = hardware_interface.tf_bridge_node:main',
            # 添加scripts目录下的Python脚本
            'ackermann_controller = hardware_interface.ackermann_controller:main',
            'motor_monitor_node = hardware_interface.motor_monitor_node:main',
        ],
    },
)
