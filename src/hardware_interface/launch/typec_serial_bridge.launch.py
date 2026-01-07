#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyTHS1',
            description='Type-C串口设备路径'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='串口波特率'
        ),
        
        DeclareLaunchArgument(
            'send_frequency',
            default_value='10.0',
            description='数据发送频率(Hz)'
        ),
        
        # Type-C串口桥接节点
        Node(
            package='hardware_interface',
            executable='typec_serial_bridge_node',
            name='typec_serial_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'send_frequency': LaunchConfiguration('send_frequency'),
                'read_timeout': 0.1
            }],
            remappings=[
                ('/chassis_feedback', '/chassis_feedback'),
                ('/typec_connection_status', '/typec_connection_status'),
                ('/steer_angle', '/steer_angle'),
                ('/wheel_speed', '/wheel_speed')
            ]
        )
    ]) 