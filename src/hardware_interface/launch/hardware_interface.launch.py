#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取acamana包的共享目录以找到配置文件
    acamana_dir = get_package_share_directory('acamana')

    # 声明launch参数
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    send_frequency = LaunchConfiguration('send_frequency')
    use_stm32_protocol = LaunchConfiguration('use_stm32_protocol')
    
    # (新) 声明中心参数文件的路径
    acamana_params_file = LaunchConfiguration('acamana_params_file')
    
    return LaunchDescription([
        # 定义参数，设置默认值
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyTHS1',
            description='底盘控制串口设备'),
            
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='串口波特率'),
            
        DeclareLaunchArgument(
            'send_frequency',
            default_value='10.0',
            description='串口发送频率(Hz)'),
            
        DeclareLaunchArgument(
            'use_stm32_protocol',
            default_value='true',
            description='是否使用STM32协议'),
            
        # (新) 指向中心配置文件
        DeclareLaunchArgument(
            'acamana_params_file',
            default_value=os.path.join(acamana_dir, 'config', 'acamana_params.yaml'),
            description='Acamana核心参数文件路径'
        ),
        
        # 注释掉重复的阿克曼控制器 - 使用启动脚本中的unified_ackermann_controller.py
        # Node(
        #     package='hardware_interface',
        #     executable='ackermann_controller',
        #     name='ackermann_controller',
        #     parameters=[acamana_params_file],
        #     output='screen'
        # ),
        
        # 串口桥接节点
        Node(
            package='hardware_interface',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'send_frequency': send_frequency,
                'use_stm32_protocol': use_stm32_protocol,
                'read_timeout': 0.1
            }],
            output='screen'
        )
    ]) 