#!/usr/bin/env python3
"""
🛡️ 稳定探索启动文件
基于2024年最新研究的容错探索系统
Author: Aurora Explorer Team
Date: 2025-01-20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    exploration_timeout_arg = DeclareLaunchArgument(
        'exploration_timeout',
        default_value='300.0',
        description='探索超时时间（秒）'
    )
    
    frontier_detection_interval_arg = DeclareLaunchArgument(
        'frontier_detection_interval', 
        default_value='3.0',
        description='前沿检测间隔（秒）'
    )
    
    min_frontier_size_arg = DeclareLaunchArgument(
        'min_frontier_size',
        default_value='5',
        description='最小前沿点大小'
    )
    
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.3',
        description='机器人半径（米）'
    )
    
    max_recovery_attempts_arg = DeclareLaunchArgument(
        'max_recovery_attempts',
        default_value='3',
        description='最大恢复尝试次数'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 稳定探索节点
    robust_explore_node = Node(
        package='aurora_explorer',
        executable='robust_explore_node.py',
        name='robust_explore_node',
        output='screen',
        parameters=[{
            'exploration_timeout': LaunchConfiguration('exploration_timeout'),
            'frontier_detection_interval': LaunchConfiguration('frontier_detection_interval'),
            'min_frontier_size': LaunchConfiguration('min_frontier_size'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'max_recovery_attempts': LaunchConfiguration('max_recovery_attempts'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel'),
            ('/navigate_to_pose', '/navigate_to_pose'),
        ]
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            '🛡️ 启动稳定探索系统\n',
            '基于2024年最新研究的容错设计\n',
            '特性:\n',
            '  - 行为树架构提供模块化和反应性\n',
            '  - 状态机提供稳定的错误恢复\n', 
            '  - 多层次异常处理机制\n',
            '  - 自适应前沿检测优化\n',
            '参数:\n',
            '  - 探索超时: ', LaunchConfiguration('exploration_timeout'), '秒\n',
            '  - 前沿检测间隔: ', LaunchConfiguration('frontier_detection_interval'), '秒\n',
            '  - 最小前沿大小: ', LaunchConfiguration('min_frontier_size'), '\n',
            '  - 机器人半径: ', LaunchConfiguration('robot_radius'), '米\n',
            '  - 最大恢复次数: ', LaunchConfiguration('max_recovery_attempts'), '\n'
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        exploration_timeout_arg,
        frontier_detection_interval_arg,
        min_frontier_size_arg,
        robot_radius_arg,
        max_recovery_attempts_arg,
        use_sim_time_arg,
        
        # 启动信息
        start_info,
        
        # 节点
        robust_explore_node,
    ])
