#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.11.1',
        description='Aurora雷达的IP地址'
    )
    
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.3',
        description='机器人半径(m)'
    )
    
    cell_size_arg = DeclareLaunchArgument(
        'cell_size',
        default_value='0.2',
        description='采样网格大小(m)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='是否自动开始规划'
    )
    
    path_smoothing_arg = DeclareLaunchArgument(
        'path_smoothing',
        default_value='true',
        description='是否启用路径平滑'
    )
    
    smoothing_factor_arg = DeclareLaunchArgument(
        'smoothing_factor',
        default_value='0.2',
        description='路径平滑系数 (0.0-1.0)'
    )
    
    optimize_path_arg = DeclareLaunchArgument(
        'optimize_path',
        default_value='true',
        description='是否优化路径'
    )
    
    coverage_overlap_arg = DeclareLaunchArgument(
        'coverage_overlap',
        default_value='0.5',
        description='覆盖重叠系数 (0.0-1.0)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 获取配置
    ip_address = LaunchConfiguration('ip_address')
    robot_radius = LaunchConfiguration('robot_radius')
    cell_size = LaunchConfiguration('cell_size')
    auto_start = LaunchConfiguration('auto_start')
    path_smoothing = LaunchConfiguration('path_smoothing')
    smoothing_factor = LaunchConfiguration('smoothing_factor')
    optimize_path = LaunchConfiguration('optimize_path')
    coverage_overlap = LaunchConfiguration('coverage_overlap')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 启动Aurora雷达节点
    slamware_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slamware_ros_sdk'),
                'launch',
                'slamware_ros_sdk_server_node.py'
            ])
        ]),
        launch_arguments={
            'ip_address': ip_address,
            'map_topic': 'map',
            'map_pub_period': '0.1',
            'scan_pub_period': '0.05',
            'robot_pose_pub_period': '0.05',
            'map_frame': 'map',
            'robot_frame': 'base_link'
        }.items()
    )
    
    # 启动Nav2导航堆栈
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_configs'),
                'launch',
                'nav2_ackermann_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )
    
    # 启动可视化
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('room_coverage_planner'),
            'config',
            'coverage.rviz'
        ])]
    )
    
    # 启动覆盖路径规划节点
    planner_node = Node(
        package='room_coverage_planner',
        executable='coverage_planner_node',
        name='coverage_planner',
        parameters=[{
            'robot_radius': robot_radius,
            'cell_size': cell_size,
            'min_path_points': 10,
            'free_space_threshold': 0,
            'auto_start': auto_start,
            'path_smoothing': path_smoothing,
            'smoothing_factor': smoothing_factor,
            'optimize_path': optimize_path,
            'dbscan_eps_factor': 2.0,
            'dbscan_min_samples': 3,
            'coverage_overlap': coverage_overlap,
            'min_dist_between_points': 0.1
        }],
        output='screen'
    )
    
    # 启动路径可视化节点
    visualizer_node = Node(
        package='room_coverage_planner',
        executable='path_visualizer',
        name='path_visualizer',
        parameters=[{
            'save_path': '~/path_images',
            'auto_save': True
        }],
        output='screen'
    )
    
    # 创建启动描述
    return LaunchDescription([
        ip_address_arg,
        robot_radius_arg,
        cell_size_arg,
        auto_start_arg,
        path_smoothing_arg,
        smoothing_factor_arg,
        optimize_path_arg,
        coverage_overlap_arg,
        use_sim_time_arg,
        slamware_sdk_launch,
        nav2_launch,
        planner_node,
        visualizer_node,
        rviz_launch
    ])
