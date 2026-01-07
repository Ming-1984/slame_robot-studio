from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Declare launch arguments
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.11.1'
    )
    
    raw_ladar_data_arg = DeclareLaunchArgument(
        'raw_ladar_data',
        default_value='false'
    )
    
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='map'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map'
    )
    
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link'
    )
    
    map_pub_period_arg = DeclareLaunchArgument(
        'map_pub_period',
        default_value='0.1'
    )
    
    scan_pub_period_arg = DeclareLaunchArgument(
        'scan_pub_period',
        default_value='0.05'
    )
    
    robot_pose_pub_period_arg = DeclareLaunchArgument(
        'robot_pose_pub_period',
        default_value='0.05'
    )
    
    # Create node
    slamware_node = Node(
        package='slamware_ros_sdk',
        executable='slamware_ros_sdk_server_node',
        name='slamware_ros_sdk_server_node',
        output='both',
        parameters=[{
            'ip_address': LaunchConfiguration('ip_address'),
            'angle_compensate': True,
            'raw_ladar_data': LaunchConfiguration('raw_ladar_data'),
            'map_frame': LaunchConfiguration('map_frame'),
            'robot_frame': LaunchConfiguration('robot_frame'),
            'odom_frame': 'odom',
            'laser_frame': 'laser',
            'imu_frame': 'imu_link',
            'camera_left': 'camera_left',
            'camera_right': 'camera_right',
            'robot_pose_pub_period': LaunchConfiguration('robot_pose_pub_period'),
            'scan_pub_period': LaunchConfiguration('scan_pub_period'),
            'map_pub_period': LaunchConfiguration('map_pub_period'),
            'imu_raw_data_period': 0.005,
            'ladar_data_clockwise': True,
            'robot_pose_topic': 'robot_pose',
        }],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('map', LaunchConfiguration('map_topic')),
            ('map_metadata', 'map_metadata'),
        ]
    )
    
    # Static transform publishers
    map2odom_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map2odom_static',
        arguments=['0', '0', '0', '0', '0', '0', '1', LaunchConfiguration('map_frame'), 'odom']
    )
    
    laser2base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser2base',
        arguments=['--x', '0', '--y', '0', '--z', '0.0315', 
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                   '--frame-id', LaunchConfiguration('robot_frame'), '--child-frame-id', 'laser']
    )
    
    leftcam2base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='leftcam2base',
        arguments=['--x', '0.0418', '--y', '0.03', '--z', '0', 
                   '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5', 
                   '--frame-id', LaunchConfiguration('robot_frame'), '--child-frame-id', 'camera_left']
    )
    
    rightcam2Leftcam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rightcam2Leftcam',
        arguments=['--x', '0.06', '--y', '0', '--z', '0', 
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                   '--frame-id', 'camera_left', '--child-frame-id', 'camera_right']
    )
    
    imu2Leftcam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu2Leftcam',
        arguments=['--x', '0.03', '--y', '0', '--z', '0', 
                   '--qx', '0', '--qy', '0', '--qz', '-0.7071068', '--qw', '0.7071068', 
                   '--frame-id', 'camera_left', '--child-frame-id', 'imu_link']
    )
    
    return LaunchDescription([
        ip_address_arg,
        raw_ladar_data_arg,
        map_topic_arg,
        map_frame_arg,
        robot_frame_arg,
        map_pub_period_arg,
        scan_pub_period_arg,
        robot_pose_pub_period_arg,
        slamware_node,
        map2odom_static,
        laser2base,
        leftcam2base,
        rightcam2Leftcam,
        imu2Leftcam
    ])
