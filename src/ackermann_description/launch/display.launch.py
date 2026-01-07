from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 获取包路径
    pkg_share = FindPackageShare('ackermann_description').find('ackermann_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'ackermann_robot.urdf.xacro'])
    
    # 解析URDF
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ', 
            urdf_path
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # 启动节点
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'display.rviz'])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]) 