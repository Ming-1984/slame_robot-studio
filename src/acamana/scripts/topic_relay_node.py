#!/usr/bin/env python3
"""
Topic Relay Node (工程化版本)
- 从参数服务器读取Topic名称
- 将硬件驱动的Topic桥接到标准的ROS Topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

class TopicRelayNode(Node):
    def __init__(self):
        super().__init__('topic_relay_node')
        
        # 声明并获取参数
        self.declare_parameter('raw_scan_topic', '/slamware_ros_sdk_server_node/scan')
        self.declare_parameter('raw_odom_topic', '/slamware_ros_sdk_server_node/odom')
        self.declare_parameter('raw_map_topic', '/slamware_ros_sdk_server_node/map')
        self.declare_parameter('standard_scan_topic', '/scan')
        self.declare_parameter('standard_odom_topic', '/odom')
        self.declare_parameter('standard_map_topic', '/map')
        
        raw_scan_topic = self.get_parameter('raw_scan_topic').value
        raw_odom_topic = self.get_parameter('raw_odom_topic').value
        raw_map_topic = self.get_parameter('raw_map_topic').value
        standard_scan_topic = self.get_parameter('standard_scan_topic').value
        standard_odom_topic = self.get_parameter('standard_odom_topic').value
        standard_map_topic = self.get_parameter('standard_map_topic').value
        
        # 创建订阅者
        self.scan_sub = self.create_subscription(LaserScan, raw_scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, raw_odom_topic, self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, raw_map_topic, self.map_callback, 10)
        
        # 创建发布者
        self.scan_pub = self.create_publisher(LaserScan, standard_scan_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, standard_odom_topic, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, standard_map_topic, 10)
        
        self.get_logger().info('工程化的Topic中继节点已启动')
        self.get_logger().info(f'  Scan: {raw_scan_topic} -> {standard_scan_topic}')
        self.get_logger().info(f'  Odom: {raw_odom_topic} -> {standard_odom_topic}')
        self.get_logger().info(f'  Map:  {raw_map_topic} -> {standard_map_topic}')

    def scan_callback(self, msg):
        self.scan_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)

    def map_callback(self, msg):
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 安全关闭：只有当context仍然有效时才调用shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 