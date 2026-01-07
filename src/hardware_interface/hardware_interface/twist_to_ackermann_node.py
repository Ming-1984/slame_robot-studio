#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class TwistToAckermannNode(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann_node')
        
        # --- 参数 ---
        self.declare_parameter('wheelbase', 0.21333)  # 轴距（单位：米）- 统一参数
        self.declare_parameter('max_steer_angle', 0.6108)  # 最大转向角（单位：弧度）- 统一参数
        self.declare_parameter('min_speed', 0.05)  # 最小速度阈值
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.min_speed = self.get_parameter('min_speed').value
        
        self.get_logger().info(f"轴距: {self.wheelbase}m, 最大转向角: {self.max_steer_angle}rad")
        
        # --- 订阅者 ---
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # --- 发布者 ---
        self.steer_pub = self.create_publisher(
            Float32,
            '/steer_angle',
            10)
            
        self.speed_pub = self.create_publisher(
            Float32,
            '/wheel_speed',
            10)
    
    def cmd_vel_callback(self, msg):
        """将cmd_vel转换为阿克曼控制信号"""
        linear_x = msg.linear.x   # 前进速度
        angular_z = msg.angular.z  # 角速度
        
        # 速度转换 - 直接使用线速度
        wheel_speed = linear_x
        
        # 转向角计算 - 使用自行车模型公式：tan(steer_angle) = wheelbase * angular_z / linear_x
        if abs(linear_x) > self.min_speed:
            # 当速度足够大时，使用标准阿克曼公式
            steer_angle = math.atan2(angular_z * self.wheelbase, linear_x)
        elif abs(angular_z) > 0.001:
            # 速度很小但有转向需求时，根据角速度方向设置最大转向角
            steer_angle = self.max_steer_angle if angular_z > 0 else -self.max_steer_angle
        else:
            # 速度很小且无转向需求，保持直行
            steer_angle = 0.0
        
        # 限制转向角在合理范围内
        steer_angle = max(-self.max_steer_angle, min(self.max_steer_angle, steer_angle))
        
        # 发布消息
        steer_msg = Float32()
        steer_msg.data = steer_angle
        self.steer_pub.publish(steer_msg)
        
        speed_msg = Float32()
        speed_msg.data = wheel_speed
        self.speed_pub.publish(speed_msg)
        
        # self.get_logger().debug(f"转换: 线速度={linear_x}, 角速度={angular_z} → 转向角={steer_angle}, 车轮速度={wheel_speed}")

def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 