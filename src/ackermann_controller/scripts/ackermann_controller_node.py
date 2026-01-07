#!/usr/bin/env python3
"""
🎯 增强的阿克曼控制器节点
基于优化规划第一阶段：集成增强运动学模型，优化Pure Pursuit算法
Author: Acamana-Bot Development Team  
Date: 2025-01-15
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float32, Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from threading import Lock
import time
import yaml
import os

# 导入增强的运动学模型
from ackermann_controller.enhanced_kinematics import EnhancedAckermannKinematics


class EnhancedAckermannController(Node):
    """增强的阿克曼控制器"""
    
    def __init__(self):
        super().__init__('enhanced_ackermann_controller')
        
        # 🔧 加载统一配置参数
        self._load_unified_parameters()
        
        # 🚗 初始化增强运动学模型
        self.kinematics = EnhancedAckermannKinematics(
            wheelbase=self.wheelbase,
            max_steer_angle=self.max_steer_angle,
            min_turning_radius=self.min_turning_radius
        )
        
        # 📊 控制状态变量
        self.current_pose = None
        self.current_velocity = 0.0
        self.current_steering_angle = 0.0
        self.last_cmd_time = None
        self.is_following_path = False
        self.goal_reached = False
        self.stuck_recovery_active = False
        self.stuck_start_time = None
        self.stuck_attempts = 0
        
        # 🎯 路径跟踪变量
        self.current_path = None
        self.current_waypoint_index = 0
        self.lookahead_point = None
        
        # 🔒 线程安全
        self.state_lock = Lock()
        
        # 📡 TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 🎛️ 创建订阅者
        self._setup_subscribers()
        
        # 📤 创建发布者
        self._setup_publishers()
        
        # ⏰ 创建定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, 
            self.control_loop
        )
        
        # 📈 性能监控定时器
        self.monitor_timer = self.create_timer(
            1.0,  # 每秒监控一次
            self.performance_monitor
        )
        
        self.get_logger().info("🎯 增强阿克曼控制器已启动")
        self.get_logger().info(f"   轴距: {self.wheelbase:.3f}m")
        self.get_logger().info(f"   最大转向角: {math.degrees(self.max_steer_angle):.1f}°")
        self.get_logger().info(f"   最小转弯半径: {self.min_turning_radius:.3f}m")

    def _load_unified_parameters(self):
        """加载统一的配置参数"""
        # 🔗 尝试加载acamana_params.yaml中的统一参数
        try:
            config_path = os.path.join(
                self.get_namespace() if self.get_namespace() != '/' else '',
                'src/acamana/config/acamana_params.yaml'
            )
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    config = yaml.safe_load(file)
                    physical_params = config.get('physical_properties', {}).get('ros__parameters', {})
                    
                    # 从统一配置加载参数
                    self.wheelbase = physical_params.get('wheelbase', 0.21333)
                    self.max_steer_angle = physical_params.get('max_steer_angle', 0.6108)
                    self.min_turning_radius = physical_params.get('min_turning_radius', 0.21333)
                    self.max_speed = physical_params.get('max_linear_velocity', 0.6)
                    self.min_speed = physical_params.get('min_speed', 0.1)
                    self.lookahead_distance_base = physical_params.get('lookahead_distance_base', 0.8)
                    self.lookahead_velocity_scaling = physical_params.get('lookahead_velocity_scaling', 0.5)
                    self.steering_filter_alpha = physical_params.get('steering_filter_alpha', 0.3)
                    self.max_steering_rate = physical_params.get('max_steering_rate', 2.0)
                    
                    self.get_logger().info("✅ 已加载统一配置参数")
            else:
                raise FileNotFoundError("统一配置文件不存在")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ 无法加载统一配置: {e}，使用默认参数")
            # 后备参数声明
            self._declare_fallback_parameters()
    
    def _declare_fallback_parameters(self):
        """声明后备参数"""
        self.declare_parameter('wheelbase', 0.21333)
        self.declare_parameter('max_steer_angle', 0.6108)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('min_speed', 0.1)
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('control_timeout', 2.0)
        self.declare_parameter('smooth_factor', 0.7)
        self.declare_parameter('max_angular_velocity', 0.5)
        
        # 加载参数
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.min_turning_radius = self.wheelbase / math.tan(self.max_steer_angle)
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.lookahead_distance_base = self.get_parameter('lookahead_distance').value
        self.lookahead_velocity_scaling = 0.5
        self.steering_filter_alpha = 0.3
        self.max_steering_rate = 2.0
        
        # 其他控制参数
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.control_timeout = self.get_parameter('control_timeout').value
        self.smooth_factor = self.get_parameter('smooth_factor').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # 脱困参数
        self.declare_parameter('stuck_backup_speed', -0.15)
        self.declare_parameter('stuck_backup_time', 1.0)
        self.declare_parameter('stuck_max_attempts', 3)
        self.declare_parameter('stuck_turn_time', 1.5)
        self.declare_parameter('stuck_realign_time', 0.5)
        
        self.stuck_backup_speed = self.get_parameter('stuck_backup_speed').value
        self.stuck_backup_time = self.get_parameter('stuck_backup_time').value
        self.stuck_max_attempts = self.get_parameter('stuck_max_attempts').value
        self.stuck_turn_time = self.get_parameter('stuck_turn_time').value
        self.stuck_realign_time = self.get_parameter('stuck_realign_time').value

    def _setup_subscribers(self):
        """设置订阅者"""
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

    def _setup_publishers(self):
        """设置发布者"""
        self.steer_angle_pub = self.create_publisher(
            Float32,
            '/steer_angle',
            10)
        
        self.wheel_speed_pub = self.create_publisher(
            Float32,
            '/wheel_speed',
            10)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.goal_reached_pub = self.create_publisher(
            Bool,
            '/goal_reached',
            10)
        
        self.current_waypoint_pub = self.create_publisher(
            PoseStamped,
            '/current_waypoint',
            10)

    def path_callback(self, msg):
        """路径回调函数"""
        with self.state_lock:
            self.current_path = msg
            self.current_waypoint_index = 0
            self.is_following_path = True
            self.goal_reached = False
            
            # 验证路径是否满足阿克曼约束
            is_valid, invalid_indices = self.kinematics.validate_path_curvature(msg)
            if not is_valid:
                self.get_logger().warn(f"⚠️ 路径包含{len(invalid_indices)}个违反转弯半径约束的点")
                # 平滑路径
                self.current_path = self.kinematics.smooth_path_curvature(msg)
                self.get_logger().info("✅ 路径已平滑处理")
        
        self.get_logger().info(f"📍 接收到新路径，包含{len(msg.poses)}个路径点")

    def odom_callback(self, msg):
        """里程计回调函数"""
        with self.state_lock:
            self.current_pose = msg.pose.pose
            self.current_velocity = math.sqrt(
                msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
            )
            self.last_cmd_time = self.get_clock().now()

    def goal_callback(self, msg):
        """目标点回调函数"""
        self.get_logger().info(f"🎯 接收到新目标点: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def calculate_dynamic_lookahead_distance(self) -> float:
        """计算动态前瞻距离"""
        base_distance = self.lookahead_distance_base
        velocity_scaling = self.lookahead_velocity_scaling * self.current_velocity
        
        dynamic_distance = base_distance + velocity_scaling
        
        # 限制在合理范围内
        min_distance = 0.3
        max_distance = 2.0
        
        return np.clip(dynamic_distance, min_distance, max_distance)

    def find_lookahead_point(self) -> Tuple[Point, int]:
        """寻找前瞻点"""
        if not self.current_path or not self.current_pose:
            return None, -1
        
        lookahead_distance = self.calculate_dynamic_lookahead_distance()
        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        
        # 从当前waypoint开始搜索
        for i in range(self.current_waypoint_index, len(self.current_path.poses)):
            waypoint_pos = np.array([
                self.current_path.poses[i].pose.position.x,
                self.current_path.poses[i].pose.position.y
            ])
            
            distance = np.linalg.norm(waypoint_pos - current_pos)
            
            if distance >= lookahead_distance:
                return self.current_path.poses[i].pose.position, i
        
        # 如果没找到合适距离的点，返回最后一个点
        if len(self.current_path.poses) > 0:
            return self.current_path.poses[-1].pose.position, len(self.current_path.poses) - 1
        
        return None, -1

    def calculate_pure_pursuit_control(self) -> Tuple[float, float]:
        """计算Pure Pursuit控制量"""
        if not self.current_pose:
            return 0.0, 0.0
        
        lookahead_point, waypoint_idx = self.find_lookahead_point()
        if lookahead_point is None:
            return 0.0, 0.0
        
        self.lookahead_point = lookahead_point
        
        # 计算到前瞻点的距离和角度
        dx = lookahead_point.x - self.current_pose.position.x
        dy = lookahead_point.y - self.current_pose.position.y
        lookahead_distance = math.sqrt(dx*dx + dy*dy)
        
        # 当前航向角
        current_yaw = self._get_yaw_from_pose(self.current_pose)
        
        # 目标航向角
        target_yaw = math.atan2(dy, dx)
        
        # 计算航向误差
        yaw_error = target_yaw - current_yaw
        # 角度归一化
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Pure Pursuit公式计算转向角
        if lookahead_distance > 1e-6:
            curvature = 2.0 * math.sin(yaw_error) / lookahead_distance
            steering_angle = math.atan(self.wheelbase * curvature)
        else:
            steering_angle = 0.0
        
        # 限制转向角
        steering_angle = np.clip(steering_angle, -self.max_steer_angle, self.max_steer_angle)
        
        # 自适应速度控制
        speed = self._calculate_adaptive_speed(steering_angle, lookahead_distance)
        
        return speed, steering_angle

    def _calculate_adaptive_speed(self, steering_angle: float, lookahead_distance: float) -> float:
        """计算自适应速度"""
        # 基础速度
        base_speed = self.max_speed
        
        # 根据转向角调整速度（转弯时减速）
        curvature_factor = abs(steering_angle) / self.max_steer_angle
        curvature_speed_reduction = curvature_factor * 0.5  # 最大减速50%
        
        # 根据前瞻距离调整速度（距离短时减速）
        distance_factor = min(lookahead_distance / self.lookahead_distance_base, 1.0)
        distance_speed_factor = 0.3 + 0.7 * distance_factor  # 最低30%速度
        
        # 计算最终速度
        final_speed = base_speed * (1.0 - curvature_speed_reduction) * distance_speed_factor
        
        # 确保最小速度
        final_speed = max(final_speed, self.min_speed)
        
        return final_speed

    def smooth_steering_command(self, target_steering: float) -> float:
        """平滑转向指令"""
        if self.current_steering_angle is None:
            self.current_steering_angle = target_steering
            return target_steering
        
        # 计算转向速率限制
        dt = 1.0 / self.control_frequency
        max_change = self.max_steering_rate * dt
        
        steering_diff = target_steering - self.current_steering_angle
        
        # 限制变化率
        if abs(steering_diff) > max_change:
            steering_diff = math.copysign(max_change, steering_diff)
        
        # 应用平滑滤波
        smoothed_steering = self.current_steering_angle + self.steering_filter_alpha * steering_diff
        
        self.current_steering_angle = smoothed_steering
        return smoothed_steering

    def control_loop(self):
        """主控制循环"""
        if not self.is_following_path or not self.current_path or not self.current_pose:
            return
        
        # 检查是否到达目标
        if self._check_goal_reached():
            self._handle_goal_reached()
            return
        
        # 检查是否卡住
        if self._check_stuck():
            self._handle_stuck_recovery()
            return
        
        # 计算Pure Pursuit控制
        speed, steering_angle = self.calculate_pure_pursuit_control()
        
        # 平滑转向指令
        smoothed_steering = self.smooth_steering_command(steering_angle)
        
        # 发布控制指令
        self._publish_control_commands(speed, smoothed_steering)
        
        # 发布当前前瞻点
        self._publish_current_waypoint()

    def _check_goal_reached(self) -> bool:
        """检查是否到达目标"""
        if not self.current_path or len(self.current_path.poses) == 0:
            return False
        
        goal_pose = self.current_path.poses[-1].pose.position
        current_pos = self.current_pose.position
        
        distance = math.sqrt(
            (goal_pose.x - current_pos.x)**2 + 
            (goal_pose.y - current_pos.y)**2
        )
        
        return distance < self.goal_tolerance

    def _handle_goal_reached(self):
        """处理目标到达"""
        with self.state_lock:
            self.is_following_path = False
            self.goal_reached = True
        
        # 停止车辆
        self._publish_control_commands(0.0, 0.0)
        
        # 发布目标到达消息
        goal_msg = Bool()
        goal_msg.data = True
        self.goal_reached_pub.publish(goal_msg)
        
        self.get_logger().info("🎯 目标已到达！")

    def _check_stuck(self) -> bool:
        """检查是否卡住"""
        if self.current_velocity < 0.05 and not self.stuck_recovery_active:
            if self.stuck_start_time is None:
                self.stuck_start_time = time.time()
            elif time.time() - self.stuck_start_time > 3.0:  # 3秒无运动
                return True
        else:
            self.stuck_start_time = None
        
        return False

    def _handle_stuck_recovery(self):
        """处理卡住恢复"""
        if self.stuck_attempts >= self.stuck_max_attempts:
            self.get_logger().error("❌ 脱困尝试次数已达上限，停止运行")
            self.is_following_path = False
            return
        
        self.get_logger().warn(f"⚠️ 检测到卡住，开始第{self.stuck_attempts + 1}次脱困")
        self.stuck_recovery_active = True
        
        # 简单的后退脱困策略（适用于阿克曼车）
        self._publish_control_commands(self.stuck_backup_speed, 0.0)
        
        # 设置脱困完成的定时器
        self.create_timer(
            self.stuck_backup_time,
            lambda: self._finish_stuck_recovery(),
            once=True
        )

    def _finish_stuck_recovery(self):
        """完成脱困"""
        self.stuck_recovery_active = False
        self.stuck_attempts += 1
        self.stuck_start_time = None
        self.get_logger().info("✅ 脱困完成，继续路径跟踪")

    def _publish_control_commands(self, speed: float, steering_angle: float):
        """发布控制指令"""
        # 发布转向角
        steer_msg = Float32()
        steer_msg.data = steering_angle
        self.steer_angle_pub.publish(steer_msg)
        
        # 发布轮速
        speed_msg = Float32()
        speed_msg.data = speed
        self.wheel_speed_pub.publish(speed_msg)
        
        # 同时发布Twist消息用于兼容
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        
        # 根据阿克曼模型计算角速度
        if abs(steering_angle) > 1e-6:
            turning_radius = self.kinematics.calculate_turning_radius(steering_angle)
            cmd_vel.angular.z = speed / turning_radius
            if steering_angle < 0:
                cmd_vel.angular.z = -cmd_vel.angular.z
        else:
            cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)

    def _publish_current_waypoint(self):
        """发布当前前瞻点"""
        if self.lookahead_point:
            waypoint_msg = PoseStamped()
            waypoint_msg.header.frame_id = "map"
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()
            waypoint_msg.pose.position = self.lookahead_point
            waypoint_msg.pose.orientation.w = 1.0
            
            self.current_waypoint_pub.publish(waypoint_msg)

    def performance_monitor(self):
        """性能监控"""
        if not self.is_following_path:
            return
        
        # 计算路径偏差
        if self.current_path and self.current_pose:
            deviation = self._calculate_path_deviation()
            if deviation > 0.15:  # 15cm偏差告警
                self.get_logger().warn(f"⚠️ 路径偏差较大: {deviation:.3f}m")

    def _calculate_path_deviation(self) -> float:
        """计算当前位置到路径的偏差"""
        if not self.current_path or len(self.current_path.poses) < 2:
            return 0.0
        
        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        min_distance = float('inf')
        
        # 找到最近的路径段
        for i in range(len(self.current_path.poses) - 1):
            p1 = np.array([
                self.current_path.poses[i].pose.position.x,
                self.current_path.poses[i].pose.position.y
            ])
            p2 = np.array([
                self.current_path.poses[i+1].pose.position.x,
                self.current_path.poses[i+1].pose.position.y
            ])
            
            # 计算点到线段的距离
            distance = self._point_to_line_segment_distance(current_pos, p1, p2)
            min_distance = min(min_distance, distance)
        
        return min_distance

    def _point_to_line_segment_distance(self, point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray) -> float:
        """计算点到线段的距离"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        line_len = np.linalg.norm(line_vec)
        if line_len < 1e-6:
            return np.linalg.norm(point_vec)
        
        line_unit = line_vec / line_len
        proj_length = np.dot(point_vec, line_unit)
        
        if proj_length < 0:
            return np.linalg.norm(point_vec)
        elif proj_length > line_len:
            return np.linalg.norm(point - line_end)
        else:
            proj_point = line_start + proj_length * line_unit
            return np.linalg.norm(point - proj_point)

    def _get_yaw_from_pose(self, pose: Pose) -> float:
        """从位姿中提取航向角"""
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedAckermannController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 