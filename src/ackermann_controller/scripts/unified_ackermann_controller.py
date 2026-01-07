#!/usr/bin/env python3
"""
统一阿克曼底盘控制器 - 第二阶段优化版本
整合所有阿克曼控制功能，解决参数不一致和代码重复问题
专为Acamana机器人平台设计

Author: Acamana-Bot Development Team
Date: 2025-01-15
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import math
import numpy as np
from typing import Tuple, Optional
import time

# ROS2 消息类型
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class UnifiedAckermannController(Node):
    """统一阿克曼底盘控制器"""
    
    def __init__(self):
        super().__init__('unified_ackermann_controller')
        
        # 🔧 从统一配置文件加载参数
        self._load_parameters()
        
        # 📊 状态变量初始化
        self._init_state_variables()
        
        # 🔒 线程安全锁
        self.control_lock = threading.Lock()
        
        # 📡 创建订阅者和发布者
        self._setup_subscribers()
        self._setup_publishers()
        
        # ⏰ 控制定时器
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, 
            self._control_loop
        )
        
        # 📈 诊断定时器
        self.diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        
        # 🛡️ 安全监控定时器
        self.safety_timer = self.create_timer(0.1, self._safety_check)
        
        self.get_logger().info("🚗 统一阿克曼控制器已启动")
        self._log_configuration()
        
    def _load_parameters(self):
        """加载统一参数配置"""
        # 🎯 核心几何参数 (修正为正确值)
        self.declare_parameter('ackermann_vehicle.wheelbase', 0.2133)
        self.declare_parameter('ackermann_vehicle.max_steer_angle', 0.6108)
        self.declare_parameter('ackermann_vehicle.min_turning_radius', 0.3047)
        self.declare_parameter('ackermann_vehicle.track_width', 0.18)
        self.declare_parameter('ackermann_vehicle.wheel_radius', 0.05)
        
        # ⚡ 运动性能约束
        self.declare_parameter('ackermann_vehicle.max_linear_velocity', 0.6)
        self.declare_parameter('ackermann_vehicle.max_angular_velocity', 1.0)
        self.declare_parameter('ackermann_vehicle.max_acceleration', 0.8)
        self.declare_parameter('ackermann_vehicle.max_steering_rate', 1.5)
        
        # 🎛️ 控制参数
        self.declare_parameter('ackermann_vehicle.control_frequency', 10.0)
        self.declare_parameter('ackermann_vehicle.steering_filter_alpha', 0.2)
        self.declare_parameter('ackermann_vehicle.velocity_filter_alpha', 0.3)
        
        # 🛡️ 安全参数
        self.declare_parameter('safety_constraints.cmd_timeout', 0.5)
        self.declare_parameter('safety_constraints.max_steering_change_rate', 0.8)
        self.declare_parameter('safety_constraints.steering_deadband', 0.02)
        
        # 获取参数值
        self.wheelbase = self.get_parameter('ackermann_vehicle.wheelbase').value
        self.max_steer_angle = self.get_parameter('ackermann_vehicle.max_steer_angle').value
        self.min_turning_radius = self.get_parameter('ackermann_vehicle.min_turning_radius').value
        self.max_linear_velocity = self.get_parameter('ackermann_vehicle.max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('ackermann_vehicle.max_angular_velocity').value
        self.control_frequency = self.get_parameter('ackermann_vehicle.control_frequency').value
        self.steering_filter_alpha = self.get_parameter('ackermann_vehicle.steering_filter_alpha').value
        self.velocity_filter_alpha = self.get_parameter('ackermann_vehicle.velocity_filter_alpha').value
        self.cmd_timeout = 3.0  # 进一步延长超时时间，适应导航间歇性
        self.max_steering_rate = self.get_parameter('ackermann_vehicle.max_steering_rate').value
        self.steering_deadband = self.get_parameter('safety_constraints.steering_deadband').value
        
        # ✅ 参数一致性验证
        self._validate_parameters()
        
    def _validate_parameters(self):
        """验证参数一致性"""
        # 计算理论最小转弯半径
        theoretical_min_radius = self.wheelbase / math.tan(self.max_steer_angle)
        
        # 检查参数一致性
        radius_diff = abs(theoretical_min_radius - self.min_turning_radius)
        tolerance = 0.005  # 5mm容差
        
        if radius_diff > tolerance:
            self.get_logger().warn(
                f"❌ 参数不一致检测："
                f"\n  理论最小转弯半径: {theoretical_min_radius:.4f}m"
                f"\n  配置最小转弯半径: {self.min_turning_radius:.4f}m"
                f"\n  差异: {radius_diff:.4f}m (容差: {tolerance:.4f}m)"
            )
            # 使用理论值修正
            self.min_turning_radius = theoretical_min_radius
            self.get_logger().info(f"✅ 已自动修正最小转弯半径为: {self.min_turning_radius:.4f}m")
        
    def _init_state_variables(self):
        """初始化状态变量"""
        # 🎛️ 控制状态
        self.current_cmd_vel = Twist()
        self.current_steering_angle = 0.0
        self.current_wheel_speed = 0.0
        self.target_steering_angle = 0.0
        self.target_wheel_speed = 0.0
        
        # ⏰ 时间戳
        self.last_cmd_time = self.get_clock().now()
        self.last_control_time = self.get_clock().now()
        
        # 📊 状态标志
        self.emergency_stop = False
        self.system_ready = True
        self.cmd_vel_available = False
        
        # 📈 性能监控
        self.control_loop_count = 0
        self.max_control_latency = 0.0
        self.avg_control_latency = 0.0
        

        
    def _setup_subscribers(self):
        """设置订阅者"""
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
            
    def _setup_publishers(self):
        """设置发布者"""
        # 🚗 底盘控制输出
        self.wheel_speed_pub = self.create_publisher(Float32, '/wheel_speed', 10)
        self.steer_angle_pub = self.create_publisher(Float32, '/steer_angle', 10)
        
        # 📊 状态反馈
        self.vehicle_status_pub = self.create_publisher(Bool, '/vehicle_status', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
    def _cmd_vel_callback(self, msg: Twist):
        """处理速度指令回调"""
        with self.control_lock:
            self.current_cmd_vel = msg
            self.last_cmd_time = self.get_clock().now()
            self.cmd_vel_available = True

            # 调试：记录非零命令
            if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
                self.get_logger().info(
                    f"🎮 收到控制命令: v={msg.linear.x:.3f}m/s, ω={msg.angular.z:.3f}rad/s"
                )
            
    def _odom_callback(self, msg: Odometry):
        """里程计回调"""
        # 可用于状态估计和反馈控制
        pass
        
    def _control_loop(self):
        """主控制循环"""
        loop_start_time = time.time()
        
        with self.control_lock:
            # 🛡️ 安全检查
            if not self._safety_check_internal():
                self._emergency_stop()
                return
                
            # 🎯 计算控制指令
            if self.cmd_vel_available:
                wheel_speed, steering_angle = self._calculate_ackermann_control(
                    self.current_cmd_vel)
                
                # 🎛️ 应用平滑滤波
                self.target_wheel_speed = self._apply_velocity_filter(wheel_speed)
                self.target_steering_angle = self._apply_steering_filter(steering_angle)
                
                # 📤 发布控制指令
                self._publish_control_commands()
                
        # 📈 性能监控
        self._update_performance_metrics(loop_start_time)
        
    def _calculate_ackermann_control(self, cmd_vel: Twist) -> Tuple[float, float]:
        """
        计算阿克曼控制指令
        
        Args:
            cmd_vel: 目标速度指令
            
        Returns:
            (wheel_speed, steering_angle): 车轮速度和转向角
        """
        linear_x = cmd_vel.linear.x
        angular_z = cmd_vel.angular.z
        
        # 🚗 速度限制
        linear_x = np.clip(linear_x, -self.max_linear_velocity, self.max_linear_velocity)
        
        # 🎯 阿克曼运动学转换
        if abs(angular_z) < 1e-6:
            # 直线运动
            steering_angle = 0.0
        elif abs(linear_x) < 1e-6:
            # 原地转向 (阿克曼车无法真正原地转向)
            steering_angle = math.copysign(self.max_steer_angle, angular_z)
            linear_x = 0.1 * math.copysign(1.0, angular_z)  # 给予微小前进速度
        else:
            # 标准阿克曼公式: tan(δ) = L * ω / v
            steering_angle = math.atan(self.wheelbase * angular_z / linear_x)
            
        # 🔧 限制转向角
        steering_angle = np.clip(steering_angle, -self.max_steer_angle, self.max_steer_angle)
        
        # ✅ 验证转弯半径约束
        if abs(steering_angle) > self.steering_deadband:
            turning_radius = self.wheelbase / math.tan(abs(steering_angle))
            if turning_radius < self.min_turning_radius:
                # 重新计算满足约束的转向角
                steering_angle = math.atan(self.wheelbase / self.min_turning_radius)
                steering_angle = math.copysign(steering_angle, angular_z)
                
        return linear_x, steering_angle
        
    def _apply_velocity_filter(self, target_speed: float) -> float:
        """应用速度平滑滤波"""
        return (self.velocity_filter_alpha * target_speed + 
                (1 - self.velocity_filter_alpha) * self.current_wheel_speed)
                
    def _apply_steering_filter(self, target_angle: float) -> float:
        """应用转向角平滑滤波和速率限制"""
        # 低通滤波
        filtered_angle = (self.steering_filter_alpha * target_angle + 
                         (1 - self.steering_filter_alpha) * self.current_steering_angle)
        
        # 转向速率限制
        dt = 1.0 / self.control_frequency
        max_angle_change = self.max_steering_rate * dt
        angle_diff = filtered_angle - self.current_steering_angle
        
        if abs(angle_diff) > max_angle_change:
            filtered_angle = self.current_steering_angle + math.copysign(max_angle_change, angle_diff)
            
        return filtered_angle
        
    def _publish_control_commands(self):
        """发布控制指令"""
        # 🚗 车轮速度
        speed_msg = Float32()
        speed_msg.data = float(self.target_wheel_speed)
        self.wheel_speed_pub.publish(speed_msg)
        
        # 🎛️ 转向角
        steer_msg = Float32()
        steer_msg.data = float(self.target_steering_angle)
        self.steer_angle_pub.publish(steer_msg)
        
        # 更新当前状态
        self.current_wheel_speed = self.target_wheel_speed
        self.current_steering_angle = self.target_steering_angle
        
    def _safety_check(self):
        """安全检查定时器回调"""
        # 这个方法由定时器调用，不需要加锁
        pass
        
    def _safety_check_internal(self) -> bool:
        """内部安全检查(已在锁内)"""
        current_time = self.get_clock().now()
        
        # 🛡️ 指令超时检查
        cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if cmd_age > self.cmd_timeout:
            if self.cmd_vel_available:
                self.get_logger().warn(f"⚠️ 指令超时: {cmd_age:.2f}s")
                self.cmd_vel_available = False
            return False
        
        return True
        
    def _emergency_stop(self):
        """紧急停车"""
        if not self.emergency_stop:
            self.get_logger().error("🚨 紧急停车激活")
            self.emergency_stop = True
            
        # 发布零速度指令
        speed_msg = Float32()
        speed_msg.data = 0.0
        self.wheel_speed_pub.publish(speed_msg)
        
        steer_msg = Float32()
        steer_msg.data = 0.0
        self.steer_angle_pub.publish(steer_msg)
        
    def _update_performance_metrics(self, loop_start_time: float):
        """更新性能指标"""
        latency = time.time() - loop_start_time
        self.control_loop_count += 1
        
        if latency > self.max_control_latency:
            self.max_control_latency = latency
            
        # 计算移动平均
        alpha = 0.1
        self.avg_control_latency = (alpha * latency + 
                                   (1 - alpha) * self.avg_control_latency)
                                   
    def _publish_diagnostics(self):
        """发布诊断信息"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # 🚗 控制器状态诊断
        status = DiagnosticStatus()
        status.name = "unified_ackermann_controller"
        status.hardware_id = "acamana_ackermann"
        
        if self.emergency_stop:
            status.level = DiagnosticStatus.ERROR
            status.message = "紧急停车状态"
        elif not self.cmd_vel_available:
            status.level = DiagnosticStatus.WARN
            status.message = "无指令输入"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "正常运行"
            
        # 添加关键数值
        status.values = [
            KeyValue(key="control_frequency", value=str(self.control_frequency)),
            KeyValue(key="max_latency_ms", value=f"{self.max_control_latency*1000:.2f}"),
            KeyValue(key="avg_latency_ms", value=f"{self.avg_control_latency*1000:.2f}"),
            KeyValue(key="current_speed", value=f"{self.current_wheel_speed:.3f}"),
            KeyValue(key="current_steer_angle", value=f"{math.degrees(self.current_steering_angle):.1f}"),
            KeyValue(key="loop_count", value=str(self.control_loop_count)),
        ]
        
        diag_array.status.append(status)
        self.diagnostic_pub.publish(diag_array)
        
    def _log_configuration(self):
        """记录配置信息"""
        self.get_logger().info("🔧 控制器配置:")
        self.get_logger().info(f"  轴距: {self.wheelbase:.4f}m")
        self.get_logger().info(f"  最大转向角: {math.degrees(self.max_steer_angle):.1f}°")
        self.get_logger().info(f"  最小转弯半径: {self.min_turning_radius:.4f}m")
        self.get_logger().info(f"  最大线速度: {self.max_linear_velocity:.2f}m/s")
        self.get_logger().info(f"  控制频率: {self.control_frequency:.1f}Hz")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        controller = UnifiedAckermannController()
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        controller.get_logger().info("🚀 统一阿克曼控制器启动完成")
        executor.spin()
        
    except KeyboardInterrupt:
        controller.get_logger().info("👋 用户中断")
    except Exception as e:
        controller.get_logger().error(f"❌ 运行错误: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        # 安全关闭：只有当context仍然有效时才调用shutdown
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 