#!/usr/bin/env python3
"""
Aurora激光雷达路径规划系统 - 电机舵机监控节点
功能: 实时监控向电机和舵机发布的速度和角度信息
作者: Aurora ROS2 Team
日期: 2025-01-01
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import math
import os
from datetime import datetime
import json

class MotorMonitorNode(Node):
    def __init__(self):
        super().__init__('motor_monitor_node')
        
        # --- 参数配置 ---
        self.declare_parameter('display_frequency', 2.0)  # 显示频率 (Hz)
        self.declare_parameter('max_angle_degrees', 34.4)  # 最大转向角度(度)，对应0.6弧度
        self.declare_parameter('max_speed_mps', 1.0)       # 最大速度(m/s)
        self.declare_parameter('log_to_file', True)        # 是否保存日志到文件
        self.declare_parameter('log_directory', 'logs')    # 日志目录
        
        self.display_interval = 1.0 / self.get_parameter('display_frequency').value
        self.max_angle_degrees = self.get_parameter('max_angle_degrees').value
        self.max_speed_mps = self.get_parameter('max_speed_mps').value
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_directory = self.get_parameter('log_directory').value
        
        # --- 日志设置 ---
        self.setup_logging()
        
        # --- 数据存储 ---
        self.current_steer_angle = 0.0      # 当前转向角(弧度)
        self.current_wheel_speed = 0.0      # 当前轮速(m/s)
        self.current_cmd_vel = None         # 当前速度命令
        self.last_steer_time = None         # 上次转向命令时间
        self.last_speed_time = None         # 上次速度命令时间
        self.last_cmd_vel_time = None       # 上次cmd_vel时间
        
        # 统计信息
        self.steer_command_count = 0        # 转向命令计数
        self.speed_command_count = 0        # 速度命令计数
        self.cmd_vel_count = 0              # cmd_vel命令计数
        
        # --- 订阅者 ---
        self.steer_sub = self.create_subscription(
            Float32,
            '/steer_angle',
            self.steer_callback,
            10)
            
        self.speed_sub = self.create_subscription(
            Float32,
            '/wheel_speed',
            self.speed_callback,
            10)
            
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # 订阅底盘反馈和STM32数据 (如果需要)
        self.chassis_feedback_sub = self.create_subscription(
            String,
            '/chassis_feedback',
            self.chassis_feedback_callback,
            10)
            
        self.stm32_data_sub = self.create_subscription(
            String,
            '/stm32_data',
            self.stm32_data_callback,
            10)
        
        # --- (新) 诊断信息发布者 ---
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # --- 定时显示 ---
        self.display_timer = self.create_timer(self.display_interval, self.display_and_publish_status)
        
        # 启动信息
        self.get_logger().info("🎯 电机舵机监控节点已启动 (含诊断发布功能)")
        self.get_logger().info(f"   显示频率: {self.get_parameter('display_frequency').value}Hz")
        self.get_logger().info(f"   最大转向角: ±{self.max_angle_degrees}°")
        self.get_logger().info(f"   最大速度: ±{self.max_speed_mps}m/s")
        if self.log_to_file:
            self.get_logger().info(f"   日志保存: {self.log_file_path}")
        
    def setup_logging(self):
        """设置日志文件"""
        if not self.log_to_file:
            self.log_file_path = None
            self.log_file = None
            return
            
        # 创建日志目录
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)
            
        # 生成日志文件名（带时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(self.log_directory, f"motor_monitor_{timestamp}.log")
        
        # 打开日志文件
        try:
            self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
            # 写入文件头
            header = {
                "session_start": datetime.now().isoformat(),
                "description": "Aurora激光雷达路径规划系统 - 电机舵机监控日志",
                "format": "每行一个JSON对象，包含timestamp, topic, data等字段"
            }
            self.log_file.write(f"# {json.dumps(header, ensure_ascii=False)}\n")
            self.log_file.flush()
        except Exception as e:
            self.get_logger().error(f"创建日志文件失败: {e}")
            self.log_to_file = False
            self.log_file = None
    
    def log_data(self, topic, data, data_type="float"):
        """记录数据到日志文件"""
        if not self.log_to_file or self.log_file is None:
            return
            
        try:
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "topic": topic,
                "data": data,
                "data_type": data_type
            }
            self.log_file.write(f"{json.dumps(log_entry, ensure_ascii=False)}\n")
            self.log_file.flush()
        except Exception as e:
            self.get_logger().error(f"写入日志失败: {e}")
        
    def steer_callback(self, msg):
        """转向角度回调"""
        self.current_steer_angle = msg.data
        self.last_steer_time = time.time()
        self.steer_command_count += 1
        
        # 记录到日志
        self.log_data("/steer_angle", {
            "value_rad": msg.data,
            "value_degrees": self.rad_to_degrees(msg.data),
            "command_count": self.steer_command_count
        }, "steer_angle")
        
    def speed_callback(self, msg):
        """轮速回调"""
        self.current_wheel_speed = msg.data
        self.last_speed_time = time.time()
        self.speed_command_count += 1
        
        # 记录到日志
        self.log_data("/wheel_speed", {
            "value_mps": msg.data,
            "command_count": self.speed_command_count
        }, "wheel_speed")
        
    def cmd_vel_callback(self, msg):
        """cmd_vel回调"""
        self.current_cmd_vel = msg
        self.last_cmd_vel_time = time.time()
        self.cmd_vel_count += 1
        
        # 记录到日志
        self.log_data("/cmd_vel", {
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "linear_z": msg.linear.z,
            "angular_x": msg.angular.x,
            "angular_y": msg.angular.y,
            "angular_z": msg.angular.z,
            "command_count": self.cmd_vel_count
        }, "cmd_vel")
        
    def chassis_feedback_callback(self, msg):
        """底盘反馈回调"""
        # 记录到日志
        self.log_data("/chassis_feedback", {
            "message": msg.data
        }, "chassis_feedback")
        
    def stm32_data_callback(self, msg):
        """STM32数据回调"""
        # 记录到日志
        self.log_data("/stm32_data", {
            "message": msg.data
        }, "stm32_data")
    
    def rad_to_degrees(self, radians):
        """弧度转角度"""
        return radians * 180.0 / math.pi
    
    def get_progress_bar(self, value, max_value, width=20):
        """生成进度条"""
        if max_value == 0:
            return "━" * width
            
        # 计算填充比例
        ratio = abs(value) / max_value
        filled = int(ratio * width)
        
        # 生成进度条
        if value >= 0:
            bar = "█" * filled + "░" * (width - filled)
        else:
            bar = "░" * (width - filled) + "█" * filled
            
        return bar
    
    def get_status_indicator(self, last_time, timeout=2.0):
        """获取状态指示器"""
        if last_time is None:
            return "⚫"  # 无数据
        elif time.time() - last_time < timeout:
            return "🟢"  # 活跃
        else:
            return "🔴"  # 超时
    
    def display_and_publish_status(self):
        """
        核心函数：
        1. 在终端显示当前状态
        2. 将状态发布为标准的ROS2诊断消息
        """
        # --- 1. 显示状态 (代码保持不变) ---
        current_time = time.time()
        
        # 清屏（可选）
        print("\033[2J\033[H", end="")
        
        print("=" * 80)
        print("🤖 Aurora激光雷达路径规划系统 - 电机舵机监控")
        print("=" * 80)
        
        # --- 日志状态 ---
        if self.log_to_file and self.log_file_path:
            print(f"📝 日志保存: {self.log_file_path}")
        
        # --- 实时控制数据 ---
        print("\n📊 实时控制数据:")
        
        # 转向角度信息
        steer_degrees = self.rad_to_degrees(self.current_steer_angle)
        steer_status = self.get_status_indicator(self.last_steer_time)
        steer_bar = self.get_progress_bar(steer_degrees, self.max_angle_degrees)
        
        print(f"🎯 舵机转向角: {steer_status}")
        print(f"   角度值: {self.current_steer_angle:+.3f} rad ({steer_degrees:+.1f}°)")
        print(f"   进度条: [{steer_bar}] ({abs(steer_degrees)/self.max_angle_degrees*100:.1f}%)")
        print(f"   范围: ±{self.max_angle_degrees}° (±{self.max_angle_degrees*math.pi/180:.3f} rad)")
        
        # 速度信息
        speed_status = self.get_status_indicator(self.last_speed_time)
        speed_bar = self.get_progress_bar(self.current_wheel_speed, self.max_speed_mps)
        
        wheel_speed_str = f"   {speed_status} 速度: {self.current_wheel_speed:5.2f} m/s |{speed_bar}|"
        print(wheel_speed_str)
        
        # cmd_vel 信息
        cmd_vel_status = self.get_status_indicator(self.last_cmd_vel_time)
        if self.current_cmd_vel:
            cmd_vel_str = f"   {cmd_vel_status} Nav2指令 (Vx, Wz): {self.current_cmd_vel.linear.x:5.2f} m/s, {self.current_cmd_vel.angular.z:5.2f} rad/s"
        else:
            cmd_vel_str = f"   {cmd_vel_status} Nav2指令: 暂无"
        print(cmd_vel_str)
        
        # --- 统计信息 ---
        print(f"\n📈 统计信息:")
        print(f"   转向命令数: {self.steer_command_count}")
        print(f"   速度命令数: {self.speed_command_count}")
        print(f"   cmd_vel数: {self.cmd_vel_count}")
        
        # --- 状态指示 ---
        print(f"\n🔍 话题状态:")
        print(f"   /steer_angle:  {self.get_status_indicator(self.last_steer_time)} " +
              f"(最后更新: {self.format_time_ago(self.last_steer_time)})")
        print(f"   /wheel_speed:  {self.get_status_indicator(self.last_speed_time)} " +
              f"(最后更新: {self.format_time_ago(self.last_speed_time)})")
        print(f"   /cmd_vel:      {self.get_status_indicator(self.last_cmd_vel_time)} " +
              f"(最后更新: {self.format_time_ago(self.last_cmd_vel_time)})")
        
        # --- 控制提示 ---
        print(f"\n💡 提示:")
        print(f"   • 绿色🟢 = 话题活跃  红色🔴 = 话题超时  黑色⚫ = 无数据")
        print(f"   • 转向角正值 = 左转，负值 = 右转")
        print(f"   • 速度正值 = 前进，负值 = 后退")
        print(f"   • 按 Ctrl+C 退出监控")
        if self.log_to_file:
            print(f"   • 所有数据已保存到日志文件")
        
        print("=" * 80)
        
        # --- 2. 发布诊断消息 ---
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # 创建一个总的底盘诊断状态
        chassis_status = DiagnosticStatus()
        chassis_status.name = "Acamana Chassis"
        chassis_status.hardware_id = "acamana-jetson-orin"
        
        # 根据整体状态决定诊断等级
        if self.get_status_indicator(self.last_steer_time) == "🔴" or \
           self.get_status_indicator(self.last_speed_time) == "🔴":
            chassis_status.level = DiagnosticStatus.ERROR
            chassis_status.message = "底盘控制指令超时!"
        else:
            chassis_status.level = DiagnosticStatus.OK
            chassis_status.message = "运行正常"

        # 填充键值对
        chassis_status.values.append(KeyValue(key="Nav2 Cmd Vel Status", value=cmd_vel_status))
        chassis_status.values.append(KeyValue(key="Steer Cmd Status", value=steer_status))
        chassis_status.values.append(KeyValue(key="Speed Cmd Status", value=speed_status))
        chassis_status.values.append(KeyValue(key="Steer Angle (rad)", value=str(round(self.current_steer_angle, 3))))
        chassis_status.values.append(KeyValue(key="Wheel Speed (m/s)", value=str(round(self.current_wheel_speed, 3))))
        if self.current_cmd_vel:
            chassis_status.values.append(KeyValue(key="Nav2 Linear Vel X", value=str(round(self.current_cmd_vel.linear.x, 3))))
            chassis_status.values.append(KeyValue(key="Nav2 Angular Vel Z", value=str(round(self.current_cmd_vel.angular.z, 3))))
        
        diag_array.status.append(chassis_status)
        self.diagnostics_pub.publish(diag_array)
        
    def format_time_ago(self, timestamp):
        """格式化时间差"""
        if timestamp is None:
            return "从未"
        
        ago = time.time() - timestamp
        if ago < 1:
            return f"{ago*1000:.0f}ms前"
        elif ago < 60:
            return f"{ago:.1f}s前"
        else:
            return f"{ago/60:.1f}min前"
    
    def __del__(self):
        """析构函数，关闭日志文件"""
        if hasattr(self, 'log_file') and self.log_file is not None:
            try:
                # 写入会话结束标记
                end_entry = {
                    "timestamp": datetime.now().isoformat(),
                    "event": "session_end",
                    "total_commands": {
                        "steer": self.steer_command_count,
                        "speed": self.speed_command_count,
                        "cmd_vel": self.cmd_vel_count
                    }
                }
                self.log_file.write(f"# {json.dumps(end_entry, ensure_ascii=False)}\n")
                self.log_file.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    monitor_node = MotorMonitorNode()
    
    try:
        print("🚀 启动电机舵机监控节点...")
        print("📡 正在监控以下话题:")
        print("   • /steer_angle  - 舵机转向角度")
        print("   • /wheel_speed  - 电机速度")
        print("   • /cmd_vel      - 原始速度命令")
        if monitor_node.log_to_file:
            print(f"📝 日志保存到: {monitor_node.log_file_path}")
        print("\n⏳ 等待数据...")
        
        rclpy.spin(monitor_node)
        
    except KeyboardInterrupt:
        print("\n\n🛑 用户中断，正在退出...")
    except Exception as e:
        print(f"\n❌ 监控节点异常: {e}")
    finally:
        # 确保日志文件被正确关闭
        if hasattr(monitor_node, 'log_file') and monitor_node.log_file is not None:
            try:
                end_entry = {
                    "timestamp": datetime.now().isoformat(),
                    "event": "session_end",
                    "total_commands": {
                        "steer": monitor_node.steer_command_count,
                        "speed": monitor_node.speed_command_count,
                        "cmd_vel": monitor_node.cmd_vel_count
                    }
                }
                monitor_node.log_file.write(f"# {json.dumps(end_entry, ensure_ascii=False)}\n")
                monitor_node.log_file.close()
                print(f"📝 日志已保存到: {monitor_node.log_file_path}")
            except:
                pass
                
        monitor_node.destroy_node()
        # 安全关闭：只有当context仍然有效时才调用shutdown
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ 监控节点已安全退出")

if __name__ == '__main__':
    main() 