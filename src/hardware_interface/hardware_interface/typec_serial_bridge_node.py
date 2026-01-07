#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial
import struct
import time
import threading
import json
from hardware_interface.speed_frames import get_frame_for_speed, get_nearest_valid_speed

class TypeCSerialBridgeNode(Node):
    def __init__(self):
        super().__init__('typec_serial_bridge_node')

        # --- 参数 ---
        self.declare_parameter('serial_port', '/dev/ttyTHS1')  # 使用THS1 UART串口
        self.declare_parameter('baud_rate', 115200)           # 默认波特率
        self.declare_parameter('send_frequency', 10.0)        # 发送频率 (Hz)，每秒发送10次
        self.declare_parameter('read_timeout', 0.1)          # 读取超时时间（秒）

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.send_interval = 1.0 / self.get_parameter('send_frequency').value
        self.read_timeout = self.get_parameter('read_timeout').value

        self.get_logger().info(f"THS1串口: {self.serial_port_name}, 波特率: {self.baud_rate}, 发送间隔: {self.send_interval:.2f}秒")

        # --- 变量 ---
        self.serial_connection = None
        self.last_steer_angle = 0.2  # 初始转向角度设为0.2弧度，与STM32控制器代码一致
        self.last_wheel_speed = 0.0
        self.data_lock = threading.Lock()
        self.connect_attempt_timer = None
        self.read_thread = None
        self.read_thread_running = False
        
        # 控制参数限制（与STM32控制器代码一致）
        self.max_speed = 1.0     # 最大速度1.0米/秒
        self.max_angle = 0.6     # 最大转向角度0.6弧度
        
        # --- 发布器 ---
        self.chassis_feedback_pub = self.create_publisher(
            String,
            '/chassis_feedback',
            10)

        # 发布连接状态
        self.connection_status_pub = self.create_publisher(
            String,
            '/typec_connection_status',
            10)
            
        # 新增STM32数据处理后的发布器
        self.stm32_data_pub = self.create_publisher(
            String,
            '/stm32_data',
            10)

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

        # --- 尝试连接串口 ---
        self.try_connect_serial()

        # --- 定时发送 ---
        self.send_timer = self.create_timer(self.send_interval, self.send_serial_data)

    def publish_connection_status(self, status):
        """发布连接状态"""
        msg = String()
        msg.data = status
        self.connection_status_pub.publish(msg)

    def steer_callback(self, msg):
        with self.data_lock:
            # 限制转向角度在允许范围内
            limited_angle = max(min(msg.data, self.max_angle), -self.max_angle)
            if abs(limited_angle - msg.data) > 0.01:
                self.get_logger().info(f"转向角度已限制: {msg.data:.2f} → {limited_angle:.2f} 弧度")
            self.last_steer_angle = limited_angle

    def speed_callback(self, msg):
        with self.data_lock:
            # 先获取最接近的有效速度
            valid_speed = get_nearest_valid_speed(msg.data)
            # 再限制在最大速度范围内
            limited_speed = max(min(valid_speed, self.max_speed), -self.max_speed)
            
            if abs(limited_speed - msg.data) > 0.01:
                self.get_logger().info(f"速度值已调整: {msg.data:.2f} → {limited_speed:.1f}")
            self.last_wheel_speed = limited_speed

    def create_control_frame(self, steering_angle, motor_speed):
        """创建符合STM32协议的控制数据帧"""
        # 限制角度和速度在允许范围内
        steering_angle = max(min(steering_angle, self.max_angle), -self.max_angle)
        motor_speed = max(min(motor_speed, self.max_speed), -self.max_speed)

        # 准备数据包
        # 帧头 0xAA 0x55
        packet = bytearray([0xAA, 0x55])
        
        # 数据长度 0x08 (固定8字节：4字节角度 + 4字节速度)
        packet.append(0x08)
        
        # 命令类型 0x01
        packet.append(0x01)
        
        # 转向角度 (IEEE-754 float, 小端模式)
        angle_bytes = struct.pack("<f", steering_angle)
        packet.extend(angle_bytes)
        
        # 速度 (IEEE-754 float, 小端模式)
        speed_bytes = struct.pack("<f", motor_speed)
        packet.extend(speed_bytes)
        
        # 计算校验和 - 与STM32代码中一致
        # 从数据长度开始到速度数据结束
        data_for_checksum = packet[2:]  # 从数据长度开始计算
        checksum = sum(data_for_checksum) & 0xFF
        checksum = (~checksum) & 0xFF
        packet.append(checksum)
        
        # 帧尾 0x0D 0x0A
        packet.extend([0x0D, 0x0A])
        
        # 打印发送的数据包详细信息（用于调试）
        hex_data = ' '.join([f"{b:02X}" for b in packet])
        angle_hex = ' '.join([f"{b:02X}" for b in angle_bytes])
        speed_hex = ' '.join([f"{b:02X}" for b in speed_bytes])
        
        # 校验和计算过程
        sum_value = sum(data_for_checksum) & 0xFF
        
        self.get_logger().debug(f"数据包: {hex_data}")
        self.get_logger().debug(f"转向角度: {steering_angle:.2f} rad -> {angle_hex}")
        self.get_logger().debug(f"速度: {motor_speed:.2f} m/s -> {speed_hex}")
        self.get_logger().debug(f"校验和计算: sum={sum_value:02X} -> 取反 = {checksum:02X}")
        
        return bytes(packet)

    def try_connect_serial(self):
        """尝试连接或重新连接串口"""
        if self.serial_connection is not None and self.serial_connection.is_open:
            return True

        if self.connect_attempt_timer is not None:
            self.connect_attempt_timer.cancel()
            self.connect_attempt_timer = None

        try:
            self.get_logger().info(f"尝试连接THS1串口 {self.serial_port_name}...")
            
            # 设置THS1端口权限
            import subprocess
            try:
                subprocess.run(['sudo', 'chmod', '666', self.serial_port_name], 
                             capture_output=True, timeout=5)
                self.get_logger().info(f"已设置THS1端口 {self.serial_port_name} 权限")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("设置THS1端口权限超时，继续尝试连接...")
            
            self.serial_connection = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout,
                dsrdtr=False,
                rtscts=False
            )
            
            time.sleep(1.0)  # 等待串口稳定
            
            if self.serial_connection.is_open:
                self.get_logger().info(f"THS1串口 {self.serial_port_name} 连接成功!")
                self.publish_connection_status("CONNECTED")
                self.start_read_thread()
                
                return True
            else:
                self.get_logger().warn(f"无法打开THS1串口 {self.serial_port_name}")
                self.publish_connection_status("FAILED")
                self.schedule_reconnect()
                return False

        except serial.SerialException as e:
            self.get_logger().error(f"THS1串口连接失败: {e}")
            self.serial_connection = None
            self.publish_connection_status("ERROR")
            self.schedule_reconnect()
            return False
        except Exception as e:
            self.get_logger().error(f"连接THS1串口时发生未知错误: {e}")
            self.serial_connection = None
            self.publish_connection_status("ERROR")
            self.schedule_reconnect()
            return False

    def process_stm32_data(self, data):
        """处理并解析STM32数据"""
        # 创建信息发布对象
        msg = String()
        result = {}
        
        try:
            # 尝试解析为ASCII文本
            text = data.decode('ascii', errors='ignore').strip()
            if text:
                result['text'] = text
                
                # 尝试解析为数值或键值对
                try:
                    if ':' in text:
                        parts = text.split(':')
                        if len(parts) == 2:
                            key, value = parts
                            result[key.strip()] = value.strip()
                            
                            # 尝试转换为数值
                            try:
                                result[key.strip() + '_value'] = float(value.strip())
                            except ValueError:
                                pass
                    else:
                        try:
                            result['value'] = float(text)
                        except ValueError:
                            pass
                except:
                    pass
        except:
            pass
            
        # 始终添加原始十六进制数据
        result['hex'] = data.hex()
        result['bytes'] = len(data)
        result['timestamp'] = time.time()
        result['time'] = time.strftime("%H:%M:%S", time.localtime())
        
        # 转换为字符串格式
        formatted = ""
        for k, v in result.items():
            formatted += f"{k}: {v}\n"
            
        msg.data = formatted
        
        # 发布处理后的数据
        self.stm32_data_pub.publish(msg)
        
        # 同时发布原始数据
        raw_msg = String()
        raw_msg.data = data.decode('ascii', errors='ignore').strip()
        self.chassis_feedback_pub.publish(raw_msg)
        
        return result
        
    def schedule_reconnect(self):
        """安排5秒后重试连接"""
        if self.connect_attempt_timer is None:
            self.get_logger().info("将在5秒后尝试重新连接THS1串口...")
            self.connect_attempt_timer = self.create_timer(5.0, self.reconnect_callback)

    def reconnect_callback(self):
        """定时器回调，用于尝试重新连接"""
        if self.connect_attempt_timer is not None:
             self.destroy_timer(self.connect_attempt_timer)
             self.connect_attempt_timer = None
        self.try_connect_serial()

    def start_read_thread(self):
        """启动串口读取线程"""
        if self.read_thread is not None and self.read_thread.is_alive():
            self.get_logger().warn("读取线程已经在运行")
            return
            
        self.read_thread_running = True
        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        self.get_logger().info("THS1串口读取线程已启动")
    
    def stop_read_thread(self):
        """停止串口读取线程"""
        if self.read_thread is not None:
            self.read_thread_running = False
            self.read_thread = None
            self.get_logger().info("THS1串口读取线程已停止")

    def read_serial_data(self):
        """持续读取串口数据的线程函数"""
        self.get_logger().info("开始监听STM32通过THS1返回的数据...")
        
        while self.read_thread_running and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    raw_data = self.serial_connection.read(self.serial_connection.in_waiting)
                    if raw_data:
                        # 处理并解析数据
                        result = self.process_stm32_data(raw_data)
                        
                        # 简化日志输出
                        if 'text' in result and result['text']:
                            self.get_logger().info(f"STM32返回: {result['text']}")
                        
                time.sleep(0.001)
            except serial.SerialException as e:
                self.get_logger().error(f"THS1读取数据错误: {e}")
                with self.data_lock:
                    self.serial_connection = None
                self.publish_connection_status("DISCONNECTED")
                self.schedule_reconnect()
                break
            except Exception as e:
                self.get_logger().error(f"THS1读取数据时发生未知错误: {e}")
                time.sleep(0.01)
                
        self.get_logger().info("THS1串口读取线程已结束")

    def send_serial_data(self):
        """定时将最新的控制数据通过THS1发送到STM32"""
        # 如果没有连接，尝试重新连接
        if self.serial_connection is None or not self.serial_connection.is_open:
            if self.connect_attempt_timer is None:
                self.try_connect_serial()
            return

        with self.data_lock:
            steer = self.last_steer_angle
            speed = self.last_wheel_speed

        try:
            # 使用STM32协议
            frame = self.create_control_frame(steer, speed)
            bytes_sent = self.serial_connection.write(frame)
            self.serial_connection.flush()
            
            self.get_logger().info(f"THS1发送: 转向={steer:.2f}弧度, 速度={speed:.2f}m/s")
            
        except Exception as e:
            self.get_logger().error(f"THS1发送数据时出错: {e}")
            if self.serial_connection:
                try:
                    self.serial_connection.close()
                except:
                    pass
            self.serial_connection = None
            self.publish_connection_status("DISCONNECTED")
            self.schedule_reconnect()

    def destroy_node(self):
        """节点关闭时清理资源"""
        # 停止读取线程
        self.read_thread_running = False
        if self.read_thread is not None:
            try:
                self.read_thread.join(timeout=1.0)
            except:
                pass
                
        if self.serial_connection is not None and self.serial_connection.is_open:
            try:
                # 发送停止命令
                stop_frame = self.create_control_frame(0.0, 0.0)
                self.serial_connection.write(stop_frame)
                self.get_logger().info("通过THS1发送停止命令...")
                time.sleep(0.1)
            except Exception as e:
                 self.get_logger().warn(f"关闭时发送停止命令失败: {e}")
            finally:
                 self.serial_connection.close()
                 self.get_logger().info("THS1串口已关闭")
                 
        self.publish_connection_status("DISCONNECTED")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    serial_bridge = TypeCSerialBridgeNode()
    try:
        rclpy.spin(serial_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        serial_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 