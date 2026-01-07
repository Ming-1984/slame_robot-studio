#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial
import struct
import time
import threading
from hardware_interface.speed_frames import get_frame_for_speed, get_nearest_valid_speed

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # --- 参数 ---
        # 默认使用THS1 UART串口
        self.declare_parameter('serial_port', '/dev/ttyTHS1')  # 默认使用THS1 UART
        self.declare_parameter('baud_rate', 115200)           # 默认波特率
        self.declare_parameter('send_frequency', 10.0)        # 发送频率 (Hz)，每秒发送10次
        self.declare_parameter('read_timeout', 0.1)          # 读取超时时间（秒）
        self.declare_parameter('use_stm32_protocol', True)   # 是否使用STM32协议
        self.declare_parameter('set_permissions', False)      # 是否尝试chmod串口

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.send_interval = 1.0 / self.get_parameter('send_frequency').value
        self.read_timeout = self.get_parameter('read_timeout').value
        self.use_stm32_protocol = self.get_parameter('use_stm32_protocol').value
        self.set_permissions = self.get_parameter('set_permissions').value

        self.get_logger().info(f"串口: {self.serial_port_name}, 波特率: {self.baud_rate}, 协议: {'STM32' if self.use_stm32_protocol else 'Legacy'}")
        self.get_logger().info(f"发送间隔: {self.send_interval:.2f}秒 ({self.get_parameter('send_frequency').value}Hz)")

        # --- 变量 ---
        self.serial_connection = None
        self.last_steer_angle = 0.2  # 初始转向角度设为0.2弧度，与STM32控制器代码一致
        self.last_wheel_speed = 0.0
        self.data_lock = threading.Lock()
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
            
        # 新增一个发布器，专门用于STM32数据展示
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

        # --- 初始化串口 ---
        self.connect_serial()

        # --- 定时发送 ---
        self.send_timer = self.create_timer(self.send_interval, self.send_serial_data)

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

    def create_stm32_control_frame(self, steering_angle, motor_speed):
        """创建符合STM32协议的控制数据帧"""
        # 帧头
        frame = bytearray([0xAA, 0x55])
        
        # 数据长度 (舵机角度4字节 + 电机速度4字节 = 8字节)
        data_len = 8
        frame.append(data_len)
        
        # 命令类型
        cmd_type = 0x01
        frame.append(cmd_type)
        
        # 数据载荷: 舵机角度和电机速度 (小端字节序的float)
        steering_bytes = struct.pack('<f', steering_angle)
        speed_bytes = struct.pack('<f', motor_speed)
        
        frame.extend(steering_bytes)
        frame.extend(speed_bytes)
        
        # 计算校验和：按照STM32控制器代码的方式
        # 从数据长度开始到速度数据结束的所有字节
        data_for_checksum = frame[2:]  # 从数据长度开始计算
        checksum_sum = sum(data_for_checksum) & 0xFF
        checksum = (~checksum_sum) & 0xFF
        frame.append(checksum)
        
        # 添加帧尾：按照STM32控制器代码的格式
        frame.extend([0x0D, 0x0A])
        
        return bytes(frame)

    def connect_serial(self):
        """连接串口，简化版，不检测连接状态"""
        try:
            self.get_logger().info(f"连接串口 {self.serial_port_name}...")
            
            # 如果是THS1端口，设置权限
            if self.set_permissions and 'ttyTHS' in self.serial_port_name:
                import subprocess
                try:
                    subprocess.run(['sudo', 'chmod', '666', self.serial_port_name], capture_output=True, timeout=5)
                    self.get_logger().info("已设置串口权限")
                except Exception as e:
                    self.get_logger().warn(f"设置权限失败: {e}")
            
            # 打开串口
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
                self.get_logger().info(f"串口 {self.serial_port_name} 连接成功!")
                # 启动读取线程
                self.start_read_thread()
            else:
                self.get_logger().warn(f"无法打开串口 {self.serial_port_name}")
                self.serial_connection = None

        except Exception as e:
            self.get_logger().error(f"连接串口时出错: {e}")
            self.serial_connection = None

    def start_read_thread(self):
        """启动串口读取线程"""
        if self.read_thread is not None and self.read_thread.is_alive():
            self.get_logger().warn("读取线程已经在运行")
            return
            
        self.read_thread_running = True
        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        self.get_logger().info("串口读取线程已启动")
    
    def stop_read_thread(self):
        """停止串口读取线程"""
        if self.read_thread is not None:
            self.read_thread_running = False
            self.read_thread = None
            self.get_logger().info("串口读取线程已停止")

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
                
                # 尝试解析为数值
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

    def read_serial_data(self):
        """持续读取串口数据的线程函数"""
        self.get_logger().info("开始监听STM32返回的数据...")
        
        while self.read_thread_running and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    # 读取可用数据
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    if data:
                        # 处理数据
                        result = self.process_stm32_data(data)
                        
                        # 简化日志输出
                        if 'text' in result and result['text']:
                            self.get_logger().info(f"STM32返回: {result['text']}")
                
                # 短暂休眠以避免CPU占用过高
                time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f"读取数据时出错: {e}")
                time.sleep(0.1)  # 出错时休眠更长时间
                
        self.get_logger().info("串口读取线程已结束")

    def send_serial_data(self):
        """定时将最新的控制数据发送到STM32，简化版，不检测连接状态"""
        # 如果没有连接，尝试重新连接
        if self.serial_connection is None or not self.serial_connection.is_open:
            self.connect_serial()
            if self.serial_connection is None:
                return

        with self.data_lock:
            steer = self.last_steer_angle
            speed = self.last_wheel_speed

        try:
            if self.use_stm32_protocol:
                # 使用STM32协议
                frame = self.create_stm32_control_frame(steer, speed)
                bytes_sent = self.serial_connection.write(frame)
                self.serial_connection.flush()
                
                self.get_logger().info(f"STM32协议发送: 转向={steer:.2f}弧度, 速度={speed:.2f}m/s")
            else:
                # 使用旧的协议
                frame = get_frame_for_speed(speed, steer)
                bytes_sent = self.serial_connection.write(frame)
                self.serial_connection.flush()
                self.get_logger().info(f"旧协议发送: 转向={steer:.2f}弧度, 速度={speed:.2f}m/s")
                
                # 发送回车换行符
                self.serial_connection.write(b'\r\n')
                self.serial_connection.flush()
            
        except Exception as e:
            self.get_logger().error(f"发送数据时出错: {e}")
            if self.serial_connection:
                try:
                    self.serial_connection.close()
                except:
                    pass
            self.serial_connection = None

    def destroy_node(self):
        """节点关闭时清理资源"""
        # 停止读取线程
        self.read_thread_running = False
        if self.read_thread is not None:
            self.read_thread.join(timeout=1.0)
            
        if self.serial_connection is not None and self.serial_connection.is_open:
            try:
                # 发送停止命令
                if self.use_stm32_protocol:
                    stop_frame = self.create_stm32_control_frame(0.0, 0.0)
                else:
                    stop_frame = get_frame_for_speed(0.0, 0.0)
                self.serial_connection.write(stop_frame)
                self.get_logger().info("发送停止命令...")
                time.sleep(0.1)
            except Exception as e:
                 self.get_logger().warn(f"关闭时发送停止命令失败: {e}")
            finally:
                 self.serial_connection.close()
                 self.get_logger().info("串口已关闭")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    serial_bridge = SerialBridgeNode()
    try:
        rclpy.spin(serial_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        serial_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
