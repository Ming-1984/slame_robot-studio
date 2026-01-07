#!/usr/bin/env python3

"""
STM32串口通信协议实现
基于实际测试结果，使用能够成功控制STM32的数据格式
"""

import struct

def get_nearest_valid_speed(speed):
    """
    将任意速度值四舍五入到最接近的0.1倍数
    例如：0.23 → 0.2, 0.67 → 0.7, -0.15 → -0.1
    
    参数:
        speed (float): 任意速度值（支持负值后退）
    
    返回:
        float: 最接近的有效速度值（0.1的倍数）
    """
    # 支持负速度（后退）和正速度（前进）
    if speed > 1.0:
        return 1.0
    elif speed < -1.0:
        return -1.0
    
    # 四舍五入到最接近的0.1倍数，保持正负符号
    return round(speed * 10) / 10

def get_frame_for_speed_user_method(speed, steer_angle=0.0):
    """
    使用用户提供的计算方法生成数据帧
    
    用户提供的Float32转换值和校验和：
    - 0.0 rad: 数据顺序为速度在前，转向角在后，校验和0x40
    - 0.2 rad: CD CC 4C 3E, 校验和0x12
    - 0.4 rad: CD CC CD 3E, 校验和0x30
    - 0.6 rad: 9A 99 19 3F, 校验和0x78
    - 0.8 rad: CD CC 4C 4F, 校验和0x97
    - 1.0 rad: 00 00 80 3F, 校验和0xBC
    
    参数:
        speed (float): 速度值(m/s)，固定为0.5
        steer_angle (float): 转向角度(rad)
        
    返回:
        bytes: 完整的数据帧
    """
    # 用户提供的Float32转换值和校验和
    user_data = {
        0.0: {
            'angle_bytes': [0x00, 0x00, 0x00, 0x00],
            'speed_bytes': [0x3F, 0x00, 0x00, 0x00],  # 用户提供的数据顺序
            'checksum': 0x40
        },
        0.2: {
            'angle_bytes': [0xCD, 0xCC, 0x4C, 0x3E],
            'speed_bytes': [0x00, 0x00, 0x00, 0x3F],
            'checksum': 0x12
        },
        0.4: {
            'angle_bytes': [0xCD, 0xCC, 0xCD, 0x3E],  # 用户提供的值
            'speed_bytes': [0x00, 0x00, 0x00, 0x3F],
            'checksum': 0x30
        },
        0.6: {
            'angle_bytes': [0x9A, 0x99, 0x19, 0x3F],
            'speed_bytes': [0x00, 0x00, 0x00, 0x3F],
            'checksum': 0x78
        },
        0.8: {
            'angle_bytes': [0xCD, 0xCC, 0x4C, 0x4F],  # 用户提供的值
            'speed_bytes': [0x00, 0x00, 0x00, 0x3F],
            'checksum': 0x97
        },
        1.0: {
            'angle_bytes': [0x00, 0x00, 0x80, 0x3F],
            'speed_bytes': [0x00, 0x00, 0x00, 0x3F],
            'checksum': 0xBC
        }
    }
    
    # 找到最接近的角度
    closest_angle = min(user_data.keys(), key=lambda x: abs(x - steer_angle))
    data = user_data[closest_angle]
    
    # 帧头
    frame_header = b'\xAA\x55'
    
    # 数据长度和命令类型
    data_length = 0x08
    command_type = 0x01
    
    # 构建数据（按用户提供的数据顺序）
    if closest_angle == 0.0:
        # 0.0 rad特殊处理：用户将速度数据放在转向角前面
        data_bytes = bytes(data['speed_bytes'] + data['angle_bytes'])
    else:
        # 其他角度：转向角在前，速度在后
        data_bytes = bytes(data['angle_bytes'] + data['speed_bytes'])
    
    # 使用用户提供的校验和
    checksum = data['checksum']
    
    # 帧尾
    frame_tail = b'\x0D\x0A\x0D\x0A'
    
    # 组装完整帧
    complete_frame = (frame_header + 
                     bytes([data_length]) + 
                     bytes([command_type]) + 
                     data_bytes + 
                     bytes([checksum]) + 
                     frame_tail)
    
    return complete_frame

def get_frame_for_speed(speed, steer_angle=0.0):
    """
    根据实际测试结果生成能够控制STM32的数据帧
    
    现在使用用户提供的计算方法
    
    参数:
        speed (float): 速度值(m/s)，将控制M2电机
        steer_angle (float): 转向角度(rad)，将控制M1电机
        
    返回:
        bytes: 完整的数据帧
    """
    # 使用用户提供的计算方法
    return get_frame_for_speed_user_method(speed, steer_angle)

def get_frame_for_speed_alternative(speed, steer_angle=0.0):
    """
    使用整数格式的替代数据帧生成函数
    
    基于测试发现整数格式也能成功控制STM32
    
    参数:
        speed (float): 速度值(m/s)
        steer_angle (float): 转向角度(rad)
        
    返回:
        bytes: 完整的数据帧
    """
    # 确保速度值有效
    valid_speed = get_nearest_valid_speed(speed)
    
    # 帧头
    frame_header = b'\xAA\x55'
    
    # 数据长度（固定为8字节：M1控制4字节 + M2控制4字节）
    data_length = 0x08
    
    # 命令类型（0x01: 标准控制命令）
    command_type = 0x01
    
    # M1控制值：转向角*10，整数，小端格式
    m1_bytes = struct.pack('<i', int(steer_angle * 10))
    
    # M2控制值：速度*10，整数，小端格式  
    m2_bytes = struct.pack('<i', int(valid_speed * 10))
    
    # 组合数据：M1在前，M2在后
    data_bytes = m1_bytes + m2_bytes
    
    # 计算校验和
    checksum = (data_length + command_type + sum(data_bytes)) % 256
    
    # 帧尾
    frame_tail = b'\x0D\x0A\x0D\x0A'
    
    # 组装完整帧
    complete_frame = (frame_header + 
                     bytes([data_length]) + 
                     bytes([command_type]) + 
                     data_bytes + 
                     bytes([checksum]) + 
                     frame_tail)
    
    return complete_frame

def verify_frame(frame):
    """
    验证数据帧的格式和校验和
    
    参数:
        frame (bytes): 要验证的数据帧
        
    返回:
        tuple: (is_valid, info_dict) 验证结果和详细信息
    """
    if len(frame) != 17:
        return False, {"error": f"帧长度错误: {len(frame)} != 17"}
    
    # 检查帧头
    if frame[0:2] != b'\xAA\x55':
        return False, {"error": f"帧头错误: {frame[0:2].hex()} != AA55"}
    
    # 检查数据长度
    if frame[2] != 0x08:
        return False, {"error": f"数据长度错误: {frame[2]} != 8"}
    
    # 检查命令类型
    if frame[3] != 0x01:
        return False, {"error": f"命令类型错误: {frame[3]} != 1"}
    
    # 检查帧尾
    if frame[13:17] != b'\x0D\x0A\x0D\x0A':
        return False, {"error": f"帧尾错误: {frame[13:17].hex()} != 0D0A0D0A"}
    
    # 计算并验证校验和
    data_length = frame[2]
    command_type = frame[3] 
    data_bytes = frame[4:12]
    calculated_checksum = (data_length + command_type + sum(data_bytes)) % 256
    actual_checksum = frame[12]
    
    if calculated_checksum != actual_checksum:
        return False, {"error": f"校验和错误: 计算值{calculated_checksum} != 实际值{actual_checksum}"}
    
    # 解析数据（假设是float32格式）
    try:
        m1_value = struct.unpack('<f', frame[4:8])[0]
        m2_value = struct.unpack('<f', frame[8:12])[0]
        
        info = {
            "frame_hex": frame.hex(),
            "m1_value": m1_value,
            "m2_value": m2_value,
            "checksum": actual_checksum
        }
    except:
        # 如果float32解析失败，尝试整数格式
        try:
            m1_value = struct.unpack('<i', frame[4:8])[0]
            m2_value = struct.unpack('<i', frame[8:12])[0]
            
            info = {
                "frame_hex": frame.hex(),
                "m1_value": m1_value,
                "m2_value": m2_value,
                "checksum": actual_checksum,
                "format": "integer"
            }
        except:
            info = {
                "frame_hex": frame.hex(),
                "checksum": actual_checksum,
                "error": "无法解析数据"
            }
    
    return True, info

if __name__ == "__main__":
    pass 