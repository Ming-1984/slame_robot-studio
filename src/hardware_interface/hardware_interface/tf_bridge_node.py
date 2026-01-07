#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import numpy as np
from scipy.spatial.transform import Rotation
import time

class TFBridgeNode(Node):
    def __init__(self):
        super().__init__('tf_bridge_node')
        
        # TF相关组件
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 参数配置
        self.declare_parameter('publish_rate', 50.0)  # 50Hz发布频率
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom') 
        self.declare_parameter('map_frame', 'map')
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # 状态变量
        self.last_map_odom_transform = None
        self.transform_valid = False
        self.transform_timeout = 2.0  # 2秒超时
        
        self.get_logger().info(f"TF桥接节点启动 - 发布频率: {self.publish_rate}Hz")
        self.get_logger().info(f"将计算并发布: {self.map_frame} -> {self.odom_frame} 变换")
        
        # 定时器：计算并发布map->odom变换
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_map_odom_transform
        )
        
        # 状态报告定时器
        self.status_timer = self.create_timer(5.0, self.report_status)

    def compute_map_odom_transform(self):
        """
        计算map->odom变换
        数学原理: T_map_odom = T_map_base_link * T_base_link_odom
                              = T_map_base_link * inverse(T_odom_base_link)
        """
        try:
            # 使用最新可用的变换，而不是特定时间
            
            # 获取map->base_link变换 (由Aurora SDK提供)
            map_to_base = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame, 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            
            # 获取odom->base_link变换 (由Aurora SDK提供)  
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            
            # 计算base_link->odom变换 (odom->base_link的逆变换)
            base_to_odom = self.invert_transform(odom_to_base)
            
            # 计算map->odom变换 = map->base_link * base_link->odom
            map_to_odom = self.multiply_transforms(map_to_base, base_to_odom)
            
            # 更新时间戳到当前时间
            map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            
            self.last_map_odom_transform = map_to_odom
            self.transform_valid = True
            
            return True
            
        except Exception as e:
            if not self.transform_valid:
                self.get_logger().debug(f"等待TF变换可用: {e}")
            else:
                self.get_logger().warn(f"TF计算失败: {e}")
            return False

    def invert_transform(self, transform):
        """计算变换的逆"""
        # 提取平移和旋转
        t = transform.transform.translation
        r = transform.transform.rotation
        
        # 转换为numpy数组进行计算
        translation = np.array([t.x, t.y, t.z])
        rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])
        
        # 计算逆变换
        inv_rotation = rotation.inv()
        inv_translation = -inv_rotation.apply(translation)
        
        # 创建逆变换消息
        inv_transform = TransformStamped()
        inv_transform.header = transform.header
        inv_transform.transform.translation.x = inv_translation[0]
        inv_transform.transform.translation.y = inv_translation[1] 
        inv_transform.transform.translation.z = inv_translation[2]
        
        inv_quat = inv_rotation.as_quat()
        inv_transform.transform.rotation.x = inv_quat[0]
        inv_transform.transform.rotation.y = inv_quat[1]
        inv_transform.transform.rotation.z = inv_quat[2]
        inv_transform.transform.rotation.w = inv_quat[3]
        
        return inv_transform

    def multiply_transforms(self, t1, t2):
        """将两个变换相乘: result = t1 * t2"""
        # 提取第一个变换
        t1_trans = np.array([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z])
        t1_rot = Rotation.from_quat([t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w])
        
        # 提取第二个变换
        t2_trans = np.array([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z])
        t2_rot = Rotation.from_quat([t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w])
        
        # 计算组合变换
        result_rot = t1_rot * t2_rot
        result_trans = t1_trans + t1_rot.apply(t2_trans)
        
        # 创建结果变换
        result = TransformStamped()
        result.header = t1.header
        result.transform.translation.x = result_trans[0]
        result.transform.translation.y = result_trans[1]
        result.transform.translation.z = result_trans[2]
        
        result_quat = result_rot.as_quat()
        result.transform.rotation.x = result_quat[0]
        result.transform.rotation.y = result_quat[1]
        result.transform.rotation.z = result_quat[2]
        result.transform.rotation.w = result_quat[3]
        
        return result

    def publish_map_odom_transform(self):
        """发布map->odom变换"""
        if self.compute_map_odom_transform() and self.last_map_odom_transform:
            # 发布变换
            self.tf_broadcaster.sendTransform(self.last_map_odom_transform)
        elif self.last_map_odom_transform:
            # 如果计算失败但有历史数据，继续发布历史数据（更新时间戳）
            self.last_map_odom_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.last_map_odom_transform)

    def report_status(self):
        """定期报告TF桥接状态"""
        if self.transform_valid:
            self.get_logger().info("✅ TF桥接正常运行 - map->odom变换已发布")
        else:
            self.get_logger().warn("⚠️ TF桥接等待Aurora SDK数据...")

def main(args=None):
    rclpy.init(args=args)
    node = TFBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 