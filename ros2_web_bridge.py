#!/usr/bin/env python3

"""
ROS2 Web Bridge - WebSocket桥接服务
提供ROS2话题和服务的Web访问接口
"""

import asyncio
import json
import logging
import threading
import time
from typing import Dict, Any, Set
import websockets
import websockets.server
from websockets.exceptions import ConnectionClosed

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

# 添加依赖检查
try:
    import websockets
except ImportError:
    print("请安装websockets: pip install websockets")
    exit(1)

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ROS2WebBridge(Node):
    def __init__(self):
        super().__init__('ros2_web_bridge')
        
        # WebSocket连接管理
        self.websocket_clients: Set[websockets.WebSocketServerProtocol] = set()

        # 主事件循环引用
        self.main_loop = None

        # 数据缓存
        self.latest_data = {
            'map': None,
            'odom': None,
            'scan': None,
            'robot_pose': None,
            'system_status': 'unknown'
        }
        
        # 创建QoS配置
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 创建订阅者
        self.setup_subscribers()
        
        # 创建发布者
        self.setup_publishers()
        
        # 创建服务客户端
        self.setup_service_clients()
        
        self.get_logger().info('ROS2 Web Bridge 已启动')
    
    def setup_subscribers(self):
        """设置ROS2订阅者"""
        # 地图订阅 - 优先使用slamware节点的话题
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/slamware_ros_sdk_server_node/map',
            self.map_callback,
            self.qos_profile
        )

        # 里程计订阅 - 优先使用slamware节点的话题
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/slamware_ros_sdk_server_node/odom',
            self.odom_callback,
            self.qos_profile
        )

        # 激光雷达订阅 - 优先使用slamware节点的话题
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/slamware_ros_sdk_server_node/scan',
            self.scan_callback,
            self.qos_profile
        )

        # 机器人位姿订阅 - 使用Aurora SDK的话题
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/slamware_ros_sdk_server_node/robot_pose',
            self.pose_callback,
            self.qos_profile
        )
    
    def setup_publishers(self):
        """设置ROS2发布者"""
        # 速度控制发布者
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 目标点发布者
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
    
    def setup_service_clients(self):
        """设置ROS2服务客户端"""
        # 地图保存服务
        self.save_map_client = self.create_client(Empty, '/save_map')
        
        # 清除代价地图服务
        self.clear_costmap_client = self.create_client(Empty, '/clear_entirely_costmap')
    
    def map_callback(self, msg):
        """地图数据回调 - 添加节流避免频繁更新"""
        try:
            # 检查地图尺寸是否真的变化了
            current_size = (msg.info.width, msg.info.height)

            if hasattr(self, '_last_map_size'):
                if self._last_map_size == current_size:
                    # 尺寸没变，只更新数据不广播（减少网络流量）
                    map_data = {
                        'header': {
                            'stamp': {
                                'sec': msg.header.stamp.sec,
                                'nanosec': msg.header.stamp.nanosec
                            },
                            'frame_id': msg.header.frame_id
                        },
                        'info': {
                            'resolution': msg.info.resolution,
                            'width': msg.info.width,
                            'height': msg.info.height,
                            'origin': {
                                'position': {
                                    'x': msg.info.origin.position.x,
                                    'y': msg.info.origin.position.y,
                                    'z': msg.info.origin.position.z
                                },
                                'orientation': {
                                    'x': msg.info.origin.orientation.x,
                                    'y': msg.info.origin.orientation.y,
                                    'z': msg.info.origin.orientation.z,
                                    'w': msg.info.origin.orientation.w
                                }
                            }
                        },
                        'data': list(msg.data)
                    }
                    self.latest_data['map'] = map_data
                    # 节流：每5次更新才广播一次
                    if not hasattr(self, '_map_update_counter'):
                        self._map_update_counter = 0
                    self._map_update_counter += 1
                    if self._map_update_counter >= 5:
                        self._map_update_counter = 0
                        self.broadcast_map_data(map_data)
                    return
                else:
                    # 尺寸变化了，记录日志
                    self.get_logger().warn(f'⚠️ 地图尺寸变化: {self._last_map_size} → {current_size}')

            self._last_map_size = current_size
            self.get_logger().info(f'收到地图数据: {msg.info.width}x{msg.info.height}')

            # 转换地图数据为JSON格式
            map_data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'info': {
                    'resolution': msg.info.resolution,
                    'width': msg.info.width,
                    'height': msg.info.height,
                    'origin': {
                        'position': {
                            'x': msg.info.origin.position.x,
                            'y': msg.info.origin.position.y,
                            'z': msg.info.origin.position.z
                        },
                        'orientation': {
                            'x': msg.info.origin.orientation.x,
                            'y': msg.info.origin.orientation.y,
                            'z': msg.info.origin.orientation.z,
                            'w': msg.info.origin.orientation.w
                        }
                    }
                },
                'data': list(msg.data)  # 转换为列表以便JSON序列化
            }

            self.latest_data['map'] = map_data

            # 使用map_data类型广播，避免与topic_data/map冲突
            self.broadcast_map_data(map_data)

        except Exception as e:
            self.get_logger().error(f'地图数据处理错误: {e}')
    
    def odom_callback(self, msg):
        """里程计数据回调

        Odometry消息结构:
        - msg.pose 是 PoseWithCovariance
        - msg.pose.pose 是实际的 Pose
        - msg.twist 是 TwistWithCovariance
        - msg.twist.twist 是实际的 Twist
        """
        try:
            # 保持与ROS2 Odometry消息相同的结构
            # 这样前端可以使用 data.pose.pose 访问位姿
            odom_data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'child_frame_id': msg.child_frame_id,
                'pose': {
                    'pose': {
                        'position': {
                            'x': msg.pose.pose.position.x,
                            'y': msg.pose.pose.position.y,
                            'z': msg.pose.pose.position.z
                        },
                        'orientation': {
                            'x': msg.pose.pose.orientation.x,
                            'y': msg.pose.pose.orientation.y,
                            'z': msg.pose.pose.orientation.z,
                            'w': msg.pose.pose.orientation.w
                        }
                    },
                    'covariance': list(msg.pose.covariance)
                },
                'twist': {
                    'twist': {
                        'linear': {
                            'x': msg.twist.twist.linear.x,
                            'y': msg.twist.twist.linear.y,
                            'z': msg.twist.twist.linear.z
                        },
                        'angular': {
                            'x': msg.twist.twist.angular.x,
                            'y': msg.twist.twist.angular.y,
                            'z': msg.twist.twist.angular.z
                        }
                    },
                    'covariance': list(msg.twist.covariance)
                }
            }

            self.latest_data['odom'] = odom_data
            self.broadcast_to_clients('odom', odom_data)

        except Exception as e:
            self.get_logger().error(f'里程计数据处理错误: {e}')
            import traceback
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')
    
    def scan_callback(self, msg):
        """激光雷达数据回调"""
        try:
            self.get_logger().info(f'收到激光数据: {len(msg.ranges)} 个点')
            # 为了减少数据量，可以对激光数据进行采样
            ranges = list(msg.ranges)
            intensities = list(msg.intensities) if msg.intensities else []
            
            # 每隔N个点采样一次以减少数据量
            sample_rate = max(1, len(ranges) // 360)  # 最多360个点
            sampled_ranges = ranges[::sample_rate]
            sampled_intensities = intensities[::sample_rate] if intensities else []
            
            scan_data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment * sample_rate,
                'time_increment': msg.time_increment,
                'scan_time': msg.scan_time,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'ranges': sampled_ranges,
                'intensities': sampled_intensities
            }
            
            self.latest_data['scan'] = scan_data
            self.broadcast_to_clients('scan', scan_data)
            
        except Exception as e:
            self.get_logger().error(f'激光雷达数据处理错误: {e}')
    
    def pose_callback(self, msg):
        """机器人位姿回调"""
        try:
            pose_data = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'pose': {
                    'position': {
                        'x': msg.pose.position.x,
                        'y': msg.pose.position.y,
                        'z': msg.pose.position.z
                    },
                    'orientation': {
                        'x': msg.pose.orientation.x,
                        'y': msg.pose.orientation.y,
                        'z': msg.pose.orientation.z,
                        'w': msg.pose.orientation.w
                    }
                }
            }
            
            self.latest_data['robot_pose'] = pose_data
            self.broadcast_to_clients('robot_pose', pose_data)
            
        except Exception as e:
            self.get_logger().error(f'机器人位姿数据处理错误: {e}')
    
    def broadcast_map_data(self, map_data: dict):
        """向所有WebSocket客户端广播地图数据（使用map_data类型）"""
        if not self.websocket_clients:
            return

        self.get_logger().info(f'向 {len(self.websocket_clients)} 个客户端广播地图数据')

        # 使用map_data类型，避免与topic_data/map冲突
        message = {
            'type': 'map_data',
            'width': map_data['info']['width'],
            'height': map_data['info']['height'],
            'resolution': map_data['info']['resolution'],
            'origin': {
                'x': map_data['info']['origin']['position']['x'],
                'y': map_data['info']['origin']['position']['y'],
                'z': map_data['info']['origin']['position']['z']
            },
            'data': map_data['data'],
            'timestamp': time.time()
        }

        # 线程安全的异步发送给所有客户端
        try:
            if hasattr(self, 'main_loop') and self.main_loop and not self.main_loop.is_closed():
                # 使用allow_nan=False确保不会生成非法JSON
                # 使用separators减小JSON大小
                json_str = json.dumps(message, allow_nan=False, separators=(',', ':'))

                # 检查JSON大小
                json_size_mb = len(json_str) / (1024 * 1024)
                if json_size_mb > 1.0:
                    self.get_logger().warning(f'地图数据JSON过大: {json_size_mb:.2f} MB')

                future = asyncio.run_coroutine_threadsafe(
                    self._send_to_all_clients(json_str),
                    self.main_loop
                )
            else:
                self.get_logger().warning(f'主事件循环不可用，无法广播地图数据')
        except ValueError as e:
            self.get_logger().error(f'地图数据包含非法值（NaN/Infinity）: {e}')
        except Exception as e:
            self.get_logger().error(f'广播地图数据时发生错误: {e}')

    def broadcast_to_clients(self, topic: str, data: Any):
        """向所有WebSocket客户端广播数据"""
        if not self.websocket_clients:
            self.get_logger().info(f'没有WebSocket客户端连接，跳过广播 {topic} 数据')
            return

        self.get_logger().info(f'向 {len(self.websocket_clients)} 个客户端广播 {topic} 数据')

        message = {
            'type': 'topic_data',
            'topic': topic,
            'data': data,
            'timestamp': time.time()
        }

        # 线程安全的异步发送给所有客户端
        try:
            if hasattr(self, 'main_loop') and self.main_loop and not self.main_loop.is_closed():
                # 使用线程安全的方式调度到主事件循环
                future = asyncio.run_coroutine_threadsafe(
                    self._send_to_all_clients(json.dumps(message)),
                    self.main_loop
                )
                # 不等待结果，避免阻塞ROS2回调
            else:
                self.get_logger().warning(f'主事件循环不可用，无法广播 {topic} 数据')
        except Exception as e:
            self.get_logger().error(f'广播数据时发生错误: {e}')
    
    async def _send_to_all_clients(self, message: str):
        """异步发送消息给所有客户端"""
        if not self.websocket_clients:
            return
        
        # 创建发送任务列表
        tasks = []
        clients_to_remove = set()
        
        for client in self.websocket_clients.copy():
            try:
                tasks.append(client.send(message))
            except ConnectionClosed:
                clients_to_remove.add(client)
            except Exception as e:
                logger.error(f'发送消息到客户端失败: {e}')
                clients_to_remove.add(client)
        
        # 移除断开的客户端
        self.websocket_clients -= clients_to_remove
        
        # 执行发送任务
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
    
    async def handle_websocket_client(self, websocket, path):
        """处理WebSocket客户端连接"""
        self.websocket_clients.add(websocket)
        client_addr = websocket.remote_address
        logger.info(f'新的WebSocket客户端连接: {client_addr}')
        
        try:
            # 发送当前缓存的数据
            await self.send_cached_data(websocket)
            
            # 处理客户端消息
            async for message in websocket:
                await self.handle_client_message(websocket, message)
                
        except ConnectionClosed:
            logger.info(f'WebSocket客户端断开连接: {client_addr}')
        except Exception as e:
            logger.error(f'WebSocket客户端处理错误: {e}')
        finally:
            self.websocket_clients.discard(websocket)
    
    async def send_cached_data(self, websocket):
        """发送缓存的数据给新连接的客户端"""
        for topic, data in self.latest_data.items():
            if data is not None:
                message = {
                    'type': 'topic_data',
                    'topic': topic,
                    'data': data,
                    'timestamp': time.time()
                }
                try:
                    await websocket.send(json.dumps(message))
                except Exception as e:
                    logger.error(f'发送缓存数据失败: {e}')
    
    async def handle_client_message(self, websocket, message):
        """处理客户端消息"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'cmd_vel':
                # 处理速度控制命令
                await self.handle_cmd_vel(data)
            elif msg_type == 'goal_pose':
                # 处理目标点设置
                await self.handle_goal_pose(data)
            elif msg_type == 'service_call':
                # 处理服务调用
                await self.handle_service_call(data)
            elif msg_type == 'subscribe':
                # 处理话题订阅请求
                await self.handle_subscribe_request(websocket, data)
            elif msg_type == 'request_map':
                # 客户端请求地图数据
                await self.send_map_data(websocket)
            elif msg_type == 'request_saved_maps':
                # 客户端请求已保存的地图列表
                await self.send_saved_maps_list(websocket)
            elif msg_type == 'request_saved_clouds':
                # 客户端请求已保存的点云列表
                await self.send_saved_clouds_list(websocket)
            else:
                logger.warning(f'未知的消息类型: {msg_type}')

        except json.JSONDecodeError:
            logger.error('无效的JSON消息')
        except Exception as e:
            logger.error(f'处理客户端消息错误: {e}')
    
    async def handle_cmd_vel(self, data):
        """处理速度控制命令"""
        try:
            cmd_data = data.get('data', {})
            
            twist = Twist()
            twist.linear.x = float(cmd_data.get('linear', {}).get('x', 0.0))
            twist.linear.y = float(cmd_data.get('linear', {}).get('y', 0.0))
            twist.linear.z = float(cmd_data.get('linear', {}).get('z', 0.0))
            twist.angular.x = float(cmd_data.get('angular', {}).get('x', 0.0))
            twist.angular.y = float(cmd_data.get('angular', {}).get('y', 0.0))
            twist.angular.z = float(cmd_data.get('angular', {}).get('z', 0.0))
            
            self.cmd_vel_publisher.publish(twist)
            logger.info(f'发布速度命令: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}')
            
        except Exception as e:
            logger.error(f'处理速度命令错误: {e}')
    
    async def handle_goal_pose(self, data):
        """处理目标点设置"""
        try:
            goal_data = data.get('data', {})
            
            goal = PoseStamped()
            goal.header.frame_id = goal_data.get('frame_id', 'map')
            goal.header.stamp = self.get_clock().now().to_msg()
            
            pose = goal_data.get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            
            goal.pose.position.x = float(position.get('x', 0.0))
            goal.pose.position.y = float(position.get('y', 0.0))
            goal.pose.position.z = float(position.get('z', 0.0))
            goal.pose.orientation.x = float(orientation.get('x', 0.0))
            goal.pose.orientation.y = float(orientation.get('y', 0.0))
            goal.pose.orientation.z = float(orientation.get('z', 0.0))
            goal.pose.orientation.w = float(orientation.get('w', 1.0))
            
            self.goal_publisher.publish(goal)
            logger.info(f'发布目标点: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}')
            
        except Exception as e:
            logger.error(f'处理目标点设置错误: {e}')
    
    async def handle_service_call(self, data):
        """处理服务调用"""
        try:
            service_name = data.get('service')
            
            if service_name == 'save_map':
                if self.save_map_client.service_is_ready():
                    request = Empty.Request()
                    future = self.save_map_client.call_async(request)
                    logger.info('调用地图保存服务')
                else:
                    logger.warning('地图保存服务不可用')
            
            elif service_name == 'clear_costmap':
                if self.clear_costmap_client.service_is_ready():
                    request = Empty.Request()
                    future = self.clear_costmap_client.call_async(request)
                    logger.info('调用清除代价地图服务')
                else:
                    logger.warning('清除代价地图服务不可用')
            
            else:
                logger.warning(f'未知的服务: {service_name}')
                
        except Exception as e:
            logger.error(f'处理服务调用错误: {e}')
    
    async def handle_subscribe_request(self, websocket, data):
        """处理话题订阅请求"""
        try:
            topic = data.get('topic')
            if topic in self.latest_data and self.latest_data[topic] is not None:
                message = {
                    'type': 'topic_data',
                    'topic': topic,
                    'data': self.latest_data[topic],
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(message))
                
        except Exception as e:
            logger.error(f'处理订阅请求错误: {e}')

    async def send_map_data(self, websocket):
        """发送地图数据给客户端"""
        try:
            if self.latest_data['map'] is not None:
                # map_msg已经是字典格式
                map_msg = self.latest_data['map']

                # 创建地图数据字典
                map_data = {
                    'type': 'map_data',
                    'width': map_msg['info']['width'],
                    'height': map_msg['info']['height'],
                    'resolution': map_msg['info']['resolution'],
                    'origin': {
                        'x': map_msg['info']['origin']['position']['x'],
                        'y': map_msg['info']['origin']['position']['y'],
                        'z': map_msg['info']['origin']['position']['z']
                    },
                    'data': map_msg['data'],  # 已经是列表
                    'timestamp': time.time()
                }

                await websocket.send(json.dumps(map_data))
                logger.info(f'已发送地图数据给客户端: {map_data["width"]}x{map_data["height"]}')
            else:
                # 发送无地图数据消息
                await websocket.send(json.dumps({
                    'type': 'map_data',
                    'status': 'no_data',
                    'message': '暂无地图数据'
                }))

        except Exception as e:
            logger.error(f'发送地图数据失败: {e}')
            import traceback
            logger.error(f'详细错误: {traceback.format_exc()}')

    async def send_saved_maps_list(self, websocket):
        """发送已保存的地图列表 - 更新为使用项目管理系统"""
        try:
            # 先尝试从项目管理系统获取
            import requests
            try:
                response = requests.get('http://localhost:8000/api/files/list/maps', timeout=5)
                if response.status_code == 200:
                    api_data = response.json()
                    
                    # 转换格式匹配旧的WebSocket格式
                    saved_maps = []
                    for file in api_data['files']:
                        saved_maps.append({
                            'name': file['name'],
                            'yaml_file': file['yaml_file'],
                            'pgm_file': file['pgm_file'],
                            'yaml_size': file['yaml_size'],
                            'pgm_size': file['pgm_size'],
                            'created_time': file['modified'],
                            'created_date': file['modified'][:19].replace('T', ' '),
                            'download_url': f'/api/download/map/{file["name"]}'
                        })
                    
                    message = {
                        'type': 'saved_maps_list',
                        'maps': saved_maps,
                        'count': len(saved_maps),
                        'total_size': sum(m['yaml_size'] + m['pgm_size'] for m in saved_maps)
                    }
                    
                    await websocket.send(json.dumps(message))
                    logger.info(f'通过项目管理API发送{len(saved_maps)}个地图')
                    return
            except Exception as e:
                logger.warning(f'无法从项目管理API获取地图: {e}')
            
            # 回退到旧系统（但使用正确路径）
            import os
            import glob
            from datetime import datetime

            maps_dirs = [
                "/home/jetson/maps",  # 旧系统路径
                "/home/jetson/ros2_ws/data/maps"  # 备选路径
            ]
            saved_maps = []
            
            for maps_dir in maps_dirs:
                if os.path.exists(maps_dir):
                    # 查找所有.yaml文件
                    yaml_files = glob.glob(os.path.join(maps_dir, "*.yaml"))

                    for yaml_file in yaml_files:
                        map_name = os.path.splitext(os.path.basename(yaml_file))[0]
                        pgm_file = os.path.join(maps_dir, f"{map_name}.pgm")

                        if os.path.exists(pgm_file):
                            # 获取文件信息
                            yaml_stat = os.stat(yaml_file)
                            pgm_stat = os.stat(pgm_file)

                        saved_maps.append({
                            'name': map_name,
                            'yaml_file': yaml_file,
                            'pgm_file': pgm_file,
                            'yaml_size': yaml_stat.st_size,
                            'pgm_size': pgm_stat.st_size,
                            'created_time': yaml_stat.st_mtime,
                            'created_date': datetime.fromtimestamp(yaml_stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S'),
                            'download_url': f'/api/download/map/{map_name}'
                        })

            # 按创建时间排序（最新的在前）
            saved_maps.sort(key=lambda x: x['created_time'], reverse=True)

            await websocket.send(json.dumps({
                'type': 'saved_maps_list',
                'maps': saved_maps,
                'count': len(saved_maps),
                'total_size': sum(m['yaml_size'] + m['pgm_size'] for m in saved_maps)
            }))

            logger.info(f'已发送{len(saved_maps)}个已保存地图的列表')

        except Exception as e:
            logger.error(f'发送已保存地图列表失败: {e}')
            await websocket.send(json.dumps({
                'type': 'error',
                'message': '获取地图列表失败',
                'error': str(e)
            }))

    async def send_saved_clouds_list(self, websocket):
        """发送已保存的点云列表"""
        try:
            import os
            from datetime import datetime

            clouds_dir = "/home/jetson/ros2_ws/data/point_clouds"
            saved_clouds = []

            if os.path.exists(clouds_dir):
                # 查找所有点云目录
                for item in os.listdir(clouds_dir):
                    item_path = os.path.join(clouds_dir, item)
                    if os.path.isdir(item_path):
                        # 统计目录中的文件
                        files = []
                        total_size = 0
                        for file in os.listdir(item_path):
                            file_path = os.path.join(item_path, file)
                            if os.path.isfile(file_path):
                                file_stat = os.stat(file_path)
                                file_size = file_stat.st_size
                                total_size += file_size
                                files.append({
                                    'name': file,
                                    'size': file_size,
                                    'type': os.path.splitext(file)[1],
                                    'modified': datetime.fromtimestamp(file_stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
                                })

                        # 获取目录创建时间
                        dir_stat = os.stat(item_path)

                        saved_clouds.append({
                            'name': item,
                            'path': item_path,
                            'files': files,
                            'file_count': len(files),
                            'total_size': total_size,
                            'created_time': dir_stat.st_mtime,
                            'created_date': datetime.fromtimestamp(dir_stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S'),
                            'download_url': f'/api/download/cloud/{item}'
                        })

            # 按创建时间排序（最新的在前）
            saved_clouds.sort(key=lambda x: x['created_time'], reverse=True)

            await websocket.send(json.dumps({
                'type': 'saved_clouds_list',
                'clouds': saved_clouds,
                'count': len(saved_clouds),
                'total_size': sum(c['total_size'] for c in saved_clouds)
            }))

            logger.info(f'已发送{len(saved_clouds)}个已保存点云的列表')

        except Exception as e:
            logger.error(f'发送已保存点云列表失败: {e}')
            await websocket.send(json.dumps({
                'type': 'error',
                'message': '获取点云列表失败',
                'error': str(e)
            }))


async def start_websocket_server(bridge_node):
    """启动WebSocket服务器"""
    # 创建处理函数，兼容不同版本的websockets库
    async def websocket_handler(websocket, path=None):
        try:
            return await bridge_node.handle_websocket_client(websocket, path or "/")
        except Exception as e:
            logger.error(f"WebSocket处理器错误: {e}")

    try:
        server = await websockets.serve(
            websocket_handler,
            "0.0.0.0",
            8001,
            ping_interval=20,
            ping_timeout=10,
            max_size=10**7,  # 10MB最大消息大小，适合地图数据
            max_queue=32     # 增加队列大小
        )

        logger.info("WebSocket服务器已启动，端口: 8001")
        return server
    except Exception as e:
        logger.error(f"启动WebSocket服务器失败: {e}")
        raise


def main():
    """主函数"""
    rclpy.init()

    bridge_node = None
    ros_thread_obj = None

    try:
        # 创建ROS2节点
        bridge_node = ROS2WebBridge()

        # 在单独线程中运行ROS2
        def ros_thread():
            try:
                rclpy.spin(bridge_node)
            except Exception as e:
                logger.error(f"ROS2线程错误: {e}")

        ros_thread_obj = threading.Thread(target=ros_thread, daemon=True)
        ros_thread_obj.start()

        # 启动WebSocket服务器
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # 设置事件循环引用
        bridge_node.main_loop = loop

        server = loop.run_until_complete(start_websocket_server(bridge_node))

        logger.info("ROS2 Web Bridge 服务已启动")
        # 获取本机IP地址
        import socket
        try:
            # 连接到一个远程地址来获取本机IP
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("8.8.8.8", 80))
            local_ip = sock.getsockname()[0]
            sock.close()
            logger.info(f"WebSocket地址: ws://{local_ip}:8001")
        except Exception:
            logger.info("WebSocket地址: ws://localhost:8001")

        # 运行事件循环
        try:
            loop.run_forever()
        except KeyboardInterrupt:
            logger.info("收到中断信号，正在关闭...")

    except Exception as e:
        logger.error(f"主函数错误: {e}")
    finally:
        try:
            if bridge_node:
                bridge_node.destroy_node()
        except Exception as e:
            logger.error(f"销毁节点错误: {e}")

        try:
            rclpy.shutdown()
        except Exception as e:
            logger.error(f"关闭ROS2错误: {e}")


if __name__ == '__main__':
    main()
