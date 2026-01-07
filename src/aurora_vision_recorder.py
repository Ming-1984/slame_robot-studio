#!/usr/bin/env python3

"""
Aurora双目视觉数据录制节点
录制左目、右目和特征点图像数据，保存为视频文件和图片序列
"""

import os
import cv2
import numpy as np
import threading
import time
from datetime import datetime
from pathlib import Path
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from std_msgs.msg import String
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StereoImageRecorder(Node):
    def __init__(self):
        super().__init__('stereo_image_recorder')
        
        self.bridge = CvBridge()
        self.recording = False
        self.record_lock = threading.Lock()
        
        # 录制配置 - 支持环境变量指定的输出目录
        video_output_dir = os.environ.get('VIDEO_OUTPUT_DIR', '/home/jetson/videos')
        self.videos_dir = Path(video_output_dir)
        self.videos_dir.mkdir(parents=True, exist_ok=True)

        logger.info(f"视频输出目录: {self.videos_dir}")
        
        self.record_session = None
        self.video_writers = {}
        self.frame_counts = {}
        self.recording_start_time = None
        
        # 图像缓存
        self.left_image = None
        self.right_image = None
        self.keypoints_image = None
        
        # QoS配置 - 使用可靠传输
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅图像话题 - 修复为Aurora SDK的正确话题名称
        self.left_image_sub = self.create_subscription(
            Image,
            '/slamware_ros_sdk_server_node/left_image_raw',
            self.left_image_callback,
            qos_profile
        )

        self.right_image_sub = self.create_subscription(
            Image,
            '/slamware_ros_sdk_server_node/right_image_raw',
            self.right_image_callback,
            qos_profile
        )

        self.keypoints_sub = self.create_subscription(
            Image,
            '/slamware_ros_sdk_server_node/stereo_keypoints',
            self.keypoints_callback,
            qos_profile
        )
        
        # 录制状态定时器
        self.status_timer = self.create_timer(1.0, self.log_status)

        # 创建录制控制服务
        self.recording_service = self.create_service(
            SetBool,
            'aurora_recording_control',
            self.recording_control_callback
        )

        # 创建会话名称订阅
        self.session_name_sub = self.create_subscription(
            String,
            'aurora_recording_session_name',
            self.session_name_callback,
            10
        )

        self.pending_session_name = None

        logger.info("Aurora双目视觉录制节点已启动")
        logger.info(f"视频保存目录: {self.videos_dir}")
        logger.info("录制控制服务: aurora_recording_control")
        logger.info("会话名称话题: aurora_recording_session_name")
    
    def left_image_callback(self, msg):
        """左目图像回调"""
        try:
            # Aurora SDK发布RGB8格式，需要转换为BGR8用于OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.left_image = cv_image

            if self.recording:
                self.record_frame('left', cv_image, msg.header.stamp)

        except Exception as e:
            logger.error(f"处理左目图像失败: {e}")

    def right_image_callback(self, msg):
        """右目图像回调"""
        try:
            # Aurora SDK发布RGB8格式，需要转换为BGR8用于OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.right_image = cv_image

            if self.recording:
                self.record_frame('right', cv_image, msg.header.stamp)

        except Exception as e:
            logger.error(f"处理右目图像失败: {e}")

    def keypoints_callback(self, msg):
        """特征点图像回调"""
        try:
            # Aurora SDK发布RGB8格式，需要转换为BGR8用于OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.keypoints_image = cv_image

            if self.recording:
                self.record_frame('keypoints', cv_image, msg.header.stamp)

        except Exception as e:
            logger.error(f"处理特征点图像失败: {e}")

    def session_name_callback(self, msg):
        """接收会话名称"""
        self.pending_session_name = msg.data
        logger.info(f"接收到会话名称: {self.pending_session_name}")

    def recording_control_callback(self, request, response):
        """录制控制服务回调"""
        try:
            if request.data:  # 开始录制
                session_name = self.pending_session_name or f"stereo_recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                success, message = self.start_recording(session_name)
                response.success = success
                response.message = message
                logger.info(f"录制控制 - 开始: {message}")
            else:  # 停止录制
                success, message = self.stop_recording()
                response.success = success
                response.message = message
                logger.info(f"录制控制 - 停止: {message}")

        except Exception as e:
            response.success = False
            response.message = f"录制控制失败: {e}"
            logger.error(f"录制控制服务失败: {e}")

        return response

    def start_recording(self, session_name=None):
        """开始录制"""
        with self.record_lock:
            if self.recording:
                logger.warning("录制已在进行中")
                return False, "录制已在进行中"
            
            try:
                # 生成会话名称
                if not session_name:
                    session_name = f"stereo_recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                
                self.record_session = session_name
                session_dir = self.videos_dir / session_name
                session_dir.mkdir(parents=True, exist_ok=True)
                
                # 初始化视频写入器
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                fps = 15.0  # 根据stereo_image_pub_period=0.067约等于15fps
                
                # 获取图像分辨率（假设640x480，实际运行时会自动调整）
                frame_size = (640, 480)
                if self.left_image is not None:
                    frame_size = (self.left_image.shape[1], self.left_image.shape[0])

                # 特征点图像是左右拼接的，分辨率是双倍宽度
                keypoints_frame_size = (1280, 480)
                if self.keypoints_image is not None:
                    keypoints_frame_size = (self.keypoints_image.shape[1], self.keypoints_image.shape[0])

                self.video_writers = {
                    'left': cv2.VideoWriter(
                        str(session_dir / 'left_camera.avi'),
                        fourcc, fps, frame_size
                    ),
                    'right': cv2.VideoWriter(
                        str(session_dir / 'right_camera.avi'),
                        fourcc, fps, frame_size
                    ),
                    'keypoints': cv2.VideoWriter(
                        str(session_dir / 'stereo_keypoints.avi'),
                        fourcc, fps, keypoints_frame_size
                    )
                }
                
                self.frame_counts = {'left': 0, 'right': 0, 'keypoints': 0}
                self.recording_start_time = datetime.now()
                self.recording = True
                
                # 保存录制配置信息
                config = {
                    'session_name': session_name,
                    'start_time': self.recording_start_time.isoformat(),
                    'fps': fps,
                    'frame_size': frame_size,
                    'topics': {
                        'left_image_raw': '/left_image_raw',
                        'right_image_raw': '/right_image_raw', 
                        'stereo_keypoints': '/stereo_keypoints'
                    }
                }
                
                with open(session_dir / 'recording_config.json', 'w') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                
                logger.info(f"开始录制会话: {session_name}")
                logger.info(f"录制目录: {session_dir}")
                
                return True, f"录制已开始: {session_name}"
                
            except Exception as e:
                logger.error(f"启动录制失败: {e}")
                self.recording = False
                return False, f"启动录制失败: {e}"
    
    def stop_recording(self):
        """停止录制"""
        with self.record_lock:
            if not self.recording:
                return False, "没有正在进行的录制"
            
            try:
                self.recording = False
                
                # 关闭视频写入器
                for writer in self.video_writers.values():
                    writer.release()
                
                recording_duration = datetime.now() - self.recording_start_time
                
                # 更新配置信息
                session_dir = self.videos_dir / self.record_session
                config_file = session_dir / 'recording_config.json'
                
                if config_file.exists():
                    with open(config_file, 'r') as f:
                        config = json.load(f)
                    
                    config.update({
                        'end_time': datetime.now().isoformat(),
                        'duration_seconds': recording_duration.total_seconds(),
                        'frame_counts': self.frame_counts,
                        'files': {
                            'left_camera': 'left_camera.avi',
                            'right_camera': 'right_camera.avi',
                            'stereo_keypoints': 'stereo_keypoints.avi'
                        }
                    })
                    
                    with open(config_file, 'w') as f:
                        json.dump(config, f, indent=2, ensure_ascii=False)
                
                logger.info(f"录制完成: {self.record_session}")
                logger.info(f"录制时长: {recording_duration}")
                logger.info(f"帧数统计: {self.frame_counts}")
                
                session_name = self.record_session
                self.record_session = None
                self.video_writers = {}
                self.frame_counts = {}
                
                return True, f"录制完成: {session_name}"
                
            except Exception as e:
                logger.error(f"停止录制失败: {e}")
                return False, f"停止录制失败: {e}"
    
    def record_frame(self, camera, frame, timestamp):
        """录制单帧图像"""
        if camera in self.video_writers:
            writer = self.video_writers[camera]

            # 检查VideoWriter是否有效
            if not writer.isOpened():
                logger.error(f"{camera}相机VideoWriter未正确初始化")
                return

            # 直接写入帧，不检查尺寸（因为我们在初始化时已经设置了正确的尺寸）
            writer.write(frame)
            self.frame_counts[camera] += 1
    
    def get_recording_status(self):
        """获取录制状态"""
        with self.record_lock:
            if not self.recording:
                return {
                    'recording': False,
                    'session': None,
                    'duration': 0,
                    'frame_counts': {}
                }
            else:
                duration = (datetime.now() - self.recording_start_time).total_seconds()
                return {
                    'recording': True,
                    'session': self.record_session,
                    'duration': duration,
                    'frame_counts': self.frame_counts.copy()
                }
    
    def list_recordings(self):
        """列出所有录制会话"""
        sessions = []
        for session_dir in self.videos_dir.iterdir():
            if session_dir.is_dir():
                config_file = session_dir / 'recording_config.json'
                if config_file.exists():
                    try:
                        with open(config_file, 'r') as f:
                            config = json.load(f)
                        sessions.append(config)
                    except Exception as e:
                        logger.warning(f"读取录制配置失败 {session_dir}: {e}")
        
        return sorted(sessions, key=lambda x: x.get('start_time', ''), reverse=True)
    
    def log_status(self):
        """定时记录状态"""
        if self.recording:
            duration = (datetime.now() - self.recording_start_time).total_seconds()
            logger.info(f"录制中: {self.record_session} - {duration:.1f}s - 帧数: {self.frame_counts}")

def main():
    rclpy.init()
    
    # 创建录制节点
    recorder = StereoImageRecorder()
    
    try:
        # 启动ROS2节点
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        logger.info("收到中断信号，正在停止...")
    finally:
        # 如果正在录制，先停止录制
        if recorder.recording:
            recorder.stop_recording()
        
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()