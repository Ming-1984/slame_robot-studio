#!/usr/bin/env python3

"""
Robot API Server - FastAPI后端服务
提供机器人控制和数据访问的RESTful API接口
"""

import asyncio
import json
import logging
import os
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any
import zipfile
import tempfile

# 添加依赖检查
try:
    from fastapi import FastAPI, HTTPException, UploadFile, File, BackgroundTasks
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import FileResponse, JSONResponse
    from fastapi.staticfiles import StaticFiles
    from pydantic import BaseModel
    import uvicorn
except ImportError:
    print("请安装FastAPI相关依赖:")
    print("pip install fastapi uvicorn python-multipart")
    exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
# from sensor_msgs.msg import LaserScan  # 激光雷达功能已移除
from std_srvs.srv import Empty

# 导入项目管理器
from project_manager import get_project_manager, ProjectManager

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Pydantic模型定义
class VelocityCommand(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

class GoalPose(BaseModel):
    x: float
    y: float
    z: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0
    frame_id: str = "map"

class SystemCommand(BaseModel):
    command: str
    parameters: Optional[Dict[str, Any]] = None

class VideoRecordRequest(BaseModel):
    action: str  # "start" or "stop"
    session_name: Optional[str] = None

# 全局变量：视频录制进程
video_recorder_process = None
video_recorder_lock = threading.Lock()

class RobotAPINode(Node):
    def __init__(self):
        super().__init__('robot_api_node')
        
        # 数据缓存
        self.latest_data = {
            'map': None,
            'odom': None,
            # 'scan': None,  # 激光雷达功能已移除
            'robot_pose': None,
            'system_status': 'running'
        }
        
        # 系统状态
        self.system_processes = {}
        
        # 创建QoS配置
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 设置订阅者和发布者
        self.setup_ros_interfaces()
        
        self.get_logger().info('Robot API Node 已启动')
    
    def setup_ros_interfaces(self):
        """设置ROS2接口"""
        # 订阅者 - 修复地图话题名称以匹配Aurora LIDAR
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/slamware_ros_sdk_server_node/map', self.map_callback, self.qos_profile)
        
        self.odom_subscription = self.create_subscription(
            Odometry, '/slamware_ros_sdk_server_node/odom', self.odom_callback, self.qos_profile)

        # 激光雷达订阅已移除
        # self.scan_subscription = self.create_subscription(
        #     LaserScan, '/slamware_ros_sdk_server_node/scan', self.scan_callback, self.qos_profile)
        
        # 发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 服务客户端
        self.save_map_client = self.create_client(Empty, '/save_map')
    
    def map_callback(self, msg):
        """地图数据回调"""
        self.latest_data['map'] = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'z': msg.info.origin.position.z
            },
            'timestamp': time.time()
        }
    
    def odom_callback(self, msg):
        """里程计数据回调"""
        self.latest_data['odom'] = {
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
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            },
            'timestamp': time.time()
        }
    
    def scan_callback(self, msg):
        """激光雷达数据回调 - 已禁用"""
        # 激光雷达功能已移除
        pass
    
    def publish_velocity(self, vel_cmd: VelocityCommand):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = vel_cmd.linear_x
        twist.linear.y = vel_cmd.linear_y
        twist.linear.z = vel_cmd.linear_z
        twist.angular.x = vel_cmd.angular_x
        twist.angular.y = vel_cmd.angular_y
        twist.angular.z = vel_cmd.angular_z
        
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'发布速度命令: linear_x={vel_cmd.linear_x}, angular_z={vel_cmd.angular_z}')
    
    def publish_goal(self, goal: GoalPose):
        """发布目标点"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = goal.frame_id
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.position.z = goal.z
        goal_msg.pose.orientation.x = goal.orientation_x
        goal_msg.pose.orientation.y = goal.orientation_y
        goal_msg.pose.orientation.z = goal.orientation_z
        goal_msg.pose.orientation.w = goal.orientation_w
        
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f'发布目标点: x={goal.x}, y={goal.y}')

# 创建FastAPI应用
app = FastAPI(
    title="Robot Control API",
    description="机器人控制和监控API",
    version="1.0.0"
)

# 配置CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 全局变量
robot_node: Optional[RobotAPINode] = None
project_manager: Optional[ProjectManager] = None

@app.on_event("startup")
async def startup_event():
    """应用启动事件"""
    global robot_node, project_manager
    
    # 初始化项目管理器
    try:
        project_manager = get_project_manager()
        
        # 如果没有当前项目，创建一个新项目
        if not project_manager.get_current_project():
            project_id = project_manager.create_new_project()
            logger.info(f"创建新项目: {project_id}")
        else:
            logger.info(f"当前项目: {project_manager.get_current_project()}")
            
    except Exception as e:
        logger.error(f"初始化项目管理器失败: {e}")
    
    # 初始化ROS2节点
    try:
        # 设置ROS2环境
        import os
        os.environ['ROS_DOMAIN_ID'] = '99'

        # 检查ROS2是否已经初始化
        if not rclpy.ok():
            rclpy.init()
            logger.info("ROS2已初始化")
        else:
            logger.info("ROS2已经初始化，跳过")

        robot_node = RobotAPINode()

        # 在后台线程运行ROS2
        def ros_thread():
            try:
                executor = rclpy.executors.SingleThreadedExecutor()
                executor.add_node(robot_node)
                executor.spin()
            except Exception as e:
                logger.error(f"ROS2线程错误: {e}")
                import traceback
                logger.error(f"详细错误: {traceback.format_exc()}")

        threading.Thread(target=ros_thread, daemon=True).start()
        logger.info("Robot API Server 已启动")
    except Exception as e:
        logger.error(f"启动失败: {e}")
        import traceback
        logger.error(f"详细错误: {traceback.format_exc()}")

@app.on_event("shutdown")
async def shutdown_event():
    """应用关闭事件"""
    global robot_node
    try:
        if robot_node:
            robot_node.destroy_node()
            logger.info("ROS2节点已销毁")

        # 只有在ROS2仍然运行时才关闭
        if rclpy.ok():
            rclpy.shutdown()
            logger.info("ROS2已关闭")
    except Exception as e:
        logger.error(f"关闭时出错: {e}")

    logger.info("Robot API Server 已关闭")

# API路由定义

@app.get("/")
async def root():
    """根路径"""
    return {
        "message": "Robot Control API Server",
        "version": "1.0.0",
        "status": "running",
        "timestamp": datetime.now().isoformat()
    }

@app.get("/api/status")
async def get_system_status():
    """获取系统状态"""
    if not robot_node:
        raise HTTPException(status_code=503, detail="ROS2节点未初始化")
    
    return {
        "system_status": robot_node.latest_data.get('system_status', 'unknown'),
        "ros2_status": "running" if robot_node else "stopped",
        "data_status": {
            "map": robot_node.latest_data['map'] is not None,
            "odom": robot_node.latest_data['odom'] is not None,
            # "scan": robot_node.latest_data['scan'] is not None  # 激光雷达功能已移除
        },
        "timestamp": datetime.now().isoformat()
    }

@app.get("/api/robot/pose")
async def get_robot_pose():
    """获取机器人位姿"""
    if not robot_node or not robot_node.latest_data['odom']:
        raise HTTPException(status_code=404, detail="机器人位姿数据不可用")
    
    return robot_node.latest_data['odom']

@app.get("/api/robot/scan")
async def get_laser_scan():
    """获取激光雷达数据 - 已禁用"""
    raise HTTPException(status_code=404, detail="激光雷达功能已移除")

@app.get("/api/map/info")
async def get_map_info():
    """获取地图信息"""
    if not robot_node or not robot_node.latest_data['map']:
        raise HTTPException(status_code=404, detail="地图数据不可用")
    
    return robot_node.latest_data['map']

@app.post("/api/robot/velocity")
async def set_robot_velocity(velocity: VelocityCommand):
    """设置机器人速度"""
    if not robot_node:
        raise HTTPException(status_code=503, detail="ROS2节点未初始化")
    
    robot_node.publish_velocity(velocity)
    return {"status": "success", "message": "速度命令已发送"}

@app.post("/api/robot/goal")
async def set_robot_goal(goal: GoalPose):
    """设置机器人目标点"""
    if not robot_node:
        raise HTTPException(status_code=503, detail="ROS2节点未初始化")
    
    robot_node.publish_goal(goal)
    return {"status": "success", "message": "目标点已设置"}

@app.post("/api/robot/stop")
async def stop_robot():
    """停止机器人"""
    if not robot_node:
        raise HTTPException(status_code=503, detail="ROS2节点未初始化")
    
    stop_cmd = VelocityCommand()  # 所有速度为0
    robot_node.publish_velocity(stop_cmd)
    return {"status": "success", "message": "机器人已停止"}

@app.post("/api/system/command")
async def execute_system_command(command: SystemCommand):
    """执行系统命令"""
    try:
        from pathlib import Path
        import asyncio

        if command.command == "start_path_planning":
            # 启动路径规划系统
            # 使用异步方式启动路径规划，避免超时
            process = await asyncio.create_subprocess_exec(
                "bash", "/home/jetson/ros2_ws/start_path_planning.sh",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                cwd="/home/jetson/ros2_ws"
            )

            # 等待进程启动（不等待完成）
            await asyncio.sleep(2)

            if process.returncode is None:
                # 进程正在运行
                return {
                    "status": "success",
                    "message": "路径规划系统启动中...",
                    "output": "系统正在后台启动，请等待30-60秒"
                }
            else:
                # 进程已结束，可能有错误
                stdout, stderr = await process.communicate()
                return {
                    "status": "error" if process.returncode != 0 else "success",
                    "output": stdout.decode() if stdout else "",
                    "error": stderr.decode() if stderr else ""
                }
        
        elif command.command == "stop_path_planning":
            # 停止路径规划系统 - 增强版，确保机器人完全停止
            # 1. 首先立即发送停止命令给机器人
            try:
                if robot_node:
                    # 发送零速度命令
                    zero_twist = Twist()
                    zero_twist.linear.x = 0.0
                    zero_twist.linear.y = 0.0
                    zero_twist.linear.z = 0.0
                    zero_twist.angular.x = 0.0
                    zero_twist.angular.y = 0.0
                    zero_twist.angular.z = 0.0

                    # 连续发送3次确保停止
                    for _ in range(3):
                        robot_node.cmd_vel_publisher.publish(zero_twist)
                        await asyncio.sleep(0.1)

                    logger.info("已发送紧急停止命令")
            except Exception as e:
                logger.warning(f"发送停止命令失败: {e}")

            # 2. 然后执行停止脚本
            process = await asyncio.create_subprocess_exec(
                "bash", "/home/jetson/ros2_ws/stop_path_planning.sh",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                cwd="/home/jetson/ros2_ws"
            )

            # 等待停止完成
            stdout, stderr = await process.communicate()

            # 3. 最后再次确保机器人停止
            try:
                if robot_node:
                    zero_twist = Twist()
                    robot_node.cmd_vel_publisher.publish(zero_twist)
                    logger.info("最终停止命令已发送")
            except Exception as e:
                logger.warning(f"最终停止命令失败: {e}")

            return {
                "status": "success" if process.returncode == 0 else "error",
                "message": "路径规划系统已停止，机器人已停止运动" if process.returncode == 0 else "停止失败",
                "output": stdout.decode() if stdout else "",
                "error": stderr.decode() if stderr else ""
            }

        elif command.command == "start_exploration":
            # 启动自动探索（仅启动探索节点，要求导航系统已运行）
            import asyncio
            import os

            # 1. 如果已在运行，直接返回
            running_check = subprocess.run(
                ["pgrep", "-f", "robust_explore_node.py"],
                capture_output=True,
                text=True
            )
            if running_check.returncode == 0:
                return {
                    "status": "success",
                    "message": "自动探索已在运行",
                    "output": running_check.stdout.strip()
                }

            # 2. 检查 Nav2 是否已启动（探索依赖 /controller_server）
            if not robot_node:
                return {
                    "status": "error",
                    "message": "ROS2节点未初始化，无法启动探索",
                    "error": "请先启动 Robot Studio 主服务"
                }

            try:
                nodes = robot_node.get_node_names_and_namespaces()
            except Exception as e:
                logger.warning(f"获取ROS2节点列表失败: {e}")
                nodes = []

            nav2_running = False
            for node_name, node_ns in nodes:
                full_name = f"{node_ns.rstrip('/')}/{node_name}"
                full_name = full_name.replace("//", "/")
                if full_name == "/controller_server":
                    nav2_running = True
                    break

            if not nav2_running:
                return {
                    "status": "error",
                    "message": "导航系统未运行，无法启动探索",
                    "error": "请先点击“启动导航/探索”启动 Nav2，再启动探索"
                }

            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = "99"
            env["LD_LIBRARY_PATH"] = "/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:" + env.get("LD_LIBRARY_PATH", "")

            # 3. 启动探索节点（与 start_path_planning.sh 使用同一配置）
            log_dir = Path("/home/jetson/ros2_ws/logs")
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / "optimized_robust_explore.log"

            start_cmd = (
                "set -e; "
                "export ROS_DOMAIN_ID=99; "
                "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
                "source /home/jetson/ros2_ws/install/setup.bash 2>/dev/null || true; "
                "export LD_LIBRARY_PATH=/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH; "
                "cd /home/jetson/ros2_ws; "
                "nohup python3 src/aurora_explorer/scripts/robust_explore_node.py "
                "--ros-args --params-file src/aurora_explorer/config/optimized_robust_params.yaml "
                f"> {log_path} 2>&1 &"
            )
            subprocess.Popen(["bash", "-c", start_cmd], env=env, cwd="/home/jetson/ros2_ws")

            await asyncio.sleep(1)
            verify = subprocess.run(
                ["pgrep", "-f", "robust_explore_node.py"],
                capture_output=True,
                text=True
            )

            if verify.returncode == 0:
                return {
                    "status": "success",
                    "message": "自动探索已启动",
                    "output": f"PID: {verify.stdout.strip()} | 日志: {log_path}"
                }

            return {
                "status": "warning",
                "message": "已下发探索启动命令，但暂未检测到进程",
                "output": f"请查看日志: {log_path}"
            }

        elif command.command == "stop_exploration":
            # 停止自动探索（只停止探索节点，保留导航功能）- 增强版
            import asyncio

            # 1. 首先发送停止命令给机器人
            try:
                if robot_node:
                    # 发送零速度命令
                    zero_twist = Twist()
                    zero_twist.linear.x = 0.0
                    zero_twist.angular.z = 0.0

                    # 连续发送确保停止
                    for _ in range(3):
                        robot_node.cmd_vel_publisher.publish(zero_twist)
                        await asyncio.sleep(0.1)

                    logger.info("探索停止：已发送停止命令")
            except Exception as e:
                logger.warning(f"发送探索停止命令失败: {e}")

            # 2. 温和地停止探索节点（发送SIGTERM而不是SIGKILL）
            result = subprocess.run(
                ["pkill", "-TERM", "-f", "robust_explore_node.py"],
                capture_output=True,
                text=True
            )

            # 3. 等待一下让探索节点正常退出
            await asyncio.sleep(2)

            # 4. 检查是否还有残留进程
            check_result = subprocess.run(
                ["pgrep", "-f", "robust_explore_node.py"],
                capture_output=True,
                text=True
            )

            if check_result.returncode == 0:
                # 如果还有进程，强制停止
                subprocess.run(
                    ["pkill", "-KILL", "-f", "robust_explore_node.py"],
                    capture_output=True,
                    text=True
                )
                message = "自动探索已强制停止"
            else:
                message = "自动探索已正常停止"

            # 5. 最后确保机器人停止
            try:
                if robot_node:
                    zero_twist = Twist()
                    robot_node.cmd_vel_publisher.publish(zero_twist)
                    logger.info("探索停止：最终停止命令已发送")
            except Exception as e:
                logger.warning(f"最终停止命令失败: {e}")

            return {
                "status": "success",
                "message": f"{message}，导航功能保持运行，机器人已停止运动",
                "output": "探索节点已停止，可继续手动控制"
            }

        elif command.command == "emergency_stop":
            # 紧急停止 - 立即停止机器人运动
            try:
                if robot_node:
                    # 发送零速度命令
                    zero_twist = Twist()
                    zero_twist.linear.x = 0.0
                    zero_twist.linear.y = 0.0
                    zero_twist.linear.z = 0.0
                    zero_twist.angular.x = 0.0
                    zero_twist.angular.y = 0.0
                    zero_twist.angular.z = 0.0

                    # 连续发送多次确保停止
                    for _ in range(5):
                        robot_node.cmd_vel_publisher.publish(zero_twist)
                        import time
                        time.sleep(0.05)  # 50ms间隔

                    logger.info("紧急停止命令已发送")

                    return {
                        "status": "success",
                        "message": "紧急停止已执行，机器人已停止运动",
                        "output": "已发送5次零速度命令确保停止"
                    }
                else:
                    return {
                        "status": "error",
                        "message": "机器人节点未初始化",
                        "error": "无法发送停止命令"
                    }
            except Exception as e:
                logger.error(f"紧急停止失败: {e}")
                return {
                    "status": "error",
                    "message": "紧急停止失败",
                    "error": str(e)
                }

        elif command.command == "shutdown_system":
            # 系统关机 - 安全关闭所有应用后关机
            import os

            try:
                logger.info("开始系统关机流程...")

                # 1. 首先发送停止命令给机器人
                if robot_node:
                    zero_twist = Twist()
                    for _ in range(5):
                        robot_node.cmd_vel_publisher.publish(zero_twist)
                        await asyncio.sleep(0.05)
                    logger.info("机器人已停止")

                # 2. 创建关机标志文件
                shutdown_flag = "/tmp/robot_shutdown_request"
                with open(shutdown_flag, "w") as f:
                    f.write("shutdown_requested")

                # 3. 启动独立的关机脚本（避免 self-kill 导致关机未执行）
                script_path = "/home/jetson/ros2_ws/safe_shutdown.sh"
                if not os.path.exists(script_path):
                    return {
                        "status": "error",
                        "message": "关机脚本不存在",
                        "error": f"缺少: {script_path}"
                    }

                # 关机是高危操作：必须显式确认；否则不执行任何停止/关机动作，避免误触导致断联/关机
                delay_seconds = 2
                confirm_token = None
                plan_only = False
                dry_run = False
                network_cleanup = True

                if command.parameters:
                    try:
                        delay_seconds = int(command.parameters.get("delay", delay_seconds))
                    except Exception:
                        delay_seconds = 2
                    confirm_token = command.parameters.get("confirm")
                    plan_only = bool(command.parameters.get("plan_only", False))
                    dry_run = bool(command.parameters.get("dry_run", False))
                    # 默认：真实关机做网络清理；dry-run 不动网络避免断连（仍会停止服务）
                    network_cleanup = bool(command.parameters.get("network_cleanup", (not dry_run)))

                if (not plan_only) and (confirm_token != "关机"):
                    return {
                        "status": "error",
                        "message": "关机操作需要确认令牌",
                        "error": "请在 parameters 中传入 confirm=关机（可选 dry_run=true 仅停止服务不关机）"
                    }

                args = ["bash", script_path, "--delay", str(max(0, delay_seconds))]
                if plan_only:
                    args.append("--plan-only")
                if dry_run:
                    args.append("--dry-run")
                if not network_cleanup:
                    args.append("--no-network-cleanup")

                log_paths = ["/var/log/robot-studio-safe-shutdown.log", "/tmp/robot-studio-safe-shutdown.log"]
                log_fp = None
                for p in log_paths:
                    try:
                        log_fp = open(p, "a")
                        break
                    except Exception:
                        continue

                proc = subprocess.Popen(
                    args,
                    stdout=log_fp if log_fp else subprocess.DEVNULL,
                    stderr=log_fp if log_fp else subprocess.DEVNULL,
                    start_new_session=True,
                    close_fds=True,
                )

                if log_fp:
                    log_fp.close()

                return {
                    "status": "success",
                    "message": (
                        "系统关机流程已启动"
                        + ("（plan-only：仅检查，不执行）" if plan_only else "")
                        + ("（dry-run：仅停止服务，不关机）" if (dry_run and (not plan_only)) else "")
                    ),
                    "output": (
                        f"关机脚本已启动 (PID: {proc.pid})，预计 {max(0, delay_seconds)} 秒后执行停止/关机。"
                    )
                }

            except Exception as e:
                logger.error(f"系统关机失败: {e}")
                return {
                    "status": "error",
                    "message": "系统关机失败",
                    "error": str(e)
                }

        elif command.command == "enter_test_mode_once":
            # 一次性测试模式 - 触发重启：下次开机不启热点/不上位机，仅启动SSH；且只生效一次（下次再开机自动恢复正常自启）
            import os

            try:
                script_path = "/home/jetson/ros2_ws/enter_test_mode_once.sh"
                if not os.path.exists(script_path):
                    return {
                        "status": "error",
                        "message": "测试模式脚本不存在",
                        "error": f"缺少: {script_path}"
                    }

                delay_seconds = 3
                confirm_token = None
                plan_only = False
                dry_run = False

                if command.parameters:
                    try:
                        delay_seconds = int(command.parameters.get("delay", delay_seconds))
                    except Exception:
                        delay_seconds = 3
                    confirm_token = command.parameters.get("confirm")
                    plan_only = bool(command.parameters.get("plan_only", False))
                    dry_run = bool(command.parameters.get("dry_run", False))

                # 高危操作：非 plan-only 时必须显式确认
                if (not plan_only) and (confirm_token != "测试模式"):
                    return {
                        "status": "error",
                        "message": "进入测试模式需要确认令牌",
                        "error": "请在 parameters 中传入 confirm=测试模式"
                    }

                args = ["bash", script_path, "--delay", str(max(0, delay_seconds))]
                if plan_only:
                    args.append("--plan-only")
                if dry_run:
                    args.append("--dry-run")

                log_paths = ["/var/log/robot-studio-test-mode.log", "/tmp/robot-studio-test-mode.log"]
                log_fp = None
                for p in log_paths:
                    try:
                        log_fp = open(p, "a")
                        break
                    except Exception:
                        continue

                proc = subprocess.Popen(
                    args,
                    stdout=log_fp if log_fp else subprocess.DEVNULL,
                    stderr=log_fp if log_fp else subprocess.DEVNULL,
                    start_new_session=True,
                    close_fds=True,
                )

                if log_fp:
                    log_fp.close()

                return {
                    "status": "success",
                    "message": (
                        "测试模式流程已启动"
                        + ("（plan-only：仅检查，不执行）" if plan_only else "")
                        + ("（dry-run：不重启）" if (dry_run and (not plan_only)) else "")
                    ),
                    "output": f"测试模式脚本已启动 (PID: {proc.pid})，预计 {max(0, delay_seconds)} 秒后重启"
                }

            except Exception as e:
                logger.error(f"进入测试模式失败: {e}")
                return {
                    "status": "error",
                    "message": "进入测试模式失败",
                    "error": str(e)
                }

        elif command.command == "start_point_cloud_processing":
            # 启动点云处理 - 使用异步方式避免超时
            import os
            import asyncio
            from datetime import datetime

            # 设置工作目录为点云处理项目目录
            processing_dir = "/home/jetson/ros2_ws/robotstudio/aurora_project"
            script_path = os.path.join(processing_dir, "complete_processing.sh")

            # 检查脚本是否存在
            if not os.path.exists(script_path):
                return {
                    "status": "error",
                    "message": "点云处理脚本不存在",
                    "error": f"脚本路径: {script_path}"
                }

            # 检查必要的工具
            bin_dir = "/home/jetson/ros2_ws/build/bin"
            cloud_tool = os.path.join(bin_dir, "cloud_processor_tool")

            if not os.path.exists(cloud_tool):
                return {
                    "status": "error",
                    "message": "点云处理工具不存在",
                    "error": f"请先构建项目: ./build_project.sh\n缺失工具: {cloud_tool}"
                }

            # 确保脚本可执行
            os.chmod(script_path, 0o755)

            # 创建输出目录
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = f"/home/jetson/ros2_ws/data/point_clouds/processing_{timestamp}"
            os.makedirs(output_dir, exist_ok=True)

            try:
                # 使用异步方式启动点云处理
                process = await asyncio.create_subprocess_exec(
                    "bash", script_path,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                    cwd=processing_dir
                )

                # 等待处理开始（不等待完成）
                await asyncio.sleep(2)

                if process.returncode is None:
                    # 进程正在运行
                    return {
                        "status": "success",
                        "message": "点云处理已启动",
                        "output": f"处理正在后台运行...\n输出目录: {output_dir}\n请等待处理完成（可能需要1-3分钟）",
                        "data": {
                            "processing_dir": processing_dir,
                            "output_dir": output_dir,
                            "timestamp": timestamp
                        }
                    }
                else:
                    # 进程已结束，可能有错误
                    stdout, stderr = await process.communicate()
                    return {
                        "status": "error" if process.returncode != 0 else "success",
                        "message": "点云处理失败" if process.returncode != 0 else "点云处理完成",
                        "output": stdout.decode() if stdout else "",
                        "error": stderr.decode() if stderr else ""
                    }

            except Exception as e:
                return {
                    "status": "error",
                    "message": "启动点云处理失败",
                    "error": str(e)
                }

        elif command.command == "complete_point_cloud_processing":
            # 完整的点云处理流程：提取 → 墙体增强 → 自动保存
            import os
            import asyncio
            import shutil
            from datetime import datetime
            from pathlib import Path

            # 设置工作目录
            processing_dir = "/home/jetson/ros2_ws/robotstudio/aurora_project"
            script_path = os.path.join(processing_dir, "complete_processing.sh")
            data_dir = os.path.join(processing_dir, "data")

            # 检查脚本和工具
            if not os.path.exists(script_path):
                return {
                    "status": "error",
                    "message": "点云处理脚本不存在",
                    "error": f"缺失脚本: {script_path}"
                }

            bin_dir = "/home/jetson/ros2_ws/build/bin"
            cloud_tool = os.path.join(bin_dir, "cloud_processor_tool")

            if not os.path.exists(cloud_tool):
                return {
                    "status": "error",
                    "message": "点云处理工具不存在",
                    "error": f"请先构建项目: ./build_project.sh\n缺失工具: {cloud_tool}"
                }

            # 确保脚本可执行
            os.chmod(script_path, 0o755)

            try:
                # 执行完整的点云处理流程
                process = await asyncio.create_subprocess_exec(
                    "bash", script_path,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                    cwd=processing_dir
                )

                # 等待处理完成
                stdout, stderr = await process.communicate()
                stdout_text = stdout.decode('utf-8', errors='ignore') if stdout else ""
                stderr_text = stderr.decode('utf-8', errors='ignore') if stderr else ""

                if process.returncode != 0:
                    return {
                        "status": "error",
                        "message": "点云处理失败",
                        "error": f"退出码: {process.returncode}\n标准输出: {stdout_text}\n错误输出: {stderr_text}"
                    }

                # 检查生成的文件
                generated_files = []
                if os.path.exists(data_dir):
                    for ext in ['*.ply', '*.xyz']:
                        for file_path in Path(data_dir).glob(ext):
                            file_stat = os.stat(file_path)
                            generated_files.append({
                                "name": file_path.name,
                                "size": file_stat.st_size,
                                "size_mb": round(file_stat.st_size / 1024 / 1024, 2),
                                "path": str(file_path)
                            })

                # 自动保存到项目管理器
                saved_info = None
                if project_manager and generated_files:
                    try:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        temp_save_dir = f"/tmp/pointcloud_complete_{timestamp}"

                        # 复制处理结果到临时目录
                        shutil.copytree(data_dir, temp_save_dir)

                        # 保存到项目目录
                        saved_path = project_manager.save_file(temp_save_dir, 'point_clouds', f"complete_processing_{timestamp}")

                        # 清理临时目录
                        shutil.rmtree(temp_save_dir)

                        saved_info = {
                            "saved_path": saved_path,
                            "timestamp": timestamp,
                            "file_count": len(generated_files)
                        }
                    except Exception as save_error:
                        logger.warning(f"自动保存失败: {save_error}")

                return {
                    "status": "success",
                    "message": "完整点云处理流程已完成",
                    "output": stdout_text,
                    "data": {
                        "generated_files": generated_files,
                        "file_count": len(generated_files),
                        "saved_info": saved_info,
                        "processing_dir": data_dir
                    }
                }

            except Exception as e:
                return {
                    "status": "error",
                    "message": "完整点云处理失败",
                    "error": str(e)
                }

        elif command.command == "clear_map":
            # 清除地图 - 发送清除地图命令到Aurora SLAM
            # 参考 start_path_planning.sh 第157行的实现
            import os

            try:
                logger.info("开始清除地图...")

                # 设置环境变量
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = '99'

                # 发送清除地图命令（与 start_path_planning.sh 一致）
                logger.info("发送清除地图命令到 /slamware_ros_sdk_server_node/clear_map...")

                # 使用简单的命令，不使用timeout（避免超时问题）
                clear_result = subprocess.run(
                    [
                        "bash", "-c",
                        "source /opt/ros/humble/setup.bash && "
                        "source /home/jetson/ros2_ws/install/setup.bash && "
                        "ros2 topic pub --once /slamware_ros_sdk_server_node/clear_map "
                        "slamware_ros_sdk/msg/ClearMapRequest '{}'"
                    ],
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=15  # 增加超时时间
                )

                logger.info(f"清除地图命令返回码: {clear_result.returncode}")
                logger.info(f"标准输出: {clear_result.stdout}")
                if clear_result.stderr:
                    logger.info(f"标准错误: {clear_result.stderr}")

                # 无论返回码如何，都认为命令已发送
                # ros2 topic pub --once 通常返回0表示成功
                if clear_result.returncode == 0 or "publishing" in clear_result.stderr.lower():
                    logger.info("地图清除命令发送成功")

                    # 等待地图清除完成
                    import time
                    time.sleep(1)

                    # 清除本地缓存的地图数据
                    if robot_node:
                        robot_node.latest_data['map'] = None
                        logger.info("本地地图缓存已清除")

                    return {
                        "status": "success",
                        "message": "地图清除命令已发送",
                        "output": f"stdout: {clear_result.stdout}\nstderr: {clear_result.stderr}"
                    }
                else:
                    logger.warning(f"地图清除命令可能失败，返回码: {clear_result.returncode}")
                    return {
                        "status": "warning",
                        "message": "地图清除命令已发送，但返回码异常",
                        "output": f"返回码: {clear_result.returncode}\nstdout: {clear_result.stdout}\nstderr: {clear_result.stderr}"
                    }

            except subprocess.TimeoutExpired:
                logger.error("地图清除命令超时（15秒）")
                return {
                    "status": "error",
                    "message": "地图清除命令超时",
                    "error": "命令执行超过15秒"
                }
            except Exception as e:
                logger.error(f"清除地图失败: {str(e)}")
                import traceback
                logger.error(f"详细错误: {traceback.format_exc()}")
                return {
                    "status": "error",
                    "message": "清除地图失败",
                    "error": str(e)
                }

        elif command.command == "save_map":
            # 保存地图 - 使用真实ROS2接口保存到项目
            import os
            import tempfile
            from datetime import datetime

            if not project_manager:
                return {
                    "status": "error",
                    "message": "项目管理器未初始化",
                    "error": "系统配置错误"
                }

            try:
                logger.info("开始真实地图保存流程...")

                # 1. 检查ROS2环境
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = '99'
                env['LD_LIBRARY_PATH'] = '/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:' + env.get('LD_LIBRARY_PATH', '')

                # 2. 检查地图话题是否存在
                logger.info("检查地图话题...")
                topic_check = subprocess.run(
                    ["bash", "-c", """
                    export ROS_DOMAIN_ID=99
                    source /opt/ros/humble/setup.bash 2>/dev/null || true
                    source /home/jetson/ros2_ws/install/setup.bash 2>/dev/null || true
                    timeout 5 ros2 topic list | grep -q "/map"
                    """],
                    capture_output=True,
                    text=True,
                    timeout=10
                )

                if topic_check.returncode != 0:
                    return {
                        "status": "error",
                        "message": "地图话题不存在",
                        "error": "请确保路径规划系统已启动并有地图数据发布"
                    }

                # 3. 检查地图话题是否有数据
                logger.info("检查地图数据...")
                data_check = subprocess.run(
                    ["bash", "-c", """
                    export ROS_DOMAIN_ID=99
                    source /opt/ros/humble/setup.bash 2>/dev/null || true
                    source /home/jetson/ros2_ws/install/setup.bash 2>/dev/null || true
                    timeout 8 ros2 topic echo /map --once >/dev/null 2>&1
                    """],
                    capture_output=True,
                    text=True,
                    timeout=10
                )

                if data_check.returncode != 0:
                    return {
                        "status": "error", 
                        "message": "地图话题无数据",
                        "error": "请确保SLAM系统正在运行并生成地图数据"
                    }

                # 4. 确保当前有活动项目
                current_project = project_manager.get_current_project()
                if not current_project:
                    current_project = project_manager.create_new_project()

                # 5. 生成时间戳文件名
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                map_name = f"map_{timestamp}"

                # 6. 检查地图保存器
                map_saver_path = "/home/jetson/ros2_ws/simple_map_saver.py"
                if not os.path.exists(map_saver_path):
                    return {
                        "status": "error",
                        "message": "地图保存工具不存在",
                        "error": f"找不到文件: {map_saver_path}"
                    }

                # 7. 创建临时目录保存地图
                with tempfile.TemporaryDirectory() as temp_dir:
                    temp_map_path = os.path.join(temp_dir, map_name)

                    # 8. 执行地图保存到临时目录
                    logger.info(f"保存真实地图数据: {map_name}")
                    bash_command = f"""
                    export ROS_DOMAIN_ID=99
                    export LD_LIBRARY_PATH=/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
                    source /opt/ros/humble/setup.bash 2>/dev/null || true
                    source /home/jetson/ros2_ws/install/setup.bash 2>/dev/null || true
                    cd /home/jetson/ros2_ws
                    echo "开始执行地图保存..."
                    python3 simple_map_saver.py {temp_map_path} --timeout 15
                    echo "地图保存命令完成，返回码: $?"
                    """

                    result = subprocess.run(
                        ["bash", "-c", bash_command],
                        capture_output=True,
                        text=True,
                        timeout=60,  # 增加超时到60秒
                        cwd="/home/jetson/ros2_ws"
                    )

                    # 9. 检查临时保存结果
                    temp_yaml_file = f"{temp_map_path}.yaml"
                    temp_pgm_file = f"{temp_map_path}.pgm"

                    if result.returncode == 0 and os.path.exists(temp_yaml_file) and os.path.exists(temp_pgm_file):
                        # 保存YAML文件到项目
                        yaml_saved_path = project_manager.save_file(temp_yaml_file, 'maps', f"{map_name}.yaml")
                        
                        # 保存PGM文件到项目
                        pgm_saved_path = project_manager.save_file(temp_pgm_file, 'maps', f"{map_name}.pgm")

                        # 获取文件信息
                        yaml_size = os.path.getsize(yaml_saved_path)
                        pgm_size = os.path.getsize(pgm_saved_path)

                        logger.info(f"真实地图保存成功: {map_name}")

                        return {
                            "status": "success",
                            "message": f"真实地图保存成功到项目 {current_project}: {map_name}",
                            "output": f"YAML文件: {yaml_saved_path} ({yaml_size}B)\nPGM文件: {pgm_saved_path} ({pgm_size}B)\n地图保存工具输出: {result.stdout}",
                            "data": {
                                "project_id": current_project,
                                "map_name": map_name,
                                "yaml_file": yaml_saved_path,
                                "pgm_file": pgm_saved_path,
                                "yaml_size": yaml_size,
                                "pgm_size": pgm_size,
                                "timestamp": timestamp
                            }
                        }
                    else:
                        return {
                            "status": "error",
                            "message": "地图保存失败",
                            "error": f"地图保存工具执行失败: {result.stderr or result.stdout}",
                            "output": f"返回码: {result.returncode}\n标准输出: {result.stdout}\n错误输出: {result.stderr}"
                        }

            except subprocess.TimeoutExpired:
                return {
                    "status": "error",
                    "message": "地图保存超时",
                    "error": "地图保存操作超过60秒限制"
                }
            except Exception as e:
                logger.error(f"地图保存异常: {e}")
                return {
                    "status": "error",
                    "message": "地图保存异常",
                    "error": str(e)
                }

        elif command.command == "check_point_cloud_status":
            # 检查点云处理状态
            import os
            import glob

            processing_dir = "/home/jetson/ros2_ws/robotstudio/aurora_project/data"
            output_dir = "/home/jetson/ros2_ws/data/point_clouds"

            status_info = {
                "processing_files": [],
                "output_files": [],
                "recent_processing": None
            }

            # 检查处理目录中的文件
            if os.path.exists(processing_dir):
                ply_files = glob.glob(os.path.join(processing_dir, "*.ply"))

                for file_path in ply_files:
                    file_stat = os.stat(file_path)
                    status_info["processing_files"].append({
                        "name": os.path.basename(file_path),
                        "size": file_stat.st_size,
                        "modified": file_stat.st_mtime
                    })

            # 检查输出目录
            if os.path.exists(output_dir):
                for item in os.listdir(output_dir):
                    item_path = os.path.join(output_dir, item)
                    if os.path.isdir(item_path):
                        status_info["output_files"].append({
                            "name": item,
                            "path": item_path,
                            "files": len(os.listdir(item_path)) if os.path.isdir(item_path) else 0
                        })

            # 查找最近的处理结果
            if status_info["processing_files"]:
                recent_file = max(status_info["processing_files"], key=lambda x: x["modified"])
                status_info["recent_processing"] = recent_file

            return {
                "status": "success",
                "message": "点云状态检查完成",
                "data": status_info
            }

        elif command.command == "save_point_cloud":
            # 保存点云 - 使用项目管理器保存到当前项目
            import os
            import shutil
            from datetime import datetime

            if not project_manager:
                return {
                    "status": "error",
                    "message": "项目管理器未初始化",
                    "error": "系统配置错误"
                }

            processing_dir = "/home/jetson/ros2_ws/robotstudio/aurora_project/data"

            # 检查是否有处理结果
            if not os.path.exists(processing_dir) or not os.listdir(processing_dir):
                return {
                    "status": "error",
                    "message": "没有可保存的点云数据",
                    "error": "请先运行点云处理"
                }

            try:
                # 确保当前有活动项目
                current_project = project_manager.get_current_project()
                if not current_project:
                    current_project = project_manager.create_new_project()

                # 生成时间戳文件夹名
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                temp_save_dir = f"/tmp/pointcloud_{timestamp}"
                
                # 先复制到临时目录
                shutil.copytree(processing_dir, temp_save_dir)
                
                # 保存到项目目录
                saved_path = project_manager.save_file(temp_save_dir, 'point_clouds', f"pointcloud_{timestamp}")
                
                # 清理临时目录
                shutil.rmtree(temp_save_dir)

                # 统计保存的文件
                saved_files = []
                saved_path_obj = Path(saved_path)
                for file_path in saved_path_obj.rglob('*'):
                    if file_path.is_file():
                        saved_files.append({
                            "name": file_path.name,
                            "size": file_path.stat().st_size,
                            "type": file_path.suffix,
                            "relative_path": str(file_path.relative_to(saved_path_obj))
                        })

                return {
                    "status": "success",
                    "message": f"点云数据已保存到项目 {current_project}",
                    "output": f"保存位置: {saved_path}\n保存文件数: {len(saved_files)}",
                    "data": {
                        "project_id": current_project,
                        "save_path": saved_path,
                        "timestamp": timestamp,
                        "files": saved_files
                    }
                }

            except Exception as e:
                return {
                    "status": "error",
                    "message": "点云保存失败",
                    "error": str(e)
                }
        

        
        else:
            raise HTTPException(status_code=400, detail=f"未知命令: {command.command}")
            
    except HTTPException:
        raise
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=408, detail="命令执行超时")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"命令执行失败: {str(e)}")

@app.get("/api/files/maps")
async def list_map_files():
    """列出地图文件"""
    maps_dir = Path("/home/jetson/maps")
    if not maps_dir.exists():
        return {"files": []}
    
    files = []
    for file_path in maps_dir.glob("*"):
        if file_path.is_file():
            files.append({
                "name": file_path.name,
                "size": file_path.stat().st_size,
                "modified": datetime.fromtimestamp(file_path.stat().st_mtime).isoformat(),
                "type": file_path.suffix
            })
    
    return {"files": files}

@app.get("/api/files/download/{filename}")
async def download_file(filename: str):
    """下载文件"""
    # 安全检查文件路径
    safe_filename = os.path.basename(filename)
    file_path = Path("/home/jetson/maps") / safe_filename

    if not file_path.exists():
        # 也检查robotstudio数据目录
        file_path = Path("/home/jetson/ros2_ws/robotstudio/aurora_project/data") / safe_filename

    if not file_path.exists():
        # 检查data/maps目录
        file_path = Path("/home/jetson/ros2_ws/data/maps") / safe_filename

    if not file_path.exists():
        # 检查data/point_clouds目录
        file_path = Path("/home/jetson/ros2_ws/data/point_clouds") / safe_filename

    if not file_path.exists():
        raise HTTPException(status_code=404, detail="文件不存在")

    return FileResponse(
        path=str(file_path),
        filename=safe_filename,
        media_type='application/octet-stream'
    )

@app.get("/api/download/map/{map_name}")
async def download_map_package(map_name: str):
    """下载地图文件包（YAML + PGM）"""
    try:
        if not project_manager:
            raise HTTPException(status_code=503, detail="项目管理器未初始化")
        
        current_project = project_manager.get_current_project()
        if not current_project:
            raise HTTPException(status_code=404, detail="没有活动项目")
        
        # 从当前项目中查找地图文件
        maps_dir = project_manager.base_dir / current_project / "maps"
        yaml_file = maps_dir / f"{map_name}.yaml"
        pgm_file = maps_dir / f"{map_name}.pgm"

        # 检查文件是否存在
        if not yaml_file.exists() or not pgm_file.exists():
            raise HTTPException(status_code=404, detail=f"地图文件不完整: {map_name}")

        # 创建临时ZIP文件
        import tempfile
        with tempfile.NamedTemporaryFile(delete=False, suffix='.zip') as tmp_file:
            with zipfile.ZipFile(tmp_file.name, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                zip_file.write(str(yaml_file), f"{map_name}.yaml")
                zip_file.write(str(pgm_file), f"{map_name}.pgm")

            return FileResponse(
                path=tmp_file.name,
                filename=f"{map_name}.zip",
                media_type="application/zip"
            )

    except Exception as e:
        logger.error(f"下载地图包失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/download/cloud/{cloud_name}")
async def download_cloud_package(cloud_name: str):
    """下载点云文件包"""
    try:
        if not project_manager:
            raise HTTPException(status_code=503, detail="项目管理器未初始化")
        
        current_project = project_manager.get_current_project()
        if not current_project:
            raise HTTPException(status_code=404, detail="没有活动项目")
        
        # 从当前项目中查找点云目录
        clouds_dir = project_manager.base_dir / current_project / "point_clouds"
        cloud_path = clouds_dir / cloud_name

        # 检查目录是否存在
        if not cloud_path.exists() or not cloud_path.is_dir():
            raise HTTPException(status_code=404, detail=f"点云目录不存在: {cloud_name}")

        # 创建临时ZIP文件
        import tempfile
        with tempfile.NamedTemporaryFile(delete=False, suffix='.zip') as tmp_file:
            with zipfile.ZipFile(tmp_file.name, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                for file_path in cloud_path.rglob('*'):
                    if file_path.is_file():
                        arcname = file_path.relative_to(cloud_path)
                        zip_file.write(str(file_path), str(arcname))

            return FileResponse(
                path=tmp_file.name,
                filename=f"{cloud_name}.zip",
                media_type="application/zip"
            )

    except Exception as e:
        logger.error(f"下载点云包失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/files/list/{file_type}")
async def list_data_files(file_type: str):
    """获取数据文件列表（支持项目管理系统）"""
    try:
        if not project_manager:
            # 回退到旧的文件系统
            return await list_legacy_files(file_type)
        
        # 获取当前项目的文件
        files = []
        current_project = project_manager.get_current_project()
        
        if current_project:
            try:
                if file_type == "maps":
                    project_files = project_manager.list_project_files(current_project, 'maps')
                    # 按地图名称分组
                    map_groups = {}
                    for file_info in project_files:
                        if file_info['name'].endswith('.yaml'):
                            map_name = file_info['name'][:-5]  # 移除.yaml扩展名
                            # 查找对应的pgm文件
                            pgm_file = next((f for f in project_files if f['name'] == f"{map_name}.pgm"), None)
                            if pgm_file:
                                map_groups[map_name] = {
                                    "name": map_name,
                                    "yaml_file": file_info['full_path'],
                                    "pgm_file": pgm_file['full_path'],
                                    "yaml_size": file_info['size'],
                                    "pgm_size": pgm_file['size'],
                                    "modified": file_info['modified_at'],
                                    "project_id": current_project
                                }
                    files = list(map_groups.values())
                    
                elif file_type == "clouds":
                    project_files = project_manager.list_project_files(current_project, 'point_clouds')
                    # 按目录分组
                    cloud_dirs = {}
                    for file_info in project_files:
                        # 获取第一级子目录名
                        path_parts = file_info['path'].split('/')
                        if len(path_parts) > 2:  # point_clouds/dirname/file
                            dir_name = path_parts[1]
                            if dir_name not in cloud_dirs:
                                cloud_dirs[dir_name] = {
                                    "name": dir_name,
                                    "path": str(project_manager.get_subdir_path('point_clouds', current_project) / dir_name),
                                    "files": [],
                                    "total_size": 0,
                                    "modified": file_info['modified_at'],
                                    "project_id": current_project
                                }
                            cloud_dirs[dir_name]['files'].append(file_info)
                            cloud_dirs[dir_name]['total_size'] += file_info['size']
                            # 更新最新修改时间
                            if file_info['modified_at'] > cloud_dirs[dir_name]['modified']:
                                cloud_dirs[dir_name]['modified'] = file_info['modified_at']
                    
                    for cloud_info in cloud_dirs.values():
                        cloud_info['file_count'] = len(cloud_info['files'])
                        del cloud_info['files']  # 移除详细文件列表
                    
                    files = list(cloud_dirs.values())
                    
                else:
                    return JSONResponse(status_code=400, content={"error": "不支持的文件类型"})
                    
            except Exception as e:
                logger.warning(f"获取项目文件失败: {e}, 回退到传统模式")
                return await list_legacy_files(file_type)
        
        # 如果当前项目没有文件，返回空列表（不显示其他项目的文件）
        # 这样新建项目后会显示为空，而不是显示旧项目的文件
        if not files and current_project:
            logger.info(f"当前项目 {current_project} 没有 {file_type} 文件")
            return {"files": [], "current_project": current_project}

        # 如果没有当前项目，显示最近项目的文件作为参考
        if not files and not current_project:
            all_projects = project_manager.list_projects()
            for project in all_projects[:3]:  # 只显示最近的3个项目
                try:
                    project_id = project['project_id']
                    if file_type == "maps":
                        project_files = project_manager.list_project_files(project_id, 'maps')
                        # 处理地图文件...
                        map_groups = {}
                        for file_info in project_files:
                            if file_info['name'].endswith('.yaml'):
                                map_name = file_info['name'][:-5]
                                pgm_file = next((f for f in project_files if f['name'] == f"{map_name}.pgm"), None)
                                if pgm_file:
                                    map_groups[f"{project_id}_{map_name}"] = {
                                        "name": f"{project_id}_{map_name}",
                                        "yaml_file": file_info['full_path'],
                                        "pgm_file": pgm_file['full_path'],
                                        "yaml_size": file_info['size'],
                                        "pgm_size": pgm_file['size'],
                                        "modified": file_info['modified_at'],
                                        "project_id": project_id
                                    }
                        files.extend(list(map_groups.values()))
                        
                    elif file_type == "clouds":
                        project_files = project_manager.list_project_files(project_id, 'point_clouds')
                        cloud_dirs = {}
                        for file_info in project_files:
                            path_parts = file_info['path'].split('/')
                            if len(path_parts) > 2:
                                dir_name = f"{project_id}_{path_parts[1]}"
                                if dir_name not in cloud_dirs:
                                    cloud_dirs[dir_name] = {
                                        "name": dir_name,
                                        "path": str(project_manager.get_subdir_path('point_clouds', project_id) / path_parts[1]),
                                        "file_count": 0,
                                        "total_size": 0,
                                        "modified": file_info['modified_at'],
                                        "project_id": project_id
                                    }
                                cloud_dirs[dir_name]['file_count'] += 1
                                cloud_dirs[dir_name]['total_size'] += file_info['size']
                        files.extend(list(cloud_dirs.values()))
                except Exception as e:
                    logger.warning(f"处理项目 {project.get('project_id', 'unknown')} 失败: {e}")
        
        return {"files": files}

    except Exception as e:
        logger.error(f"获取文件列表失败: {e}")
        # 回退到旧系统
        return await list_legacy_files(file_type)

async def list_legacy_files(file_type: str):
    """传统文件系统兼容性函数"""
    try:
        if file_type == "maps":
            data_dir = Path("/home/jetson/ros2_ws/data/maps")
        elif file_type == "clouds":
            data_dir = Path("/home/jetson/ros2_ws/data/point_clouds")
        else:
            return JSONResponse(status_code=400, content={"error": "不支持的文件类型"})

        if not data_dir.exists():
            return {"files": []}

        files = []
        if file_type == "maps":
            # 按地图名称分组
            map_groups = {}
            for file_path in data_dir.glob("*.yaml"):
                map_name = file_path.stem
                pgm_file = data_dir / f"{map_name}.pgm"
                if pgm_file.exists():
                    map_groups[map_name] = {
                        "name": map_name,
                        "yaml_file": str(file_path),
                        "pgm_file": str(pgm_file),
                        "yaml_size": file_path.stat().st_size,
                        "pgm_size": pgm_file.stat().st_size,
                        "modified": datetime.fromtimestamp(file_path.stat().st_mtime).isoformat(),
                        "project_id": "legacy"
                    }
            files = list(map_groups.values())

        elif file_type == "clouds":
            # 列出点云目录
            for dir_path in data_dir.iterdir():
                if dir_path.is_dir():
                    file_count = len(list(dir_path.rglob('*')))
                    total_size = sum(f.stat().st_size for f in dir_path.rglob('*') if f.is_file())
                    files.append({
                        "name": dir_path.name,
                        "path": str(dir_path),
                        "file_count": file_count,
                        "total_size": total_size,
                        "modified": datetime.fromtimestamp(dir_path.stat().st_mtime).isoformat(),
                        "project_id": "legacy"
                    })

        return {"files": files}

    except Exception as e:
        logger.error(f"获取传统文件列表失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ==================== 项目管理相关API ====================

@app.get("/api/projects")
async def list_projects():
    """获取所有项目列表"""
    if not project_manager:
        return {"projects": [], "current_project": None}
    
    try:
        projects = project_manager.list_projects()
        current_project = project_manager.get_current_project()
        
        return {
            "projects": projects,
            "current_project": current_project,
            "total_count": len(projects)
        }
    except Exception as e:
        logger.error(f"获取项目列表失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/projects/new")
async def create_new_project():
    """创建新项目"""
    if not project_manager:
        raise HTTPException(status_code=503, detail="项目管理器未初始化")
    
    try:
        project_id = project_manager.create_new_project()
        return {
            "status": "success",
            "message": f"新项目创建成功: {project_id}",
            "project_id": project_id
        }
    except Exception as e:
        logger.error(f"创建项目失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/projects/{project_id}/switch")
async def switch_project(project_id: str):
    """切换到指定项目"""
    if not project_manager:
        raise HTTPException(status_code=503, detail="项目管理器未初始化")
    
    try:
        success = project_manager.switch_project(project_id)
        if success:
            return {
                "status": "success",
                "message": f"已切换到项目: {project_id}",
                "current_project": project_id
            }
        else:
            raise HTTPException(status_code=404, detail="项目不存在")
    except Exception as e:
        logger.error(f"切换项目失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/projects/{project_id}/files")
async def get_project_files(project_id: str, subdir: str = None):
    """获取指定项目的文件列表"""
    if not project_manager:
        raise HTTPException(status_code=503, detail="项目管理器未初始化")
    
    try:
        files = project_manager.list_project_files(project_id, subdir)
        return {
            "project_id": project_id,
            "subdir": subdir,
            "files": files,
            "total_count": len(files)
        }
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        logger.error(f"获取项目文件失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/projects/current")
async def get_current_project():
    """获取当前项目信息"""
    if not project_manager:
        return {"current_project": None}
    
    try:
        current_project = project_manager.get_current_project()
        if not current_project:
            return {"current_project": None}
        
        # 获取项目详细信息
        projects = project_manager.list_projects()
        project_info = next((p for p in projects if p['project_id'] == current_project), None)
        
        if project_info:
            # 添加文件统计信息
            project_info['file_stats'] = {}
            for subdir in project_manager.subdirs:
                try:
                    files = project_manager.list_project_files(current_project, subdir)
                    project_info['file_stats'][subdir] = len(files)
                except:
                    project_info['file_stats'][subdir] = 0
        
        return {
            "current_project": current_project,
            "project_info": project_info
        }
    except Exception as e:
        logger.error(f"获取当前项目信息失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/projects/history")
async def get_project_history():
    """获取项目历史记录"""
    try:
        if not project_manager:
            raise HTTPException(status_code=503, detail="项目管理器未初始化")

        # 获取项目历史记录（限制20个）
        history = project_manager.get_project_history(limit=20)

        # 为每个项目添加摘要信息
        detailed_history = []
        for project in history:
            project_id = project['project_id']
            summary = project_manager.get_project_summary(project_id)
            detailed_history.append(summary)

        return {
            "history": detailed_history,
            "total_count": len(detailed_history),
            "current_project": project_manager.get_current_project()
        }
    except Exception as e:
        logger.error(f"获取项目历史记录失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/projects/switch/{project_id}")
async def switch_project(project_id: str):
    """切换到指定项目"""
    try:
        if not project_manager:
            raise HTTPException(status_code=503, detail="项目管理器未初始化")

        # 验证项目ID格式
        safe_project_id = os.path.basename(project_id)

        success = project_manager.switch_project(safe_project_id)

        if success:
            # 获取切换后的项目信息
            project_summary = project_manager.get_project_summary(safe_project_id)

            return {
                "status": "success",
                "message": f"已切换到项目: {safe_project_id}",
                "current_project": safe_project_id,
                "project_info": project_summary
            }
        else:
            raise HTTPException(status_code=404, detail=f"项目不存在: {safe_project_id}")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"切换项目失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/projects/create")
async def create_new_project():
    """创建新项目"""
    try:
        if not project_manager:
            raise HTTPException(status_code=503, detail="项目管理器未初始化")

        # 创建新项目
        project_id = project_manager.create_new_project()

        # 获取新项目信息
        project_summary = project_manager.get_project_summary(project_id)

        return {
            "status": "success",
            "message": f"新项目已创建: {project_id}",
            "project_id": project_id,
            "project_info": project_summary
        }
    except Exception as e:
        logger.error(f"创建新项目失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ==================== 视频录制相关API ====================

@app.post("/api/video/record")
async def control_video_recording(request: VideoRecordRequest):
    """控制视频录制（支持项目管理系统）"""
    global video_recorder_process, video_recorder_lock
    
    try:
        with video_recorder_lock:
            if request.action == "start":
                if video_recorder_process and video_recorder_process.poll() is None:
                    return {"status": "error", "message": "录制已在进行中"}
                
                # 确保有当前项目
                if project_manager:
                    current_project = project_manager.get_current_project()
                    if not current_project:
                        current_project = project_manager.create_new_project()
                        logger.info(f"为视频录制创建新项目: {current_project}")
                
                # 设置ROS2环境并启动录制节点
                env = os.environ.copy()
                env['ROS_DOMAIN_ID'] = '99'
                
                # 如果有项目管理器，设置视频保存路径
                if project_manager and current_project:
                    video_output_dir = str(project_manager.get_subdir_path('videos', current_project))
                    env['VIDEO_OUTPUT_DIR'] = video_output_dir
                
                cmd = [
                    'python3',
                    'src/aurora_vision_recorder.py'
                ]
                
                video_recorder_process = subprocess.Popen(
                    cmd,
                    env=env,
                    cwd='/home/jetson/ros2_ws',
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                # 等待节点启动
                await asyncio.sleep(3)

                # 检查进程是否正常运行
                if video_recorder_process.poll() is None:
                    session_name = request.session_name or f"stereo_recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

                    # 通过ROS2服务发送会话名称和开始录制命令
                    try:
                        # 发送会话名称
                        subprocess.run([
                            'ros2', 'topic', 'pub', '--once',
                            'aurora_recording_session_name',
                            'std_msgs/msg/String',
                            f'{{data: "{session_name}"}}'
                        ], env=env, cwd='/home/jetson/ros2_ws', timeout=5)

                        # 开始录制
                        result = subprocess.run([
                            'ros2', 'service', 'call',
                            'aurora_recording_control',
                            'std_srvs/srv/SetBool',
                            '{data: true}'
                        ], env=env, cwd='/home/jetson/ros2_ws', capture_output=True, text=True, timeout=10)

                        if result.returncode != 0:
                            logger.error(f"启动录制服务调用失败: {result.stderr}")

                    except Exception as e:
                        logger.error(f"ROS2服务调用失败: {e}")
                    
                    result = {
                        "status": "success",
                        "message": f"视频录制已开始: {session_name}",
                        "session_name": session_name
                    }
                    
                    # 如果有项目管理，添加项目信息
                    if project_manager and current_project:
                        result["project_id"] = current_project
                        result["output_dir"] = video_output_dir
                    
                    return result
                else:
                    stdout, stderr = video_recorder_process.communicate()
                    logger.error(f"录制节点启动失败: {stderr.decode()}")
                    return {"status": "error", "message": "录制节点启动失败"}
            
            elif request.action == "stop":
                if not video_recorder_process or video_recorder_process.poll() is not None:
                    return {"status": "error", "message": "没有正在进行的录制"}
                
                # 通过ROS2服务停止录制
                try:
                    env = os.environ.copy()
                    env['ROS_DOMAIN_ID'] = '99'

                    result = subprocess.run([
                        'ros2', 'service', 'call',
                        'aurora_recording_control',
                        'std_srvs/srv/SetBool',
                        '{data: false}'
                    ], env=env, cwd='/home/jetson/ros2_ws', capture_output=True, text=True, timeout=10)

                    if result.returncode != 0:
                        logger.error(f"停止录制服务调用失败: {result.stderr}")

                except Exception as e:
                    logger.error(f"ROS2停止录制服务调用失败: {e}")

                # 等待一段时间让录制完成保存
                await asyncio.sleep(2)

                # 优雅地停止录制节点
                video_recorder_process.terminate()

                # 等待进程结束，最多5秒
                try:
                    video_recorder_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    video_recorder_process.kill()
                    video_recorder_process.wait()

                video_recorder_process = None
                
                result = {
                    "status": "success", 
                    "message": "视频录制已停止"
                }
                
                # 如果有项目管理，添加项目信息
                if project_manager:
                    current_project = project_manager.get_current_project()
                    if current_project:
                        result["project_id"] = current_project
                        # 更新项目元数据
                        project_manager.update_project_metadata(current_project)
                
                return result
            
            else:
                return {"status": "error", "message": "无效的操作"}
                
    except Exception as e:
        logger.error(f"控制视频录制失败: {e}")
        return {"status": "error", "message": str(e)}

@app.get("/api/video/status")
async def get_video_recording_status():
    """获取视频录制状态"""
    global video_recorder_process
    
    try:
        if not video_recorder_process or video_recorder_process.poll() is not None:
            return {
                "recording": False,
                "session": None,
                "duration": 0
            }
        else:
            # 尝试从录制节点获取详细状态（这里简化处理）
            return {
                "recording": True,
                "session": "current_recording",
                "duration": 0  # 实际应该从节点获取
            }
            
    except Exception as e:
        logger.error(f"获取录制状态失败: {e}")
        return {"recording": False, "session": None, "duration": 0}

@app.get("/api/video/sessions")
async def list_video_sessions():
    """列出所有视频录制会话"""
    try:
        # 获取当前项目
        current_project = None
        if project_manager:
            current_project = project_manager.get_current_project()

        # 优先使用项目管理器的视频目录
        if project_manager and current_project:
            videos_dir = project_manager.get_subdir_path('videos', current_project)
        else:
            videos_dir = Path("/home/jetson/videos")

        if not videos_dir.exists():
            logger.info(f"视频目录不存在: {videos_dir}")
            return {"sessions": []}
        
        sessions = []
        for session_dir in videos_dir.iterdir():
            if session_dir.is_dir():
                config_file = session_dir / 'recording_config.json'
                if config_file.exists():
                    try:
                        with open(config_file, 'r') as f:
                            config = json.load(f)
                        
                        # 获取视频文件信息
                        files = {}
                        total_size = 0
                        for video_file in ['left_camera.avi', 'right_camera.avi', 'stereo_keypoints.avi']:
                            file_path = session_dir / video_file
                            if file_path.exists():
                                size = file_path.stat().st_size
                                files[video_file] = {
                                    'size': size,
                                    'size_mb': round(size / 1024 / 1024, 2)
                                }
                                total_size += size
                        
                        config.update({
                            'files': files,
                            'total_size': total_size,
                            'total_size_mb': round(total_size / 1024 / 1024, 2)
                        })
                        
                        sessions.append(config)
                    except Exception as e:
                        logger.warning(f"读取录制配置失败 {session_dir}: {e}")
        
        # 按时间倒序排列
        sessions.sort(key=lambda x: x.get('start_time', ''), reverse=True)
        logger.info(f"找到 {len(sessions)} 个视频会话，目录: {videos_dir}")
        return {"sessions": sessions}
        
    except Exception as e:
        logger.error(f"获取视频会话列表失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/video/download/{session_name}")
async def download_video_session(session_name: str):
    """下载视频录制会话（打包为ZIP）"""
    try:
        logger.info(f"开始下载视频会话: {session_name}")

        # 安全检查文件路径
        safe_session_name = os.path.basename(session_name)
        logger.info(f"安全会话名称: {safe_session_name}")

        # 获取当前项目
        current_project = None
        if project_manager:
            current_project = project_manager.get_current_project()

        # 优先使用项目管理器的视频目录
        if project_manager and current_project:
            videos_dir = project_manager.get_subdir_path('videos', current_project)
            logger.info(f"使用项目管理器视频目录: {videos_dir}")
        else:
            videos_dir = Path("/home/jetson/videos")
            logger.info(f"使用默认视频目录: {videos_dir}")

        session_dir = videos_dir / safe_session_name
        logger.info(f"会话目录: {session_dir}")

        if not session_dir.exists():
            logger.error(f"会话目录不存在: {session_dir}")
            raise HTTPException(status_code=404, detail=f"录制会话不存在: {safe_session_name}")

        if not session_dir.is_dir():
            logger.error(f"会话路径不是目录: {session_dir}")
            raise HTTPException(status_code=404, detail=f"录制会话路径无效: {safe_session_name}")

        # 检查目录中的文件
        files_in_dir = list(session_dir.iterdir())
        logger.info(f"会话目录中的文件: {[f.name for f in files_in_dir]}")

        if not files_in_dir:
            logger.warning(f"会话目录为空: {session_dir}")
            raise HTTPException(status_code=404, detail=f"录制会话为空: {safe_session_name}")

        # 创建临时ZIP文件
        logger.info("开始创建ZIP文件...")
        with tempfile.NamedTemporaryFile(delete=False, suffix='.zip') as tmp_file:
            logger.info(f"临时ZIP文件: {tmp_file.name}")

            with zipfile.ZipFile(tmp_file.name, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                file_count = 0
                for file_path in session_dir.rglob('*'):
                    if file_path.is_file():
                        arcname = file_path.relative_to(session_dir)
                        zip_file.write(str(file_path), str(arcname))
                        file_count += 1
                        logger.info(f"添加文件到ZIP: {arcname} ({file_path.stat().st_size} bytes)")

                logger.info(f"ZIP文件创建完成，包含 {file_count} 个文件")

            # 检查ZIP文件大小
            zip_size = Path(tmp_file.name).stat().st_size
            logger.info(f"ZIP文件大小: {zip_size} bytes")

            return FileResponse(
                path=tmp_file.name,
                filename=f"stereo_video_{safe_session_name}.zip",
                media_type="application/zip"
            )

    except HTTPException:
        # 重新抛出HTTP异常
        raise
    except Exception as e:
        logger.error(f"下载视频会话失败: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"下载失败: {str(e)}")

@app.delete("/api/video/sessions/{session_name}")
async def delete_video_session(session_name: str):
    """删除视频录制会话"""
    try:
        # 安全检查文件路径
        safe_session_name = os.path.basename(session_name)

        # 获取当前项目
        current_project = None
        if project_manager:
            current_project = project_manager.get_current_project()

        # 优先使用项目管理器的视频目录
        if project_manager and current_project:
            videos_dir = project_manager.get_subdir_path('videos', current_project)
        else:
            videos_dir = Path("/home/jetson/videos")

        session_dir = videos_dir / safe_session_name
        
        if not session_dir.exists():
            raise HTTPException(status_code=404, detail="录制会话不存在")
        
        # 删除目录及其所有内容
        import shutil
        shutil.rmtree(session_dir)
        
        return {"status": "success", "message": f"录制会话已删除: {session_name}"}
    
    except Exception as e:
        logger.error(f"删除视频会话失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    # 运行服务器
    uvicorn.run(
        "robot_api_server:app",
        host="0.0.0.0",
        port=8000,
        reload=False,
        log_level="info"
    )
