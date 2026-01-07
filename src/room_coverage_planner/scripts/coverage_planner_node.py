#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
import numpy as np
import math
from sklearn.cluster import DBSCAN
from threading import Lock
import cv2
from scipy.interpolate import splprep, splev
from scipy.spatial.distance import cdist

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        
        # 参数
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('cell_size', 0.1)
        self.declare_parameter('min_path_points', 10)
        self.declare_parameter('free_space_threshold', 0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('path_smoothing', True)
        self.declare_parameter('smoothing_factor', 0.2)
        self.declare_parameter('optimize_path', True)
        self.declare_parameter('dbscan_eps_factor', 2.0)
        self.declare_parameter('dbscan_min_samples', 3)
        self.declare_parameter('coverage_overlap', 0.5)
        self.declare_parameter('min_dist_between_points', 0.1)
        
        self.map_topic = self.get_parameter('map_topic').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.cell_size = self.get_parameter('cell_size').value
        self.min_path_points = self.get_parameter('min_path_points').value
        self.free_space_threshold = self.get_parameter('free_space_threshold').value
        self.auto_start = self.get_parameter('auto_start').value
        self.path_smoothing = self.get_parameter('path_smoothing').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.optimize_path = self.get_parameter('optimize_path').value
        self.dbscan_eps_factor = self.get_parameter('dbscan_eps_factor').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.coverage_overlap = self.get_parameter('coverage_overlap').value
        self.min_dist_between_points = self.get_parameter('min_dist_between_points').value
        
        # 变量初始化
        self.map_data = None
        self.map_lock = Lock()
        self.coverage_path = []
        self.current_goal_index = 0
        self.is_navigating = False
        self.is_path_following = False
        
        # 订阅者
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10)
            
        # 发布者
        self.path_pub = self.create_publisher(
            Path,
            'coverage_path',
            10)
            
        self.path_markers_pub = self.create_publisher(
            MarkerArray,
            'coverage_path_markers',
            10)
            
        # Action客户端
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')
            
        # 服务
        self.plan_srv = self.create_service(
            Trigger,
            'plan_coverage_path',
            self.plan_coverage_path_callback)
            
        self.follow_srv = self.create_service(
            Trigger,
            'follow_coverage_path',
            self.follow_coverage_path_callback)
            
        # 自动启动规划
        if self.auto_start:
            self.timer = self.create_timer(10.0, self.auto_plan_callback)
            
        self.get_logger().info('房间覆盖路径规划器已启动')
        
    def map_callback(self, msg):
        with self.map_lock:
            self.map_data = msg
            
    def auto_plan_callback(self):
        if self.map_data is None:
            self.get_logger().info('等待地图数据...')
            return
            
        if not self.coverage_path and not self.is_path_following:
            self.get_logger().info('自动规划覆盖路径')
            self.plan_coverage_path()
            
    def plan_coverage_path_callback(self, request, response):
        result = self.plan_coverage_path()
        response.success = result
        if result:
            response.message = f"规划了{len(self.coverage_path)}个点的覆盖路径"
        else:
            response.message = "覆盖路径规划失败"
        return response
        
    def follow_coverage_path_callback(self, request, response):
        if not self.coverage_path:
            response.success = False
            response.message = "没有可用的覆盖路径"
            return response
            
        self.is_path_following = True
        self.current_goal_index = 0
        self.send_next_goal()
        
        response.success = True
        response.message = "开始按照覆盖路径导航"
        return response
        
    def plan_coverage_path(self):
        if self.map_data is None:
            self.get_logger().warn('没有可用的地图数据')
            return False
            
        with self.map_lock:
            map_data = self.map_data
            
        # 将地图转换为OpenCV格式
        grid = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        
        # 创建自由空间掩码
        free_space = (grid == self.free_space_threshold).astype(np.uint8) * 255
        
        # 对自由空间进行膨胀处理（考虑机器人半径）
        robot_radius_px = int(self.robot_radius / map_data.info.resolution)
        kernel = np.ones((robot_radius_px * 2, robot_radius_px * 2), np.uint8)
        eroded_free_space = cv2.erode(free_space, kernel, iterations=1)
        
        # 消除小的噪声区域
        # 首先找到所有连通区域
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(eroded_free_space, connectivity=8)
        
        # 过滤掉太小的区域
        min_area = (robot_radius_px * 2) ** 2  # 最小区域为机器人面积
        for i in range(1, num_labels):  # 跳过背景(0)
            if stats[i, cv2.CC_STAT_AREA] < min_area:
                eroded_free_space[labels == i] = 0
        
        # 在自由空间上进行灰度距离变换
        # 机器人离障碍物越远，值越大
        dist_transform = cv2.distanceTransform(eroded_free_space, cv2.DIST_L2, 5)
        
        # 将距离图归一化到0-255
        normalized_dist = cv2.normalize(dist_transform, None, 0, 255, cv2.NORM_MINMAX)
        
        # 计算有效采样半径 = 机器人半径 * 覆盖重叠因子
        effective_radius = robot_radius_px * (1.0 - self.coverage_overlap)
        sampling_radius_px = max(int(effective_radius), 1)
        
        # 使用距离变换和概率采样获取覆盖点
        sample_points = self.voronoi_based_sampling(normalized_dist, sampling_radius_px)
        
        if len(sample_points) < self.min_path_points:
            self.get_logger().warn(f'样本点太少 ({len(sample_points)})，无法规划路径')
            return False
            
        # 将样本点转换为numpy数组
        points = np.array(sample_points)
        
        # 使用DBSCAN聚类算法，将样本点分组
        eps = sampling_radius_px * self.dbscan_eps_factor
        clustering = DBSCAN(eps=eps, min_samples=self.dbscan_min_samples).fit(points)
        
        # 获取聚类标签
        labels = clustering.labels_
        
        # 移除噪声点(标签为-1)
        valid_points = points[labels != -1]
        valid_labels = labels[labels != -1]
        
        # 按簇排序点
        clusters = {}
        for i, label in enumerate(valid_labels):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(valid_points[i])
            
        # 根据簇的大小排序
        sorted_clusters = sorted(clusters.items(), key=lambda x: len(x[1]), reverse=True)
        
        # 从最大的簇开始，使用改进的算法构建路径
        path_points = []
        
        for label, cluster_points in sorted_clusters:
            cluster_array = np.array(cluster_points)
            
            # 如果集群中只有少量点，跳过
            if len(cluster_array) < 3:
                continue
                
            # 使用空间填充曲线（如Z曲线）来排序点以实现更均匀的覆盖
            cluster_path = self.get_cluster_path(cluster_array)
            path_points.extend(cluster_path)
            
        # 如果没有足够的路径点，返回失败
        if len(path_points) < self.min_path_points:
            self.get_logger().warn(f'有效路径点太少 ({len(path_points)})，无法生成路径')
            return False
            
        # 将像素坐标转换为世界坐标
        world_path = []
        for point in path_points:
            x = map_data.info.origin.position.x + point[0] * map_data.info.resolution
            y = map_data.info.origin.position.y + point[1] * map_data.info.resolution
            world_path.append((x, y))
        
        # 优化路径
        if self.optimize_path:
            world_path = self.optimize_coverage_path(world_path, map_data.info.resolution)
            
        # 应用路径平滑处理
        if self.path_smoothing and len(world_path) > 3:
            world_path = self.smooth_path(world_path)
            
        # 保存路径
        self.coverage_path = world_path
        
        # 发布路径可视化
        self.publish_path_markers()
        self.publish_path()
        
        self.get_logger().info(f'规划了{len(self.coverage_path)}个点的覆盖路径')
        return True
        
    def voronoi_based_sampling(self, distance_map, sampling_radius):
        """
        使用基于Voronoi图和距离变换的采样方法，获取覆盖点
        
        Args:
            distance_map: 归一化的距离变换图 (0-255)
            sampling_radius: 采样半径 (像素)
            
        Returns:
            采样点列表 [(x, y), ...]
        """
        height, width = distance_map.shape
        sample_points = []
        
        # 创建一个掩码来跟踪已经覆盖的区域
        coverage_mask = np.zeros_like(distance_map, dtype=bool)
        
        # 将距离图转换为概率图 (距离越大，概率越高)
        prob_map = distance_map.astype(float) / 255.0
        
        # 对概率图应用非线性变换以加强对中心区域的偏好
        prob_map = prob_map ** 2
        
        # 对所有可能的点按概率值排序
        candidates = []
        for y in range(height):
            for x in range(width):
                if distance_map[y, x] > 0:  # 只考虑自由空间中的点
                    candidates.append((x, y, prob_map[y, x]))
        
        # 按概率从高到低排序
        candidates.sort(key=lambda p: p[2], reverse=True)
        
        # 贪婪地选择点，确保覆盖
        for x, y, _ in candidates:
            # 如果点已被覆盖，跳过
            if coverage_mask[y, x]:
                continue
                
            # 添加采样点
            sample_points.append((x, y))
            
            # 标记这个点覆盖的区域
            y_min = max(0, y - sampling_radius)
            y_max = min(height, y + sampling_radius + 1)
            x_min = max(0, x - sampling_radius)
            x_max = min(width, x + sampling_radius + 1)
            
            for cy in range(y_min, y_max):
                for cx in range(x_min, x_max):
                    if (cx - x) ** 2 + (cy - y) ** 2 <= sampling_radius ** 2:
                        coverage_mask[cy, cx] = True
        
        return sample_points
    
    def get_cluster_path(self, points):
        """
        使用改进的路径规划算法为聚类中的点生成路径
        
        Args:
            points: 聚类中的点 numpy数组 [[x, y], ...]
            
        Returns:
            有序路径点列表 [[x, y], ...]
        """
        if len(points) <= 1:
            return points.tolist()
            
        # 计算聚类的边界框
        min_x, min_y = np.min(points, axis=0)
        max_x, max_y = np.max(points, axis=0)
        
        # 计算中心点
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        # 计算主方向 (使用PCA或简单协方差矩阵)
        centered_points = points - [center_x, center_y]
        cov = np.cov(centered_points.T)
        eigvals, eigvecs = np.linalg.eig(cov)
        
        # 主方向是特征值最大的特征向量
        main_axis = eigvecs[:, np.argmax(eigvals)]
        
        # 计算每个点到主轴的投影和垂直距离
        # 这将用于基于主方向排序点
        projections = np.dot(centered_points, main_axis)
        
        # 基于投影排序点 (沿主轴方向)
        sorted_indices = np.argsort(projections)
        sorted_points = points[sorted_indices]
        
        # 进行蛇形扫描 (先沿主轴一个方向，然后反向，以此类推)
        # 首先将点分组成若干行，每行具有相似的投影值
        proj_sorted = projections[sorted_indices]
        
        # 找出投影值的"台阶"，这些是扫描线的分隔点
        N = len(sorted_points)
        stride = max(1, N // 10)  # 尝试分成10行左右
        
        rows = []
        current_row = [sorted_points[0]]
        
        for i in range(1, N):
            # 如果投影值相似，添加到当前行
            if i % stride == 0 and i < N-1:
                rows.append(current_row)
                current_row = [sorted_points[i]]
            else:
                current_row.append(sorted_points[i])
        
        if current_row:
            rows.append(current_row)
        
        # 以蛇形模式连接行
        result = []
        for i, row in enumerate(rows):
            # 偶数行从左到右，奇数行从右到左
            if i % 2 == 0:
                result.extend(row)
            else:
                result.extend(reversed(row))
        
        return result
        
    def optimize_coverage_path(self, path, resolution):
        """
        优化覆盖路径，去除冗余点
        
        Args:
            path: 原始路径点列表 [(x, y), ...]
            resolution: 地图分辨率
            
        Returns:
            优化后的路径点列表 [(x, y), ...]
        """
        if len(path) <= 2:
            return path
            
        # 转换为numpy数组以便于操作
        path_array = np.array(path)
        
        # 计算最小距离阈值 (基于地图分辨率)
        min_dist = max(self.min_dist_between_points, resolution * 2)
        min_dist_sq = min_dist ** 2
        
        # 优化1: 移除太近的点
        optimized_path = [path[0]]  # 保留第一个点
        
        for i in range(1, len(path)):
            # 计算到上一个保留点的距离
            last = np.array(optimized_path[-1])
            current = np.array(path[i])
            dist_sq = np.sum((current - last) ** 2)
            
            # 如果距离足够远，保留这个点
            if dist_sq >= min_dist_sq:
                optimized_path.append(path[i])
        
        # 优化2: 检查是否可以跳过一些点而不影响覆盖
        if len(optimized_path) > 3:
            i = 0
            while i < len(optimized_path) - 2:
                # 检查跳过下一个点是否可行
                p1 = np.array(optimized_path[i])
                p2 = np.array(optimized_path[i+1])
                p3 = np.array(optimized_path[i+2])
                
                # 计算p1-p3的距离，以及p2到p1-p3线段的距离
                p1p3_dist = np.linalg.norm(p3 - p1)
                
                # 如果p1-p3距离不超过两倍的最小距离，可以跳过p2
                if p1p3_dist <= min_dist * 2:
                    optimized_path.pop(i+1)
                    # 不增加i，因为我们移除了一个点
                else:
                    i += 1
        
        return optimized_path
        
    def smooth_path(self, path):
        """
        使用样条插值平滑路径
        
        Args:
            path: 路径点列表 [(x, y), ...]
            
        Returns:
            平滑后的路径点列表 [(x, y), ...]
        """
        if len(path) <= 3:
            return path
            
        # 转换格式 [(x, y), ...] -> [[x, ...], [y, ...]]
        x = [p[0] for p in path]
        y = [p[1] for p in path]
        
        # 根据路径复杂度调整平滑因子
        smoothing_factor = min(len(path) * self.smoothing_factor, 0.95)
        
        # 为闭环路径处理
        is_closed = np.linalg.norm(np.array(path[0]) - np.array(path[-1])) < self.min_dist_between_points
        
        if is_closed:
            # 闭环路径，需要连接首尾
            x.append(x[0])
            y.append(y[0])
        
        # 创建参数化曲线的参数（使用累积弦长法）
        t = np.zeros(len(x))
        for i in range(1, len(t)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            t[i] = t[i-1] + np.sqrt(dx*dx + dy*dy)
        
        # 归一化参数
        if t[-1] > 0:
            t = t / t[-1]
        
        # 样条插值
        tck, u = splprep([x, y], u=t, s=smoothing_factor, per=is_closed)
        
        # 生成平滑路径点，使用比原始点更多的点
        num_points = len(path)
        u_new = np.linspace(0, 1, num_points)
        smooth_path = splev(u_new, tck)
        
        # 转换回[(x, y), ...]格式
        return list(zip(smooth_path[0], smooth_path[1]))
    
    def send_next_goal(self):
        if not self.is_path_following:
            return
            
        if self.current_goal_index >= len(self.coverage_path):
            self.get_logger().info('覆盖路径完成!')
            self.is_path_following = False
            return
            
        # 获取当前目标点
        goal_x, goal_y = self.coverage_path[self.current_goal_index]
        
        # 创建目标姿态
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        
        # 计算朝向（如果不是最后一个点，朝向下一个点）
        if self.current_goal_index < len(self.coverage_path) - 1:
            next_x, next_y = self.coverage_path[self.current_goal_index + 1]
            dx = next_x - goal_x
            dy = next_y - goal_y
            yaw = math.atan2(dy, dx)
            
            # 四元数表示朝向
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        else:
            # 最后一个点，使用默认朝向
            goal_pose.pose.orientation.w = 1.0
            
        self.get_logger().info(f'导航到路径点 {self.current_goal_index+1}/{len(self.coverage_path)}: ({goal_x:.2f}, {goal_y:.2f})')
        
        # 发送导航目标
        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            self.is_navigating = False
            self.is_path_following = False
            return
            
        self.get_logger().info('导航目标已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # 4 = 成功
            self.get_logger().info(f'导航到路径点 {self.current_goal_index+1} 成功')
            self.is_navigating = False
            self.current_goal_index += 1
            
            # 短暂延迟后继续下一个点
            timer = self.create_timer(1.0, self.timer_next_goal_callback)
        else:
            self.get_logger().error(f'导航失败，状态码: {status}')
        self.is_navigating = False
            self.is_path_following = False
            
    def timer_next_goal_callback(self):
        self.destroy_timer(self.timer)
        self.send_next_goal()
        
    def feedback_callback(self, feedback_msg):
        # 可以在此处处理导航反馈，例如记录或发布当前进度
        pass
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in self.coverage_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        
    def publish_path_markers(self):
        marker_array = MarkerArray()
        
        # 路径线条标记
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # 线宽
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        for point in self.coverage_path:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.05  # 略微抬高以便可视化
            line_marker.points.append(p)
            
        marker_array.markers.append(line_marker)
        
        # 路径点标记
        for i, point in enumerate(self.coverage_path):
            point_marker = Marker()
            point_marker.header.frame_id = 'map'
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = 'path_points'
            point_marker.id = i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = point[0]
            point_marker.pose.position.y = point[1]
            point_marker.pose.position.z = 0.05
            point_marker.pose.orientation.w = 1.0
            
            # 点大小与索引成比例
            size_ratio = min(1.0, i / max(1, len(self.coverage_path) - 1))
            point_marker.scale.x = 0.1 + 0.1 * size_ratio
            point_marker.scale.y = 0.1 + 0.1 * size_ratio
            point_marker.scale.z = 0.1 + 0.1 * size_ratio
            
            # 颜色渐变 (从绿到红)
            point_marker.color.r = size_ratio
            point_marker.color.g = 1.0 - size_ratio
            point_marker.color.b = 0.0
            point_marker.color.a = 1.0
            
            marker_array.markers.append(point_marker)
            
            # 添加文本标记
            if i % 5 == 0:  # 每5个点标记一个序号
                text_marker = Marker()
                text_marker.header.frame_id = 'map'
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = 'path_text'
                text_marker.id = i + 1000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = point[0]
                text_marker.pose.position.y = point[1]
                text_marker.pose.position.z = 0.2
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.2  # 文本大小
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = str(i)
                
                marker_array.markers.append(text_marker)
            
        self.path_markers_pub.publish(marker_array)

def main():
    rclpy.init()
    planner = CoveragePlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 