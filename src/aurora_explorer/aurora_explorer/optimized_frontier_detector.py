#!/usr/bin/env python3
"""
🔍 优化的前沿检测器
实现自适应DBSCAN参数调优、房间感知探索和前沿质量评估
Author: Acamana-Bot Development Team
Date: 2025-01-15
"""

import numpy as np
import math
import cv2
from typing import List, Tuple, Dict, Optional
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score
from scipy import ndimage
from dataclasses import dataclass


@dataclass
class OptimizedFrontierPoint:
    """优化的前沿点数据结构"""
    x: float                    # 世界坐标X
    y: float                    # 世界坐标Y
    size: int                   # 前沿点数量
    door_weight: float = 1.0    # 门洞权重
    quality_score: float = 0.0  # 质量评分
    accessibility_score: float = 0.0  # 可达性评分
    room_priority: float = 0.0  # 房间优先级
    exploration_value: float = 0.0  # 探索价值


class OptimizedFrontierDetector:
    """优化的前沿检测器"""
    
    def __init__(self, map_resolution: float = 0.05, robot_radius: float = 0.35, min_frontier_size: int = 3):
        self.map_resolution = map_resolution
        self.robot_radius = robot_radius

        # 🎯 自适应参数
        self.base_eps = 5.0              # 基础DBSCAN eps参数
        self.base_min_samples = 5        # 基础最小样本数
        self.min_frontier_size = min_frontier_size  # 最小前沿大小(可配置)

        # 🔍 多尺度检测参数
        self.scale_levels = [0.5, 1.0, 1.5]  # 多尺度检测级别
        self.adaptive_threshold_enabled = True  # 启用自适应阈值
        self.dynamic_eps_range = (3.0, 8.0)    # 动态eps范围
        
        # 🏠 房间感知参数
        self.room_boundary_threshold = 0.1   # 房间边界阈值
        self.door_width_min = 0.6           # 最小门宽
        self.door_width_max = 1.5           # 最大门宽
        
        # 📊 质量评估权重
        self.weights = {
            'size': 0.3,           # 前沿大小权重
            'accessibility': 0.3,   # 可达性权重
            'room_priority': 0.2,   # 房间优先级权重
            'door_bonus': 0.2      # 门洞加成权重
        }
        
        # 🔍 性能统计
        self.detection_stats = {
            'total_detections': 0,
            'adaptive_improvements': 0,
            'quality_improvements': 0
        }

    def detect_optimized_frontiers(self, map_data, room_info: Optional[Dict] = None) -> List[OptimizedFrontierPoint]:
        """
        检测优化的前沿点
        
        Args:
            map_data: 地图数据
            room_info: 房间信息字典
            
        Returns:
            优化的前沿点列表
        """
        grid = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        
        # 🔍 Step 1: 多尺度前沿像素检测
        frontier_pixels = self._detect_multiscale_frontier_pixels(grid)
        if len(frontier_pixels) == 0:
            return []
        
        # 🎯 Step 2: 自适应DBSCAN聚类
        frontier_clusters = self._adaptive_clustering(frontier_pixels, grid)
        if len(frontier_clusters) == 0:
            return []
        
        # 📊 Step 3: 动态阈值调整和前沿质量评估
        if self.adaptive_threshold_enabled:
            self._adjust_dynamic_thresholds(frontier_clusters, grid)

        quality_frontiers = self._evaluate_frontier_quality(frontier_clusters, grid, map_data)
        
        # 🏠 Step 4: 房间感知优先级计算
        if room_info:
            self._calculate_room_priorities(quality_frontiers, room_info, map_data)
        
        # 🎯 Step 5: 综合评分和排序
        final_frontiers = self._calculate_final_scores(quality_frontiers)
        
        # 📈 更新统计信息
        self.detection_stats['total_detections'] += 1
        
        return final_frontiers

    def _detect_multiscale_frontier_pixels(self, grid: np.ndarray) -> List[Tuple[int, int]]:
        """多尺度前沿像素检测"""
        all_frontier_pixels = set()

        for scale in self.scale_levels:
            # 🔍 在不同尺度下检测前沿点
            scale_frontiers = self._detect_frontier_pixels_at_scale(grid, scale)
            all_frontier_pixels.update(scale_frontiers)

        # 🎯 去重并转换为列表
        return list(all_frontier_pixels)

    def _detect_frontier_pixels_at_scale(self, grid: np.ndarray, scale: float) -> List[Tuple[int, int]]:
        """在指定尺度下检测前沿像素 - 优化版本"""
        # 🚀 使用向量化操作大幅提升性能
        height, width = grid.shape

        # 创建掩码：自由空间
        free_space_mask = (grid == 0)

        # 使用形态学操作快速检测边界
        from scipy import ndimage

        # 创建结构元素
        detection_radius = max(1, int(scale * 2))
        struct_elem = np.ones((2*detection_radius+1, 2*detection_radius+1))

        # 检测未知区域边界
        unknown_mask = (grid == -1)
        unknown_dilated = ndimage.binary_dilation(unknown_mask, structure=struct_elem)

        # 检测障碍物边界
        obstacle_mask = (grid == 100)
        obstacle_dilated = ndimage.binary_dilation(obstacle_mask, structure=struct_elem)

        # 前沿点：自由空间 & 邻近未知区域 & 不邻近障碍物
        frontier_mask = free_space_mask & unknown_dilated & ~obstacle_dilated

        # 获取前沿点坐标
        frontier_coords = np.where(frontier_mask)
        frontier_pixels = list(zip(frontier_coords[1], frontier_coords[0]))  # (x, y)格式

        # 🎯 限制前沿点数量以提高性能
        if len(frontier_pixels) > 1000:
            # 使用步长采样减少前沿点数量
            step = len(frontier_pixels) // 1000
            frontier_pixels = frontier_pixels[::step]

        return frontier_pixels

    def _detect_frontier_pixels(self, grid: np.ndarray) -> List[Tuple[int, int]]:
        """
        检测前沿像素，使用优化的向量化检测逻辑
        """
        # 🚀 向量化前沿检测（性能优化）
        h, w = grid.shape

        # 创建掩码：自由空间 (值为0)
        free_mask = (grid == 0)

        # 创建未知区域掩码 (值为-1)
        unknown_mask = (grid == -1)

        # 使用卷积核检测前沿点

        # 4-连通邻域核
        kernel = np.array([[0, 1, 0],
                          [1, 0, 1],
                          [0, 1, 0]], dtype=np.float32)

        # 计算每个自由空间像素周围的未知区域数量
        unknown_neighbors = ndimage.convolve(unknown_mask.astype(np.float32), kernel, mode='constant', cval=0)

        # 前沿条件：自由空间且至少有2个未知邻居
        frontier_mask = free_mask & (unknown_neighbors >= 2)

        # 添加边界缓冲（避免边界像素）
        frontier_mask[0:2, :] = False
        frontier_mask[-2:, :] = False
        frontier_mask[:, 0:2] = False
        frontier_mask[:, -2:] = False

        # 获取前沿像素坐标
        frontier_coords = np.where(frontier_mask)
        frontier_pixels = [(int(c), int(r)) for r, c in zip(frontier_coords[0], frontier_coords[1])]

        return frontier_pixels

    def _adaptive_clustering(self, frontier_pixels: List[Tuple[int, int]], grid: np.ndarray) -> List[List[Tuple[int, int]]]:
        """
        自适应DBSCAN聚类
        """
        if len(frontier_pixels) < self.base_min_samples:
            return []
        
        points = np.array(frontier_pixels)
        
        # 🎯 多维度环境分析
        map_density = self._calculate_map_density(grid)
        map_scale = self._estimate_map_scale(grid)
        frontier_distribution = self._analyze_frontier_distribution(points)
        map_complexity = self._calculate_map_complexity(grid)

        # 🔄 动态参数计算
        eps, min_samples = self._calculate_dynamic_parameters(
            map_density, map_scale, frontier_distribution, map_complexity, len(points)
        )
        
        # 🎯 多策略聚类尝试
        best_clusters = self._multi_strategy_clustering(points, eps, min_samples)

        return best_clusters

    def _analyze_frontier_distribution(self, points: np.ndarray) -> Dict:
        """分析前沿点分布特征 - 优化版本"""
        if len(points) < 2:
            return {'spread': 0.0, 'density': 0.0, 'uniformity': 0.0}

        # 🔍 计算分布范围
        x_range = np.max(points[:, 0]) - np.min(points[:, 0])
        y_range = np.max(points[:, 1]) - np.min(points[:, 1])
        spread = math.sqrt(x_range**2 + y_range**2)

        # 🔍 计算密度
        area = max(x_range * y_range, 1)
        density = len(points) / area

        # 🚀 优化均匀性计算：使用采样而不是全距离计算
        if len(points) > 100:
            # 对于大量点，随机采样计算均匀性
            sample_size = min(100, len(points))
            indices = np.random.choice(len(points), sample_size, replace=False)
            sample_points = points[indices]
        else:
            sample_points = points

        # 使用简化的均匀性度量：标准差/均值
        center = np.mean(sample_points, axis=0)
        distances_to_center = np.linalg.norm(sample_points - center, axis=1)
        uniformity = 1.0 / (1.0 + np.std(distances_to_center) / (np.mean(distances_to_center) + 1e-6))

        return {
            'spread': spread,
            'density': density,
            'uniformity': uniformity
        }

    def _calculate_map_complexity(self, grid: np.ndarray) -> float:
        """计算地图复杂度 - 优化版本"""
        # 🚀 使用简化的复杂度计算，避免昂贵的边缘检测
        height, width = grid.shape

        # 对大地图进行下采样以提高性能
        if height > 200 or width > 200:
            # 下采样到合理尺寸
            scale_factor = min(200 / height, 200 / width)
            new_height = int(height * scale_factor)
            new_width = int(width * scale_factor)
            grid_small = cv2.resize(grid.astype(np.uint8), (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        else:
            grid_small = grid

        # 使用简单的梯度计算代替Canny边缘检测
        obstacle_mask = (grid_small == 100)

        # 计算梯度（简化版边缘检测）
        grad_x = np.abs(np.diff(obstacle_mask.astype(float), axis=1))
        grad_y = np.abs(np.diff(obstacle_mask.astype(float), axis=0))

        edge_density = (np.sum(grad_x) + np.sum(grad_y)) / (grid_small.size)

        return min(edge_density * 5, 1.0)  # 归一化到0-1

    def _calculate_dynamic_parameters(self, map_density: float, map_scale: float,
                                    frontier_dist: Dict, map_complexity: float,
                                    point_count: int) -> Tuple[float, int]:
        """动态计算DBSCAN参数"""
        # 🎯 基础参数调整
        base_eps = self.base_eps
        base_min_samples = self.base_min_samples

        # 🔄 根据地图密度调整
        if map_density > 0.4:  # 高密度环境
            eps_factor = 0.6
            min_samples_factor = 0.8
        elif map_density < 0.15:  # 低密度环境
            eps_factor = 1.4
            min_samples_factor = 0.6
        else:  # 中等密度环境
            eps_factor = 1.0
            min_samples_factor = 1.0

        # 🔄 根据前沿点分布调整
        if frontier_dist['spread'] > 100:  # 分布很广
            eps_factor *= 1.2
        elif frontier_dist['spread'] < 20:  # 分布很密
            eps_factor *= 0.8

        # 🔄 根据地图复杂度调整
        if map_complexity > 0.5:  # 复杂环境
            min_samples_factor *= 1.2

        # 🔄 根据点数量调整
        if point_count > 200:  # 大量前沿点
            min_samples_factor *= 1.1
        elif point_count < 50:  # 少量前沿点
            min_samples_factor *= 0.8

        # 🎯 计算最终参数
        final_eps = base_eps * eps_factor * map_scale
        final_min_samples = max(2, int(base_min_samples * min_samples_factor))

        # 🔒 限制在合理范围内
        final_eps = max(self.dynamic_eps_range[0], min(self.dynamic_eps_range[1], final_eps))
        final_min_samples = max(2, min(10, final_min_samples))

        return final_eps, final_min_samples

    def _multi_strategy_clustering(self, points: np.ndarray, eps: float, min_samples: int) -> List[List[Tuple[int, int]]]:
        """多策略聚类尝试"""
        strategies = [
            (eps, min_samples),                    # 主策略
            (eps * 0.8, min_samples),             # 更紧密聚类
            (eps * 1.2, max(2, min_samples - 1)), # 更松散聚类
            (self.base_eps, self.base_min_samples) # 备选策略
        ]

        best_clusters = []
        best_quality = -1.0

        for strategy_eps, strategy_min_samples in strategies:
            try:
                # 🚀 并行DBSCAN聚类
                clustering = DBSCAN(eps=strategy_eps, min_samples=strategy_min_samples, n_jobs=-1).fit(points)
                labels = clustering.labels_
                clusters = self._extract_clusters(points, labels)

                if len(clusters) > 0:
                    # 🎯 评估聚类质量
                    quality = self._evaluate_clustering_quality(points, labels)

                    if quality > best_quality:
                        best_quality = quality
                        best_clusters = clusters

                    # 如果质量足够好，提前返回
                    if quality > 0.7:
                        break

            except Exception:
                continue

        return best_clusters

    def _adjust_dynamic_thresholds(self, frontier_clusters: List[List[Tuple[int, int]]], grid: np.ndarray) -> None:
        """动态调整检测阈值"""
        if not frontier_clusters:
            return

        # 🔍 分析当前检测结果
        cluster_sizes = [len(cluster) for cluster in frontier_clusters]
        avg_cluster_size = np.mean(cluster_sizes)
        cluster_count = len(frontier_clusters)

        # 🎯 根据检测结果调整参数
        if cluster_count > 20:  # 检测到太多小聚类
            self.min_frontier_size = min(self.min_frontier_size + 1, 8)
            self.base_eps = min(self.base_eps * 1.1, self.dynamic_eps_range[1])
        elif cluster_count < 3 and avg_cluster_size > 15:  # 检测到太少大聚类
            self.min_frontier_size = max(self.min_frontier_size - 1, 2)
            self.base_eps = max(self.base_eps * 0.9, self.dynamic_eps_range[0])

        # 🔄 根据地图探索进度调整
        map_exploration_ratio = self._calculate_exploration_ratio(grid)
        if map_exploration_ratio > 0.8:  # 后期探索，需要更精细的检测
            self.min_frontier_size = max(2, self.min_frontier_size - 1)
        elif map_exploration_ratio < 0.3:  # 早期探索，可以更粗糙
            self.min_frontier_size = min(6, self.min_frontier_size + 1)

    def _calculate_exploration_ratio(self, grid: np.ndarray) -> float:
        """计算地图探索比例"""
        total_cells = grid.size
        unknown_cells = np.sum(grid == -1)
        return 1.0 - (unknown_cells / max(total_cells, 1))

    def _calculate_map_density(self, grid: np.ndarray) -> float:
        """计算地图密度"""
        total_cells = grid.size
        free_cells = np.sum(grid == 0)
        occupied_cells = np.sum(grid > 0)
        
        if total_cells == 0:
            return 0.0
        
        return (free_cells + occupied_cells) / total_cells

    def _estimate_map_scale(self, grid: np.ndarray) -> float:
        """估计地图尺度因子"""
        # 基于地图大小的简单尺度估计
        map_area = grid.shape[0] * grid.shape[1]
        
        # 归一化到0.5-2.0范围
        if map_area < 100000:      # 小地图
            return 0.5
        elif map_area > 1000000:   # 大地图
            return 2.0
        else:                      # 中等地图
            return 1.0

    def _evaluate_clustering_quality(self, points: np.ndarray, labels: np.ndarray) -> float:
        """评估聚类质量"""
        try:
            # 排除噪声点
            mask = labels != -1
            if np.sum(mask) < 2:
                return 0.0
            
            filtered_points = points[mask]
            filtered_labels = labels[mask]
            
            # 检查是否有足够的聚类
            unique_labels = len(set(filtered_labels))
            if unique_labels < 2:
                return 0.0
            
            # 使用轮廓系数评估
            silhouette = silhouette_score(filtered_points, filtered_labels)
            
            # 额外考虑聚类数量的合理性
            cluster_count_penalty = 1.0
            if unique_labels > 10:  # 太多聚类
                cluster_count_penalty = 0.8
            elif unique_labels < 2:  # 太少聚类
                cluster_count_penalty = 0.6
            
            return max(0.0, silhouette * cluster_count_penalty)
            
        except Exception:
            return 0.0

    def _extract_clusters(self, points: np.ndarray, labels: np.ndarray) -> List[List[Tuple[int, int]]]:
        """从聚类标签中提取聚类"""
        clusters = []
        unique_labels = set(labels)
        
        for label in unique_labels:
            if label == -1:  # 跳过噪声点
                continue
            
            cluster_points = points[labels == label]
            if len(cluster_points) >= self.min_frontier_size:
                cluster_list = [(int(p[0]), int(p[1])) for p in cluster_points]
                clusters.append(cluster_list)
        
        return clusters

    def _evaluate_frontier_quality(self, clusters: List[List[Tuple[int, int]]], 
                                 grid: np.ndarray, map_data) -> List[OptimizedFrontierPoint]:
        """评估前沿质量并创建优化的前沿点"""
        quality_frontiers = []
        
        for cluster in clusters:
            # 🎯 计算安全的质心目标点
            target_x, target_y = self._find_safe_centroid_target(grid, cluster)
            if target_x is None:
                continue
            
            # 转换为世界坐标
            world_x = map_data.info.origin.position.x + target_x * map_data.info.resolution
            world_y = map_data.info.origin.position.y + target_y * map_data.info.resolution
            
            # 🔍 创建优化的前沿点
            frontier = OptimizedFrontierPoint(
                x=world_x,
                y=world_y,
                size=len(cluster)
            )
            
            # 📊 评估各项质量指标
            frontier.door_weight = self._compute_enhanced_door_weight(grid, target_x, target_y, map_data)
            frontier.quality_score = self._compute_frontier_quality_score(cluster, grid, target_x, target_y)
            frontier.accessibility_score = self._compute_accessibility_score(grid, target_x, target_y)
            
            # 📈 只保留质量足够高的前沿点 (降低阈值以确保前沿点可见)
            if frontier.quality_score > 0.1 and frontier.accessibility_score > 0.1:
                quality_frontiers.append(frontier)
                self.detection_stats['quality_improvements'] += 1
        
        return quality_frontiers

    def _find_safe_centroid_target(self, grid: np.ndarray, cluster: List[Tuple[int, int]]) -> Tuple[Optional[int], Optional[int]]:
        """寻找安全的质心目标点"""
        if not cluster:
            return None, None
        
        # 计算质心
        cluster_array = np.array(cluster)
        center_x = np.mean(cluster_array[:, 0])
        center_y = np.mean(cluster_array[:, 1])
        
        # 按距离质心的距离排序
        sorted_points = sorted(cluster, key=lambda p: (p[0] - center_x)**2 + (p[1] - center_y)**2)
        
        # 寻找第一个安全的点
        robot_radius_cells = int(self.robot_radius / self.map_resolution)
        
        for px, py in sorted_points:
            if self._is_safe_target(grid, px, py, robot_radius_cells):
                return px, py
        
        return None, None

    def _is_safe_target(self, grid: np.ndarray, x: int, y: int, radius_cells: int) -> bool:
        """检查目标点是否安全"""
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                check_x, check_y = x + dx, y + dy
                if 0 <= check_x < grid.shape[1] and 0 <= check_y < grid.shape[0]:
                    if grid[check_y, check_x] > 0:  # 遇到障碍物
                        return False
                # 超出边界也认为不安全
                elif not (0 <= check_x < grid.shape[1] and 0 <= check_y < grid.shape[0]):
                    return False
        return True

    def _compute_enhanced_door_weight(self, grid: np.ndarray, x: int, y: int, map_data) -> float:
        """增强的门洞权重计算"""
        try:
            # 🚪 多方向门洞检测
            directions = [
                (1, 0),   # 水平
                (0, 1),   # 垂直
                (1, 1),   # 对角线
                (1, -1)   # 反对角线
            ]
            
            max_door_score = 1.0
            
            for dx, dy in directions:
                # 正向和负向扫描
                pos_dist = self._scan_free_space(grid, x, y, dx, dy)
                neg_dist = self._scan_free_space(grid, x, y, -dx, -dy)
                
                total_width = (pos_dist + neg_dist) * self.map_resolution
                
                # 检查是否符合门洞特征
                if self.door_width_min < total_width < self.door_width_max:
                    # 计算门洞评分
                    door_score = 1.5 + min(0.5, (total_width - self.door_width_min) / (self.door_width_max - self.door_width_min))
                    max_door_score = max(max_door_score, door_score)
            
            return max_door_score
            
        except Exception:
            return 1.0

    def _scan_free_space(self, grid: np.ndarray, start_x: int, start_y: int, dx: int, dy: int) -> int:
        """扫描自由空间距离"""
        distance = 0
        x, y = start_x, start_y
        
        while 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
            if grid[y, x] != 0:  # 遇到非自由空间
                break
            distance += 1
            x += dx
            y += dy
            
            # 防止无限扫描
            if distance > 50:
                break
        
        return distance

    def _compute_frontier_quality_score(self, cluster: List[Tuple[int, int]], 
                                      grid: np.ndarray, target_x: int, target_y: int) -> float:
        """计算前沿质量评分"""
        try:
            # 🎯 基础评分因子
            size_score = min(1.0, len(cluster) / 20.0)  # 大小评分
            
            # 🔍 周围未知区域密度
            unknown_density = self._calculate_unknown_density(grid, target_x, target_y, radius=5)
            
            # 📊 前沿形状紧凑性
            compactness = self._calculate_cluster_compactness(cluster)
            
            # 🎯 综合质量评分
            quality = (size_score * 0.4 + unknown_density * 0.4 + compactness * 0.2)
            
            return min(1.0, max(0.0, quality))
            
        except Exception:
            return 0.5

    def _calculate_unknown_density(self, grid: np.ndarray, x: int, y: int, radius: int = 5) -> float:
        """计算周围未知区域密度"""
        unknown_count = 0
        total_count = 0
        
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                check_x, check_y = x + dx, y + dy
                if 0 <= check_x < grid.shape[1] and 0 <= check_y < grid.shape[0]:
                    total_count += 1
                    if grid[check_y, check_x] == -1:
                        unknown_count += 1
        
        return unknown_count / total_count if total_count > 0 else 0.0

    def _calculate_cluster_compactness(self, cluster: List[Tuple[int, int]]) -> float:
        """计算聚类紧凑性"""
        if len(cluster) <= 1:
            return 1.0
        
        cluster_array = np.array(cluster)
        center = np.mean(cluster_array, axis=0)
        
        # 计算平均距离和标准差
        distances = [np.linalg.norm(point - center) for point in cluster_array]
        avg_distance = np.mean(distances)
        std_distance = np.std(distances)
        
        # 紧凑性 = 1 / (1 + 变异系数)
        if avg_distance == 0:
            return 1.0
        
        coefficient_variation = std_distance / avg_distance
        compactness = 1.0 / (1.0 + coefficient_variation)
        
        return compactness

    def _compute_accessibility_score(self, grid: np.ndarray, x: int, y: int) -> float:
        """计算可达性评分"""
        try:
            # 🛣️ 检查多个方向的可达性
            directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
            accessible_directions = 0
            
            for dx, dy in directions:
                if self._check_direction_accessibility(grid, x, y, dx, dy, max_distance=10):
                    accessible_directions += 1
            
            # 可达性评分基于可达方向的比例
            accessibility = accessible_directions / len(directions)
            
            # 🎯 额外检查是否有足够的自由空间
            free_space_ratio = self._calculate_nearby_free_space(grid, x, y, radius=3)
            
            # 综合可达性评分
            final_score = (accessibility * 0.7 + free_space_ratio * 0.3)
            
            return min(1.0, max(0.0, final_score))
            
        except Exception:
            return 0.5

    def _check_direction_accessibility(self, grid: np.ndarray, start_x: int, start_y: int, 
                                     dx: int, dy: int, max_distance: int = 10) -> bool:
        """检查特定方向的可达性"""
        x, y = start_x, start_y
        
        for _ in range(max_distance):
            x += dx
            y += dy
            
            if not (0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]):
                return False
            
            if grid[y, x] > 0:  # 遇到障碍物
                return False
            
            if grid[y, x] == 0:  # 到达自由空间
                return True
        
        return True

    def _calculate_nearby_free_space(self, grid: np.ndarray, x: int, y: int, radius: int = 3) -> float:
        """计算附近自由空间比例"""
        free_count = 0
        total_count = 0
        
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                check_x, check_y = x + dx, y + dy
                if 0 <= check_x < grid.shape[1] and 0 <= check_y < grid.shape[0]:
                    total_count += 1
                    if grid[check_y, check_x] == 0:
                        free_count += 1
        
        return free_count / total_count if total_count > 0 else 0.0

    def _calculate_room_priorities(self, frontiers: List[OptimizedFrontierPoint], 
                                 room_info: Dict, map_data) -> None:
        """计算房间优先级"""
        if not room_info or 'room_centroids' not in room_info:
            return
        
        room_centroids = room_info['room_centroids']
        room_stats = room_info.get('room_stats', {})
        
        for frontier in frontiers:
            # 🏠 找到最近的房间
            min_distance = float('inf')
            closest_room_id = None
            
            for room_id, (room_x, room_y) in room_centroids.items():
                distance = math.sqrt((frontier.x - room_x)**2 + (frontier.y - room_y)**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_room_id = room_id
            
            # 📊 计算房间优先级
            if closest_room_id and closest_room_id in room_stats:
                stats = room_stats[closest_room_id]
                
                # 基于房间面积、边界因子和未知比例计算优先级
                area_factor = min(1.0, stats.get('area', 0) / 1000.0)
                boundary_factor = stats.get('boundary_factor', 1.0)
                unknown_ratio = stats.get('unknown_ratio', 0.0)
                
                # 距离惩罚
                distance_penalty = 1.0 / (1.0 + min_distance / 5.0)
                
                room_priority = (area_factor * 0.3 + 
                               boundary_factor * 0.3 + 
                               unknown_ratio * 0.2 + 
                               distance_penalty * 0.2)
                
                frontier.room_priority = min(1.0, max(0.0, room_priority))

    def _calculate_final_scores(self, frontiers: List[OptimizedFrontierPoint]) -> List[OptimizedFrontierPoint]:
        """计算最终综合评分"""
        for frontier in frontiers:
            # 🎯 综合评分计算
            size_component = min(1.0, frontier.size / 20.0) * self.weights['size']
            accessibility_component = frontier.accessibility_score * self.weights['accessibility']
            room_component = frontier.room_priority * self.weights['room_priority']
            door_component = (frontier.door_weight - 1.0) * self.weights['door_bonus']
            
            # 📊 最终探索价值
            frontier.exploration_value = (size_component + 
                                        accessibility_component + 
                                        room_component + 
                                        door_component + 
                                        frontier.quality_score * 0.1)
        
        # 按探索价值排序
        frontiers.sort(key=lambda f: f.exploration_value, reverse=True)
        
        return frontiers

    def get_detection_stats(self) -> Dict:
        """获取检测统计信息"""
        return self.detection_stats.copy()

    def reset_stats(self) -> None:
        """重置统计信息"""
        self.detection_stats = {
            'total_detections': 0,
            'adaptive_improvements': 0,
            'quality_improvements': 0
        } 