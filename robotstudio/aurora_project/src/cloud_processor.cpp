#include "cloud_processor.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <set>
#include <map>
#include <memory>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <random>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "cloud_visualizer.h"

CloudProcessor::CloudProcessor() {
    // 初始化点云对象
    m_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    m_processedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

CloudProcessor::~CloudProcessor() {
    // 清理资源
}

// 定义平面点到平面的距离函数
static float distanceToPlane(const PlaneStruct& plane, const Point3D& point) {
    return std::abs(plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d) / 
           std::sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);
}

// 判断点是否在平面上
static bool isPointOnPlane(const PlaneStruct& plane, const Point3D& point, float threshold) {
    return distanceToPlane(plane, point) < threshold;
}

bool CloudProcessor::loadFromXYZ(const std::string& filename) {
    // 打开XYZ文件
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }
    
    // 清空原有数据
    m_points.clear();
    m_planes.clear();
    
    // 读取XYZ文件中的每一行
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z;
        int r = 255, g = 255, b = 255;  // 默认为白色
        
        // 尝试读取坐标和颜色
        if (iss >> x >> y >> z) {
            // 尝试读取颜色（如果有）
            iss >> r >> g >> b;
            
            // 添加点到集合中
            Point3D point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = r / 255.0f;
            point.g = g / 255.0f;
            point.b = b / 255.0f;
            m_points.push_back(point);
        }
    }
    
    std::cout << "Loaded " << m_points.size() << " points from XYZ file" << std::endl;
    
    // 创建PCL点云
    m_cloud->clear();
    m_cloud->points.reserve(m_points.size());
    
    for (const auto& p : m_points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = static_cast<uint8_t>(p.r * 255);
        point.g = static_cast<uint8_t>(p.g * 255);
        point.b = static_cast<uint8_t>(p.b * 255);
        m_cloud->points.push_back(point);
    }
    
    m_cloud->width = m_cloud->points.size();
    m_cloud->height = 1;
    m_cloud->is_dense = true;
    
    // 初始化处理后的点云
    m_processedCloud = m_cloud;
    
    return true;
}

bool CloudProcessor::loadFromPLY(const std::string& filename) {
    // 使用PCL库加载PLY文件
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(filename, *m_cloud) == -1) {
        std::cerr << "Cannot load PLY file: " << filename << std::endl;
        return false;
    }
    
    // 清空原有数据
    m_points.clear();
    m_planes.clear();
    
    // 将PCL点云转换为内部点表示
    m_points.reserve(m_cloud->points.size());
    for (const auto& p : m_cloud->points) {
        Point3D point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = p.r / 255.0f;
        point.g = p.g / 255.0f;
        point.b = p.b / 255.0f;
        m_points.push_back(point);
    }
    
    std::cout << "Loaded " << m_points.size() << " points from PLY file" << std::endl;
    
    // 初始化处理后的点云
    m_processedCloud = m_cloud;
    
    return true;
}

bool CloudProcessor::loadFromSTCM(const std::string& filename) {
    std::vector<Point3D> points;
    if (!readSTCM(filename, points)) {
        return false;
    }
    
    // 成功读取后，更新内部数据
    m_points = points;
    
    // 创建PCL点云
    m_cloud->clear();
    m_cloud->points.reserve(m_points.size());
    
    for (const auto& p : m_points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = static_cast<uint8_t>(p.r * 255);
        point.g = static_cast<uint8_t>(p.g * 255);
        point.b = static_cast<uint8_t>(p.b * 255);
        m_cloud->points.push_back(point);
    }
    
    m_cloud->width = m_cloud->points.size();
    m_cloud->height = 1;
    m_cloud->is_dense = true;
    
    // 初始化处理后的点云
    m_processedCloud = m_cloud;
    
    return true;
}

bool CloudProcessor::readSTCM(const std::string& filename, std::vector<Point3D>& points) {
    // 打开STCM文件
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Cannot open STCM file: " << filename << std::endl;
        return false;
    }
    
    // 检查文件头
    char header[4];
    file.read(header, 4);
    if (header[0] != 'S' || header[1] != 'T' || header[2] != 'C' || header[3] != 'M') {
        std::cerr << "Invalid STCM file format" << std::endl;
        return false;
    }
    
    // 读取版本号
    uint32_t version;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    
    // 读取点云数据数量
    uint32_t pointCount;
    file.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
    
    // 准备点集合
    points.clear();
    points.reserve(pointCount);
    
    // 读取点云数据
    for (uint32_t i = 0; i < pointCount; i++) {
        // 读取点坐标
        float x, y, z;
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        
        // 读取点颜色
        uint8_t r, g, b, a;
        file.read(reinterpret_cast<char*>(&r), sizeof(uint8_t));
        file.read(reinterpret_cast<char*>(&g), sizeof(uint8_t));
        file.read(reinterpret_cast<char*>(&b), sizeof(uint8_t));
        file.read(reinterpret_cast<char*>(&a), sizeof(uint8_t));
        
        // 添加点到集合中
        Point3D point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.r = r / 255.0f;
        point.g = g / 255.0f;
        point.b = b / 255.0f;
        points.push_back(point);
    }
    
    std::cout << "Loaded " << points.size() << " points from STCM file" << std::endl;
    return true;
}

bool CloudProcessor::loadFromPoints(const std::vector<Point3D>& points) {
    // 设置内部点集合
    m_points = points;
    
    // 创建PCL点云
    m_cloud->clear();
    m_cloud->points.reserve(points.size());
    
    for (const auto& p : points) {
        pcl::PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = static_cast<uint8_t>(p.r * 255);
        point.g = static_cast<uint8_t>(p.g * 255);
        point.b = static_cast<uint8_t>(p.b * 255);
        m_cloud->points.push_back(point);
    }
    
    m_cloud->width = m_cloud->points.size();
    m_cloud->height = 1;
    m_cloud->is_dense = true;
    
    // 初始化处理后的点云
    m_processedCloud = m_cloud;
    
    std::cout << "Loaded " << m_points.size() << " points" << std::endl;
    return true;
}

void CloudProcessor::denoise(float outlierThreshold) {
    if (m_cloud->empty()) {
        std::cerr << "Cloud is empty, cannot process" << std::endl;
        return;
    }
    
    // 保存原始点云大小
    size_t originalSize = m_cloud->points.size();
    
    // 创建临时点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // 应用统计离群点去除
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(m_cloud);
    sor.setMeanK(25);  // 使用较小值保留更多点
    sor.setStddevMulThresh(outlierThreshold);
    sor.filter(*filteredCloud);
    
    // 检查是否过度过滤
    if (filteredCloud->points.size() < originalSize * 0.5) {
        std::cout << "Warning: Denoising removed over 50% of points, keeping original cloud" << std::endl;
        // 不更新点云，保留原始数据
        return;
    }
    
    // 更新点云
    m_cloud = filteredCloud;
    
    // 更新内部点表示
    m_points.clear();
    m_points.reserve(m_cloud->points.size());
    
    for (const auto& p : m_cloud->points) {
        Point3D point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = p.r / 255.0f;
        point.g = p.g / 255.0f;
        point.b = p.b / 255.0f;
        m_points.push_back(point);
    }
    
    std::cout << "Denoising kept " << m_cloud->points.size() << " points" << std::endl;
}

bool CloudProcessor::processCloud(float leafSize, int meanK, float stddevMulThresh) {
    if (m_cloud->empty()) {
        std::cerr << "Cloud is empty, cannot process" << std::endl;
        return false;
    }
    
    // 保存原始点云副本
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZRGB>(*m_cloud));
    
    // 处理1: 去噪
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoisedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(m_cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*denoisedCloud);
    
    // 检查去噪结果
    if (denoisedCloud->points.size() < m_cloud->points.size() * 0.5) {
        std::cout << "Warning: Denoising removed over 50%, using original cloud" << std::endl;
        denoisedCloud = originalCloud;
    }
    
    // 处理2: 下采样
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(denoisedCloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*downsampledCloud);
    
    // 检查下采样结果
    if (downsampledCloud->points.size() < denoisedCloud->points.size() * 0.3) {
        std::cout << "Warning: Downsampling removed over 70%, using denoised cloud" << std::endl;
        downsampledCloud = denoisedCloud;
    }
    
    // 更新处理后的点云
    m_processedCloud = downsampledCloud;
    
    std::cout << "Processing kept " << m_processedCloud->points.size() << " points" << std::endl;
    return true;
}

bool CloudProcessor::process() {
    // 默认处理流程
    return processCloud(0.005, 25, 1.0);
}

std::vector<PlaneStruct> CloudProcessor::extractPlanes(float distanceThreshold) {
    std::vector<PlaneStruct> planes;
    
    if (!m_processedCloud || m_processedCloud->empty()) {
        return planes;
    }
    
    // 使用更低的阈值来提高平面检测灵敏度
    float actualThreshold = std::min(distanceThreshold, 0.01f); // 使用更小的阈值
    
    // 创建一个临时点云用于平面分割
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr workingCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*m_processedCloud, *workingCloud);
    
    // 使用RANSAC进行平面分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    
    // 设置分割参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(actualThreshold);
    seg.setMaxIterations(2000); // 增加迭代次数
    
    // 最小平面点数 - 降低以检测更多平面
    const int minPointsPerPlane = 50;
    
    // 最多提取10个平面
    const int maxPlanes = 10;
    int remainingPoints = workingCloud->size();
    
    for (int i = 0; i < maxPlanes && remainingPoints > minPointsPerPlane; ++i) {
        // 分割最大平面
        seg.setInputCloud(workingCloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty()) {
            break; // 没有更多平面
        }
        
        // 提取平面点
        PlaneStruct plane;
        plane.a = coefficients->values[0];
        plane.b = coefficients->values[1];
        plane.c = coefficients->values[2];
        plane.d = coefficients->values[3];
        
        for (size_t idx : inliers->indices) {
            const auto& pt = workingCloud->points[idx];
            Point3D point;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            point.r = pt.r / 255.0f;
            point.g = pt.g / 255.0f;
            point.b = pt.b / 255.0f;
            plane.points.push_back(point);
        }
        
        // 只添加有足够点的平面
        if (plane.points.size() > minPointsPerPlane) {
            planes.push_back(plane);
            std::cout << "Found plane " << i + 1 << " with " << plane.points.size() << " points" << std::endl;
        }
        
        // 从工作点云中移除这些点
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(workingCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*tempCloud);
        workingCloud = tempCloud;
        
        remainingPoints = workingCloud->size();
    }
    
    // 保存提取的平面
    m_planes = planes;
    
    return planes;
}

bool CloudProcessor::saveToPLY(const std::string& filename) {
    // 确保有点云数据
    if (m_processedCloud->empty()) {
        std::cerr << "Processed cloud is empty, cannot save" << std::endl;
        return false;
    }
    
    // 保存为PLY格式
    pcl::PLYWriter writer;
    int result = writer.write(filename, *m_processedCloud);
    
    if (result != 0) {
        std::cerr << "Failed to save PLY file: " << filename << std::endl;
        return false;
    }
    
    std::cout << "Successfully saved " << m_processedCloud->points.size() << " points to PLY file: " << filename << std::endl;
    return true;
}

size_t CloudProcessor::getPointCount() const {
    return m_points.size();
}

size_t CloudProcessor::getPlaneCount() const {
    return m_planes.size();
}

size_t CloudProcessor::getWallCount() const {
    return m_walls.size();
}

// 新增：密度过滤函数
void filterByDensity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, 
                    float radius, int min_neighbors) {
    std::cout << "Filtering by point density (radius=" << radius 
              << ", min_neighbors=" << min_neighbors << ")..." << std::endl;
    
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_in);
    
    cloud_out->clear();
    cloud_out->header = cloud_in->header;
    
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    
    // 计算每个点的邻居数量并保留密度高的点
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        int neighbors = kdtree.radiusSearch(cloud_in->points[i], radius, point_indices, point_distances);
        
        if (neighbors >= min_neighbors) {
            cloud_out->points.push_back(cloud_in->points[i]);
        }
    }
    
    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = false;
    
    std::cout << "Density filtering complete. Points remaining: " 
              << cloud_out->size() << " (removed " 
              << cloud_in->size() - cloud_out->size() << " points)" << std::endl;
}

void removeNoise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, int meanK, double stddevMulThresh) {
    std::cout << "Removing noise (meanK=" << meanK << ", stddev=" << stddevMulThresh << ")..." << std::endl;
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*cloud_out);
    
    std::cout << "Noise removal complete. Points remaining: " 
              << cloud_out->size() << " (removed " 
              << cloud_in->size() - cloud_out->size() << " points)" << std::endl;
}

void downsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float leafSize) {
    std::cout << "Downsampling cloud (leaf size=" << leafSize << ")..." << std::endl;
    
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*cloud_out);
    
    std::cout << "Downsampling complete. Points remaining: " 
              << cloud_out->size() << " (removed " 
              << cloud_in->size() - cloud_out->size() << " points)" << std::endl;
}

void extractPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float distanceThreshold = 0.01) {
    std::cout << "Extracting planes (distance threshold=" << distanceThreshold << ")..." << std::endl;
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(2000);
    
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_filtered = *cloud_in;
    
    int plane_count = 0;
    int remaining_points = cloud_filtered->size();
    
    // While 30% of the original cloud is still there
    while (remaining_points > 0.3 * cloud_in->size() && plane_count < 10) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < 50) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        
        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        
        // Get the points that are on the plane
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        
        std::cout << "Plane " << plane_count << " with " << cloud_plane->size() << " points." << std::endl;
        
        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
        
        remaining_points = cloud_filtered->size();
        plane_count++;
    }
    
    *cloud_out = *cloud_filtered;
    std::cout << "Plane extraction complete. Extracted " << plane_count << " planes." << std::endl;
    std::cout << "Points remaining: " << cloud_out->size() << std::endl;
}

void printHelp() {
    std::cout << "Usage: cloud_processor_tool <input_file> <output_file> [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --denoise <meanK> <stddev>      Apply statistical outlier removal" << std::endl;
    std::cout << "  --downsample <leaf_size>        Apply voxel grid downsampling" << std::endl;
    std::cout << "  --extract-planes <threshold>    Extract planes from the point cloud" << std::endl;
    std::cout << "  --density-filter <radius> <min_neighbors>  Filter by point density" << std::endl;
    std::cout << "  --help                          Show this help message" << std::endl;
}

// 门窗检测实现
std::vector<DoorWindow> CloudProcessor::detectDoorWindows(const std::vector<PlaneStruct>& walls) {
    std::vector<DoorWindow> doorWindows;

    if (!m_processedCloud || m_processedCloud->empty()) {
        return doorWindows;
    }

    std::cout << "开始检测门窗..." << std::endl;

    // 对每个墙面平面进行门窗检测
    for (const auto& wall : walls) {
        if (wall.points.size() < 100) continue; // 跳过太小的平面

        // 检查是否为垂直墙面（法线向量接近水平）
        float normalZ = std::abs(wall.c);
        if (normalZ > 0.3f) continue; // 不是垂直墙面，跳过

        // 在墙面上检测门窗开口
        auto wallDoorWindows = detectOpeningsInWall(wall);
        doorWindows.insert(doorWindows.end(), wallDoorWindows.begin(), wallDoorWindows.end());
    }

    std::cout << "检测到 " << doorWindows.size() << " 个门窗" << std::endl;
    return doorWindows;
}

// 在单个墙面上检测门窗开口
std::vector<DoorWindow> CloudProcessor::detectOpeningsInWall(const PlaneStruct& wall) {
    std::vector<DoorWindow> openings;

    // 将3D点投影到2D平面
    std::vector<std::pair<float, float>> projectedPoints;
    float minX = FLT_MAX, maxX = -FLT_MAX;
    float minZ = FLT_MAX, maxZ = -FLT_MAX;

    for (const auto& point : wall.points) {
        // 简化投影：使用X和Z坐标
        projectedPoints.push_back({point.x, point.z});
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }

    // 创建2D网格来检测空洞
    const int gridWidth = 50;
    const int gridHeight = 30;
    std::vector<std::vector<int>> grid(gridHeight, std::vector<int>(gridWidth, 0));

    float stepX = (maxX - minX) / gridWidth;
    float stepZ = (maxZ - minZ) / gridHeight;

    if (stepX <= 0 || stepZ <= 0) return openings;

    // 填充网格
    for (const auto& point : projectedPoints) {
        int gridX = std::min(gridWidth - 1, (int)((point.first - minX) / stepX));
        int gridZ = std::min(gridHeight - 1, (int)((point.second - minZ) / stepZ));
        grid[gridZ][gridX]++;
    }

    // 检测矩形空洞区域
    for (int z = 1; z < gridHeight - 1; z++) {
        for (int x = 1; x < gridWidth - 1; x++) {
            if (grid[z][x] == 0) { // 空洞点
                // 尝试扩展矩形区域
                DoorWindow opening = expandRectangularOpening(grid, x, z, gridWidth, gridHeight);
                if (opening.confidence > 0.5f) {
                    // 转换回世界坐标
                    opening.center.x = minX + (x + opening.width/2) * stepX;
                    opening.center.z = minZ + (z + opening.height/2) * stepZ;
                    opening.center.y = 0; // 墙面的Y坐标

                    // 根据尺寸判断是门还是窗
                    float realWidth = opening.width * stepX;
                    float realHeight = opening.height * stepZ;

                    if (realHeight > 1.8f && opening.center.z < 0.3f) {
                        opening.type = DoorWindow::DOOR;
                    } else if (realHeight > 0.8f && opening.center.z > 0.8f) {
                        opening.type = DoorWindow::WINDOW;
                    }

                    openings.push_back(opening);
                }
            }
        }
    }

    return openings;
}

// 扩展矩形开口区域
DoorWindow CloudProcessor::expandRectangularOpening(const std::vector<std::vector<int>>& grid,
                                                   int startX, int startZ, int gridWidth, int gridHeight) {
    DoorWindow opening;

    // 向右扩展
    int endX = startX;
    while (endX < gridWidth - 1 && grid[startZ][endX + 1] == 0) {
        endX++;
    }

    // 向下扩展
    int endZ = startZ;
    while (endZ < gridHeight - 1 && grid[endZ + 1][startX] == 0) {
        endZ++;
    }

    // 检查矩形区域的完整性
    int emptyCount = 0;
    int totalCount = 0;

    for (int z = startZ; z <= endZ; z++) {
        for (int x = startX; x <= endX; x++) {
            totalCount++;
            if (grid[z][x] == 0) {
                emptyCount++;
            }
        }
    }

    // 计算置信度
    float emptyRatio = (float)emptyCount / totalCount;
    opening.confidence = emptyRatio;

    // 设置尺寸
    opening.width = endX - startX + 1;
    opening.height = endZ - startZ + 1;

    // 只有当空洞比例足够高且尺寸合理时才认为是有效开口
    if (emptyRatio > 0.7f && opening.width >= 3 && opening.height >= 5) {
        opening.confidence = emptyRatio;
    } else {
        opening.confidence = 0.0f;
    }

    return opening;
}

// 检测门窗并标记颜色
std::vector<DoorWindow> CloudProcessor::detectAndMarkDoorWindows() {
    // 首先提取平面
    auto planes = extractPlanes(0.02f);

    // 检测门窗
    m_doorWindows = detectDoorWindows(planes);

    // 为门窗点着色
    colorDoorWindowPoints(m_doorWindows);

    return m_doorWindows;
}

// 完整建筑元素检测和分类
bool CloudProcessor::detectBuildingElements() {
    if (!m_processedCloud || m_processedCloud->empty()) {
        std::cerr << "No processed cloud data available" << std::endl;
        return false;
    }

    std::cout << "开始完整建筑元素检测..." << std::endl;

    // 1. 提取所有平面
    auto allPlanes = extractPlanes(0.015f); // 使用更精确的阈值
    std::cout << "检测到 " << allPlanes.size() << " 个平面" << std::endl;

    // 2. 分类平面
    classifyPlanes(allPlanes);

    // 3. 高级门窗检测
    m_doorWindows = detectAdvancedDoorWindows();

    // 4. 分离门和窗
    m_doors.clear();
    m_windows.clear();
    for (const auto& dw : m_doorWindows) {
        if (dw.type == DoorWindow::DOOR) {
            m_doors.push_back(dw);
        } else if (dw.type == DoorWindow::WINDOW) {
            m_windows.push_back(dw);
        }
    }

    // 5. 应用智能颜色编码
    applyIntelligentColoring();

    std::cout << "建筑元素检测完成:" << std::endl;
    std::cout << "  墙面: " << m_wallPlanes.size() << " 个" << std::endl;
    std::cout << "  地面: " << m_floorPlanes.size() << " 个" << std::endl;
    std::cout << "  屋顶: " << m_ceilingPlanes.size() << " 个" << std::endl;
    std::cout << "  门: " << m_doors.size() << " 个" << std::endl;
    std::cout << "  窗: " << m_windows.size() << " 个" << std::endl;

    return true;
}

// 分类平面为墙面、地面、屋顶 - 改进版
void CloudProcessor::classifyPlanes(const std::vector<PlaneStruct>& planes) {
    m_wallPlanes.clear();
    m_floorPlanes.clear();
    m_ceilingPlanes.clear();

    // 计算点云的整体高度范围
    float minZ = FLT_MAX, maxZ = -FLT_MAX;
    for (const auto& point : m_processedCloud->points) {
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }
    float heightRange = maxZ - minZ;
    float floorThreshold = minZ + heightRange * 0.2f;  // 底部20%认为是地面
    float ceilingThreshold = maxZ - heightRange * 0.2f; // 顶部20%认为是天花板

    std::cout << "Height analysis: min=" << minZ << ", max=" << maxZ << ", range=" << heightRange << std::endl;
    std::cout << "Floor threshold: " << floorThreshold << ", Ceiling threshold: " << ceilingThreshold << std::endl;

    for (const auto& plane : planes) {
        // 计算法向量的角度
        float normalZ = std::abs(plane.c);
        float normalXY = std::sqrt(plane.a * plane.a + plane.b * plane.b);

        // 计算平面中心点的Z坐标和Z坐标分布
        float avgZ = 0.0f;
        float minPlaneZ = FLT_MAX, maxPlaneZ = -FLT_MAX;
        for (const auto& point : plane.points) {
            avgZ += point.z;
            minPlaneZ = std::min(minPlaneZ, point.z);
            maxPlaneZ = std::max(maxPlaneZ, point.z);
        }
        avgZ /= plane.points.size();
        float planeZRange = maxPlaneZ - minPlaneZ;

        // 改进的分类逻辑
        if (normalZ > 0.7f) {
            // 水平平面 (法向量主要指向Z方向)
            if (avgZ < floorThreshold && planeZRange < heightRange * 0.1f) {
                // 低位置且相对平坦的水平平面 -> 地面
                m_floorPlanes.push_back(plane);
                std::cout << "Classified as FLOOR: avgZ=" << avgZ << ", normalZ=" << normalZ << ", points=" << plane.points.size() << std::endl;
            } else if (avgZ > ceilingThreshold && planeZRange < heightRange * 0.1f) {
                // 高位置且相对平坦的水平平面 -> 屋顶/天花板
                m_ceilingPlanes.push_back(plane);
                std::cout << "Classified as CEILING: avgZ=" << avgZ << ", normalZ=" << normalZ << ", points=" << plane.points.size() << std::endl;
            }
        } else if (normalXY > 0.7f && normalZ < 0.4f) {
            // 垂直平面 (法向量主要在XY平面) -> 墙面
            if (plane.points.size() > 50 && planeZRange > heightRange * 0.3f) {
                // 足够大且有一定高度的垂直平面
                m_wallPlanes.push_back(plane);
                std::cout << "Classified as WALL: avgZ=" << avgZ << ", normalXY=" << normalXY << ", points=" << plane.points.size() << ", zRange=" << planeZRange << std::endl;
            }
        }
    }

    std::cout << "平面分类完成:" << std::endl;
    std::cout << "  墙面: " << m_wallPlanes.size() << " 个" << std::endl;
    std::cout << "  地面: " << m_floorPlanes.size() << " 个" << std::endl;
    std::cout << "  屋顶: " << m_ceilingPlanes.size() << " 个" << std::endl;
}

// 高级门窗检测算法
std::vector<DoorWindow> CloudProcessor::detectAdvancedDoorWindows() {
    std::vector<DoorWindow> doorWindows;

    std::cout << "开始高级门窗检测..." << std::endl;

    // 对每个墙面进行门窗检测
    for (const auto& wall : m_wallPlanes) {
        if (wall.points.size() < 200) continue; // 跳过太小的墙面

        // 创建墙面的2D投影网格
        const int gridSize = 50;
        std::vector<std::vector<int>> grid(gridSize, std::vector<int>(gridSize, 0));

        // 计算墙面边界
        float minX = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float minZ = std::numeric_limits<float>::max();
        float maxZ = std::numeric_limits<float>::lowest();

        for (const auto& point : wall.points) {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minZ = std::min(minZ, point.z);
            maxZ = std::max(maxZ, point.z);
        }

        float rangeX = maxX - minX;
        float rangeZ = maxZ - minZ;

        if (rangeX < 0.5f || rangeZ < 0.5f) continue; // 墙面太小

        // 填充网格
        for (const auto& point : wall.points) {
            int gridX = static_cast<int>((point.x - minX) / rangeX * (gridSize - 1));
            int gridZ = static_cast<int>((point.z - minZ) / rangeZ * (gridSize - 1));

            gridX = std::max(0, std::min(gridSize - 1, gridX));
            gridZ = std::max(0, std::min(gridSize - 1, gridZ));

            grid[gridZ][gridX]++;
        }

        // 检测矩形空洞区域
        for (int z = 5; z < gridSize - 15; z++) {
            for (int x = 3; x < gridSize - 8; x++) {
                // 检查不同尺寸的矩形区域
                std::vector<std::pair<int, int>> sizes = {{6, 12}, {8, 15}, {5, 10}, {4, 8}};

                for (const auto& size : sizes) {
                    int width = size.first;
                    int height = size.second;

                    if (x + width >= gridSize || z + height >= gridSize) continue;

                    // 计算区域内的点密度
                    int totalPoints = 0;
                    int emptyCount = 0;

                    for (int dz = 0; dz < height; dz++) {
                        for (int dx = 0; dx < width; dx++) {
                            int count = grid[z + dz][x + dx];
                            totalPoints += count;
                            if (count == 0) emptyCount++;
                        }
                    }

                    float emptyRatio = static_cast<float>(emptyCount) / (width * height);

                    // 如果空洞比例足够高，认为是门窗开口
                    if (emptyRatio > 0.6f && totalPoints < 50) {
                        DoorWindow opening;

                        // 计算实际坐标
                        opening.center.x = minX + (x + width / 2.0f) / gridSize * rangeX;
                        opening.center.y = 0; // 墙面的Y坐标
                        opening.center.z = minZ + (z + height / 2.0f) / gridSize * rangeZ;

                        opening.width = width * rangeX / gridSize;
                        opening.height = height * rangeZ / gridSize;
                        opening.confidence = emptyRatio;

                        // 根据尺寸和位置判断门窗类型
                        if (opening.height > 1.5f && opening.center.z < 0.5f) {
                            opening.type = DoorWindow::DOOR;
                        } else if (opening.height < 1.5f && opening.center.z > 0.8f) {
                            opening.type = DoorWindow::WINDOW;
                        } else {
                            opening.type = DoorWindow::UNKNOWN;
                        }

                        // 避免重复检测
                        bool isDuplicate = false;
                        for (const auto& existing : doorWindows) {
                            float dist = std::sqrt(
                                std::pow(existing.center.x - opening.center.x, 2) +
                                std::pow(existing.center.z - opening.center.z, 2)
                            );
                            if (dist < 0.5f) {
                                isDuplicate = true;
                                break;
                            }
                        }

                        if (!isDuplicate && opening.confidence > 0.7f) {
                            doorWindows.push_back(opening);
                        }
                    }
                }
            }
        }
    }

    std::cout << "高级门窗检测完成，检测到 " << doorWindows.size() << " 个开口" << std::endl;
    return doorWindows;
}

// 智能颜色编码系统
void CloudProcessor::applyIntelligentColoring() {
    std::cout << "应用智能颜色编码..." << std::endl;

    // 为不同建筑元素着色
    colorWalls(m_wallPlanes);
    colorFloors(m_floorPlanes);
    colorCeilings(m_ceilingPlanes);
    colorDoors(m_doors);
    colorWindows(m_windows);

    std::cout << "颜色编码完成" << std::endl;
}

// 为墙面着色 (灰色) - 改进版
void CloudProcessor::colorWalls(const std::vector<PlaneStruct>& walls) {
    std::cout << "为 " << walls.size() << " 个墙面着色..." << std::endl;
    int coloredPoints = 0;

    for (const auto& wall : walls) {
        for (const auto& point : wall.points) {
            // 在处理后的点云中找到对应点并着色
            for (auto& cloudPoint : m_processedCloud->points) {
                if (std::abs(cloudPoint.x - point.x) < 0.02f &&
                    std::abs(cloudPoint.y - point.y) < 0.02f &&
                    std::abs(cloudPoint.z - point.z) < 0.02f) {
                    // 灰色 (128, 128, 128)
                    cloudPoint.r = 128;
                    cloudPoint.g = 128;
                    cloudPoint.b = 128;
                    coloredPoints++;
                    break;
                }
            }
        }
    }
    std::cout << "墙面着色完成，共着色 " << coloredPoints << " 个点" << std::endl;
}

// 为地面着色 (绿色) - 改进版
void CloudProcessor::colorFloors(const std::vector<PlaneStruct>& floors) {
    std::cout << "为 " << floors.size() << " 个地面着色..." << std::endl;
    int coloredPoints = 0;

    for (const auto& floor : floors) {
        for (const auto& point : floor.points) {
            for (auto& cloudPoint : m_processedCloud->points) {
                if (std::abs(cloudPoint.x - point.x) < 0.02f &&
                    std::abs(cloudPoint.y - point.y) < 0.02f &&
                    std::abs(cloudPoint.z - point.z) < 0.02f) {
                    // 绿色 (0, 255, 0)
                    cloudPoint.r = 0;
                    cloudPoint.g = 255;
                    cloudPoint.b = 0;
                    coloredPoints++;
                    break;
                }
            }
        }
    }
    std::cout << "地面着色完成，共着色 " << coloredPoints << " 个点" << std::endl;
}

// 为屋顶着色 (红色) - 改进版
void CloudProcessor::colorCeilings(const std::vector<PlaneStruct>& ceilings) {
    std::cout << "为 " << ceilings.size() << " 个天花板着色..." << std::endl;
    int coloredPoints = 0;

    for (const auto& ceiling : ceilings) {
        for (const auto& point : ceiling.points) {
            for (auto& cloudPoint : m_processedCloud->points) {
                if (std::abs(cloudPoint.x - point.x) < 0.02f &&
                    std::abs(cloudPoint.y - point.y) < 0.02f &&
                    std::abs(cloudPoint.z - point.z) < 0.02f) {
                    // 红色 (255, 0, 0)
                    cloudPoint.r = 255;
                    cloudPoint.g = 0;
                    cloudPoint.b = 0;
                    coloredPoints++;
                    break;
                }
            }
        }
    }
    std::cout << "天花板着色完成，共着色 " << coloredPoints << " 个点" << std::endl;
}

// 为门着色 (棕色) - 改进版
void CloudProcessor::colorDoors(const std::vector<DoorWindow>& doors) {
    std::cout << "为 " << doors.size() << " 个门着色..." << std::endl;
    int coloredPoints = 0;

    for (const auto& door : doors) {
        // 为门周围的点着色，扩大搜索范围
        for (auto& cloudPoint : m_processedCloud->points) {
            float dist = std::sqrt(
                std::pow(cloudPoint.x - door.center.x, 2) +
                std::pow(cloudPoint.y - door.center.y, 2) +
                std::pow(cloudPoint.z - door.center.z, 2)
            );

            if (dist < std::max(door.width, door.height) / 2.0f + 0.3f) {
                // 棕色 (139, 69, 19)
                cloudPoint.r = 139;
                cloudPoint.g = 69;
                cloudPoint.b = 19;
                coloredPoints++;
            }
        }
    }
    std::cout << "门着色完成，共着色 " << coloredPoints << " 个点" << std::endl;
}

// 为窗着色 (蓝色) - 改进版
void CloudProcessor::colorWindows(const std::vector<DoorWindow>& windows) {
    std::cout << "为 " << windows.size() << " 个窗着色..." << std::endl;
    int coloredPoints = 0;

    for (const auto& window : windows) {
        // 为窗周围的点着色，扩大搜索范围
        for (auto& cloudPoint : m_processedCloud->points) {
            float dist = std::sqrt(
                std::pow(cloudPoint.x - window.center.x, 2) +
                std::pow(cloudPoint.y - window.center.y, 2) +
                std::pow(cloudPoint.z - window.center.z, 2)
            );

            if (dist < std::max(window.width, window.height) / 2.0f + 0.3f) {
                // 蓝色 (0, 0, 255)
                cloudPoint.r = 0;
                cloudPoint.g = 0;
                cloudPoint.b = 255;
                coloredPoints++;
            }
        }
    }
    std::cout << "窗着色完成，共着色 " << coloredPoints << " 个点" << std::endl;
}

// 获取建筑元素统计信息
void CloudProcessor::getBuildingElementStats(int& wallCount, int& doorCount, int& windowCount,
                                           int& floorCount, int& ceilingCount) {
    wallCount = m_wallPlanes.size();
    doorCount = m_doors.size();
    windowCount = m_windows.size();
    floorCount = m_floorPlanes.size();
    ceilingCount = m_ceilingPlanes.size();
}

// 为门窗点着色
void CloudProcessor::colorDoorWindowPoints(const std::vector<DoorWindow>& doorWindows) {
    if (!m_processedCloud || m_processedCloud->empty()) {
        return;
    }

    std::cout << "为门窗点着色..." << std::endl;

    for (const auto& doorWindow : doorWindows) {
        // 定义颜色
        uint8_t r, g, b;
        if (doorWindow.type == DoorWindow::DOOR) {
            r = 255; g = 0; b = 0;    // 红色表示门
        } else if (doorWindow.type == DoorWindow::WINDOW) {
            r = 0; g = 0; b = 255;    // 蓝色表示窗
        } else {
            r = 255; g = 255; b = 0;  // 黄色表示未知类型
        }

        // 为门窗区域附近的点着色
        float searchRadius = std::max(doorWindow.width, doorWindow.height) * 0.6f;

        for (auto& point : m_processedCloud->points) {
            float dx = point.x - doorWindow.center.x;
            float dy = point.y - doorWindow.center.y;
            float dz = point.z - doorWindow.center.z;
            float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (distance < searchRadius) {
                point.r = r;
                point.g = g;
                point.b = b;
            }
        }
    }

    std::cout << "门窗点着色完成" << std::endl;
}
