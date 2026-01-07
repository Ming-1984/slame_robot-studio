#include "cloud_visualizer.h"
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <iostream>

CloudVisualizer::CloudVisualizer() {
    // 初始化点云
    m_cloud.reset(new CloudT);
    m_processedCloud.reset(new CloudT);
    
    // 初始化可视化器
    m_viewer.reset(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    m_viewer->setBackgroundColor(0.05, 0.05, 0.05);
    m_viewer->addCoordinateSystem(0.5);
    m_viewer->initCameraParameters();
    
    // 设置默认参数
    m_enableDownsampling = true;
    m_downsamplingLeafSize = 0.01f;
    m_enableDenoising = true;
    m_meanK = 25; // 降低到25可能更合适
    m_stddevMulThresh = 1.0f;
}

CloudVisualizer::~CloudVisualizer() {
    if (m_viewer) {
        m_viewer->close();
    }
}

void CloudVisualizer::setInputCloud(const CloudPtr& cloud) {
    m_cloud = cloud;
    m_processedCloud = cloud;
    
    std::cout << "设置点云: " << cloud->points.size() << " 个点" << std::endl;
    
    // 更新显示
    processCloud();
}

void CloudVisualizer::setInputCloud(const std::vector<Point3D>& points) {
    // 创建一个新的PCL点云
    CloudPtr cloud(new CloudT);
    
    // 填充点云数据
    for (const auto& p : points) {
        PointT point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = static_cast<uint8_t>(p.r * 255);
        point.g = static_cast<uint8_t>(p.g * 255);
        point.b = static_cast<uint8_t>(p.b * 255);
        cloud->points.push_back(point);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    
    // 设置点云
    setInputCloud(cloud);
}

void CloudVisualizer::setDownsampling(bool enable, float leafSize) {
    m_enableDownsampling = enable;
    m_downsamplingLeafSize = leafSize;
    
    // 如果已经有点云，更新处理
    if (m_cloud && !m_cloud->empty()) {
        processCloud();
    }
}

void CloudVisualizer::setDenoising(bool enable, int meanK, float stddevThresh) {
    m_enableDenoising = enable;
    m_meanK = meanK;
    m_stddevMulThresh = stddevThresh;
    
    // 如果已经有点云，更新处理
    if (m_cloud && !m_cloud->empty()) {
        processCloud();
    }
}

void CloudVisualizer::processCloud() {
    if (!m_cloud || m_cloud->empty()) {
        std::cerr << "点云为空，无法处理" << std::endl;
        return;
    }
    
    // 复制原始点云作为处理起点
    CloudPtr processedCloud(new CloudT(*m_cloud));
    
    // 应用去噪处理
    if (m_enableDenoising) {
        applyDenoising();
    }
    
    // 应用下采样
    if (m_enableDownsampling) {
        applyDownsampling();
    }
    
    // 分割平面（可选）
    // segmentPlanes();
    
    std::cout << "处理后的点云: " << m_processedCloud->points.size() << " 个点" << std::endl;
}

void CloudVisualizer::applyDenoising() {
    if (!m_cloud || m_cloud->empty()) return;
    
    // 使用统计离群点去除
    CloudPtr filteredCloud(new CloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(m_cloud);
    sor.setMeanK(m_meanK);
    sor.setStddevMulThresh(m_stddevMulThresh);
    sor.filter(*filteredCloud);
    
    // 检查是否过度过滤
    if (filteredCloud->points.size() < m_cloud->points.size() * 0.5) {
        std::cout << "警告: 去噪过滤了超过50%的点，使用原始点云" << std::endl;
        return;
    }
    
    m_processedCloud = filteredCloud;
}

void CloudVisualizer::applyDownsampling() {
    if (!m_processedCloud || m_processedCloud->empty()) return;
    
    // 使用体素网格下采样
    CloudPtr downsampledCloud(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(m_processedCloud);
    vg.setLeafSize(m_downsamplingLeafSize, m_downsamplingLeafSize, m_downsamplingLeafSize);
    vg.filter(*downsampledCloud);
    
    // 检查是否过度下采样
    if (downsampledCloud->points.size() < m_processedCloud->points.size() * 0.3) {
        std::cout << "警告: 下采样过滤了超过70%的点，使用去噪后的点云" << std::endl;
        return;
    }
    
    m_processedCloud = downsampledCloud;
}

void CloudVisualizer::segmentPlanes() {
    if (!m_processedCloud || m_processedCloud->empty()) return;
    
    // 清除之前的分割结果
    m_segmentedClouds.clear();
    
    // 使用RANSAC分割平面
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.02);
    
    // 复制点云进行分割
    CloudPtr workingCloud(new CloudT(*m_processedCloud));
    
    // 最多检测5个平面
    int maxPlanes = 5;
    int planeCount = 0;
    
    while (planeCount < maxPlanes && workingCloud->points.size() > 100) {
        // 分割平面
        seg.setInputCloud(workingCloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty()) {
            std::cout << "无法找到更多平面" << std::endl;
            break;
        }
        
        // 提取平面点云
        CloudPtr planeCloud(new CloudT);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(workingCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*planeCloud);
        
        std::cout << "平面 " << planeCount << " 包含 " << planeCloud->points.size() << " 个点" << std::endl;
        
        // 添加到分割结果
        m_segmentedClouds.push_back(planeCloud);
        
        // 从工作点云中移除当前平面点
        extract.setNegative(true);
        extract.filter(*workingCloud);
        
        planeCount++;
    }
}

void CloudVisualizer::visualize() {
    if (!m_processedCloud || m_processedCloud->empty()) {
        std::cerr << "处理后的点云为空，无法显示" << std::endl;
        return;
    }
    
    // 清除之前的可视化
    m_viewer->removeAllPointClouds();
    m_viewer->removeAllShapes();
    
    // 添加处理后的点云
    m_viewer->addPointCloud<PointT>(m_processedCloud, "cloud");
    m_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    
    // 如果有分割结果，显示各个平面
    for (size_t i = 0; i < m_segmentedClouds.size(); i++) {
        std::string id = "plane_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerRandom<PointT> color(m_segmentedClouds[i]);
        m_viewer->addPointCloud<PointT>(m_segmentedClouds[i], color, id);
        m_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    }
    
    // 刷新显示
    m_viewer->spinOnce();
}

bool CloudVisualizer::wasStopped() const {
    return m_viewer->wasStopped();
}

void CloudVisualizer::spinOnce(int time) {
    m_viewer->spinOnce(time);
}

bool CloudVisualizer::savePointCloudPLY(const std::string& filename) {
    if (!m_processedCloud || m_processedCloud->empty()) {
        std::cerr << "处理后的点云为空，无法保存" << std::endl;
        return false;
    }
    
    pcl::PLYWriter writer;
    int result = writer.write(filename, *m_processedCloud);
    
    if (result == 0) {
        std::cout << "成功保存点云到: " << filename << std::endl;
        return true;
    } else {
        std::cerr << "保存点云失败: " << filename << std::endl;
        return false;
    }
}

int CloudVisualizer::getPointCount() const {
    if (m_processedCloud) {
        return m_processedCloud->points.size();
    }
    return 0;
}

std::vector<Point3D> CloudVisualizer::getPoints() const {
    std::vector<Point3D> points;
    
    if (!m_processedCloud || m_processedCloud->empty()) {
        return points;
    }
    
    points.reserve(m_processedCloud->points.size());
    for (const auto& p : m_processedCloud->points) {
        Point3D point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = p.r / 255.0f;
        point.g = p.g / 255.0f;
        point.b = p.b / 255.0f;
        points.push_back(point);
    }
    
    return points;
} 