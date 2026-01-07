#pragma once

#include <string>
#include <memory>
#include <vector>

// PCL头文件
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

#include "cloud_processor.h"

// PCL点云类型定义
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = pcl::PointCloud<PointT>::Ptr;

// 使用CloudProcessor中的PlaneStruct作为平面结构体
using PCLPlane = struct {
    pcl::ModelCoefficients::Ptr coefficients;
    CloudPtr cloud;
};

class CloudVisualizer {
private:
    // PCL点云数据
    CloudPtr m_cloud;
    CloudPtr m_processedCloud;
    
    // PCL可视化器
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
    
    // 分割后的平面点云集合
    std::vector<std::shared_ptr<CloudT>> m_segmentedClouds;
    
    // 平面参数
    float m_distThreshold;
    
    // 处理参数
    bool m_enableDownsampling;
    float m_downsamplingLeafSize;
    bool m_enableDenoising;
    int m_meanK;
    float m_stddevMulThresh;
    
    // 处理函数
    void applyDenoising();
    void applyDownsampling();
    void segmentPlanes();
    void processCloud();
    
    // 从XYZ文件加载到PCL点云
    bool loadFromXYZToPCL(const std::string& filename);
    
    // 从PLY文件加载到PCL点云
    bool loadFromPLYToPCL(const std::string& filename);
    
    // 从我们的点云处理器加载点云数据
    bool loadFromCloudProcessor(const CloudProcessor& processor);
    
    // 使用PCL进行点云降噪
    void denoisePCL(float stddev_mult = 0.8);
    
    // 使用PCL进行点云下采样
    void downsamplePCL(float leaf_size = 0.005f);
    
    // 使用PCL进行平面分割
    void segmentPlanesPCL(float distance_threshold = 0.01,
                          int max_iterations = 1000,
                          int min_plane_size = 100);
    
    // 初始化可视化器
    void initializeViewer();

public:
    // Constructor and destructor
    CloudVisualizer();
    ~CloudVisualizer();
    
    // Set input cloud
    void setInputCloud(const CloudPtr& cloud);
    void setInputCloud(const std::vector<Point3D>& points);
    
    // Set downsampling parameters
    void setDownsampling(bool enable, float leafSize = 0.01f);
    
    // Set denoising parameters
    void setDenoising(bool enable, int meanK = 50, float stddevThresh = 1.0f);
    
    // Visualize the point cloud
    void visualize();
    
    // Check if visualizer was stopped
    bool wasStopped() const;
    
    // Process spinOnce event
    void spinOnce(int time = 100);
    
    // Save point cloud to PLY file
    bool savePointCloudPLY(const std::string& filename);

    // 获取处理后的点云
    CloudPtr getProcessedCloud() const { return m_processedCloud; }
    
    // 获取点的数量
    int getPointCount() const;
    
    // 获取平面数量
    size_t getPlaneCount() const { return m_segmentedClouds.size(); }
    
    // 获取点集合
    std::vector<Point3D> getPoints() const;
    
    // 从文件加载点云
    bool loadCloud(const std::string& filename);
    
    // 从CloudProcessor加载点云
    bool loadCloud(const CloudProcessor& processor);
    
    // 显示原始点云
    void showOriginalCloud();
    
    // 显示处理后的点云
    void showProcessedCloud();
    
    // 显示分割后的平面
    void showSegmentedPlanes();
    
    // 保存处理后的点云
    bool saveProcessedCloud(const std::string& filename);
    
    // 保存为PCD格式
    bool saveToPCD(const std::string& filename);
    
    // Initialize the visualizer
    void init(int argc, char** argv);
    
    // Run the visualizer
    void run();
}; 