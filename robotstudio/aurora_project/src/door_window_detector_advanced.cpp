#include "door_window_detector_advanced.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

AdvancedDoorWindowDetector::AdvancedDoorWindowDetector() 
    : m_originalCloud(new PointCloudT)
    , m_processedCloud(new PointCloudT)
    , m_coloredCloud(new PointCloudT)
    , m_normalCloud(new PointCloudNT)
    , m_kdtree(new pcl::search::KdTree<PointT>)
    , m_isLoaded(false)
    , m_isProcessed(false)
    , m_isDetected(false)
{
    // 初始化PCL对象
    m_voxelFilter.setLeafSize(m_params.voxelSize, m_params.voxelSize, m_params.voxelSize);
    m_statisticalFilter.setMeanK(m_params.statisticalK);
    m_statisticalFilter.setStddevMulThresh(m_params.statisticalStddev);
    
    m_normalEstimation.setSearchMethod(m_kdtree);
    m_normalEstimation.setKSearch(20);
    
    m_segmentation.setOptimizeCoefficients(true);
    m_segmentation.setModelType(pcl::SACMODEL_PLANE);
    m_segmentation.setMethodType(pcl::SAC_RANSAC);
    m_segmentation.setMaxIterations(1000);
    m_segmentation.setDistanceThreshold(m_params.wallDistanceThreshold);
    
    m_regionGrowing.setMinClusterSize(50);
    m_regionGrowing.setMaxClusterSize(1000000);
    m_regionGrowing.setSearchMethod(m_kdtree);
    m_regionGrowing.setNumberOfNeighbours(30);
    m_regionGrowing.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    m_regionGrowing.setCurvatureThreshold(1.0);
}

AdvancedDoorWindowDetector::~AdvancedDoorWindowDetector() {
}

bool AdvancedDoorWindowDetector::loadPointCloud(const std::string& filename) {
    std::cout << "Loading point cloud from: " << filename << std::endl;
    
    if (pcl::io::loadPLYFile<PointT>(filename, *m_originalCloud) == -1) {
        std::cerr << "Error: Could not load point cloud file " << filename << std::endl;
        return false;
    }
    
    std::cout << "Loaded " << m_originalCloud->size() << " points" << std::endl;
    m_isLoaded = true;
    m_isProcessed = false;
    m_isDetected = false;
    
    return true;
}

bool AdvancedDoorWindowDetector::savePointCloud(const std::string& filename) {
    if (!m_isLoaded) {
        std::cerr << "Error: No point cloud loaded" << std::endl;
        return false;
    }
    
    PointCloudT::Ptr cloudToSave = m_isDetected ? m_coloredCloud : m_processedCloud;
    if (!cloudToSave || cloudToSave->empty()) {
        cloudToSave = m_originalCloud;
    }
    
    if (pcl::io::savePLYFileBinary(filename, *cloudToSave) == -1) {
        std::cerr << "Error: Could not save point cloud file " << filename << std::endl;
        return false;
    }
    
    std::cout << "Saved " << cloudToSave->size() << " points to " << filename << std::endl;
    return true;
}

bool AdvancedDoorWindowDetector::detectDoorWindows(const DetectionParams& params) {
    if (!m_isLoaded) {
        std::cerr << "Error: No point cloud loaded" << std::endl;
        return false;
    }
    
    m_params = params;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    std::cout << "Starting advanced door window detection..." << std::endl;
    
    // 步骤1: 预处理
    std::cout << "Step 1: Preprocessing point cloud..." << std::endl;
    if (!preprocessPointCloud()) {
        std::cerr << "Error: Preprocessing failed" << std::endl;
        return false;
    }
    
    // 步骤2: 墙面分割
    std::cout << "Step 2: Segmenting walls..." << std::endl;
    if (!segmentWalls()) {
        std::cerr << "Error: Wall segmentation failed" << std::endl;
        return false;
    }
    
    // 步骤3: 开口检测
    std::cout << "Step 3: Detecting openings in walls..." << std::endl;
    if (!detectOpeningsInWalls()) {
        std::cerr << "Error: Opening detection failed" << std::endl;
        return false;
    }
    
    // 步骤4: 开口分类
    std::cout << "Step 4: Classifying openings..." << std::endl;
    if (!classifyOpenings()) {
        std::cerr << "Error: Opening classification failed" << std::endl;
        return false;
    }
    
    // 步骤5: 后处理
    std::cout << "Step 5: Post-processing results..." << std::endl;
    if (!postProcessResults()) {
        std::cerr << "Error: Post-processing failed" << std::endl;
        return false;
    }
    
    // 步骤6: 点云着色
    std::cout << "Step 6: Colorizing point cloud..." << std::endl;
    colorizePointCloud();
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Detection completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Found " << m_walls.size() << " walls and " << m_openings.size() << " openings" << std::endl;
    
    m_isDetected = true;
    return true;
}

bool AdvancedDoorWindowDetector::preprocessPointCloud() {
    if (!m_originalCloud || m_originalCloud->empty()) {
        return false;
    }
    
    // 复制原始点云
    *m_processedCloud = *m_originalCloud;
    
    std::cout << "  Original points: " << m_processedCloud->size() << std::endl;
    
    // 体素滤波降采样
    m_voxelFilter.setLeafSize(m_params.voxelSize, m_params.voxelSize, m_params.voxelSize);
    m_voxelFilter.setInputCloud(m_processedCloud);
    m_voxelFilter.filter(*m_processedCloud);
    
    std::cout << "  After voxel filtering: " << m_processedCloud->size() << std::endl;
    
    // 统计滤波去噪
    m_statisticalFilter.setMeanK(m_params.statisticalK);
    m_statisticalFilter.setStddevMulThresh(m_params.statisticalStddev);
    m_statisticalFilter.setInputCloud(m_processedCloud);
    m_statisticalFilter.filter(*m_processedCloud);
    
    std::cout << "  After statistical filtering: " << m_processedCloud->size() << std::endl;
    
    // 计算法向量
    m_normalEstimation.setInputCloud(m_processedCloud);
    m_normalEstimation.setSearchMethod(m_kdtree);
    m_normalEstimation.compute(*m_normalCloud);
    
    std::cout << "  Computed normals for " << m_normalCloud->size() << " points" << std::endl;
    
    m_isProcessed = true;
    return true;
}

bool AdvancedDoorWindowDetector::segmentWalls() {
    if (!m_isProcessed) {
        return false;
    }
    
    m_walls.clear();
    
    // 使用RANSAC提取平面
    auto planes = extractPlanesRANSAC();
    
    // 过滤出垂直墙面
    for (const auto& plane : planes) {
        // 检查是否为垂直平面（法向量Z分量接近0）
        if (std::abs(plane.normal.z()) < m_params.wallNormalThreshold && 
            plane.pointIndices.size() >= m_params.wallMinPoints) {
            m_walls.push_back(plane);
        }
    }
    
    std::cout << "  Extracted " << m_walls.size() << " wall planes" << std::endl;
    
    // 细化墙面平面
    refineWallPlanes();
    
    // 计算墙面边界
    computeWallBoundaries();
    
    return !m_walls.empty();
}

std::vector<WallPlane> AdvancedDoorWindowDetector::extractPlanesRANSAC() {
    std::vector<WallPlane> planes;
    
    PointCloudT::Ptr remainingCloud(new PointCloudT(*m_processedCloud));
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    int planeCount = 0;
    const int maxPlanes = 10; // 最多提取10个平面
    
    while (remainingCloud->size() > m_params.wallMinPoints && planeCount < maxPlanes) {
        m_segmentation.setInputCloud(remainingCloud);
        m_segmentation.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < m_params.wallMinPoints) {
            break;
        }
        
        // 创建墙面平面
        WallPlane wall;
        wall.coefficients = Eigen::Vector4f(coefficients->values[0], coefficients->values[1], 
                                           coefficients->values[2], coefficients->values[3]);
        wall.normal = Eigen::Vector3f(coefficients->values[0], coefficients->values[1], 
                                     coefficients->values[2]).normalized();
        
        // 计算中心点和边界
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*remainingCloud, inliers->indices, centroid);
        wall.center = centroid.head<3>();
        
        // 存储点索引（需要转换为原始点云的索引）
        wall.pointIndices = inliers->indices;
        
        planes.push_back(wall);
        
        // 移除已分割的点
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remainingCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*remainingCloud);
        
        planeCount++;
    }
    
    return planes;
}

void AdvancedDoorWindowDetector::refineWallPlanes() {
    for (auto& wall : m_walls) {
        // 使用区域增长细化墙面分割
        std::vector<pcl::PointIndices> clusters;

        // 创建仅包含当前墙面点的点云
        PointCloudT::Ptr wallCloud(new PointCloudT);
        pcl::copyPointCloud(*m_processedCloud, wall.pointIndices, *wallCloud);

        // 使用区域增长进一步细化
        m_regionGrowing.setInputCloud(wallCloud);

        PointCloudNT::Ptr wallNormals(new PointCloudNT);
        pcl::copyPointCloud(*m_normalCloud, wall.pointIndices, *wallNormals);
        m_regionGrowing.setInputNormals(wallNormals);

        m_regionGrowing.extract(clusters);

        // 选择最大的聚类作为主墙面
        if (!clusters.empty()) {
            auto maxCluster = std::max_element(clusters.begin(), clusters.end(),
                [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                    return a.indices.size() < b.indices.size();
                });

            // 更新墙面点索引
            std::vector<int> refinedIndices;
            for (int idx : maxCluster->indices) {
                refinedIndices.push_back(wall.pointIndices[idx]);
            }
            wall.pointIndices = refinedIndices;
        }
    }
}

void AdvancedDoorWindowDetector::computeWallBoundaries() {
    for (auto& wall : m_walls) {
        if (wall.pointIndices.empty()) continue;

        // 计算边界框
        PointT minPt, maxPt;
        pcl::getMinMax3D(*m_processedCloud, wall.pointIndices, minPt, maxPt);

        wall.minBound = Eigen::Vector3f(minPt.x, minPt.y, minPt.z);
        wall.maxBound = Eigen::Vector3f(maxPt.x, maxPt.y, maxPt.z);

        // 计算面积（简化为边界框面积）
        Eigen::Vector3f size = wall.maxBound - wall.minBound;
        wall.area = size.x() * size.z(); // 假设Y是深度方向

        // 重新计算中心点
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*m_processedCloud, wall.pointIndices, centroid);
        wall.center = centroid.head<3>();
    }
}

bool AdvancedDoorWindowDetector::detectOpeningsInWalls() {
    m_openings.clear();

    for (size_t i = 0; i < m_walls.size(); ++i) {
        std::cout << "  Processing wall " << (i + 1) << "/" << m_walls.size() << std::endl;

        auto wallOpenings = detectOpeningsInWall(m_walls[i]);

        // 设置墙面索引
        for (auto& opening : wallOpenings) {
            opening.wallIndex = static_cast<int>(i);
        }

        m_openings.insert(m_openings.end(), wallOpenings.begin(), wallOpenings.end());
    }

    std::cout << "  Found " << m_openings.size() << " potential openings" << std::endl;
    return true;
}

std::vector<Opening> AdvancedDoorWindowDetector::detectOpeningsInWall(const WallPlane& wall) {
    std::vector<Opening> openings;

    if (wall.pointIndices.size() < 100) {
        return openings;
    }

    // 将3D点投影到2D平面
    auto projected2D = project3DTo2D(wall.pointIndices, wall);

    if (projected2D.size() < 100) {
        return openings;
    }

    // 创建2D网格进行密度分析
    const int gridResolution = 100;
    std::vector<std::vector<int>> densityGrid(gridResolution, std::vector<int>(gridResolution, 0));

    // 计算2D边界
    float minX = FLT_MAX, maxX = -FLT_MAX;
    float minZ = FLT_MAX, maxZ = -FLT_MAX;

    for (const auto& point : projected2D) {
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minZ = std::min(minZ, point.z());
        maxZ = std::max(maxZ, point.z());
    }

    float stepX = (maxX - minX) / gridResolution;
    float stepZ = (maxZ - minZ) / gridResolution;

    if (stepX <= 0 || stepZ <= 0) {
        return openings;
    }

    // 填充密度网格
    for (const auto& point : projected2D) {
        int gridX = std::min(gridResolution - 1, static_cast<int>((point.x() - minX) / stepX));
        int gridZ = std::min(gridResolution - 1, static_cast<int>((point.z() - minZ) / stepZ));
        densityGrid[gridZ][gridX]++;
    }

    // 检测低密度区域（潜在开口）
    std::vector<std::vector<bool>> visited(gridResolution, std::vector<bool>(gridResolution, false));

    for (int z = 0; z < gridResolution; ++z) {
        for (int x = 0; x < gridResolution; ++x) {
            if (!visited[z][x] && densityGrid[z][x] == 0) {
                // 使用连通组件分析找到开口区域
                Opening opening = extractOpeningRegion(densityGrid, visited, x, z,
                                                     gridResolution, minX, minZ, stepX, stepZ);

                if (isValidOpening(opening)) {
                    // 计算几何特征
                    computeOpeningFeatures(opening);
                    openings.push_back(opening);
                }
            }
        }
    }

    return openings;
}
