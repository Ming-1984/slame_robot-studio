#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/region_growing.h>
#include <Eigen/Dense>

// 点云类型定义
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

// 3D点结构
struct Point3D {
    float x, y, z;
    uint8_t r, g, b;
    
    Point3D() : x(0), y(0), z(0), r(255), g(255), b(255) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_), r(255), g(255), b(255) {}
    Point3D(float x_, float y_, float z_, uint8_t r_, uint8_t g_, uint8_t b_) 
        : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {}
};

// 墙面结构
struct WallPlane {
    Eigen::Vector4f coefficients;  // 平面方程系数 ax + by + cz + d = 0
    Eigen::Vector3f normal;        // 法向量
    Eigen::Vector3f center;        // 中心点
    std::vector<int> pointIndices; // 属于该墙面的点索引
    float area;                    // 墙面面积
    Eigen::Vector3f minBound;      // 边界框最小值
    Eigen::Vector3f maxBound;      // 边界框最大值
    
    WallPlane() : area(0) {}
};

// 开口结构（门窗的基础结构）
struct Opening {
    enum Type { UNKNOWN = 0, DOOR = 1, WINDOW = 2, ARCH = 3, NICHE = 4 };
    
    Type type;
    Eigen::Vector3f center;        // 中心点
    Eigen::Vector3f size;          // 尺寸 (width, height, depth)
    float area;                    // 面积
    float aspectRatio;             // 宽高比
    float bottomHeight;            // 底部离地高度
    float confidence;              // 检测置信度
    int wallIndex;                 // 所属墙面索引
    
    // 边界信息
    std::vector<Eigen::Vector3f> boundaryPoints;  // 边界点
    std::vector<int> interiorPoints;              // 内部点索引
    std::vector<int> boundaryIndices;             // 边界点索引
    
    // 几何特征
    float rectangularity;          // 矩形度 (0-1)
    float symmetry;                // 对称性 (0-1)
    float edgeSharpness;           // 边缘锐度
    
    Opening() : type(UNKNOWN), area(0), aspectRatio(1), bottomHeight(0), 
                confidence(0), wallIndex(-1), rectangularity(0), 
                symmetry(0), edgeSharpness(0) {}
};

// 检测参数结构
struct DetectionParams {
    // 预处理参数
    float voxelSize = 0.01f;           // 体素大小
    int statisticalK = 50;             // 统计滤波K值
    float statisticalStddev = 1.0f;    // 统计滤波标准差
    
    // 墙面检测参数
    float wallDistanceThreshold = 0.02f;    // 墙面距离阈值
    int wallMinPoints = 1000;               // 墙面最小点数
    float wallNormalThreshold = 0.1f;       // 墙面法向量阈值
    
    // 开口检测参数
    float openingMinWidth = 0.6f;           // 开口最小宽度
    float openingMaxWidth = 3.0f;           // 开口最大宽度
    float openingMinHeight = 1.0f;          // 开口最小高度
    float openingMaxHeight = 3.5f;          // 开口最大高度
    float openingMinArea = 0.8f;            // 开口最小面积
    
    // 门窗分类参数
    float doorMinHeight = 1.8f;             // 门最小高度
    float doorMaxHeight = 2.5f;             // 门最大高度
    float windowMinHeight = 0.8f;           // 窗最小高度
    float windowMaxHeight = 2.0f;           // 窗最大高度
    float windowMinBottomHeight = 0.5f;     // 窗最小离地高度
    float windowMaxBottomHeight = 1.5f;     // 窗最大离地高度
    
    // 置信度参数
    float minConfidence = 0.6f;             // 最小置信度阈值
    float rectangularityWeight = 0.3f;      // 矩形度权重
    float symmetryWeight = 0.2f;            // 对称性权重
    float sizeWeight = 0.3f;                // 尺寸权重
    float positionWeight = 0.2f;            // 位置权重
};

// 高级门窗检测器类
class AdvancedDoorWindowDetector {
public:
    AdvancedDoorWindowDetector();
    ~AdvancedDoorWindowDetector();
    
    // 主要接口
    bool loadPointCloud(const std::string& filename);
    bool savePointCloud(const std::string& filename);
    bool detectDoorWindows(const DetectionParams& params = DetectionParams());
    
    // 结果获取
    const std::vector<Opening>& getDetectedOpenings() const { return m_openings; }
    const std::vector<WallPlane>& getWallPlanes() const { return m_walls; }
    PointCloudT::Ptr getColoredPointCloud() const { return m_coloredCloud; }
    
    // 参数设置
    void setDetectionParams(const DetectionParams& params) { m_params = params; }
    const DetectionParams& getDetectionParams() const { return m_params; }
    
    // 结果分析
    void printDetectionReport() const;
    bool exportToReport(const std::string& filename) const;
    
    // 可视化
    void colorizePointCloud();
    void highlightOpenings(const std::vector<int>& openingIndices);
    
private:
    // 核心算法
    bool preprocessPointCloud();
    bool segmentWalls();
    bool detectOpeningsInWalls();
    bool classifyOpenings();
    bool postProcessResults();
    
    // 墙面分割相关
    std::vector<WallPlane> extractPlanesRANSAC();
    void refineWallPlanes();
    void computeWallBoundaries();
    
    // 开口检测相关
    std::vector<Opening> detectOpeningsInWall(const WallPlane& wall);
    Opening extractOpeningRegion(const std::vector<std::vector<int>>& densityGrid,
                                std::vector<std::vector<bool>>& visited,
                                int startX, int startZ, int gridResolution,
                                float minX, float minZ, float stepX, float stepZ);
    bool isValidOpening(const Opening& opening) const;
    void computeOpeningFeatures(Opening& opening);
    
    // 分类相关
    Opening::Type classifyOpening(const Opening& opening) const;
    float computeConfidence(const Opening& opening) const;
    
    // 几何计算
    float computeRectangularity(const std::vector<Eigen::Vector3f>& points) const;
    float computeSymmetry(const std::vector<Eigen::Vector3f>& points) const;
    float computeEdgeSharpness(const std::vector<int>& boundaryIndices) const;
    
    // 工具函数
    Eigen::Vector3f projectPointToPlane(const Eigen::Vector3f& point, 
                                       const Eigen::Vector4f& plane) const;
    std::vector<Eigen::Vector3f> project3DTo2D(const std::vector<int>& indices, 
                                              const WallPlane& wall) const;
    
private:
    // 数据成员
    PointCloudT::Ptr m_originalCloud;      // 原始点云
    PointCloudT::Ptr m_processedCloud;     // 处理后点云
    PointCloudT::Ptr m_coloredCloud;       // 着色后点云
    PointCloudNT::Ptr m_normalCloud;       // 法向量点云
    
    std::vector<WallPlane> m_walls;        // 检测到的墙面
    std::vector<Opening> m_openings;       // 检测到的开口
    
    DetectionParams m_params;              // 检测参数
    
    // PCL对象
    pcl::VoxelGrid<PointT> m_voxelFilter;
    pcl::StatisticalOutlierRemoval<PointT> m_statisticalFilter;
    pcl::NormalEstimation<PointT, PointNT> m_normalEstimation;
    pcl::SACSegmentation<PointT> m_segmentation;
    pcl::RegionGrowing<PointT, PointNT> m_regionGrowing;
    pcl::search::KdTree<PointT>::Ptr m_kdtree;
    
    // 状态标志
    bool m_isLoaded;
    bool m_isProcessed;
    bool m_isDetected;
};
