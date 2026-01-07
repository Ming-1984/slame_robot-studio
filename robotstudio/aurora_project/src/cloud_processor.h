#pragma once

#include <vector>
#include <string>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 3D点结构体
struct Point3D {
    float x;
    float y;
    float z;
    float r, g, b;  // 颜色信息
};

// 平面结构体
struct PlaneStruct {
    float a, b, c, d;  // ax + by + cz + d = 0
    std::vector<Point3D> points;
};

// 3D鐐圭粨鏋勪綋
struct Plane {
    float a, b, c, d;  // ax + by + cz + d = 0
    std::vector<Point3D> points;
    
    Plane();
    Plane(float _a, float _b, float _c, float _d);
    
    // 璁＄畻鐐瑰埌骞抽潰鐨勮窛绂?
    float distanceToPoint(const Point3D& p) const;
    
    // 妫€鏌ョ偣鏄惁鍦ㄥ钩闈笂
    bool containsPoint(const Point3D& p, float threshold = 0.05f) const;
};

// 杞粨鐐?
struct ContourPoint {
    float x, y;  // 2D 骞抽潰鍧愭爣
    
    ContourPoint();
    ContourPoint(float _x, float _y);
    
    bool operator==(const ContourPoint& other) const;
    bool operator<(const ContourPoint& other) const;
};

// 墙体线段结构体
struct WallSegment {
    float x1, y1, z1;
    float x2, y2, z2;
    float thickness;
    float height;
};

// 门窗结构体
struct DoorWindow {
    enum Type { DOOR, WINDOW, UNKNOWN };

    Type type;
    Point3D center;          // 中心点
    float width;             // 宽度
    float height;            // 高度
    float bottomHeight;      // 底部离地高度
    float confidence;        // 检测置信度
    std::vector<Point3D> boundaryPoints;  // 边界点

    DoorWindow() : type(UNKNOWN), width(0), height(0), bottomHeight(0), confidence(0) {}
};

class CloudProcessor {
private:
    // 内部点云数据
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> m_cloud;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> m_processedCloud;
    
    std::vector<Point3D> m_points;
    std::vector<PlaneStruct> m_planes;
    std::vector<WallSegment> m_walls;
    std::vector<DoorWindow> m_doorWindows;

    // 建筑元素分类存储
    std::vector<PlaneStruct> m_wallPlanes;      // 墙面
    std::vector<PlaneStruct> m_floorPlanes;     // 地面
    std::vector<PlaneStruct> m_ceilingPlanes;   // 屋顶/天花板
    std::vector<DoorWindow> m_doors;            // 门
    std::vector<DoorWindow> m_windows;          // 窗
    
    // 读取STCM文件内的点云数据
    bool readSTCM(const std::string& filename, std::vector<Point3D>& points);
    
    // 平面参数估计 (使用RANSAC算法)
    std::vector<Plane> estimatePlanes(float distanceThreshold = 0.02f);
    
    // 墙体提取
    std::vector<WallSegment> extractWalls();

    // 门窗检测
    std::vector<DoorWindow> detectDoorWindows(const std::vector<PlaneStruct>& walls);

    // 在单个墙面上检测门窗开口
    std::vector<DoorWindow> detectOpeningsInWall(const PlaneStruct& wall);

    // 扩展矩形开口区域
    DoorWindow expandRectangularOpening(const std::vector<std::vector<int>>& grid,
                                       int startX, int startZ, int gridWidth, int gridHeight);

public:
    CloudProcessor();
    ~CloudProcessor();
    
    // 从XYZ文件加载点云
    bool loadFromXYZ(const std::string& filename);
    
    // 从PLY文件加载点云
    bool loadFromPLY(const std::string& filename);
    
    // 从STCM文件加载
    bool loadFromSTCM(const std::string& filename);
    
    // 从点集合加载
    bool loadFromPoints(const std::vector<Point3D>& points);
    
    // 降噪处理
    void denoise(float outlierThreshold = 1.5f);
    
    // 处理点云（降采样、去噪等）
    bool processCloud(float leafSize = 0.01, int meanK = 50, float stddevMulThresh = 1.0);
    
    // 处理点云并提取特征
    bool process();
    
    // 提取平面
    std::vector<PlaneStruct> extractPlanes(float distanceThreshold = 0.02);

    // 检测门窗并标记颜色
    std::vector<DoorWindow> detectAndMarkDoorWindows();

    // 为门窗点着色
    void colorDoorWindowPoints(const std::vector<DoorWindow>& doorWindows);

    // 完整建筑元素检测和分类
    bool detectBuildingElements();

    // 分类平面为墙面、地面、屋顶
    void classifyPlanes(const std::vector<PlaneStruct>& planes);

    // 高级门窗检测算法
    std::vector<DoorWindow> detectAdvancedDoorWindows();

    // 智能颜色编码系统
    void applyIntelligentColoring();

    // 为不同建筑元素着色
    void colorWalls(const std::vector<PlaneStruct>& walls);
    void colorFloors(const std::vector<PlaneStruct>& floors);
    void colorCeilings(const std::vector<PlaneStruct>& ceilings);
    void colorDoors(const std::vector<DoorWindow>& doors);
    void colorWindows(const std::vector<DoorWindow>& windows);

    // 获取建筑元素统计信息
    void getBuildingElementStats(int& wallCount, int& doorCount, int& windowCount,
                                int& floorCount, int& ceilingCount);
    
    // 保存到PLY文件
    bool saveToPLY(const std::string& filename);
    
    // 获取点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> getCloud() const { return m_cloud; }
    
    // 获取处理后的点云
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> getProcessedCloud() const { return m_processedCloud; }
    
    // 获取点云数量
    size_t getPointCount() const;
    
    // 获取平面数量
    size_t getPlaneCount() const;
    
    // 获取墙体线段数量
    size_t getWallCount() const;
    
    // 获取原始点集合
    const std::vector<Point3D>& getPoints() const { return m_points; }
    
    // 获取平面
    const std::vector<PlaneStruct>& getPlanes() const { return m_planes; }
}; 
