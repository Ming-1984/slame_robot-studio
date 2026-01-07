#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudNormalT;

// 自定义 deg2rad 函数，以防 PCL 版本不提供
inline float custom_deg2rad(float degrees) {
    return degrees * M_PI / 180.0f;
}

// 自定义密度过滤器
void densityFilter(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out, 
                  float radius, int min_neighbors) {
    std::cout << "执行密度过滤..." << std::endl;
    
    // 创建KD树
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_in);
    
    // 存储每个点的邻居数量
    std::vector<int> point_density(cloud_in->size());
    
    // 计算每个点的密度
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cloud_in->size()); ++i) {
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree.radiusSearch(cloud_in->points[i], radius, indices, distances);
        point_density[i] = indices.size();
    }
    
    // 计算密度统计信息
    std::vector<int> sorted_density = point_density;
    std::sort(sorted_density.begin(), sorted_density.end());
    
    // 计算四分位数
    int q1_idx = sorted_density.size() * 0.25;
    int q3_idx = sorted_density.size() * 0.75;
    int q1 = sorted_density[q1_idx];
    int q3 = sorted_density[q3_idx];
    int iqr = q3 - q1;
    
    // 自适应密度阈值，但设置较低的阈值以保留更多点
    int adaptive_min_neighbors = std::max(min_neighbors, q1 - (int)(0.5 * iqr));
    
    std::cout << "密度统计: Q1=" << q1 << ", Q3=" << q3 << ", IQR=" << iqr << std::endl;
    std::cout << "自适应密度阈值: " << adaptive_min_neighbors << std::endl;
    
    // 根据密度过滤点
    cloud_out->clear();
    cloud_out->reserve(cloud_in->size());
    
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        if (point_density[i] >= adaptive_min_neighbors) {
            cloud_out->push_back(cloud_in->points[i]);
        }
    }
    
    std::cout << "密度过滤后保留了 " << cloud_out->size() << " 个点 (原始: " << cloud_in->size() << ")" << std::endl;
}

// 移除远离中心的异常点
void removeDistantOutliers(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out, 
                          float distance_threshold_factor = 2.0) {
    std::cout << "移除远离中心的异常点..." << std::endl;
    
    if (cloud_in->empty()) {
        std::cout << "警告: 输入点云为空!" << std::endl;
        return;
    }
    
    // 计算点云重心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_in, centroid);
    std::cout << "点云重心: [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "]" << std::endl;
    
    // 计算到重心的距离
    std::vector<float> distances(cloud_in->size());
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        const auto& point = cloud_in->points[i];
        float dx = point.x - centroid[0];
        float dy = point.y - centroid[1];
        float dz = point.z - centroid[2];
        distances[i] = std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    // 计算距离统计信息
    std::vector<float> sorted_distances = distances;
    std::sort(sorted_distances.begin(), sorted_distances.end());
    
    // 使用四分位数范围来确定异常值
    float q1 = sorted_distances[sorted_distances.size() * 0.25];
    float q3 = sorted_distances[sorted_distances.size() * 0.75];
    float iqr = q3 - q1;
    float threshold = q3 + (distance_threshold_factor * iqr);
    
    std::cout << "距离统计: Q1=" << q1 << ", Q3=" << q3 << ", IQR=" << iqr << std::endl;
    std::cout << "距离阈值: " << threshold << std::endl;
    
    // 过滤远离重心的点
    cloud_out->clear();
    cloud_out->reserve(cloud_in->size());
    
    int removed_points = 0;
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        if (distances[i] <= threshold) {
            cloud_out->push_back(cloud_in->points[i]);
        } else {
            removed_points++;
        }
    }
    
    std::cout << "移除了 " << removed_points << " 个远离点 (原始: " << cloud_in->size() 
              << ", 保留: " << cloud_out->size() << ")" << std::endl;
}

// 提取并保留地面和天花板
void extractFloorAndCeiling(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out,
                           float angle_threshold = 10.0, float distance_threshold = 0.1,
                           bool remove_instead_of_extract = false) {
    std::cout << "提取地面和天花板..." << std::endl;
    
    // 计算法线
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    
    ne.setInputCloud(cloud_in);
    ne.setSearchMethod(tree);
    ne.setKSearch(15);  // 法线估计的K近邻点数
    ne.compute(*normals);
    
    // 使用RANSAC寻找水平平面(地面和天花板)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distance_threshold);
    
    // 设置轴约束(寻找与Y轴平行的平面)
    Eigen::Vector3f axis(0.0, 1.0, 0.0);  // Y轴作为法向量方向
    seg.setAxis(axis);
    // 使用 pcl::deg2rad 或自定义函数，取决于 PCL 版本
    try {
        seg.setEpsAngle(pcl::deg2rad(angle_threshold));
    } catch (...) {
        seg.setEpsAngle(custom_deg2rad(angle_threshold));
    }
    
    // 准备保存水平平面点或非水平平面点
    pcl::PointIndices::Ptr horizontal_indices(new pcl::PointIndices);
    
    // 分别寻找地面和天花板
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() > 0) {
        std::cout << "找到第一个水平平面(地面), 包含 " << inliers->indices.size() << " 个点" << std::endl;
        // 将地面点添加到水平点索引中
        horizontal_indices->indices.insert(
            horizontal_indices->indices.end(),
            inliers->indices.begin(),
            inliers->indices.end()
        );
        
        // 创建临时点云，移除地面点后再寻找天花板
        PointCloudT::Ptr temp_cloud(new PointCloudT);
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true);  // 提取非地面点
        extract.filter(*temp_cloud);
        
        // 在剩余点云中寻找天花板
        seg.setInputCloud(temp_cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() > 0) {
            std::cout << "找到第二个水平平面(天花板), 包含 " << inliers->indices.size() << " 个点" << std::endl;
            
            // 将天花板点的索引转换为原始点云的索引
            std::vector<bool> is_ground_point(cloud_in->size(), false);
            for (const auto& idx : horizontal_indices->indices) {
                is_ground_point[idx] = true;
            }
            
            // 计算天花板点在原始点云中的索引
            for (const auto& temp_idx : inliers->indices) {
                int orig_idx = 0;
                int temp_count = 0;
                
                // 找到对应的原始索引
                for (size_t i = 0; i < cloud_in->size(); ++i) {
                    if (!is_ground_point[i]) {
                        if (temp_count == temp_idx) {
                            orig_idx = i;
                            break;
                        }
                        temp_count++;
                    }
                }
                
                horizontal_indices->indices.push_back(orig_idx);
            }
        }
    }
    
    // 根据模式提取点云(保留水平平面点或移除水平平面点)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(horizontal_indices);
    extract.setNegative(remove_instead_of_extract);  // 如果remove_instead_of_extract为true，则移除水平平面点
    extract.filter(*cloud_out);
    
    std::cout << "水平平面处理后保留了 " << cloud_out->size() << " 个点 (原始: " << cloud_in->size() << ")" << std::endl;
}

// 高度过滤器
void heightFilter(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out,
                 float min_pct, float max_pct) {
    std::cout << "执行高度过滤..." << std::endl;
    
    // 找到点云的高度范围 (Y轴)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
    
    float min_y = min_pt.y;
    float max_y = max_pt.y;
    float height_range = max_y - min_y;
    
    // 计算高度阈值
    float min_threshold = min_y + min_pct * height_range;
    float max_threshold = min_y + max_pct * height_range;
    
    std::cout << "高度范围: " << min_y << " 到 " << max_y << std::endl;
    std::cout << "高度阈值: " << min_threshold << " 到 " << max_threshold << std::endl;
    
    // 过滤点
    cloud_out->clear();
    cloud_out->reserve(cloud_in->size());
    
    for (const auto& point : cloud_in->points) {
        if (point.y >= min_threshold && point.y <= max_threshold) {
            cloud_out->push_back(point);
        }
    }
    
    std::cout << "高度过滤后保留了 " << cloud_out->size() << " 个点 (原始: " << cloud_in->size() << ")" << std::endl;
}

// 保留墙面点 - 优化版
void preserveWalls(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out,
                  int k_search, float vertical_threshold) {
    std::cout << "识别并保留墙面点(优化版)..." << std::endl;
    
    // 计算法线
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    
    ne.setInputCloud(cloud_in);
    ne.setSearchMethod(tree);
    ne.setKSearch(k_search);
    ne.compute(*normals);
    
    std::cout << "法线计算完成" << std::endl;
    
    // 创建带法线的点云
    PointCloudNormalT::Ptr cloud_with_normals(new PointCloudNormalT);
    cloud_with_normals->resize(cloud_in->size());
    
    // 合并点和法线
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        cloud_with_normals->points[i].x = cloud_in->points[i].x;
        cloud_with_normals->points[i].y = cloud_in->points[i].y;
        cloud_with_normals->points[i].z = cloud_in->points[i].z;
        cloud_with_normals->points[i].r = cloud_in->points[i].r;
        cloud_with_normals->points[i].g = cloud_in->points[i].g;
        cloud_with_normals->points[i].b = cloud_in->points[i].b;
        cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
        cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
        cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
    }
    
    // 识别墙面点 - 墙面法线应该与Y轴垂直
    cloud_out->clear();
    cloud_out->reserve(cloud_in->size());
    
    int wall_points = 0;
    int non_wall_points = 0;
    int preserved_non_wall_points = 0;
    
    // 创建按高度分层的点云颜色增强
    std::vector<PointT> wall_point_layers[10]; // 最多10层
    
    // 更智能的墙面点识别
    float height_min = FLT_MAX;
    float height_max = -FLT_MAX;
    
    // 先计算高度范围
    for (const auto& point : cloud_in->points) {
        height_min = std::min(height_min, point.y);
        height_max = std::max(height_max, point.y);
    }
    
    float height_range = height_max - height_min;
    
    for (size_t i = 0; i < cloud_with_normals->size(); ++i) {
        const auto& point = cloud_with_normals->points[i];
        
        // 计算法线与Y轴的夹角余弦值
        float cos_angle_y = std::abs(point.normal_y);
        
        // 计算高度层
        int layer = 0;
        if (height_range > 0) {
            layer = std::min(9, (int)(((point.y - height_min) / height_range) * 10));
        }
        
        // 如果法线与Y轴接近垂直，则认为是墙面点
        // 余弦值越小，越接近垂直
        if (cos_angle_y < vertical_threshold) {
            PointT p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.r = point.r;
            p.g = point.g;
            p.b = point.b;
            
            // 增强墙面点的颜色 - 使墙面点在点云中更明显
            p.r = std::min(255, p.r + 30);
            p.g = std::min(255, p.g + 30);
            
            cloud_out->push_back(p);
            wall_point_layers[layer].push_back(p);
            wall_points++;
        } else {
            // 即使不是墙面点，也根据高度层和密度决定是否保留
            // 中间层(1.2m-1.8m左右)的点保留概率更高，因为这通常是门窗和重要细节区域
            float keep_probability = 0.7; // 基础保留概率
            
            // 根据高度层调整保留概率
            if (layer >= 3 && layer <= 7) {
                // 中间层保留更多非墙面点
                keep_probability = 0.8;
            } else if (layer <= 1 || layer >= 9) {
                // 最底层和最顶层(地面和天花板附近)保留较少非墙面点
                keep_probability = 0.4;
            }
            
            if ((rand() % 100) < (keep_probability * 100)) {
                PointT p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                p.r = point.r;
                p.g = point.g;
                p.b = point.b;
                
                // 对非墙面点稍微调暗，让墙面点更突出
                p.r = std::max(0, p.r - 10);
                p.g = std::max(0, p.g - 10);
                p.b = std::max(0, p.b - 10);
                
                cloud_out->push_back(p);
                preserved_non_wall_points++;
            }
            
            non_wall_points++;
        }
    }
    
    std::cout << "墙面点: " << wall_points << ", 非墙面点: " << non_wall_points 
              << ", 保留的非墙面点: " << preserved_non_wall_points << std::endl;
    std::cout << "墙面保留后的点数: " << cloud_out->size() << " (原始: " << cloud_in->size() << ")" << std::endl;
    
    // 检查每层的墙面点数量
    std::cout << "墙面点层分布:" << std::endl;
    for (int i = 0; i < 10; i++) {
        std::cout << "  层 " << i << ": " << wall_point_layers[i].size() << " 个点" << std::endl;
    }
}

// 欧氏聚类分割
void euclideanClusterExtraction(const PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_out,
                              float tolerance, int min_cluster_size, int max_cluster_size) {
    std::cout << "执行欧氏聚类分割..." << std::endl;
    
    // 创建KD树
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_in);
    
    // 执行欧氏聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);
    
    std::cout << "找到 " << cluster_indices.size() << " 个聚类" << std::endl;
    
    // 合并所有聚类
    cloud_out->clear();
    
    // 安全检查 - 如果没有找到足够的聚类，保留原始点云
    if (cluster_indices.size() < 2) {
        std::cout << "警告: 聚类数量过少，保留原始点云" << std::endl;
        cloud_out->points = cloud_in->points;
        cloud_out->width = cloud_in->width;
        cloud_out->height = cloud_in->height;
        cloud_out->is_dense = cloud_in->is_dense;
        return;
    }
    
    // 提取并合并聚类
    for (const auto& indices : cluster_indices) {
        for (const auto& idx : indices.indices) {
            cloud_out->push_back(cloud_in->points[idx]);
        }
    }
    
    std::cout << "欧氏聚类后保留了 " << cloud_out->size() << " 个点 (原始: " << cloud_in->size() << ")" << std::endl;
}

void printHelp() {
    std::cout << "Point Cloud Processor Tool" << std::endl;
    std::cout << "用法: cloud_processor_tool input.ply output.ply [options]" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "--denoise k stddev      : 统计离群点去除 (k=邻居数, stddev=标准差)" << std::endl;
    std::cout << "--downsample size       : 体素网格下采样 (size=叶子大小)" << std::endl;
    std::cout << "--density-filter r n    : 密度过滤 (r=半径, n=最小邻居数)" << std::endl;
    std::cout << "--height-filter min max : 高度过滤 (min=最小百分比, max=最大百分比)" << std::endl;
    std::cout << "--preserve-walls k t    : 保留墙面点 (k=邻居数, t=垂直阈值)" << std::endl;
    std::cout << "--cluster d min max     : 欧氏聚类 (d=距离阈值, min=最小簇大小, max=最大簇大小)" << std::endl;
    std::cout << "--remove-distant f      : 移除远离中心点 (f=阈值因子, 默认2.0)" << std::endl;
    std::cout << "--floor-ceiling a d r   : 地面天花板处理 (a=角度阈值, d=距离阈值, r=是否移除)" << std::endl;
    std::cout << "--help                  : 显示此帮助信息" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3 || pcl::console::find_switch(argc, argv, "--help")) {
        printHelp();
        return 0;
    }
    
    // 解析输入和输出文件名
    std::string input_file = argv[1];
    std::string output_file = argv[2];
    
    std::cout << "输入文件: " << input_file << std::endl;
    std::cout << "输出文件: " << output_file << std::endl;
    
    // 加载点云
    PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT::Ptr cloud_processed(new PointCloudT);
    
    std::cout << "加载点云..." << std::endl;
    if (pcl::io::loadPLYFile<PointT>(input_file, *cloud) == -1) {
        std::cerr << "无法加载点云文件: " << input_file << std::endl;
        return -1;
    }
    
    std::cout << "加载了 " << cloud->size() << " 个点" << std::endl;
    
    // 初始化处理后的点云
    *cloud_processed = *cloud;
    
    // 移除远离中心的异常点
    if (pcl::console::find_switch(argc, argv, "--remove-distant")) {
        float threshold_factor = 2.0f;  // 默认值
        pcl::console::parse_argument(argc, argv, "--remove-distant", threshold_factor);
        
        PointCloudT::Ptr cloud_no_distant(new PointCloudT);
        removeDistantOutliers(cloud_processed, cloud_no_distant, threshold_factor);
        cloud_processed = cloud_no_distant;
    }
    
    // 统计离群点去除
    if (pcl::console::find_switch(argc, argv, "--denoise")) {
        int k = 15;  // 默认值
        float stddev = 1.0f;  // 默认值
        
        pcl::console::parse_argument(argc, argv, "--denoise", k);
        
        // 获取第二个参数
        int stddev_idx = pcl::console::find_argument(argc, argv, "--denoise") + 2;
        if (stddev_idx < argc) {
            stddev = static_cast<float>(atof(argv[stddev_idx]));
        }
        
        std::cout << "执行统计离群点去除 (k=" << k << ", stddev=" << stddev << ")..." << std::endl;
        
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_processed);
        sor.setMeanK(k);
        sor.setStddevMulThresh(stddev);
        sor.filter(*cloud_processed);
        
        std::cout << "离群点去除后保留了 " << cloud_processed->size() << " 个点" << std::endl;
    }
    
    // 高度过滤
    if (pcl::console::find_switch(argc, argv, "--height-filter")) {
        float min_pct = 0.05f;  // 默认值
        float max_pct = 0.95f;  // 默认值
        
        pcl::console::parse_argument(argc, argv, "--height-filter", min_pct);
        
        // 获取第二个参数
        int max_pct_idx = pcl::console::find_argument(argc, argv, "--height-filter") + 2;
        if (max_pct_idx < argc) {
            max_pct = static_cast<float>(atof(argv[max_pct_idx]));
        }
        
        PointCloudT::Ptr cloud_height_filtered(new PointCloudT);
        heightFilter(cloud_processed, cloud_height_filtered, min_pct, max_pct);
        cloud_processed = cloud_height_filtered;
    }
    
    // 地面天花板处理
    if (pcl::console::find_switch(argc, argv, "--floor-ceiling")) {
        float angle_threshold = 10.0f;  // 默认角度阈值(度)
        float distance_threshold = 0.1f;  // 默认距离阈值(米)
        bool remove_planes = false;  // 默认保留平面而不是移除
        
        pcl::console::parse_argument(argc, argv, "--floor-ceiling", angle_threshold);
        
        // 获取第二个参数
        int dist_idx = pcl::console::find_argument(argc, argv, "--floor-ceiling") + 2;
        if (dist_idx < argc) {
            distance_threshold = static_cast<float>(atof(argv[dist_idx]));
        }
        
        // 获取第三个参数
        int remove_idx = pcl::console::find_argument(argc, argv, "--floor-ceiling") + 3;
        if (remove_idx < argc) {
            remove_planes = static_cast<bool>(atoi(argv[remove_idx]));
        }
        
        PointCloudT::Ptr cloud_floor_ceiling(new PointCloudT);
        extractFloorAndCeiling(cloud_processed, cloud_floor_ceiling, 
                              angle_threshold, distance_threshold, remove_planes);
        cloud_processed = cloud_floor_ceiling;
    }
    
    // 体素网格下采样
    if (pcl::console::find_switch(argc, argv, "--downsample")) {
        float leaf_size = 0.025f;  // 默认值
        pcl::console::parse_argument(argc, argv, "--downsample", leaf_size);
        
        std::cout << "执行体素网格下采样 (leaf_size=" << leaf_size << ")..." << std::endl;
        
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_processed);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*cloud_processed);
        
        std::cout << "下采样后保留了 " << cloud_processed->size() << " 个点" << std::endl;
    }
    
    // 保留墙面点
    if (pcl::console::find_switch(argc, argv, "--preserve-walls")) {
        int k_search = 20;  // 默认值
        float vertical_threshold = 0.6f;  // 默认值
        
        pcl::console::parse_argument(argc, argv, "--preserve-walls", k_search);
        
        // 获取第二个参数
        int threshold_idx = pcl::console::find_argument(argc, argv, "--preserve-walls") + 2;
        if (threshold_idx < argc) {
            vertical_threshold = static_cast<float>(atof(argv[threshold_idx]));
        }
        
        PointCloudT::Ptr cloud_walls(new PointCloudT);
        preserveWalls(cloud_processed, cloud_walls, k_search, vertical_threshold);
        cloud_processed = cloud_walls;
    }
    
    // 欧氏聚类
    if (pcl::console::find_switch(argc, argv, "--cluster")) {
        float tolerance = 0.2f;  // 默认值
        int min_cluster_size = 5;  // 默认值
        int max_cluster_size = 100000;  // 默认值
        
        pcl::console::parse_argument(argc, argv, "--cluster", tolerance);
        
        // 获取第二个和第三个参数
        int min_size_idx = pcl::console::find_argument(argc, argv, "--cluster") + 2;
        int max_size_idx = min_size_idx + 1;
        
        if (min_size_idx < argc) {
            min_cluster_size = atoi(argv[min_size_idx]);
        }
        
        if (max_size_idx < argc) {
            max_cluster_size = atoi(argv[max_size_idx]);
        }
        
        PointCloudT::Ptr cloud_clustered(new PointCloudT);
        euclideanClusterExtraction(cloud_processed, cloud_clustered, tolerance, min_cluster_size, max_cluster_size);
        cloud_processed = cloud_clustered;
    }
    
    // 密度过滤
    if (pcl::console::find_switch(argc, argv, "--density-filter")) {
        float radius = 0.06f;  // 默认值
        int min_neighbors = 1;  // 默认值
        
        pcl::console::parse_argument(argc, argv, "--density-filter", radius);
        
        // 获取第二个参数
        int neighbors_idx = pcl::console::find_argument(argc, argv, "--density-filter") + 2;
        if (neighbors_idx < argc) {
            min_neighbors = atoi(argv[neighbors_idx]);
        }
        
        PointCloudT::Ptr cloud_density_filtered(new PointCloudT);
        densityFilter(cloud_processed, cloud_density_filtered, radius, min_neighbors);
        cloud_processed = cloud_density_filtered;
    }
    
    // 保存处理后的点云
    std::cout << "保存处理后的点云到 " << output_file << std::endl;
    pcl::io::savePLYFileBinary(output_file, *cloud_processed);
    
    std::cout << "处理完成！最终点云包含 " << cloud_processed->size() << " 个点" << std::endl;
    
    return 0;
} 
