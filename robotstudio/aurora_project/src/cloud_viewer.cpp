#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cctype>
#include "cloud_processor.h"
#include "cloud_visualizer.h"

void printHelp() {
    std::cout << "点云查看器使用方法:" << std::endl;
    std::cout << "cloud_viewer <点云文件路径> [选项]" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  --no-denoise       禁用去噪" << std::endl;
    std::cout << "  --no-downsample    禁用下采样" << std::endl;
    std::cout << "  --denoise <meanK> <stddev>  自定义去噪参数" << std::endl;
    std::cout << "  --downsample <leafSize>     自定义下采样参数" << std::endl;
    std::cout << "  --save <filename>           保存处理后的点云" << std::endl;
    std::cout << "  --help                      显示帮助信息" << std::endl;
}

int main(int argc, char** argv) {
    // 检查参数
    if (argc < 2) {
        std::cerr << "错误: 缺少点云文件路径" << std::endl;
        printHelp();
        return 1;
    }

    std::string cloudFile = argv[1];

    // 解析选项参数
    bool enableDenoise = true;
    bool enableDownsample = true;
    bool enableSegmentation = false;
    int meanK = 25;
    float stddevMul = 1.0f;
    float leafSize = 0.01f;
    std::string saveFile = "";

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--no-denoise") {
            enableDenoise = false;
        } else if (arg == "--no-downsample") {
            enableDownsample = false;
        } else if (arg == "--denoise" && i + 2 < argc) {
            meanK = std::stoi(argv[i+1]);
            stddevMul = std::stof(argv[i+2]);
            i += 2;
        } else if (arg == "--downsample" && i + 1 < argc) {
            leafSize = std::stof(argv[i+1]);
            i += 1;
        } else if (arg == "--save" && i + 1 < argc) {
            saveFile = argv[i+1];
            i += 1;
        } else if (arg == "--help") {
            printHelp();
            return 0;
        }
    }

    // 创建点云处理器
    CloudProcessor processor;
    
    // 加载点云文件
    std::cout << "加载点云文件: " << cloudFile << std::endl;
    
    bool loaded = false;
    
    // 根据文件扩展名选择加载方式
    std::string ext = cloudFile.substr(cloudFile.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });
    
    if (ext == "xyz") {
        loaded = processor.loadFromXYZ(cloudFile);
    } else if (ext == "ply") {
        loaded = processor.loadFromPLY(cloudFile);
    } else if (ext == "stcm") {
        loaded = processor.loadFromSTCM(cloudFile);
    } else {
        std::cerr << "不支持的文件格式: " << ext << std::endl;
        return 1;
    }
    
    if (!loaded) {
        std::cerr << "无法加载点云文件" << std::endl;
        return 1;
    }
    
    std::cout << "成功加载点云，点数量: " << processor.getPointCount() << std::endl;
    
    // 处理点云（去噪、降采样）
    if (enableDenoise || enableDownsample) {
        std::cout << "处理点云..." << std::endl;
        processor.processCloud(leafSize, meanK, stddevMul);
    }
    
    // 创建可视化器
    std::cout << "初始化可视化器..." << std::endl;
    CloudVisualizer visualizer;
    
    // 设置处理参数
    visualizer.setDenoising(enableDenoise, meanK, stddevMul);
    visualizer.setDownsampling(enableDownsample, leafSize);
    
    // 加载点云
    visualizer.setInputCloud(processor.getPoints());
    
    // 可视化
    std::cout << "显示点云，按q退出..." << std::endl;
    
    while (!visualizer.wasStopped()) {
        visualizer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 如果指定了保存文件，保存处理后的点云
    if (!saveFile.empty()) {
        std::cout << "保存点云到: " << saveFile << std::endl;
        if (!visualizer.savePointCloudPLY(saveFile)) {
            std::cerr << "保存点云失败" << std::endl;
            return 1;
        }
    }
    
    return 0;
} 