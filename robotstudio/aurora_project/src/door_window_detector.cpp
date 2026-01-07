#include <iostream>
#include <string>
#include <vector>
#include "cloud_processor.h"

void printUsage() {
    std::cout << "Door Window Detector" << std::endl;
    std::cout << "Usage: door_window_detector <input_file> <output_file> [options]" << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  input_file   Input point cloud file (PLY format)" << std::endl;
    std::cout << "  output_file  Output point cloud file (PLY format with door/window markers)" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --verbose    Show detailed information" << std::endl;
    std::cout << "  --help       Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Color markers:" << std::endl;
    std::cout << "  Red - Door" << std::endl;
    std::cout << "  Blue - Window" << std::endl;
    std::cout << "  Yellow - Unknown opening" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage();
        return 1;
    }
    
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    bool verbose = false;
    
    // 解析命令行参数
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--help") {
            printUsage();
            return 0;
        }
    }
    
    if (verbose) {
        std::cout << "Door Window Detector starting..." << std::endl;
        std::cout << "Input file: " << inputFile << std::endl;
        std::cout << "Output file: " << outputFile << std::endl;
    }
    
    // 创建点云处理器
    CloudProcessor processor;
    
    // Load point cloud file
    if (verbose) {
        std::cout << "Loading point cloud file..." << std::endl;
    }

    if (!processor.loadFromPLY(inputFile)) {
        std::cerr << "Error: Cannot load point cloud file " << inputFile << std::endl;
        return 1;
    }

    // Process point cloud
    if (verbose) {
        std::cout << "Processing point cloud data..." << std::endl;
    }

    if (!processor.processCloud(0.02, 25, 0.8)) {
        std::cerr << "Error: Point cloud processing failed" << std::endl;
        return 1;
    }

    // Detect doors and windows and mark colors
    if (verbose) {
        std::cout << "Detecting doors and windows..." << std::endl;
    }
    
    auto doorWindows = processor.detectAndMarkDoorWindows();
    
    // Output detection results
    std::cout << "Detection results:" << std::endl;
    int doorCount = 0, windowCount = 0, unknownCount = 0;

    for (const auto& dw : doorWindows) {
        switch (dw.type) {
            case DoorWindow::DOOR:
                doorCount++;
                if (verbose) {
                    std::cout << "  Door: Position(" << dw.center.x << ", " << dw.center.y << ", " << dw.center.z
                              << ") Size(" << dw.width << "x" << dw.height << ") Confidence:" << dw.confidence << std::endl;
                }
                break;
            case DoorWindow::WINDOW:
                windowCount++;
                if (verbose) {
                    std::cout << "  Window: Position(" << dw.center.x << ", " << dw.center.y << ", " << dw.center.z
                              << ") Size(" << dw.width << "x" << dw.height << ") Confidence:" << dw.confidence << std::endl;
                }
                break;
            default:
                unknownCount++;
                if (verbose) {
                    std::cout << "  Unknown opening: Position(" << dw.center.x << ", " << dw.center.y << ", " << dw.center.z
                              << ") Size(" << dw.width << "x" << dw.height << ") Confidence:" << dw.confidence << std::endl;
                }
                break;
        }
    }

    std::cout << "Total: " << doorCount << " doors, " << windowCount << " windows, "
              << unknownCount << " unknown openings" << std::endl;
    
    // Save results
    if (verbose) {
        std::cout << "Saving results to " << outputFile << std::endl;
    }

    if (!processor.saveToPLY(outputFile)) {
        std::cerr << "Error: Cannot save result file " << outputFile << std::endl;
        return 1;
    }

    std::cout << "Door window detection completed!" << std::endl;
    std::cout << "Please open " << outputFile << " in a point cloud viewer to see the marked results" << std::endl;
    
    return 0;
}
