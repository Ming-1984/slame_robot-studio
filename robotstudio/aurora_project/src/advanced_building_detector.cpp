/*
 * Advanced Building Element Detector
 * 高级建筑元素检测工具 - 检测门窗、墙面、地面、屋顶并进行智能着色
 */

#include "cloud_processor.h"
#include <iostream>
#include <string>
#include <chrono>

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <input_file> <output_file> [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --verbose, -v     Enable verbose output" << std::endl;
    std::cout << "  --help, -h        Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Input file formats supported: .ply" << std::endl;
    std::cout << "Output file formats: .ply (colored point cloud)" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << programName << " input.ply output_colored.ply --verbose" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    bool verbose = false;

    // Parse command line arguments
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }

    if (verbose) {
        std::cout << "=== Aurora Advanced Building Element Detector ===" << std::endl;
        std::cout << "Input file: " << inputFile << std::endl;
        std::cout << "Output file: " << outputFile << std::endl;
        std::cout << std::endl;
    }

    // Initialize processor
    CloudProcessor processor;

    // Load point cloud
    if (verbose) {
        std::cout << "Loading point cloud from: " << inputFile << std::endl;
    }

    if (!processor.loadFromPLY(inputFile)) {
        std::cerr << "Error: Cannot load point cloud file " << inputFile << std::endl;
        return 1;
    }

    // Process point cloud
    if (verbose) {
        std::cout << "Processing point cloud data..." << std::endl;
    }

    auto start = std::chrono::high_resolution_clock::now();

    if (!processor.processCloud(0.015, 30, 0.8)) {
        std::cerr << "Error: Point cloud processing failed" << std::endl;
        return 1;
    }

    // Detect building elements
    if (verbose) {
        std::cout << "Detecting building elements (walls, doors, windows, floors, ceilings)..." << std::endl;
    }

    if (!processor.detectBuildingElements()) {
        std::cerr << "Error: Building element detection failed" << std::endl;
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Get detection statistics
    int wallCount, doorCount, windowCount, floorCount, ceilingCount;
    processor.getBuildingElementStats(wallCount, doorCount, windowCount, floorCount, ceilingCount);

    // Output detection results
    std::cout << std::endl;
    std::cout << "=== Building Element Detection Results ===" << std::endl;
    std::cout << "Processing time: " << duration.count() << " ms" << std::endl;
    std::cout << std::endl;
    std::cout << "Detected elements:" << std::endl;
    std::cout << "  🧱 Walls: " << wallCount << std::endl;
    std::cout << "  🚪 Doors: " << doorCount << std::endl;
    std::cout << "  🪟 Windows: " << windowCount << std::endl;
    std::cout << "  🟢 Floors: " << floorCount << std::endl;
    std::cout << "  🔴 Ceilings: " << ceilingCount << std::endl;
    std::cout << std::endl;
    
    std::cout << "Color coding applied:" << std::endl;
    std::cout << "  🧱 Walls: Gray (128, 128, 128)" << std::endl;
    std::cout << "  🚪 Doors: Brown (139, 69, 19)" << std::endl;
    std::cout << "  🪟 Windows: Blue (0, 0, 255)" << std::endl;
    std::cout << "  🟢 Floors: Green (0, 255, 0)" << std::endl;
    std::cout << "  🔴 Ceilings: Red (255, 0, 0)" << std::endl;

    // Save colored point cloud
    if (verbose) {
        std::cout << std::endl;
        std::cout << "Saving colored point cloud to: " << outputFile << std::endl;
    }

    if (!processor.saveToPLY(outputFile)) {
        std::cerr << "Error: Failed to save colored point cloud" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "✅ Advanced building element detection completed successfully!" << std::endl;
    std::cout << "📁 Colored point cloud saved to: " << outputFile << std::endl;

    return 0;
}
