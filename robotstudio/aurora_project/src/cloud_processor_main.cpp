#include "cloud_processor.h"
#include <iostream>
#include <string>
#include <cstring>

void printUsage() {
    std::cout << "Point cloud processing tool usage:" << std::endl;
    std::cout << "cloud_processor <input_file> <output_file> [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --denoise <meanK> <stddev>    Apply denoising with parameters" << std::endl;
    std::cout << "  --downsample <leaf_size>      Apply downsampling with voxel size" << std::endl;
    std::cout << "  --extract-planes <threshold>  Extract planes with distance threshold" << std::endl;
    std::cout << "  --help                        Show this help information" << std::endl;
}

int main(int argc, char** argv) {
    // Check arguments
    if (argc < 3) {
        std::cerr << "Insufficient arguments" << std::endl;
        printUsage();
        return 1;
    }
    
    // Parse input and output files
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    
    // Create cloud processor
    CloudProcessor processor;
    
    // Process parameters
    bool applyDenoising = false;
    bool applyDownsampling = false;
    bool extractPlanes = false;
    
    float denoiseStddev = 1.0f;
    int denoiseMeanK = 25;
    float downsampleLeafSize = 0.005f;
    float planeThreshold = 0.02f;
    
    // Parse options
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--denoise" && i + 2 < argc) {
            applyDenoising = true;
            denoiseMeanK = std::stoi(argv[i + 1]);
            denoiseStddev = std::stof(argv[i + 2]);
            i += 2;
        } else if (arg == "--downsample" && i + 1 < argc) {
            applyDownsampling = true;
            downsampleLeafSize = std::stof(argv[i + 1]);
            i += 1;
        } else if (arg == "--extract-planes" && i + 1 < argc) {
            extractPlanes = true;
            planeThreshold = std::stof(argv[i + 1]);
            i += 1;
        } else if (arg == "--help") {
            printUsage();
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage();
            return 1;
        }
    }
    
    // Load point cloud
    std::string fileExt = inputFile.substr(inputFile.find_last_of(".") + 1);
    bool loaded = false;
    
    std::cout << "Loading point cloud: " << inputFile << std::endl;
    if (fileExt == "xyz") {
        loaded = processor.loadFromXYZ(inputFile);
    } else if (fileExt == "ply") {
        loaded = processor.loadFromPLY(inputFile);
    } else if (fileExt == "stcm") {
        loaded = processor.loadFromSTCM(inputFile);
    } else {
        std::cerr << "Unsupported file format: " << fileExt << std::endl;
        return 1;
    }
    
    if (!loaded) {
        std::cerr << "Failed to load point cloud" << std::endl;
        return 1;
    }
    
    // Process point cloud
    if (applyDenoising || applyDownsampling) {
        std::cout << "Processing point cloud..." << std::endl;
        processor.processCloud(downsampleLeafSize, denoiseMeanK, denoiseStddev);
    }
    
    // Extract planes
    if (extractPlanes) {
        std::cout << "Extracting planes..." << std::endl;
        auto planes = processor.extractPlanes(planeThreshold);
        std::cout << "Extracted " << planes.size() << " planes" << std::endl;
    }
    
    // Save processed point cloud
    std::cout << "Saving processed point cloud: " << outputFile << std::endl;
    if (!processor.saveToPLY(outputFile)) {
        std::cerr << "Failed to save point cloud" << std::endl;
        return 1;
    }
    
    std::cout << "Point cloud processing complete" << std::endl;
    return 0;
} 
 
