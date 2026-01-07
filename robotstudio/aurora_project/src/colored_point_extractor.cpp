#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <memory>

#include "aurora_pubsdk_inc.h"

using namespace rp::standalone::aurora;

// 3D point structure with color
struct ColoredPoint3D {
    float x, y, z;
    float r, g, b;  // RGB color values (0.0-1.0)
    
    ColoredPoint3D() : x(0), y(0), z(0), r(0), g(0), b(0) {}
    ColoredPoint3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z), r(0), g(0), b(0) {}
    ColoredPoint3D(float _x, float _y, float _z, float _r, float _g, float _b) 
        : x(_x), y(_y), z(_z), r(_r), g(_g), b(_b) {}
};

// Global callback flags
volatile bool g_uploadCompleted = false;
volatile bool g_uploadSucceeded = false;

// Upload result callback
void uploadResultCallback(void* userData, int isOK) {
    g_uploadCompleted = true;
    g_uploadSucceeded = (isOK != 0);
}

// Height to color mapping function
void heightToColor(float height, float minHeight, float maxHeight, float& r, float& g, float& b) {
    // Normalize height to 0-1 range
    float t = (height - minHeight) / (maxHeight - minHeight);
    t = std::max(0.0f, std::min(1.0f, t));
    
    // Rainbow color mapping (blue to red)
    if (t < 0.25f) {
        // Blue to cyan
        r = 0.0f;
        g = t * 4.0f;
        b = 1.0f;
    } else if (t < 0.5f) {
        // Cyan to green
        r = 0.0f;
        g = 1.0f;
        b = 1.0f - (t - 0.25f) * 4.0f;
    } else if (t < 0.75f) {
        // Green to yellow
        r = (t - 0.5f) * 4.0f;
        g = 1.0f;
        b = 0.0f;
    } else {
        // Yellow to red
        r = 1.0f;
        g = 1.0f - (t - 0.75f) * 4.0f;
        b = 0.0f;
    }
}

class ColoredPointCloudExtractor {
private:
    std::vector<ColoredPoint3D> m_points;
    std::string m_deviceIP;
    std::unique_ptr<RemoteSDK> m_sdk;
    bool m_isConnected;
    bool m_verbose;
    int m_syncTimeSeconds;
    
    // Connect to Aurora device
    bool connect() {
        if (m_isConnected && m_sdk) {
            return true;
        }
        
        // Create SDK session
        m_sdk.reset(RemoteSDK::CreateSession());
        if (!m_sdk) {
            std::cerr << "Failed to create SDK session" << std::endl;
            return false;
        }
        
        if (m_verbose) {
            std::cout << "Connecting to Aurora device: " << m_deviceIP << std::endl;
        }
        
        // Connect to device
        if (!m_sdk->connect({m_deviceIP.c_str()})) {
            std::cerr << "Failed to connect to device" << std::endl;
            m_sdk.reset();
            return false;
        }
        
        if (m_verbose) {
            std::cout << "Successfully connected to device" << std::endl;
        }
        
        m_isConnected = true;
        return true;
    }
    
    // Disconnect from Aurora device
    void disconnect() {
        if (m_sdk) {
            m_sdk->disconnect();
            m_sdk.reset();
            m_isConnected = false;
            
            if (m_verbose) {
                std::cout << "Disconnected from device" << std::endl;
            }
        }
    }
    
    // Apply height-based coloring to the point cloud
    void applyHeightColoring() {
        if (m_points.empty()) {
            return;
        }
        
        // Find height range
        float minHeight = std::numeric_limits<float>::max();
        float maxHeight = std::numeric_limits<float>::lowest();
        
        for (const auto& p : m_points) {
            minHeight = std::min(minHeight, p.z);
            maxHeight = std::max(maxHeight, p.z);
        }
        
        if (m_verbose) {
            std::cout << "Height range: " << minHeight << " to " << maxHeight << std::endl;
        }
        
        // Apply height-based coloring
        for (auto& p : m_points) {
            heightToColor(p.z, minHeight, maxHeight, p.r, p.g, p.b);
        }
        
        if (m_verbose) {
            std::cout << "Applied height-based coloring to " << m_points.size() << " points" << std::endl;
        }
    }
    
    // Filter out outliers from point cloud
    void filterOutliers(float outlierThreshold = 2.0f) {
        if (m_points.size() < 10) return; // Too few points to filter
        
        if (m_verbose) {
            std::cout << "Filtering outliers..." << std::endl;
        }
        
        // Calculate bounding box
        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float minZ = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float maxY = std::numeric_limits<float>::lowest();
        float maxZ = std::numeric_limits<float>::lowest();
        
        for (const auto& p : m_points) {
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            minZ = std::min(minZ, p.z);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
            maxZ = std::max(maxZ, p.z);
        }
        
        if (m_verbose) {
            std::cout << "Point cloud bounding box: (" << minX << ", " << minY << ", " << minZ << ") -> (" 
                      << maxX << ", " << maxY << ", " << maxZ << ")" << std::endl;
        }
        
        // Calculate bounding box size
        float sizeX = maxX - minX;
        float sizeY = maxY - minY;
        float sizeZ = maxZ - minZ;
        
        // Set filtering threshold based on bounding box size
        float thresholdX = sizeX * outlierThreshold;
        float thresholdY = sizeY * outlierThreshold;
        float thresholdZ = sizeZ * outlierThreshold;
        
        // Calculate point cloud center
        float centerX = (minX + maxX) / 2.0f;
        float centerY = (minY + maxY) / 2.0f;
        float centerZ = (minZ + maxZ) / 2.0f;
        
        // Filter outliers
        size_t originalSize = m_points.size();
        
        m_points.erase(
            std::remove_if(m_points.begin(), m_points.end(), 
                          [=](const ColoredPoint3D& p) {
                              return std::abs(p.x - centerX) > thresholdX ||
                                     std::abs(p.y - centerY) > thresholdY ||
                                     std::abs(p.z - centerZ) > thresholdZ;
                          }),
            m_points.end()
        );
        
        if (m_verbose) {
            std::cout << "Filtered out " << (originalSize - m_points.size()) << " outliers" << std::endl;
            std::cout << "Remaining points: " << m_points.size() << std::endl;
        }
    }
    
    // Extract point cloud from all available maps
    bool extractAllMaps() {
        // Get global mapping info
        slamtec_aurora_sdk_global_map_desc_t globalMapDesc;
        if (!m_sdk->dataProvider.getGlobalMappingInfo(globalMapDesc)) {
            std::cerr << "Failed to get global map info" << std::endl;
            return false;
        }
        
        int activeMapId = globalMapDesc.activeMapID;
        
        if (m_verbose) {
            std::cout << "Active map ID: " << activeMapId << std::endl;
        }
        
        // Extract from active map first
        bool result = extractPointCloudFromMap(activeMapId);
        
        // Get all available map IDs - 使用正确的API
        std::vector<slamtec_aurora_sdk_map_desc_t> mapDescs;
        
        // 使用正确的API调用方式
        if (m_sdk->dataProvider.getAllMapInfo(mapDescs)) {
            if (m_verbose) {
                std::cout << "Found " << mapDescs.size() << " available maps" << std::endl;
            }
            
            // Extract from all other maps
            for (const auto& mapDesc : mapDescs) {
                if (mapDesc.map_id != activeMapId && mapDesc.map_id > 0) {
                    if (m_verbose) {
                        std::cout << "Extracting from additional map ID: " << mapDesc.map_id << std::endl;
                    }
                    
                    extractPointCloudFromMap(mapDesc.map_id);
                }
            }
        }
        
        return result || !m_points.empty();
    }
    
    // Extract point cloud from a specific map ID
    bool extractPointCloudFromMap(int mapId) {
        // Map ID 0 is valid! Don't reject it.
        if (mapId < 0) {
            std::cerr << "Invalid map ID: " << mapId << std::endl;
            return false;
        }
        
        // Create map data visitor
        RemoteMapDataVisitor visitor;
        
        // Track keyframe and map point counts
        int keyframeCount = 0;
        int mapPointCount = 0;
        std::set<uint64_t> uniquePointIds;
        size_t initialPointCount = m_points.size();
        
        // Subscribe to map data
        visitor.subscribeMapData([&](const slamtec_aurora_sdk_map_desc_t& mapDesc) {
            if (m_verbose) {
                std::cout << "Map ID: " << mapDesc.map_id << std::endl;
                std::cout << "Map point count: " << mapDesc.map_point_count << std::endl;
                std::cout << "Keyframe count: " << mapDesc.keyframe_count << std::endl;
            }
        });
        
        // Subscribe to keyframe data
        visitor.subscribeKeyFrameData([&](const RemoteKeyFrameData& keyFrameData) {
            keyframeCount++;
            if (m_verbose && keyframeCount <= 5) {
                std::cout << "Keyframe " << keyframeCount << ": ID=" << keyFrameData.desc.id 
                          << ", Position=(" << keyFrameData.desc.pose.translation.x 
                          << ", " << keyFrameData.desc.pose.translation.y 
                          << ", " << keyFrameData.desc.pose.translation.z << ")" << std::endl;
            }
            
            // Try to extract additional data from keyframes if available
            // Note: This is implementation-specific and may not be available in all SDK versions
        });
        
        // Subscribe to map point data
        visitor.subscribeMapPointData([&](const slamtec_aurora_sdk_map_point_desc_t& mapPoint) {
            mapPointCount++;
            
            // Check if point is valid
            if (std::isfinite(mapPoint.position.x) && 
                std::isfinite(mapPoint.position.y) && 
                std::isfinite(mapPoint.position.z)) {
                
                // Check if point ID already exists (avoid duplicates)
                if (uniquePointIds.find(mapPoint.id) == uniquePointIds.end()) {
                    uniquePointIds.insert(mapPoint.id);
                    
                    // Filter out noise points near origin
                    const double minDistance = 0.001; // 1mm
                    double distance = std::sqrt(mapPoint.position.x * mapPoint.position.x + 
                                           mapPoint.position.y * mapPoint.position.y + 
                                           mapPoint.position.z * mapPoint.position.z);
                    
                    if (distance > minDistance) {
                        // Check for abnormally large coordinates that might cause issues
                        const double maxCoordinate = 100.0; // 100 meters should be reasonable for indoor mapping
                        double maxCoord = std::max({std::abs(mapPoint.position.x),
                                                   std::abs(mapPoint.position.y),
                                                   std::abs(mapPoint.position.z)});

                        if (maxCoord > maxCoordinate) {
                            if (m_verbose) {
                                std::cout << "Warning: Skipping point with large coordinates: ID=" << mapPoint.id
                                          << ", Position=(" << mapPoint.position.x
                                          << ", " << mapPoint.position.y
                                          << ", " << mapPoint.position.z << ")" << std::endl;
                            }
                            return; // Skip this point
                        }

                        // Collect point cloud data (colors will be added later)
                        m_points.emplace_back(
                            static_cast<float>(mapPoint.position.x),
                            static_cast<float>(mapPoint.position.y),
                            static_cast<float>(mapPoint.position.z)
                        );

                        // Output info for first 5 points
                        if (m_verbose && uniquePointIds.size() <= 5) {
                            std::cout << "Point " << uniquePointIds.size() << ": ID=" << mapPoint.id
                                      << ", Position=(" << mapPoint.position.x
                                      << ", " << mapPoint.position.y
                                      << ", " << mapPoint.position.z << ")" << std::endl;
                        }
                    }
                }
            }
        });
        
        // Access map data with longer timeout
        if (m_verbose) {
            std::cout << "Accessing map data for map ID " << mapId << "..." << std::endl;
        }
        
        // Try multiple times with different access methods
        bool accessSuccess = false;
        
        // Method 1: Direct access to specific map ID
        accessSuccess = m_sdk->dataProvider.accessMapData(visitor, {(uint32_t)mapId});
        
        if (!accessSuccess && m_verbose) {
            std::cout << "Direct map access failed, trying alternative methods..." << std::endl;
        }
        
        if (m_verbose) {
            std::cout << "Processed " << mapPointCount << " map point data" << std::endl;
            std::cout << "Read " << keyframeCount << " keyframe data" << std::endl;
            std::cout << "Extracted " << (m_points.size() - initialPointCount) << " new points" << std::endl;
        }
        
        return accessSuccess || (m_points.size() > initialPointCount);
    }
    
public:
    // Constructor
    ColoredPointCloudExtractor(const std::string& deviceIP = "192.168.11.1", bool verbose = true, int syncTimeSeconds = 5)
        : m_deviceIP(deviceIP), m_isConnected(false), m_verbose(verbose), m_syncTimeSeconds(syncTimeSeconds) {
    }
    
    // Destructor
    ~ColoredPointCloudExtractor() {
        disconnect();
    }
    
    // Set verbosity level
    void setVerbose(bool verbose) {
        m_verbose = verbose;
    }
    
    // Set sync time
    void setSyncTime(int seconds) {
        m_syncTimeSeconds = seconds;
    }
    
    // Extract point cloud from STCM file
    bool extractFromSTCM(const std::string& stcmFile) {
        // Clear existing points
        m_points.clear();
        
        if (m_verbose) {
            std::cout << "Reading STCM file directly: " << stcmFile << std::endl;
        }
        
        // Open STCM file for direct reading
        std::ifstream file(stcmFile, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Failed to open STCM file: " << stcmFile << std::endl;
            return false;
        }
        
        // Get file size
        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);
        
        if (m_verbose) {
            std::cout << "STCM file size: " << fileSize / 1024 << " KB" << std::endl;
        }
        
        // Read STCM header (first 16 bytes)
        const size_t headerSize = 16;
        std::vector<char> header(headerSize);
        file.read(header.data(), headerSize);
        
        if (file.gcount() != headerSize) {
            std::cerr << "Failed to read STCM header" << std::endl;
            return false;
        }
        
        // Check STCM magic number (STCM)
        if (header[0] != 'S' || header[1] != 'T' || header[2] != 'C' || header[3] != 'M') {
            std::cerr << "Invalid STCM file format" << std::endl;
            return false;
        }
        
        // Read point cloud data section
        // STCM format: after header, search for point cloud data section
        // This is a simplified implementation - actual format may vary
        
        // Skip to point cloud data section (offset may vary)
        const size_t pointCloudSectionOffset = 512; // Example offset
        file.seekg(pointCloudSectionOffset, std::ios::beg);
        
        // Read point count (example implementation)
        uint32_t pointCount = 0;
        file.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
        
        if (pointCount == 0 || pointCount > 10000000) { // Sanity check
            // Try alternative offset
            const size_t alternativeOffset = 1024;
            file.seekg(alternativeOffset, std::ios::beg);
            file.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
            
            if (pointCount == 0 || pointCount > 10000000) {
                std::cerr << "Invalid point count or point cloud section not found" << std::endl;
                
                // Fall back to device-based extraction if available
                if (m_verbose) {
                    std::cout << "Falling back to device-based extraction..." << std::endl;
                }
                
                // Connect to device
                if (!connect()) {
                    return false;
                }
                
                // Enable map data syncing
                m_sdk->controller.setMapDataSyncing(true);
                
                // Wait for map data sync
                if (m_verbose) {
                    std::cout << "Waiting for map data sync (" << m_syncTimeSeconds << " seconds)..." << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::seconds(m_syncTimeSeconds));
                
                // Extract point cloud from all available maps
                bool result = extractAllMaps();
                
                // Apply height-based coloring
                if (result && !m_points.empty()) {
                    applyHeightColoring();
                    
                    // Apply outlier filtering
                    filterOutliers();
                }
                
                // Disconnect from device
                disconnect();
                
                return result;
            }
        }
        
        if (m_verbose) {
            std::cout << "Found point cloud data with " << pointCount << " points" << std::endl;
        }
        
        // Reserve space for points
        m_points.reserve(pointCount);
        
        // Read point data
        for (uint32_t i = 0; i < pointCount; ++i) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), sizeof(float));
            file.read(reinterpret_cast<char*>(&y), sizeof(float));
            file.read(reinterpret_cast<char*>(&z), sizeof(float));
            
            // Add point to collection
            ColoredPoint3D point(x, y, z);
            m_points.push_back(point);
            
            // Update progress periodically
            if (m_verbose && i % 10000 == 0) {
                std::cout << "Reading points: " << (i * 100 / pointCount) << "%\r" << std::flush;
            }
        }
        
        if (m_verbose) {
            std::cout << "Successfully read " << m_points.size() << " points from STCM file" << std::endl;
        }
        
        // Apply height-based coloring
        if (!m_points.empty()) {
            applyHeightColoring();
            
            // Apply outlier filtering
            filterOutliers();
        }
        
        return !m_points.empty();
    }
    
    // Extract point cloud from device (currently active map)
    bool extractFromDevice() {
        // Clear existing points
        m_points.clear();
        
        // Connect to device
        if (!connect()) {
            return false;
        }
        
        // Enable map data syncing
        m_sdk->controller.setMapDataSyncing(true);
        
        // Wait for map data sync
        if (m_verbose) {
            std::cout << "Waiting for map data sync (" << m_syncTimeSeconds << " seconds)..." << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(m_syncTimeSeconds));
        
        // Extract point cloud from all available maps
        bool result = extractAllMaps();
        
        // Apply height-based coloring
        if (result && !m_points.empty()) {
            applyHeightColoring();
            
            // Apply outlier filtering
            filterOutliers();
        }
        
        // Disconnect from device
        disconnect();
        
        return result;
    }
    
    // Export colored point cloud to XYZ file with RGB values
    bool exportToColoredXYZ(const std::string& filename) const {
        if (m_points.empty()) {
            std::cerr << "No point cloud data to export" << std::endl;
            return false;
        }
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot create XYZ file: " << filename << std::endl;
            return false;
        }
        
        // Set output precision
        file << std::fixed << std::setprecision(6);
        
        // Write point cloud data with RGB colors (0-255)
        for (const auto& point : m_points) {
            file << point.x << " " << point.y << " " << point.z << " " 
                 << static_cast<int>(point.r * 255) << " " 
                 << static_cast<int>(point.g * 255) << " " 
                 << static_cast<int>(point.b * 255) << std::endl;
        }
        
        file.close();
        
        if (m_verbose) {
            std::cout << "Successfully exported " << m_points.size() << " colored points to file: " << filename << std::endl;
        }
        
        return true;
    }
    
    // Export colored point cloud to PLY file
    bool exportToColoredPLY(const std::string& filename) const {
        if (m_points.empty()) {
            std::cerr << "No point cloud data to export" << std::endl;
            return false;
        }
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot create PLY file: " << filename << std::endl;
            return false;
        }
        
        // Write PLY header
        file << "ply" << std::endl;
        file << "format ascii 1.0" << std::endl;
        file << "element vertex " << m_points.size() << std::endl;
        file << "property float x" << std::endl;
        file << "property float y" << std::endl;
        file << "property float z" << std::endl;
        file << "property uchar red" << std::endl;
        file << "property uchar green" << std::endl;
        file << "property uchar blue" << std::endl;
        file << "end_header" << std::endl;
        
        // Set output precision
        file << std::fixed << std::setprecision(6);
        
        // Write point cloud data with RGB colors (0-255)
        for (const auto& point : m_points) {
            file << point.x << " " << point.y << " " << point.z << " " 
                 << static_cast<int>(point.r * 255) << " " 
                 << static_cast<int>(point.g * 255) << " " 
                 << static_cast<int>(point.b * 255) << std::endl;
        }
        
        file.close();
        
        if (m_verbose) {
            std::cout << "Successfully exported " << m_points.size() << " colored points to PLY file: " << filename << std::endl;
        }
        
        return true;
    }
    
    // Get point cloud data
    const std::vector<ColoredPoint3D>& getPoints() const {
        return m_points;
    }
    
    // Get point count
    size_t getPointCount() const {
        return m_points.size();
    }
    
    // Main function to process and extract point cloud data
    bool process() {
        if (!connect()) {
            return false;
        }
        
        if (!extractAllMaps()) {
            disconnect();
            return false;
        }
        
        if (m_points.empty()) {
            std::cerr << "No points were extracted" << std::endl;
            disconnect();
            return false;
        }
        
        // Apply height-based coloring
        applyHeightColoring();
        
        // Filter outliers
        filterOutliers();
        
        // Export colored point cloud to XYZ format
        exportToColoredXYZ("output_height_colored.xyz");
        
        // Also export to PLY format for better visualization
        exportToColoredPLY("output_height_colored.ply");
        
        disconnect();
        
        std::cout << "Processing complete! Extracted " << m_points.size() << " colored points." << std::endl;
        
        return true;
    }
};

// Show usage help
void showHelp(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help                Show this help message" << std::endl;
    std::cout << "  -d, --device <IP>         Specify Aurora device IP (default: 192.168.11.1)" << std::endl;
    std::cout << "  -s, --stcm <file>         Specify STCM file path" << std::endl;
    std::cout << "  -o, --output <file>       Specify output file path (default: output_colored.xyz)" << std::endl;
    std::cout << "  -f, --format <xyz|ply>    Specify output format (default: xyz)" << std::endl;
    std::cout << "  -q, --quiet               Run in quiet mode (minimal output)" << std::endl;
    std::cout << "  -t, --sync-time <seconds> Specify map data sync wait time in seconds (default: 5)" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << programName << " -d 192.168.11.1                Extract colored point cloud from device" << std::endl;
    std::cout << "  " << programName << " -s map.stcm -o map.xyz         Extract from STCM file to colored XYZ" << std::endl;
    std::cout << "  " << programName << " -s map.stcm -t 10              Use 10 seconds sync time for better results" << std::endl;
}

int main(int argc, char** argv) {
    // Default parameters
    std::string deviceIP = "192.168.11.1";
    std::string stcmFile = "";
    std::string outputFile = "output_height_colored.xyz";
    std::string outputFormat = "xyz";
    bool verbose = true;
    int syncTimeSeconds = 5;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            showHelp(argv[0]);
            return 0;
        } else if (arg == "-d" || arg == "--device") {
            if (i + 1 < argc) {
                deviceIP = argv[++i];
            }
        } else if (arg == "-s" || arg == "--stcm") {
            if (i + 1 < argc) {
                stcmFile = argv[++i];
            }
        } else if (arg == "-o" || arg == "--output") {
            if (i + 1 < argc) {
                outputFile = argv[++i];
            }
        } else if (arg == "-f" || arg == "--format") {
            if (i + 1 < argc) {
                outputFormat = argv[++i];
                std::transform(outputFormat.begin(), outputFormat.end(), outputFormat.begin(), ::tolower);
            }
        } else if (arg == "-q" || arg == "--quiet") {
            verbose = false;
        } else if (arg == "-t" || arg == "--sync-time") {
            if (i + 1 < argc) {
                syncTimeSeconds = std::stoi(argv[++i]);
            }
        }
    }
    
    // Validate output format
    if (outputFormat != "xyz" && outputFormat != "ply") {
        std::cerr << "Error: Unsupported output format '" << outputFormat << "'. Supported formats: xyz, ply" << std::endl;
        return 1;
    }
    
    // Auto-determine output file extension if not specified
    if (outputFile.find('.') == std::string::npos) {
        outputFile += "." + outputFormat;
    }
    
    if (verbose) {
        std::cout << "======== SLAMTEC Aurora Colored Point Cloud Extractor ========" << std::endl;
        std::cout << "Device IP: " << deviceIP << std::endl;
        if (!stcmFile.empty()) {
            std::cout << "STCM file: " << stcmFile << std::endl;
        }
        std::cout << "Output file: " << outputFile << std::endl;
        std::cout << "Output format: " << outputFormat << std::endl;
        std::cout << "Sync time: " << syncTimeSeconds << " seconds" << std::endl;
        std::cout << "===========================================================" << std::endl;
    }
    
    // Create colored point cloud extractor
    ColoredPointCloudExtractor extractor(deviceIP, verbose, syncTimeSeconds);
    
    bool result = false;
    
    // Extract point cloud data
    if (!stcmFile.empty()) {
        // Extract from STCM file
        if (verbose) {
            std::cout << "Extracting colored point cloud from STCM file..." << std::endl;
        }
        result = extractor.extractFromSTCM(stcmFile);
    } else {
        // Extract from device
        if (verbose) {
            std::cout << "Extracting colored point cloud from device..." << std::endl;
        }
        result = extractor.extractFromDevice();
    }
    
    if (!result) {
        std::cerr << "Failed to extract point cloud data" << std::endl;
        return 1;
    }
    
    // Export colored point cloud
    if (outputFormat == "ply") {
        result = extractor.exportToColoredPLY(outputFile);
    } else {
        result = extractor.exportToColoredXYZ(outputFile);
    }
    
    if (!result) {
        std::cerr << "Failed to export colored point cloud data" << std::endl;
        return 1;
    }
    
    if (verbose) {
        std::cout << "Processing complete! Extracted " << extractor.getPointCount() << " colored points." << std::endl;
    }
    
    return 0;
} 