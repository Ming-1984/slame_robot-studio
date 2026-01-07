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

class AuroraPointExtractor {
private:
    std::unique_ptr<RemoteSDK> m_sdk;
    std::string m_deviceIP;
    std::vector<ColoredPoint3D> m_points;
    bool m_isConnected;
    bool m_verbose;
    int m_syncTimeSeconds;

public:
    AuroraPointExtractor(const std::string& deviceIP, bool verbose = true, int syncTime = 10) 
        : m_deviceIP(deviceIP), m_isConnected(false), m_verbose(verbose), m_syncTimeSeconds(syncTime) {}

    ~AuroraPointExtractor() {
        disconnect();
    }

    // Connect to Aurora device using direct IP
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

        // Direct IP connection
        SDKServerConnectionDesc directConnection(m_deviceIP.c_str());
        if (!m_sdk->connect(directConnection)) {
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

    // Extract point cloud from device using official method
    bool extractFromDevice() {
        // Clear existing points
        m_points.clear();

        // Connect to device
        if (!connect()) {
            return false;
        }

        // Enable map data syncing (CRITICAL - this was missing!)
        m_sdk->controller.setMapDataSyncing(true);

        // Wait for map data sync
        if (m_verbose) {
            std::cout << "Waiting for map data sync (" << m_syncTimeSeconds << " seconds)..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(m_syncTimeSeconds));

        // Get global mapping info to find active map
        slamtec_aurora_sdk_global_map_desc_t globalMapDesc;
        if (!m_sdk->dataProvider.getGlobalMappingInfo(globalMapDesc)) {
            std::cerr << "Failed to get global mapping info" << std::endl;
            disconnect();
            return false;
        }

        uint32_t activeMapId = globalMapDesc.activeMapID;
        if (m_verbose) {
            std::cout << "Active map ID: " << activeMapId << std::endl;
        }

        // Extract from active map using official method (allow map ID 0!)
        bool result = extractPointCloudFromMap(activeMapId);

        // Also try to extract from all available maps
        std::vector<slamtec_aurora_sdk_map_desc_t> mapDescs;
        if (m_sdk->dataProvider.getAllMapInfo(mapDescs)) {
            if (m_verbose) {
                std::cout << "Found " << mapDescs.size() << " available maps" << std::endl;
            }

            // Extract from all maps (including map ID 0)
            for (const auto& mapDesc : mapDescs) {
                if (mapDesc.map_id != activeMapId) {
                    if (m_verbose) {
                        std::cout << "Extracting from additional map ID: " << mapDesc.map_id << std::endl;
                    }
                    extractPointCloudFromMap(mapDesc.map_id);
                }
            }
        }

        // Apply height-based coloring
        if (!m_points.empty()) {
            applyHeightColoring();
            filterOutliers();
        }

        // Disconnect from device
        disconnect();

        return result || !m_points.empty();
    }

    // Extract point cloud from specific map using official method
    bool extractPointCloudFromMap(uint32_t mapId) {
        if (m_verbose) {
            std::cout << "Extracting from map ID: " << mapId << std::endl;
        }

        // Create map data visitor
        RemoteMapDataVisitor visitor;
        
        // Track counts
        int keyframeCount = 0;
        int mapPointCount = 0;
        size_t initialPointCount = m_points.size();

        // Subscribe to map data callbacks
        visitor.subscribeMapData([&](const slamtec_aurora_sdk_map_desc_t& mapDesc) {
            if (m_verbose) {
                std::cout << "Map ID: " << mapDesc.map_id << std::endl;
                std::cout << "Map point count: " << mapDesc.map_point_count << std::endl;
                std::cout << "Keyframe count: " << mapDesc.keyframe_count << std::endl;
            }
        });

        visitor.subscribeKeyFrameData([&](const RemoteKeyFrameData& keyframeData) {
            keyframeCount++;
            // We could extract pose information here if needed
        });

        visitor.subscribeMapPointData([&](const slamtec_aurora_sdk_map_point_desc_t& mapPointDesc) {
            mapPointCount++;
            
            // Convert map point to colored point
            ColoredPoint3D point;
            point.x = mapPointDesc.position.x;
            point.y = mapPointDesc.position.y;
            point.z = mapPointDesc.position.z;
            
            // Default color (will be overridden by height coloring)
            point.r = 0.5f;
            point.g = 0.5f;
            point.b = 0.5f;
            
            m_points.push_back(point);
        });

        // Access map data for specific map ID
        bool accessSuccess = m_sdk->dataProvider.accessMapData(visitor, {mapId});

        if (m_verbose) {
            std::cout << "Processed " << mapPointCount << " map points" << std::endl;
            std::cout << "Read " << keyframeCount << " keyframes" << std::endl;
            std::cout << "Extracted " << (m_points.size() - initialPointCount) << " new points" << std::endl;
        }

        return accessSuccess || (m_points.size() > initialPointCount);
    }

    // Apply height-based coloring
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

    // Convert height to color (blue=low, green=mid, red=high)
    void heightToColor(float height, float minHeight, float maxHeight, float& r, float& g, float& b) {
        if (maxHeight <= minHeight) {
            r = g = b = 0.5f;
            return;
        }

        float normalized = (height - minHeight) / (maxHeight - minHeight);
        normalized = std::max(0.0f, std::min(1.0f, normalized));

        if (normalized < 0.5f) {
            // Blue to Green
            float t = normalized * 2.0f;
            r = 0.0f;
            g = t;
            b = 1.0f - t;
        } else {
            // Green to Red
            float t = (normalized - 0.5f) * 2.0f;
            r = t;
            g = 1.0f - t;
            b = 0.0f;
        }
    }

    // Filter outliers
    void filterOutliers() {
        if (m_points.size() < 10) {
            return;
        }

        size_t originalSize = m_points.size();
        
        // Simple statistical outlier removal based on distance from centroid
        // Calculate centroid
        float cx = 0, cy = 0, cz = 0;
        for (const auto& p : m_points) {
            cx += p.x;
            cy += p.y;
            cz += p.z;
        }
        cx /= m_points.size();
        cy /= m_points.size();
        cz /= m_points.size();

        // Calculate distances and find threshold
        std::vector<float> distances;
        distances.reserve(m_points.size());
        
        for (const auto& p : m_points) {
            float dx = p.x - cx;
            float dy = p.y - cy;
            float dz = p.z - cz;
            distances.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
        }

        std::sort(distances.begin(), distances.end());
        float threshold = distances[static_cast<size_t>(distances.size() * 0.95)]; // 95th percentile

        // Remove outliers
        auto it = std::remove_if(m_points.begin(), m_points.end(), 
            [cx, cy, cz, threshold](const ColoredPoint3D& p) {
                float dx = p.x - cx;
                float dy = p.y - cy;
                float dz = p.z - cz;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                return dist > threshold;
            });
        
        m_points.erase(it, m_points.end());

        if (m_verbose) {
            std::cout << "Filtered out " << (originalSize - m_points.size()) << " outliers" << std::endl;
            std::cout << "Remaining points: " << m_points.size() << std::endl;
        }
    }

    // Export to XYZ format
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

        file << std::fixed << std::setprecision(6);

        for (const auto& point : m_points) {
            file << point.x << " " << point.y << " " << point.z << " "
                 << static_cast<int>(point.r * 255) << " "
                 << static_cast<int>(point.g * 255) << " "
                 << static_cast<int>(point.b * 255) << std::endl;
        }

        file.close();
        return true;
    }

    // Export to PLY format
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

        // Write point data
        file << std::fixed << std::setprecision(6);
        for (const auto& point : m_points) {
            file << point.x << " " << point.y << " " << point.z << " "
                 << static_cast<int>(point.r * 255) << " "
                 << static_cast<int>(point.g * 255) << " "
                 << static_cast<int>(point.b * 255) << std::endl;
        }

        file.close();
        return true;
    }

    size_t getPointCount() const {
        return m_points.size();
    }
};

// Show help information
void showHelp(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -d <device_ip>    Aurora device IP address (default: 192.168.11.1)" << std::endl;
    std::cout << "  -o <output_file>  Output file path" << std::endl;
    std::cout << "  -f <format>       Output format: xyz or ply (default: xyz)" << std::endl;
    std::cout << "  -t <seconds>      Sync time in seconds (default: 10)" << std::endl;
    std::cout << "  -v                Verbose output" << std::endl;
    std::cout << "  -h                Show this help" << std::endl;
}

int main(int argc, char** argv) {
    std::string deviceIP = "192.168.11.1";
    std::string outputFile;
    std::string outputFormat = "xyz";
    int syncTime = 10;
    bool verbose = false;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-d" && i + 1 < argc) {
            deviceIP = argv[++i];
        } else if (std::string(argv[i]) == "-o" && i + 1 < argc) {
            outputFile = argv[++i];
        } else if (std::string(argv[i]) == "-f" && i + 1 < argc) {
            outputFormat = argv[++i];
        } else if (std::string(argv[i]) == "-t" && i + 1 < argc) {
            syncTime = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "-v") {
            verbose = true;
        } else if (std::string(argv[i]) == "-h") {
            showHelp(argv[0]);
            return 0;
        }
    }

    // Check required parameters
    if (outputFile.empty()) {
        std::cerr << "Error: Output file must be specified with -o option" << std::endl;
        showHelp(argv[0]);
        return 1;
    }

    // Validate format
    if (outputFormat != "xyz" && outputFormat != "ply") {
        std::cerr << "Error: Invalid format. Use 'xyz' or 'ply'" << std::endl;
        return 1;
    }

    std::cout << "======== SLAMTEC Aurora Colored Point Cloud Extractor (Fixed) ========" << std::endl;
    std::cout << "Device IP: " << deviceIP << std::endl;
    std::cout << "Output file: " << outputFile << std::endl;
    std::cout << "Output format: " << outputFormat << std::endl;
    std::cout << "Sync time: " << syncTime << " seconds" << std::endl;
    std::cout << "===========================================================" << std::endl;

    // Create extractor
    AuroraPointExtractor extractor(deviceIP, verbose, syncTime);

    // Extract point cloud from device
    std::cout << "Extracting colored point cloud from device..." << std::endl;
    bool result = extractor.extractFromDevice();

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
