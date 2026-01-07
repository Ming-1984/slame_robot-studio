#pragma once

#include <string>

namespace rpos { namespace robot_platforms { namespace detail { namespace objects {

    struct LogInfo
    {
        std::string filename;
        float size_mb;
        LogInfo():size_mb(0)
        {}
        LogInfo(const std::string& file, float size):filename(file),size_mb(size)
        {}
    };

    struct FetchLogStatus
    {
        bool isComplete;
        int progress;
        FetchLogStatus():isComplete(false),progress(0)
        {}
    };

    struct FloorIdentify
    {
        std::string building;
        std::string floor;
        int order;
    };

    struct FloorInfo
    {
        std::string building;
        std::string floor;
        std::string elevator;
        std::string map_id;
    };

    enum CameraDataType
    {
        Depth,
        RGB
    };

    struct CameraData
    {
        std::string camera_id;
        std::vector<uint8_t> camera_data;
    };

}}}}