
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "server_map_holder.h"
#include "msg_convert.h"

#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>

namespace slamware_ros_sdk {

    struct ServerWorkData
    {
    public:
        slamware_ros_sdk::msg::RobotDeviceInfo robotDeviceInfo;

        rpos::core::Pose robotPose;

        boost::atomic<bool> syncMapRequested;
        ServerMapHolder exploreMapHolder;

    public:
        ServerWorkData();

    public:
        static inline bool sfIsDigitalSensorValueImpact(float fVal) { return fVal < FLT_EPSILON; }
    };

    typedef boost::shared_ptr<ServerWorkData>               ServerWorkData_Ptr;
    typedef boost::shared_ptr<const ServerWorkData>         ServerWorkData_ConstPtr;
    
}
