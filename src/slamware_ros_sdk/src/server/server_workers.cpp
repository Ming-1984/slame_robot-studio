#include "server_workers.h"
#include "slamware_ros_sdk_server.h"

#include <boost/assert.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
using namespace rp::standalone::aurora;
namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    ServerRobotDeviceInfoWorker::ServerRobotDeviceInfoWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
    {
        auto nhRos = rosNodeHandle();
        pubRobotDeviceInfo_ = nhRos->create_publisher<slamware_ros_sdk::msg::RobotDeviceInfo>("/slamware_ros_sdk_server_node/robot_device_info", 1);
    }

    ServerRobotDeviceInfoWorker::~ServerRobotDeviceInfoWorker()
    {
        //
    }

    bool ServerRobotDeviceInfoWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;
        isWorkLoopInitOk_ = false;

        auto wkDat = mutableWorkData();
        auto& msgRobotDeviceInfo = wkDat->robotDeviceInfo;
        auto pltfm = rosSdkServer()->safeGetSlamwarePlatform();
        const auto devInfo = pltfm.getDeviceInfo();
        msgRobotDeviceInfo.device_id = devInfo.deviceID();
        msgRobotDeviceInfo.model_id = devInfo.modelID();
        msgRobotDeviceInfo.model_name = devInfo.modelName();
        msgRobotDeviceInfo.manufacturer_id = devInfo.manufacturerID();
        msgRobotDeviceInfo.manufacturer_name = devInfo.manufacturerName();
        msgRobotDeviceInfo.hardware_version = devInfo.hardwareVersion();
        msgRobotDeviceInfo.software_version = devInfo.softwareVersion();

        msgRobotDeviceInfo.sdp_version = pltfm.getSDPVersion();
        msgRobotDeviceInfo.sdk_version = pltfm.getSDKVersion();

        pubRobotDeviceInfo_->publish(msgRobotDeviceInfo);
        RCLCPP_INFO(rclcpp::get_logger("server workers"), "device_id: %s, hardware_version: %s, software_version: %s, sdp_version: %s, sdk_version: %s."
            , msgRobotDeviceInfo.device_id.c_str()
            , msgRobotDeviceInfo.hardware_version.c_str()
            , msgRobotDeviceInfo.software_version.c_str()
            , msgRobotDeviceInfo.sdp_version.c_str()
            , msgRobotDeviceInfo.sdk_version.c_str()
            );
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerRobotDeviceInfoWorker::doPerform()
    {
        // do nothing
    }

    //////////////////////////////////////////////////////////////////////////

     ServerOdometryWorker::ServerOdometryWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , lastPose_(rpos::core::Pose())
        , firstPoseReceived_(false)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubOdometry_ = nhRos->create_publisher<nav_msgs::msg::Odometry>(srvParams.getParameter<std::string>(std::string("odom_topic")), 10);
    }

    ServerOdometryWorker::~ServerOdometryWorker()
    {
        //
    }

    bool ServerOdometryWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;
        firstPoseReceived_ = false;
        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerOdometryWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        rclcpp::Time currentTime = rclcpp::Clock().now();
        auto wkDat = mutableWorkData();
        auto currentPose = wkDat->robotPose;

        if(!firstPoseReceived_) {
            lastPose_ = currentPose;
            lastPoseTime_ = currentTime;
            firstPoseReceived_ = true;
            return;
        }

        float deltaX = currentPose.x() - lastPose_.x();
        float deltaY = currentPose.y() - lastPose_.y();
        double deltaYaw = currentPose.yaw() - lastPose_.yaw();

        rclcpp::Duration dt = currentTime - lastPoseTime_;
        double vx = deltaX / dt.seconds();
        double vy = deltaY / dt.seconds();
        double vth = deltaYaw / dt.seconds();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = currentTime;
        odom.header.frame_id = srvParams.getParameter<std::string>("odom_frame");
        odom.child_frame_id = srvParams.getParameter<std::string>("robot_frame");

        sltcToRosMsg(currentPose, odom.pose.pose);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        pubOdometry_->publish(odom);

        auto &tfBrdcst = tfBroadcaster();
        geometry_msgs::msg::TransformStamped odomTrans;
        odomTrans.header.stamp = currentTime;
        odomTrans.header.frame_id = srvParams.getParameter<std::string>("odom_frame");
        odomTrans.child_frame_id = srvParams.getParameter<std::string>("robot_frame");
        odomTrans.transform.translation.x = currentPose.x();
        odomTrans.transform.translation.y = currentPose.y();
        odomTrans.transform.translation.z = currentPose.z();
        sltcToRosMsg(currentPose.rotation(), odomTrans.transform.rotation);
        tfBrdcst->sendTransform(odomTrans);

        lastPose_ = currentPose;
        lastPoseTime_ = currentTime;
    }

    //////////////////////////////////////////////////////////////////////////

    ServerRobotPoseWorker::ServerRobotPoseWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubRobotPose_ = nhRos->create_publisher<geometry_msgs::msg::PoseStamped>(srvParams.getParameter<std::string>(std::string("robot_pose_topic")), 10);
    }

    ServerRobotPoseWorker::~ServerRobotPoseWorker()
    {
        //
    }

    void ServerRobotPoseWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = mutableWorkData();
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "doPerform(), auroraSDK is null.");
            return;
        }

        slamtec_aurora_sdk_pose_t pose;
        auroraSDK->dataProvider.getCurrentPose(pose);
        //current pose from opencv axis to ros2 axis
        wkDat->robotPose = rpos::core::Pose(rpos::core::Location(pose.translation.x, pose.translation.y, pose.translation.z),
             rpos::core::Rotation(pose.rpy.yaw, pose.rpy.pitch, pose.rpy.roll));
        tf2::Transform vslam_pose;
        vslam_pose.setOrigin(tf2::Vector3(pose.translation.x, pose.translation.y, pose.translation.z));
        tf2::Quaternion ros_quat;
        ros_quat.setRPY(pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw);
        vslam_pose.setRotation(ros_quat);

        // publish robot_pose transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = rclcpp::Clock().now();
        t.header.frame_id = srvParams.getParameter<std::string>("map_frame");
        t.child_frame_id = srvParams.getParameter<std::string>("robot_frame");
        t.transform.translation.x = vslam_pose.getOrigin().x();
        t.transform.translation.y = vslam_pose.getOrigin().y();
        t.transform.translation.z = vslam_pose.getOrigin().z();

        t.transform.rotation.x = vslam_pose.getRotation().x();
        t.transform.rotation.y = vslam_pose.getRotation().y();
        t.transform.rotation.z = vslam_pose.getRotation().z();
        t.transform.rotation.w = vslam_pose.getRotation().w();
        tfBrdcst->sendTransform(t);

        geometry_msgs::msg::PoseStamped ros_pose;
        ros_pose.header.frame_id = srvParams.getParameter<std::string>("map_frame");
        ros_pose.header.stamp = rclcpp::Clock().now();
        ros_pose.pose.position.x = vslam_pose.getOrigin().x();
        ros_pose.pose.position.y = vslam_pose.getOrigin().y();
        ros_pose.pose.position.z = vslam_pose.getOrigin().z();
        ros_pose.pose.orientation.x = vslam_pose.getRotation().x();
        ros_pose.pose.orientation.y = vslam_pose.getRotation().y();
        ros_pose.pose.orientation.z = vslam_pose.getRotation().z();
        ros_pose.pose.orientation.w = vslam_pose.getRotation().w();
        pubRobotPose_->publish(ros_pose);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapUpdateWorker::ServerExploreMapUpdateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : super_t(pRosSdkServer, wkName, triggerInterval)
        , shouldReinitMap_(true)
        , hasSyncedWholeMap_(false)
    {
        //
    }

    ServerExploreMapUpdateWorker::~ServerExploreMapUpdateWorker()
    {
        //
    }

    bool ServerExploreMapUpdateWorker::reinitWorkLoop()
    {
        
        if (!this->super_t::reinitWorkLoop())
            return false;
        isWorkLoopInitOk_ = false;
        hasSyncedWholeMap_ = false;

        requestReinitMap_();

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerExploreMapUpdateWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto wkDat = mutableWorkData();

        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK) {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get aurora sdk");
            return;
        }

        if (!checkToReinitMap_(auroraSDK, wkDat))
            return;

        if(!hasSyncedWholeMap_) {
            rclcpp::Duration dt = rclcpp::Clock().now() - mapInitTime_;
            if(dt.seconds() > 10) {
                wkDat->syncMapRequested.store(true);
                hasSyncedWholeMap_ = true;
                RCLCPP_INFO(rclcpp::get_logger("server workers"), "sync the map after preview map generated.");
            }
        }

        if (wkDat->syncMapRequested.load())
        {
            RCLCPP_INFO(rclcpp::get_logger("server workers"), "try to sync whold explore map.");
            if (syncWholeMap_(srvParams, auroraSDK, wkDat))
            {
                wkDat->syncMapRequested.store(false);
                RCLCPP_INFO(rclcpp::get_logger("server workers"), "whold explore map synchronized.");
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("server workers"), "failed to sync whole explore map.");
            }
            return;
        }

        updateMapNearRobot_(srvParams, auroraSDK, wkDat);
    }

    void ServerExploreMapUpdateWorker::requestReinitMap_()
    {
        shouldReinitMap_ = true;
    }

    bool ServerExploreMapUpdateWorker::checkToReinitMap_(rp::standalone::aurora::RemoteSDK *sdk, const ServerWorkData_Ptr &wkDat)
    {
        if (!shouldReinitMap_)
            return true;

        RCLCPP_INFO(rclcpp::get_logger("server workers"), "try to reinit explore map.");
        wkDat->syncMapRequested.store(true);

        sdk->lidar2DMapBuilder.requireRedrawPreviewMap();
        LIDAR2DGridMapGenerationOptions genOption;
        sdk->lidar2DMapBuilder.getPreviewMapGenerationOptions(genOption);
        wkDat->exploreMapHolder.reinit(genOption.resolution);

        RCLCPP_INFO(
            rclcpp::get_logger("server workers"),
            "explore map initialized, resolution: %f, moreCellCntToExtend: %d.",
            genOption.resolution,
            wkDat->exploreMapHolder.getMoreCellCountToExtend());
        shouldReinitMap_ = false;
        mapInitTime_ = rclcpp::Clock().now();
        return true;
    }

    bool ServerExploreMapUpdateWorker::checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr& wkDat)
    {
        const auto fResolution = wkDat->exploreMapHolder.resolution();
        if (rpos::system::types::fequal(fResolution, recvResolution))
            return true;

        RCLCPP_ERROR(rclcpp::get_logger("server workers"), "local resolution: %f, received resolution: %f, request reinit.", fResolution, recvResolution);
        requestReinitMap_();
        return false;
    }

    bool ServerExploreMapUpdateWorker::updateMapInCellIdxRect_(const rp::standalone::aurora::OccupancyGridMap2DRef& prevMap
            , const rpos::core::RectangleF& reqRect
            , const ServerWorkData_Ptr& wkDat
            )
    {
        const auto& fResolution = prevMap.getResolution();
        
        static const size_t MAX_FETCH_BLOCK_CXCY = 16 * 1024; // 16*16 to get 256 MB
        size_t fetchSize = (size_t)(reqRect.width() * reqRect.height() / fResolution / fResolution);
        if (fetchSize > (MAX_FETCH_BLOCK_CXCY * MAX_FETCH_BLOCK_CXCY)) {
            // using smaller block to fetch
            float fetchLogicCXCY = MAX_FETCH_BLOCK_CXCY * fResolution;
            for (float y = 0; y < reqRect.height(); y += fetchLogicCXCY) {
                for (float x = 0; x < reqRect.width(); x += fetchLogicCXCY) {
                    slamtec_aurora_sdk_rect_t fetchRect;
                    fetchRect.x = x + reqRect.x();
                    fetchRect.y = y + reqRect.y();
                    fetchRect.width = fetchLogicCXCY;
                    fetchRect.height = fetchLogicCXCY;
                    std::vector<uint8_t> mapData;
                    slamtec_aurora_sdk_2d_gridmap_fetch_info_t info;
                    prevMap.readCellData(fetchRect, info, mapData, false);
                    wkDat->exploreMapHolder.setMapData(info.real_x, info.real_y, fResolution, info.cell_width, info.cell_height, mapData.data());
                }
            }
        }
        else {
            slamtec_aurora_sdk_rect_t fetchRect;
            fetchRect.x = reqRect.x();
            fetchRect.y = reqRect.y();
            fetchRect.width = reqRect.width();
            fetchRect.height = reqRect.height();
            std::vector<uint8_t> mapData;
            slamtec_aurora_sdk_2d_gridmap_fetch_info_t info;
            prevMap.readCellData(fetchRect, info, mapData, false);
            wkDat->exploreMapHolder.setMapData(info.real_x, info.real_y, fResolution, info.cell_width, info.cell_height, mapData.data());
        }
        
        if (!checkRecvResolution_(fResolution, wkDat))
            return false;

        return true;
    }

    bool ServerExploreMapUpdateWorker::syncWholeMap_(const ServerParams& srvParams
        , rp::standalone::aurora::RemoteSDK *sdk
        , const ServerWorkData_Ptr& wkDat
        )
    {
        auto &prevMap = sdk->lidar2DMapBuilder.getPreviewMap();

        slamtec_aurora_sdk_2dmap_dimension_t mapDimension;
        prevMap.getMapDimension(mapDimension);
        rpos::core::RectangleF knownArea(mapDimension.min_x, mapDimension.min_y,
            mapDimension.max_x - mapDimension.min_x,
            mapDimension.max_y - mapDimension.min_y);

        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
        RCLCPP_INFO(rclcpp::get_logger("server workers"), "known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))"
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "sync map, knownCellIdxRect is empty.");
            return false;
        }

        wkDat->exploreMapHolder.clear();
        wkDat->exploreMapHolder.reserveByCellIdxRect(knownCellIdxRect);

        return updateMapInCellIdxRect_(prevMap, knownArea, wkDat);
    }

    bool ServerExploreMapUpdateWorker::updateMapNearRobot_(const ServerParams& srvParams
            , rp::standalone::aurora::RemoteSDK *sdk
            , const ServerWorkData_Ptr& wkDat
            )
    {
        float fVal;
        if (srvParams.get_parameter<float>("map_update_near_robot_half_wh", fVal)) {

        }
        const float fHalfWH = std::max<float>(2.0f, fVal);
        const float fWH = fHalfWH + fHalfWH;
        const auto nearRobotArea = rpos::core::RectangleF(static_cast<float>(wkDat->robotPose.x() - fHalfWH)
            , static_cast<float>(wkDat->robotPose.y() - fHalfWH)
            , fWH
            , fWH
            );
        const auto nearRobotIdxRect = wkDat->exploreMapHolder.calcMinBoundingCellIdxRect(nearRobotArea);

        auto &prevMap = sdk->lidar2DMapBuilder.getPreviewMap();
        slamtec_aurora_sdk_2dmap_dimension_t mapDimension;
        prevMap.getMapDimension(mapDimension);
        rpos::core::RectangleF knownArea(mapDimension.min_x, mapDimension.min_y,
            mapDimension.max_x - mapDimension.min_x,
            mapDimension.max_y - mapDimension.min_y);
        const auto knownCellIdxRect = wkDat->exploreMapHolder.calcRoundedCellIdxRect(knownArea);
    #if 0
        RCLCPP_INFO(rclcpp::get_logger("server workers"), "known area: ((%f, %f), (%f, %f)), cell rect: ((%d, %d), (%d, %d))."
            , knownArea.x(), knownArea.y(), knownArea.width(), knownArea.height()
            , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
            );
    #endif
        if (ServerMapHolder::sfIsCellIdxRectEmpty(knownCellIdxRect))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "update map, knownCellIdxRect is empty, request sync map.");
            rosSdkServer()->requestSyncMap();
            return false;
        }

        const auto reqIdxRect = ServerMapHolder::sfIntersectionOfCellIdxRect(nearRobotIdxRect, knownCellIdxRect);
        if (ServerMapHolder::sfIsCellIdxRectEmpty(reqIdxRect))
        {
            RCLCPP_WARN(rclcpp::get_logger("server workers"), "knownCellIdxRect: ((%d, %d), (%d, %d)), nearRobotIdxRect: ((%d, %d), (%d, %d)), intersection is empty."
                , knownCellIdxRect.x(), knownCellIdxRect.y(), knownCellIdxRect.width(), knownCellIdxRect.height()
                , nearRobotIdxRect.x(), nearRobotIdxRect.y(), nearRobotIdxRect.width(), nearRobotIdxRect.height()
                );
            return false;
        }
        const auto& reqArea = wkDat->exploreMapHolder.calcAreaByCellIdxRect(reqIdxRect);

        return updateMapInCellIdxRect_(prevMap, reqArea, wkDat);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerExploreMapPublishWorker::ServerExploreMapPublishWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubMapDat_ = nhRos->create_publisher<nav_msgs::msg::OccupancyGrid>(srvParams.getParameter<std::string>("map_topic"), 1); // srvParams.map_topic, 1);
        pubMapInfo_ = nhRos->create_publisher<nav_msgs::msg::MapMetaData>(srvParams.getParameter<std::string>("map_info_topic"), 1);// srvParams.map_info_topic, 1);
    }

    ServerExploreMapPublishWorker::~ServerExploreMapPublishWorker()
    {
        //
    }

    void ServerExploreMapPublishWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto wkDat = workData();

        if (wkDat->exploreMapHolder.isMapDataEmpty())
        {
            //RCLCPP_WARN(rclcpp::get_logger("server workers"), "current explore map data is empty.");
            return;
        }

        nav_msgs::srv::GetMap::Response msgMap;
        wkDat->exploreMapHolder.fillRosMapMsg(msgMap);

        // Set the header information on the map
        msgMap.map.header.stamp = rclcpp::Clock().now();
        msgMap.map.header.frame_id = srvParams.getParameter<std::string>("map_frame"); //srvParams.map_frame;

        pubMapDat_->publish(msgMap.map);
        pubMapInfo_->publish(msgMap.map.info);
    }

    //////////////////////////////////////////////////////////////////////////

    ServerLaserScanWorker::ServerLaserScanWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , compensatedAngleCnt_(720u)
        , absAngleIncrement_(C_FLT_2PI / compensatedAngleCnt_)
        , latestLidarStartTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLaserScan_ = nhRos->create_publisher<sensor_msgs::msg::LaserScan>(srvParams.getParameter<std::string>("scan_topic"), 10); // srvParams.scan_topic, 10);
    }

    ServerLaserScanWorker::~ServerLaserScanWorker()
    {
        //
    }

    void ServerLaserScanWorker::doPerform()
    {
        const auto& srvParams = serverParams();
        auto& tfBrdcst = tfBroadcaster();
        auto wkDat = workData();

        auto pltfm = rosSdkServer()->safeGetSlamwarePlatform();
        rclcpp::Time startScanTime(0), endScanTime(0);
        rpos::features::system_resource::LaserScan tLs;
        if (!srvParams.getParameter<bool>("raw_ladar_data"))
        {
            startScanTime = rclcpp::Clock().now();
            tLs = pltfm.getLaserScan();
            endScanTime = rclcpp::Clock().now();
        }
        else
        {
            tLs = pltfm.getRawLaserScan();
            std::uint64_t lidarStartTimestamp = tLs.getStartTimestamp();
            if (latestLidarStartTimestamp_ == lidarStartTimestamp)
            {
                // do not send same lidar date
                return;
            }
            else
            {
                latestLidarStartTimestamp_ = lidarStartTimestamp;
            }
            startScanTime = rclcpp::Time(lidarStartTimestamp * 1000);
            endScanTime = rclcpp::Time(tLs.getEndTimestamp() * 1000);
        }
        double dblScanDur = (endScanTime - startScanTime).seconds();

        const auto& points = tLs.getLaserPoints();
        if (points.size() < 2)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "laser points count: %u, too small, skip publish.", (unsigned int)points.size());
            return;
        }

        const auto laserPose = wkDat->robotPose;
        //RCLCPP_INFO(rclcpp::get_logger("server workers"), "has laser pose: %s, robotPose: ((%f, %f), (%f)), laserPose: ((%f, %f), (%f))."
        //    , (tLs.getHasPose() ? "true" : "false")
        //    , wkDat->robotPose.x(), wkDat->robotPose.y(), wkDat->robotPose.yaw()
        //    , laserPose.x(), laserPose.y(), laserPose.yaw()
        //    );

        sensor_msgs::msg::LaserScan msgScan;
        msgScan.header.stamp = startScanTime;
        msgScan.header.frame_id = srvParams.getParameter<std::string>("laser_frame"); //srvParams.laser_frame;
        fillRangeMinMaxInMsg_(points, msgScan);

        bool isLaserDataReverse = false;
        if ((points.back().angle() < points.front().angle() && !srvParams.getParameter<bool>("ladar_data_clockwise")) ||
            (points.back().angle() > points.front().angle() && srvParams.getParameter<bool>("ladar_data_clockwise")))
        {
            isLaserDataReverse = true;
        }

        // if (srvParams.angle_compensate)
        if (srvParams.getParameter<bool>("angle_compensate"))
        {
            compensateAndfillRangesInMsg_(points, srvParams.getParameter<bool>("ladar_data_clockwise"), isLaserDataReverse, msgScan);
        }
        else
        {
            fillOriginalRangesInMsg_(points, isLaserDataReverse, msgScan);
        }
        BOOST_ASSERT(2 <= msgScan.ranges.size());
        msgScan.scan_time = dblScanDur;
        msgScan.time_increment = dblScanDur / (double)(msgScan.ranges.size() - 1);

        pubLaserScan_->publish(msgScan);
    }

    void ServerLaserScanWorker::fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        msgScan.range_min = std::numeric_limits<float>::infinity();
        msgScan.range_max = 0.0f;
        for (auto cit = laserPoints.cbegin(), citEnd = laserPoints.cend(); citEnd != cit; ++cit)
        {
            if (cit->valid())
            {
                const float tmpDist = cit->distance();
                
                if (tmpDist < msgScan.range_min)
                {
                    msgScan.range_min = std::max<float>(0.0f, tmpDist);
                }

                if (msgScan.range_max < tmpDist)
                {
                    msgScan.range_max = tmpDist;
                }
            }
        }
    }

    float ServerLaserScanWorker::calcAngleInNegativePiToPi_(float angle) const
    {
        float fRes = std::fmod(angle + C_FLT_PI, C_FLT_2PI);
        if (fRes < 0.0f)
            fRes += C_FLT_2PI;
        fRes -= C_FLT_PI;

        if (fRes < -C_FLT_PI)
            fRes = -C_FLT_PI;
        else if (C_FLT_PI <= fRes)
            fRes = -C_FLT_PI;
        return fRes;
    }

    std::uint32_t ServerLaserScanWorker::calcCompensateDestIndexBySrcAngle_(float srcAngle
        , bool isAnglesReverse
        ) const
    {
        BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        
        float fDiff = (isAnglesReverse ? (C_FLT_PI - srcAngle) : (srcAngle + C_FLT_PI));
        fDiff = std::max<float>(0.0f, fDiff);
        fDiff = std::min<float>(fDiff, C_FLT_2PI);

        std::uint32_t destIdx = static_cast<std::uint32_t>(std::round(fDiff / absAngleIncrement_));
        if (compensatedAngleCnt_ <= destIdx)
            destIdx = 0;
        return destIdx;
    }

    bool ServerLaserScanWorker::isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const
    {
        BOOST_ASSERT(-C_FLT_PI <= srcAngle && srcAngle < C_FLT_PI);
        BOOST_ASSERT(-C_FLT_PI <= destAngle && destAngle < C_FLT_PI);
        BOOST_ASSERT(-C_FLT_PI <= oldSrcAngle && oldSrcAngle < C_FLT_PI);

        float newDiff = std::abs(destAngle - srcAngle);
        if (C_FLT_2PI <= newDiff)
            newDiff = 0.0f;
        else if (C_FLT_PI < newDiff)
            newDiff = C_FLT_2PI - newDiff;

        float oldDiff = std::abs(destAngle - oldSrcAngle);
        if (C_FLT_2PI <= oldDiff)
            oldDiff = 0.0f;
        else if (C_FLT_PI < oldDiff)
            oldDiff = C_FLT_2PI - oldDiff;

        return (newDiff < oldDiff);
    }

    void ServerLaserScanWorker::compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , bool isClockwise
            , bool isLaserDataReverse
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        BOOST_ASSERT(2 <= laserPoints.size());

        msgScan.ranges.clear();
        msgScan.intensities.clear();
        msgScan.ranges.resize(compensatedAngleCnt_, std::numeric_limits<float>::infinity());
        msgScan.intensities.resize(compensatedAngleCnt_, 0.0);

        if (!isClockwise)
        {
            msgScan.angle_min = -C_FLT_PI;
            msgScan.angle_max = C_FLT_PI - absAngleIncrement_;
            msgScan.angle_increment = absAngleIncrement_;
        }
        else
        {
            msgScan.angle_min = C_FLT_PI;
            msgScan.angle_max = (-C_FLT_PI + absAngleIncrement_); 
            msgScan.angle_increment = -absAngleIncrement_;
        }

        std::vector<float> tmpSrcAngles(compensatedAngleCnt_);
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                compensateAndfillRangeInMsg_(laserPoints[i], isClockwise, msgScan, tmpSrcAngles);
            }
        }
    }

    void ServerLaserScanWorker::compensateAndfillRangeInMsg_(const rpos::core::LaserPoint& laserPoint
            , bool isClockwise
            , sensor_msgs::msg::LaserScan& msgScan
            , std::vector<float>& tmpSrcAngles
            ) const
    {
        if (laserPoint.valid())
        {
            const float srcAngle = calcAngleInNegativePiToPi_(laserPoint.angle());
            const std::uint32_t destIdx = calcCompensateDestIndexBySrcAngle_(srcAngle, isClockwise);
            BOOST_ASSERT(destIdx < compensatedAngleCnt_);
            const float destAngle = calcAngleInNegativePiToPi_(msgScan.angle_min + msgScan.angle_increment * destIdx);

            const bool shouldWrite = (std::isinf(msgScan.ranges[destIdx])
                || isSrcAngleMoreCloseThanOldSrcAngle_(srcAngle, destAngle, tmpSrcAngles[destIdx])
                );
            if (shouldWrite)
            {
                msgScan.ranges[destIdx] = laserPoint.distance();
                msgScan.intensities[destIdx] = laserPoint.quality();
                tmpSrcAngles[destIdx] = srcAngle;
            }
        }
    }

    void ServerLaserScanWorker::fillOriginalRangesInMsg_(const std::vector<rpos::core::LaserPoint>& laserPoints
            , bool isLaserDataReverse
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        msgScan.intensities.resize(laserPoints.size());
        msgScan.ranges.resize(laserPoints.size());

        int index = 0;
        if (!isLaserDataReverse)
        {
            for (int i = 0; i < laserPoints.size(); ++i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min =  laserPoints.front().angle();
            msgScan.angle_max =  laserPoints.back().angle();
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
        else
        {
            for (int i = laserPoints.size() - 1; i >= 0; --i)
            {
                fillOriginalRangeInMsg_(laserPoints[i], index++, msgScan);
            }
            msgScan.angle_min =  laserPoints.back().angle();
            msgScan.angle_max =  laserPoints.front().angle();
            msgScan.angle_increment = (msgScan.angle_max - msgScan.angle_min) / (double)(msgScan.ranges.size() - 1);
        }
    }

    void ServerLaserScanWorker::fillOriginalRangeInMsg_(const rpos::core::LaserPoint& laserPoint
            , int index
            , sensor_msgs::msg::LaserScan& msgScan
            ) const
    {
        if (!laserPoint.valid())
        {
            msgScan.ranges[index] = std::numeric_limits<float>::infinity();
            msgScan.intensities[index] = 0.0;
        }
        else
        {
            msgScan.ranges[index] = laserPoint.distance();
            msgScan.intensities[index] = laserPoint.quality();
        }
    }

    ServerRobotBasicStateWorker::ServerRobotBasicStateWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        auto nhRos = rosNodeHandle();
        pubRobotBasicState_ = nhRos->create_publisher<slamware_ros_sdk::msg::RobotBasicState>("/slamware_ros_sdk_server_node/robot_basic_state", 1);
    }

    ServerRobotBasicStateWorker::~ServerRobotBasicStateWorker()
    {
        //
    }

    void ServerRobotBasicStateWorker::doPerform()
    {
        slamware_ros_sdk::msg::RobotBasicState msgRobotBasicState;
        auto pltfm = rosSdkServer()->safeGetSlamwarePlatform();
        msgRobotBasicState.is_map_building_enabled = pltfm.getMapUpdate(rpos::features::location_provider::EXPLORERMAP);
        msgRobotBasicState.is_localization_enabled = pltfm.getMapLocalization();
        
        msgRobotBasicState.localization_quality = pltfm.getLocalizationQuality();

        msgRobotBasicState.board_temperature = pltfm.getBoardTemperature();

        const auto pwrStatus = pltfm.getPowerStatus();
        msgRobotBasicState.battery_percentage = pwrStatus.batteryPercentage;
        msgRobotBasicState.is_dc_in = pwrStatus.isDCConnected;
        msgRobotBasicState.is_charging = pwrStatus.isCharging;

        pubRobotBasicState_->publish(msgRobotBasicState);
    }
    
    ServerReadEventsWorker::ServerReadEventsWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
       
    }

    ServerReadEventsWorker::~ServerReadEventsWorker()
    {
        //
    }

    bool ServerReadEventsWorker::reinitWorkLoop()
    {
        if (!this->super_t::reinitWorkLoop())
            return false;
        isWorkLoopInitOk_ = false;

        auto pltfm = rosSdkServer()->safeGetSlamwarePlatform();
        systemEventProvider_ = pltfm.createSystemEventProvider();
        if (!systemEventProvider_)
        {
            RCLCPP_WARN(rclcpp::get_logger("server workers"), "read events get event provider failed.");
            return isWorkLoopInitOk_;
        }

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void ServerReadEventsWorker::doPerform()
    {
        std::vector<rpos::core::DiagnosisInfoInternalSystemEvent> events;
        systemEventProvider_->readEvents(events);

        for (size_t i = 0; i < events.size(); ++i)
        {
            if (rpos::core::InternalSystemEvent::InternalSystemEventMapLoopClosure == events[i].internalSystemEvent)
            {
                rosSdkServer()->requestSyncMap();
                RCLCPP_INFO(rclcpp::get_logger("server workers"), "worker: %s, loop closure, sync map.", getWorkerName().c_str());
                return;
            } 
        }
    }


    ServerSystemStatusWorker::ServerSystemStatusWorker(SlamwareRosSdkServer* pRosSdkServer
        , const std::string& wkName
        , const boost::chrono::milliseconds& triggerInterval
        )
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval), lastDeviceStatus_(boost::none),
        lastRelocalizationStatus_(boost::none)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubSystemStatus_ = nhRos->create_publisher<slamware_ros_sdk::msg::SystemStatus>(srvParams.getParameter<std::string>("system_status_topic_name"), 1);
        pubRelocalizaitonStatus_ = nhRos->create_publisher<slamware_ros_sdk::msg::RelocalizationStatus>(srvParams.getParameter<std::string>("relocalization_status_topic_name"), 1);
    }

    ServerSystemStatusWorker::~ServerSystemStatusWorker()
    {
        //
    }

    bool ServerSystemStatusWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerSystemStatusWorker::doPerform()
    {
        auto aurora = rosSdkServer()->safeGetAuroraSdk();
        if(!aurora)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get aurora sdk");
            return;
        }

        slamtec_aurora_sdk_device_status_desc_t result;
        slamware_ros_sdk::msg::SystemStatus resp;
        if(aurora->dataProvider.peekVSLAMSystemStatus(result))
        {
            //get current time in ns
            resp.timestamp_ns = rclcpp::Clock().now().nanoseconds();
            switch (result.status)  
            {
            case SLAMTEC_AURORA_SDK_DEVICE_INITED:
            {
                if(lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == SLAMTEC_AURORA_SDK_DEVICE_INITED)
                {
                    resp.status = "DeviceRunning";
                }
                else 
                    resp.status = "DeviceInited";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_INIT_FAILED:
                resp.status = "DeviceInitFailed";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_LOOP_CLOSURE:
                resp.status = "DeviceLoopClosure";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED:
            {
                if(lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceOptimizationCompleted";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_TRACKING_LOST:
                resp.status = "DeviceTrackingLost";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED:
            {
                if(lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceTrackingRecovered";
                break;
            }
            case  SLAMTEC_AURORA_SDK_DEVICE_MAP_CLEARED:    
                resp.status = "DeviceMapCleared";
                break;
            case  SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_STARTED:
                resp.status = "DeviceMapLoadingStarted";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_STARTED:
                resp.status = "DeviceMapSavingStarted";
                break;
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED:
            {
                if (lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceMapLoadingCompleted";
                break;
            }
            case SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED:
            {
                if (lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceMapSavingCompleted";
                break;
            }
            case 13:
            {
                if (lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == 13)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocSuccess";
                break;
            }
            case 14:
            {
                if (lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == 14)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocFailed";
                break;
            }
            case 15:
            {
                if (lastDeviceStatus_.is_initialized() && lastDeviceStatus_.get() == 15)
                {
                    resp.status = "DeviceRunning";
                }
                else
                    resp.status = "DeviceRelocCancelled";
                break;
            }
            case 16:
                resp.status = "DeviceRelocRunning";
                break;
            default:
                return;
            }

            lastDeviceStatus_ = result.status;
            pubSystemStatus_->publish(resp);
        }

        slamtec_aurora_sdk_relocalization_status_t reloc_status;
        slamware_ros_sdk::msg::RelocalizationStatus reloc_resp;
        if(aurora->dataProvider.peekRelocalizationStatus(reloc_status))
        {
            reloc_resp.timestamp_ns = rclcpp::Clock().now().nanoseconds();
            {
                switch (reloc_status.status)
                {
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_NONE:
                    reloc_resp.status = "RelocalizationNone";
                    break;
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_STARTED:
                    reloc_resp.status = "RelocalizationRunning";
                    break;
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED:
                {
                    if(lastRelocalizationStatus_.is_initialized() && lastRelocalizationStatus_.get() == SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationSucceed";
                    break;
                }
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED:
                {
                    if(lastRelocalizationStatus_.is_initialized() && lastRelocalizationStatus_.get() == SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationFailed";
                    break;
                }
                case SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED:
                {
                    if(lastRelocalizationStatus_.is_initialized() && lastRelocalizationStatus_.get() == SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED)
                    {
                        reloc_resp.status = "RelocalizationNone";
                    }
                    else
                        reloc_resp.status = "RelocalizationCanceled";
                    break;
                }
                default:
                    break;
                }
            }

            lastRelocalizationStatus_ = reloc_status.status;
            pubRelocalizaitonStatus_->publish(reloc_resp);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    ServerStereoImageWorker::ServerStereoImageWorker(SlamwareRosSdkServer *pRosSdkServer,
                                                     const std::string &wkName,
                                                     const boost::chrono::milliseconds &triggerInterval)
        : super_t(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubLeftImage_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("left_image_raw_topic_name"), 1);
        pubRightImage_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("right_image_raw_topic_name"), 1);
        pubStereoKeyPoints_ = nhRos->create_publisher<sensor_msgs::msg::Image>(srvParams.getParameter<std::string>("stereo_keypoints_topic_name"), 1);
    }

    ServerStereoImageWorker::~ServerStereoImageWorker()
    {
    }

    bool ServerStereoImageWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerStereoImageWorker::doPerform()
    {
        // Fill leftImage and rightImage with actual data from Aurora SDK
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        RemoteTrackingFrameInfo trackingFrame;
        if (!auroraSDK->dataProvider.peekTrackingData(trackingFrame)) {
            return;
        }

        cv::Mat left, right;
        trackingFrame.leftImage.toMat(left);
        trackingFrame.rightImage.toMat(right);

        // Convert BGR to RGB
        // convert to BGR
        cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
        const auto& srvParams = serverParams();
        // Get left and right images from the Aurora SDK
        std_msgs::msg::Header header_left;
        cv_bridge::CvImage img_bridge_left;
        header_left.frame_id = srvParams.getParameter<std::string>("camera_left");
        header_left.stamp = rclcpp::Clock().now();
        img_bridge_left = cv_bridge::CvImage(header_left, sensor_msgs::image_encodings::RGB8, left);
        sensor_msgs::msg::Image::SharedPtr leftImage = img_bridge_left.toImageMsg();
        pubLeftImage_->publish(*leftImage);

        cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);
        std_msgs::msg::Header header_right;
        cv_bridge::CvImage img_bridge_right;
        header_right.frame_id = srvParams.getParameter<std::string>("camera_right");
        header_right.stamp = rclcpp::Clock().now();
        img_bridge_right = cv_bridge::CvImage(header_right, sensor_msgs::image_encodings::RGB8, right);
        sensor_msgs::msg::Image::SharedPtr rightImage = img_bridge_right.toImageMsg();
        pubRightImage_->publish(*rightImage);

        // Get left and right keypoints from the Aurora SDK
        for (size_t i = 0; i < trackingFrame.getKeypointsLeftCount(); i++)
        {
            auto &kp = trackingFrame.getKeypointsLeftBuffer()[i];
            if (kp.flags)
                cv::circle(left, cv::Point(kp.x, kp.y), 2, cv::Scalar(0, 255, 0), 2);
        }
        for (size_t i = 0; i < trackingFrame.getKeypointsRightCount(); i++)
        {
            auto &kp = trackingFrame.getKeypointsRightBuffer()[i];
            if (kp.flags)
                cv::circle(right, cv::Point(kp.x, kp.y), 2, cv::Scalar(0, 255, 0), 2);
        }

        cv::Mat merged;
        cv::hconcat(left, right, merged);
        std_msgs::msg::Header header_stereo_keypoints;
        cv_bridge::CvImage img_bridge_stereo;
        header_stereo_keypoints.frame_id = srvParams.getParameter<std::string>("camera_left");
        header_stereo_keypoints.stamp = rclcpp::Clock().now();
        img_bridge_stereo = cv_bridge::CvImage(header_right, sensor_msgs::image_encodings::RGB8, merged);
        sensor_msgs::msg::Image::SharedPtr mergeImage = img_bridge_stereo.toImageMsg();
        pubStereoKeyPoints_->publish(*mergeImage);
    }

    // //////////////////////////////////////////////////////////////////////////
    // ServerStereoCameraInfoWorker::ServerStereoCameraInfoWorker(sSlamwareRosSdkServer *pRosSdkServer,
    //                                                  const std::string &wkName,
    //                                                  const boost::chrono::milliseconds &triggerInterval)
    //     : super_t(pRosSdkServer, wkName, triggerInterval)
    // {
    //     pubLeftCameraInfo_ = nhRos->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/left/camera_info", 10);
    //     pubRightCameraInfo_ = nhRos->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/right/camera_info", 10);
    // }

    // ServerStereoCameraInfoWorker::~ServerStereoCameraInfoWorker()
    // {
    // }

    // bool ServerStereoCameraInfoWorker::reinitWorkLoop()
    // {
    //     // Reinitialize the work loop if necessary
    //     return true;
    // }

    // void ServerStereoCameraInfoWorker::doPerform()
    // {

    // }
    
    ServerImuRawDataWorker::ServerImuRawDataWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const boost::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
        , lastTimestamp_(0)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubImuRawData_ = nhRos->create_publisher<sensor_msgs::msg::Imu>(srvParams.getParameter<std::string>("imu_raw_data_topic"), 1);
    }

    ServerImuRawDataWorker::~ServerImuRawDataWorker()
    {
        //
    }

    bool ServerImuRawDataWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerImuRawDataWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get aurora sdk");
            return;
        }

        std::vector<slamtec_aurora_sdk_imu_data_t> imuData;
        if (auroraSDK->dataProvider.peekIMUData(imuData))
        {
            for (auto &imu : imuData)
            {
                if (imu.timestamp_ns <= lastTimestamp_)
                {
                    // ignore the old data that has been fetched before
                    continue;
                }
                lastTimestamp_ = imu.timestamp_ns;
                sensor_msgs::msg::Imu imuRawDataRos;
                imuRawDataRos.linear_acceleration.x = imu.acc[0];
                imuRawDataRos.linear_acceleration.y = imu.acc[1];
                imuRawDataRos.linear_acceleration.z = imu.acc[2];
                imuRawDataRos.angular_velocity.x = imu.gyro[0];
                imuRawDataRos.angular_velocity.y = imu.gyro[1];
                imuRawDataRos.angular_velocity.z = imu.gyro[2];
                imuRawDataRos.header.stamp = rclcpp::Clock().now();
                pubImuRawData_->publish(imuRawDataRos);
            }
        }
    }

    ServerPointCloudWorker::ServerPointCloudWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const boost::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        const auto& srvParams = serverParams();
        auto nhRos = rosNodeHandle();
        pubPointCloud_ = nhRos->create_publisher<sensor_msgs::msg::PointCloud2>(srvParams.getParameter<std::string>("point_cloud_topic_name"), 1);
    }

    ServerPointCloudWorker::~ServerPointCloudWorker()
    {
        //
    }

    bool ServerPointCloudWorker::reinitWorkLoop()
    {
        // Reinitialize the work loop if necessary
        return true;
    }

    void ServerPointCloudWorker::doPerform()
    {
        auto auroraSDK = rosSdkServer()->safeGetAuroraSdk();
        if (!auroraSDK)
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get aurora sdk");
            return;
        }

        int activeMapId = -1;
        slamtec_aurora_sdk_map_desc_t activeMapDesc;
        std::vector<slamtec_aurora_sdk_map_point_desc_t> mapPoints;
        slamtec_aurora_sdk_global_map_desc_t globalMapDesc;
        if(!auroraSDK->dataProvider.getGlobalMappingInfo(globalMapDesc))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to get global mapping info");
            return;
        }

        activeMapId = globalMapDesc.activeMapID;
        // to access the map data, we need to create a map data visitor
        RemoteMapDataVisitor mapDataVisitor;
        mapDataVisitor.subscribeMapPointData([&](const slamtec_aurora_sdk_map_point_desc_t &mapPointDesc)
                                             {
                                            // handle the map point data
                                            mapPoints.push_back(mapPointDesc); 
                                            });

        // start the accessing session, this will block the background syncing thread during the accessing process.
        if (!auroraSDK->dataProvider.accessMapData(mapDataVisitor, {(uint32_t)activeMapId}))
        {
            RCLCPP_ERROR(rclcpp::get_logger("server workers"), "Failed to start the accessing session");
            return;
        }

        // tf_vslam_to_ros.setValue(0, 1, 0,
        //                        -1, 0, 0,
        //                        0, 0, 1);
        const tf2::Matrix3x3 tf_vslam_to_ros(1, 0, 0,
                                             0, 1, 0,
                                             0, 0, 1);
        // publish map point to sensor_msgs::msg::PointCloud2
        const auto& srvParams = serverParams();
        const int num_channels = 3; // x y z
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = rclcpp::Clock().now();
        cloud.header.frame_id = srvParams.getParameter<std::string>("map_frame");
        cloud.height = 1;
        cloud.width = mapPoints.size();
        cloud.is_dense = true;
        cloud.is_bigendian = false;
        cloud.point_step = num_channels * sizeof(float);
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.fields.resize(num_channels);

        std::string channel_id[] = {"x", "y", "z"};
        for (int i = 0; i < num_channels; i++)
        {
            cloud.fields[i].name = channel_id[i];
            cloud.fields[i].offset = i * sizeof(float);
            cloud.fields[i].count = 1;
            cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        }

        cloud.data.resize(cloud.row_step * cloud.height);

        // mapPoints to cloud
        unsigned char *cloud_data_ptr = &(cloud.data[0]);

        float data_array[num_channels];
        for (unsigned int i = 0; i < cloud.width; i++)
        {
            //map points from orbslam2 aixs to ros axis
            tf2::Vector3 mapPoint(mapPoints[i].position.x, mapPoints[i].position.y, mapPoints[i].position.z);
            tf2::Vector3 mapPoint_ros = tf_vslam_to_ros * mapPoint;
            data_array[0] = mapPoint_ros.x();
            data_array[1] = mapPoint_ros.y();
            data_array[2] = mapPoint_ros.z();
            memcpy(cloud_data_ptr + (i * cloud.point_step),
                   data_array, num_channels * sizeof(float));
        }

        pubPointCloud_->publish(cloud);
    }

    //////////////////////////////////////////////////////////////////////////

    RosConnectWorker::RosConnectWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const boost::chrono::milliseconds &triggerInterval)
        : ServerWorkerBase(pRosSdkServer, wkName, triggerInterval)
    {
        auto nhRos = rosNodeHandle();
        pubRosConnect_ = nhRos->create_publisher<std_msgs::msg::String>("/slamware_ros_sdk_server_node/state", 1);
    }

    RosConnectWorker::~RosConnectWorker()
    {
        //
    }

    bool RosConnectWorker::reinitWorkLoop()
    {
        auto pltfm = rosSdkServer()->safeGetSlamwarePlatform();
        if (!this->super_t::reinitWorkLoop())
            return false;

        isWorkLoopInitOk_ = true;
        return isWorkLoopInitOk_;
    }

    void RosConnectWorker::doPerform()
    {
        auto connectStatus = std_msgs::msg::String();
        connectStatus.data = "connected";
        auto pltfm = rosSdkServer()->safeGetSlamwarePlatform();
        if (!pltfm)
        {
            connectStatus.data = "disconnected";
        }

        pubRosConnect_->publish(connectStatus);
    }

    //////////////////////////////////////////////////////////////////////////
}
