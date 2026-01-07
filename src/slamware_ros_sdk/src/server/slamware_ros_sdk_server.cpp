#include  "slamware_ros_sdk_server.h"

#include <rpos/system/io/memory_read_stream.h>
#include <rpos/system/io/memory_write_stream.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>

#include <boost/assert.hpp>

#include <stdexcept>
#include <cmath>
#include <memory>
#include <tuple>

namespace slamware_ros_sdk {

    using namespace boost::placeholders;
    //////////////////////////////////////////////////////////////////////////

    SlamwareRosSdkServer::SlamwareRosSdkServer()
        : Node("slamware_ros_sdk_server")
        , state_(ServerStateNotInit)
        , isStopRequested_(false)
        , nh_(rclcpp::Node::make_shared("slamware_ros_sdk_server") )
        , relocalization_active_(false)
        , cancel_requested_(false)
    {
        tfBrdcstr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    SlamwareRosSdkServer::~SlamwareRosSdkServer()
    {
        cleanup_();
    }

    bool SlamwareRosSdkServer::startRun(std::string& errMsg)
    {
        errMsg.clear();
        const auto oldState = state_.load();
        if (ServerStateNotInit != oldState && ServerStateStopped != oldState)
        {
            errMsg = "it is running or already initialized.";
            return false;
        }

        isStopRequested_.store(false);
        bool bRet = init_(errMsg);
        if (bRet)
        {
            state_.store(ServerStateRunning);
            workThread_ = boost::move(boost::thread(boost::bind(&SlamwareRosSdkServer::workThreadFun_, this)));
        }

        if (!bRet)
        {
            cleanup_();
        }
        return bRet;
    }

    void SlamwareRosSdkServer::requestStop()
    {
        isStopRequested_.store(true);
    }

    void SlamwareRosSdkServer::waitUntilStopped()
    {
        if (workThread_.joinable())
            workThread_.join();
        BOOST_ASSERT(!isRunning_());
    }

    void SlamwareRosSdkServer::requestSyncMap()
    {
        auto wkDat = safeGetMutableWorkData_();
        if (wkDat)
            wkDat->syncMapRequested.store(true);
        auto aurora = safeGetAuroraSdk();
        if (aurora)
            aurora->controller.resyncMapData();
    }

    boost::chrono::milliseconds SlamwareRosSdkServer::sfConvFloatSecToBoostMs_(float fSec)
    {
        if (fSec < 0.0f)
            throw std::runtime_error("invalid float value of seconds.");

        const std::uint32_t uMs = static_cast<std::uint32_t>(std::floor(fSec * 1000));
        return boost::chrono::milliseconds(uMs);
    }

    bool SlamwareRosSdkServer::shouldContinueRunning_() const
    {
        return (!isStopRequested_.load());
    }

    ServerWorkData_ConstPtr SlamwareRosSdkServer::safeGetWorkData_() const
    {
        boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
        return workDat_;
    }

    ServerWorkData_Ptr SlamwareRosSdkServer::safeGetMutableWorkData_()
    {
        boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
        return workDat_;
    }

    bool SlamwareRosSdkServer::safeIsSlamwarePlatformConnected_() const
    {
        boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
        return bool(slamwarePltfm_);
    }

    SlamwareRosSdkServer::slamware_platform_t SlamwareRosSdkServer::safeGetSlamwarePlatform_() const
    {
        boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
        return slamwarePltfm_;
    }

    void SlamwareRosSdkServer::safeSetSlamwarePlatform_(const slamware_platform_t& pltfm)
    {
        auto tmpPltfm = pltfm;
        {
            boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
            std::swap(slamwarePltfm_, tmpPltfm);
        }
    }

    void SlamwareRosSdkServer::safeReleaseSlamwarePlatform_()
    {
        slamware_platform_t tmpPltfm;
        {
            boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
            std::swap(slamwarePltfm_, tmpPltfm);
        }
        disconnectSlamwarePlatform_(tmpPltfm);
    }
    
    SlamwareRosSdkServer::slamware_platform_t SlamwareRosSdkServer::connectSlamwarePlatform_(const std::string& ip, int port) const
    {
        try
        {
            auto pltfm = slamware_platform_t::connect(ip, port);
            return pltfm;
        }
        catch (const std::exception& excp)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "connectSlamwarePlatform_(), exception: %s.", excp.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "connectSlamwarePlatform_(), unknown exception.");
        }
        return slamware_platform_t();
    }
    
    void SlamwareRosSdkServer::disconnectSlamwarePlatform_(slamware_platform_t& pltfm) const
    {
        if (pltfm)
        {
            try
            {
                pltfm.disconnect();
            }
            catch (const std::exception& excp)
            {
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "disconnectSlamwarePlatform_(), exception: %s.", excp.what());
            }
            catch (...)
            {
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "disconnectSlamwarePlatform_(), unknown exception.");
            }
        }
    }

    bool SlamwareRosSdkServer::discoverAndSelectAuroraDevice(rp::standalone::aurora::RemoteSDK *sdk, rp::standalone::aurora::SDKServerConnectionDesc &selectedDeviceDesc)
    {
        std::vector<SDKServerConnectionDesc> serverList;
        size_t count = sdk->getDiscoveredServers(serverList, 32);
        if (count == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "No aurora devices found");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Found %d aurora devices", count);
        for (size_t i = 0; i < count; i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Device %d", i);
            for (size_t j = 0; j < serverList[i].size(); ++j)
            {
                auto &connectionOption = serverList[i][j];
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "  option %d: %s", j, connectionOption.toLocatorString().c_str());
            }
        }

        // select the first device
        selectedDeviceDesc = serverList[0];
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Selected first device: %s", selectedDeviceDesc[0].toLocatorString().c_str());
        return true;
    }


    void SlamwareRosSdkServer::loopTryConnectToAuroraSdk_()
    {
        std::uint32_t tryCnt = 1;
        while (shouldContinueRunning_())
        {
            {
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "try to connect to aurora, tryCnt: %u.", tryCnt);
                connectAuroraSdk_();
                if (auroraSdkConnected_.load())
                {
                    RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "connect to aurora, OK, tryCnt: %u.", tryCnt);
                    return;
                }
                int iVal = params_.getParameter<int>("reconn_wait_ms");
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "connect to aurora, FAILED, tryCnt: %u, wait %d ms to retry."
                    , tryCnt, iVal);
            }
            int iVal = params_.getParameter<int>("reconn_wait_ms");
            const std::uint32_t maxSleepMs = (0 <= iVal ? (std::uint32_t)iVal : 0U);
            roughSleepWait_(maxSleepMs, 100U);
            ++tryCnt;
        }
    }


    void SlamwareRosSdkServer::connectAuroraSdk_()
    {
        auroraSdk_ = rp::standalone::aurora::RemoteSDK::CreateSession();
        if(auroraSdk_ == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to create aurora sdk session.");
            return;
        }

        std::string ip = params_.getParameter<std::string>("ip_address");
        int port = params_.getParameter<int>("robot_port");
        rp::standalone::aurora::SDKServerConnectionDesc selectedDeviceDesc;
        const char *connectionString = nullptr;
        if (!ip.empty())
        {
            connectionString = ip.c_str();
        }

        if (connectionString == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Device connection string not provided, try to discover aurora devices..." );
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (!discoverAndSelectAuroraDevice(auroraSdk_, selectedDeviceDesc))
            {
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to discover aurora devices");
                return;
            }
        }
        else
        {
            selectedDeviceDesc = SDKServerConnectionDesc(connectionString);
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Selected device: %s", selectedDeviceDesc[0].toLocatorString().c_str());
        }

        // connect to the selected device
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Connecting to the selected device...");
        if (!auroraSdk_->connect(selectedDeviceDesc))
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to connect to the selected device");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "Connected to the selected device");
        auroraSdk_->controller.setMapDataSyncing(true);
        LIDAR2DGridMapGenerationOptions genOption;

        // 为室内测绘优化：使用更高分辨率（3cm）的2D栅格地图，并在SDK支持范围内自适应
        float desiredResolution = 0.03f;
        auto resRange = auroraSdk_->lidar2DMapBuilder.getSupportedResolutionRange();
        const float minRes = std::get<0>(resRange);
        const float maxRes = std::get<1>(resRange);
        if (minRes > 0.0f && maxRes > 0.0f && minRes <= maxRes)
        {
            if (desiredResolution < minRes) desiredResolution = minRes;
            else if (desiredResolution > maxRes) desiredResolution = maxRes;
        }
        genOption.resolution = desiredResolution;

        if(!auroraSdk_->lidar2DMapBuilder.startPreviewMapBackgroundUpdate(genOption))
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "Failed to start preview map update");
        }
        auroraSdkConnected_.store(true);
    }

    void SlamwareRosSdkServer::disconnectAuroraSdk_()
    {
        if (auroraSdkConnected_.load())
        {
            {
                boost::lock_guard<boost::mutex> lkGuard(auroraSdkLock_);
                auroraSdk_->lidar2DMapBuilder.stopPreviewMapBackgroundUpdate();
                auroraSdk_->disconnect();
                auroraSdk_->release();
            }
            auroraSdkConnected_.store(false);
        }
    }   

    bool SlamwareRosSdkServer::init_(std::string& /*errMsg*/)
    {
        // params_.resetToDefault();  // TODO:
        std::string ip = params_.getParameter<std::string>("ip_address");
        int port = params_.getParameter<int>("robot_port");
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "ip:%s, port:%d.", ip.c_str(), port);
        
        auroraSdkConnected_.store(false);
        params_.setBy(nh_);
        {
            boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
            workDat_ = boost::make_shared<ServerWorkData>();
        }

        if (!safeIsSlamwarePlatformConnected_())
        {
            loopTryConnectToSlamwarePlatform_();
        }

        if(!auroraSdkConnected_.load())
        {
            loopTryConnectToAuroraSdk_();
        }

        // init all workers
        {
            serverWorkers_.clear();

            const auto defaultUpdateIntervalForNoneUpdateWorkers = boost::chrono::milliseconds(1000u * 60u);

            {
                auto svrWk = boost::make_shared<ServerRobotDeviceInfoWorker>(this, "RobotDeviceInfo", defaultUpdateIntervalForNoneUpdateWorkers);
                serverWorkers_.push_back(svrWk);
            }

            {
                auto svrWk = boost::make_shared<ServerOdometryWorker>(this, "Odometry", sfConvFloatSecToBoostMs_(params_.getParameter<float>("odometry_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("robot_pose_pub_period"))
            {
                auto svrWk = boost::make_shared<ServerRobotPoseWorker>(this, "RobotPose", sfConvFloatSecToBoostMs_(params_.getParameter<float>("robot_pose_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("map_update_period"))
            {
                auto svrWk = boost::make_shared<ServerExploreMapUpdateWorker>(this, "ExploreMapUpdate", sfConvFloatSecToBoostMs_(params_.getParameter<float>("map_update_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("map_pub_period"))
            {
                auto svrWk = boost::make_shared<ServerExploreMapPublishWorker>(this, "ExploreMapPublish", sfConvFloatSecToBoostMs_(params_.getParameter<float>("map_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("scan_pub_period"))
            {
                auto svrWk = boost::make_shared<ServerLaserScanWorker>(this, "LaserScan", sfConvFloatSecToBoostMs_(params_.getParameter<float>("scan_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            if (0 < params_.getParameter<float>("robot_basic_state_pub_period"))
            {
                auto svrWk = boost::make_shared<ServerRobotBasicStateWorker>(this, "RobotBasicState", sfConvFloatSecToBoostMs_(params_.getParameter<float>("robot_basic_state_pub_period")));
                serverWorkers_.push_back(svrWk);
            }
            {
                auto svrWk = boost::make_shared<ServerReadEventsWorker>(this, "ReadEvents", sfConvFloatSecToBoostMs_(params_.getParameter<float>("robot_basic_state_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                auto svrWk = boost::make_shared<ServerImuRawDataWorker>(this, "ServerImuRawDataWorker", sfConvFloatSecToBoostMs_(params_.getParameter<float>("imu_raw_data_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                auto svrWk = boost::make_shared<RosConnectWorker>(this, "RosConnectWorker", sfConvFloatSecToBoostMs_(params_.getParameter<float>("robot_basic_state_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                // add ServerSystemStatusWorker
                auto svrWk = boost::make_shared<ServerSystemStatusWorker>(this, "SystemStatus", sfConvFloatSecToBoostMs_(params_.getParameter<float>("system_status_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                //add ServerStereoImageWorker
                auto svrWk = boost::make_shared<ServerStereoImageWorker>(this, "StereoImage", sfConvFloatSecToBoostMs_(params_.getParameter<float>("stereo_image_pub_period")));
                serverWorkers_.push_back(svrWk);
            }

            {
                //add ServerPointCloudWorker
                auto svrWk = boost::make_shared<ServerPointCloudWorker>(this, "PointCloud", sfConvFloatSecToBoostMs_(params_.getParameter<float>("point_cloud_pub_period")));   
                serverWorkers_.push_back(svrWk);
            }
        }

        // init all subscriptions
        { 
            subSyncMap_ = subscribe_T_<slamware_ros_sdk::msg::SyncMapRequest>("/slamware_ros_sdk_server_node/sync_map", 1U, &SlamwareRosSdkServer::msgCbSyncMap_);
           
            subClearMap_ = subscribe_T_<slamware_ros_sdk::msg::ClearMapRequest>("/slamware_ros_sdk_server_node/clear_map", 1U, &SlamwareRosSdkServer::msgCbClearMap_);
            subSetMapUpdate_ = subscribe_T_<slamware_ros_sdk::msg::SetMapUpdateRequest>("/slamware_ros_sdk_server_node/set_map_update", 1U, &SlamwareRosSdkServer::msgCbSetMapUpdate_);
            subSetMapLocalization_ = subscribe_T_<slamware_ros_sdk::msg::SetMapLocalizationRequest>("/slamware_ros_sdk_server_node/set_map_localization", 1U, &SlamwareRosSdkServer::msgCbSetMapLocalization_);


            subRelocalizationCancel_ = subscribe_T_<slamware_ros_sdk::msg::RelocalizationCancelRequest>("/slamware_ros_sdk_server_node/relocalization/cancel", 1U, &SlamwareRosSdkServer::msgCbRelocalizationCancel_);

        }

        // init all services
        {
            srvSyncGetStcm_ = advertiseService_T_<slamware_ros_sdk::srv::SyncGetStcm>("/slamware_ros_sdk_server_node/sync_get_stcm", &SlamwareRosSdkServer::srvCbSyncGetStcm_);
            srvSyncSetStcm_ = advertiseService_T_<slamware_ros_sdk::srv::SyncSetStcm>("/slamware_ros_sdk_server_node/sync_set_stcm", &SlamwareRosSdkServer::srvCbSyncSetStcm_);
            relocalization_request_srv_ = advertiseService_T_<slamware_ros_sdk::srv::RelocalizationRequest>("/slamware_ros_sdk_server_node/relocalization", &SlamwareRosSdkServer::srvCbRelocalizationRequest_);
        }
        return true;
    }

    void SlamwareRosSdkServer::cleanup_()
    {
        if (isRunning_())
            requestStop();
        waitUntilStopped();

        safeReleaseSlamwarePlatform_();

        disconnectAuroraSdk_();

        // de-init all publishers
        {
            serverWorkers_.clear();
        }

        {
            boost::lock_guard<boost::mutex> lkGuard(workDatLock_);
            workDat_.reset();
        }

        state_.store(ServerStateNotInit);
    }

    void SlamwareRosSdkServer::workThreadFun_()
    {
        BOOST_ASSERT(ServerStateRunning == state_.load());
        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "SlamwareRosSdkServer, work thread begin.");

        while (shouldContinueRunning_()
            && rclcpp::ok() // ros::ok()
            )
        {
            if (!safeIsSlamwarePlatformConnected_())
            {
                loopTryConnectToSlamwarePlatform_();

                if (!safeIsSlamwarePlatformConnected_())
                    continue;
            }
            BOOST_ASSERT(safeIsSlamwarePlatformConnected_());

            if(!auroraSdkConnected_.load())
            {
                loopTryConnectToAuroraSdk_();
                if (!auroraSdkConnected_.load())
                    continue;
            }

            try
            {
                loopWork_();
            }
            catch (const std::exception& excp)
            {
                RCLCPP_FATAL(rclcpp::get_logger("slamware ros sdk server"), "loopWork_(), exception: %s.", excp.what());
            }
            catch (...)
            {
                RCLCPP_FATAL(rclcpp::get_logger("slamware ros sdk server"), "loopWork_(), unknown exception.");
            }

            safeReleaseSlamwarePlatform_();
            
            disconnectAuroraSdk_();

            if (shouldContinueRunning_())
            {
                const std::uint32_t maxSleepMs = (1000u * 3u);
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "wait %u ms to reconnect and restart work loop.", maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "SlamwareRosSdkServer, work thread end.");
        state_.store(ServerStateStopped);
    }

    void SlamwareRosSdkServer::roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs)
    {
        const auto durOnceSleep = boost::chrono::milliseconds(onceSleepMs);
        auto tpNow = boost::chrono::steady_clock::now();
        const auto maxSleepTimepoint = tpNow + boost::chrono::milliseconds(maxSleepMs);
        while (shouldContinueRunning_()
            && tpNow < maxSleepTimepoint
            )
        {
            boost::this_thread::sleep_for(durOnceSleep);
            tpNow = boost::chrono::steady_clock::now();
        }
    }

    void SlamwareRosSdkServer::loopTryConnectToSlamwarePlatform_()
    {
        std::uint32_t tryCnt = 1;
        while (shouldContinueRunning_())
        {
            {
                std::string ip = params_.getParameter<std::string>("ip_address");
                int port = params_.getParameter<int>("robot_port");
                RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "try to connect to %s:%d, tryCnt: %u.", ip.c_str(), port, tryCnt);
                auto pltfm = connectSlamwarePlatform_(ip, port);
                if (pltfm)
                {
                    RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "connect to %s:%d, OK, tryCnt: %u.", ip.c_str(), port, tryCnt);
                    safeSetSlamwarePlatform_(pltfm);
                    return;
                }
                int iVal = params_.getParameter<int>("reconn_wait_ms");
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "connect to %s:%d, FAILED, tryCnt: %u, wait %d ms to retry."
                    , ip.c_str(), port
                    , tryCnt, iVal);
            }
            int iVal = params_.getParameter<int>("reconn_wait_ms");
            const std::uint32_t maxSleepMs = (0 <= iVal ? (std::uint32_t)iVal : 0U);
            roughSleepWait_(maxSleepMs, 100U);
            ++tryCnt;
        }
    }

    bool SlamwareRosSdkServer::reinitWorkLoop_()
    {
        const std::uint32_t cntWorkers = static_cast<std::uint32_t>(serverWorkers_.size());

        RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "reset all %u workers on work loop begin.", cntWorkers);
        for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
        {
            const auto& svrWk = (*it);
            const auto wkName = svrWk->getWorkerName();
            try
            {
                svrWk->resetOnWorkLoopBegin();
            }
            catch (const std::exception& excp)
            {
                RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, resetOnWorkLoopBegin(), exception: %s.", wkName.c_str(), excp.what());
                return false;
            }
        }

        const std::uint32_t maxSleepMs = (1000u * 2u);
        const std::uint32_t maxLoopTryCnt = 3;
        for (std::uint32_t t = 1; (t <= maxLoopTryCnt && shouldContinueRunning_()); ++t)
        {
            std::uint32_t cntOk = 0;
            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto& svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                try
                {
                    if (svrWk->isWorkLoopInitOk())
                    {
                        ++cntOk;
                    }
                    else
                    {
                        if (svrWk->reinitWorkLoop())
                            ++cntOk;
                        else
                            RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "failed to init work loop, woker: %s.", wkName.c_str());
                    }
                }
                catch (const rpos::robot_platforms::ConnectionLostException& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                    throw;
                }
                catch (const rpos::robot_platforms::ConnectionTimeOutException& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                    throw;
                }
                catch (const rpos::robot_platforms::ConnectionFailException& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                    throw;
                }
                catch (const std::exception& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker: %s, reinitWorkLoop(), exception: %s.", wkName.c_str(), excp.what());
                }
            }
            // check if all workers are ok.            
            if (cntWorkers == cntOk)
            {
                return true;
            }
            else if (t < maxLoopTryCnt)
            {
                RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "(%u / %u) cntWorkers: %u, cntOk: %u, wait %u ms to retry.", t, maxLoopTryCnt, cntWorkers, cntOk, maxSleepMs);
                roughSleepWait_(maxSleepMs, 100U);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "(%u / %u) cntWorkers: %u, cntOk: %u.", t, maxLoopTryCnt, cntWorkers, cntOk);
            }
        }
        return false;
    }

    void SlamwareRosSdkServer::loopWork_()
    {
        if (reinitWorkLoop_())
        {
            RCLCPP_INFO(rclcpp::get_logger("slamware ros sdk server"), "successed to reinit all workers on work loop begin.");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "failed or cancelled to reinit work loop.");
            return;
        }

        while (shouldContinueRunning_())
        {
            boost::chrono::steady_clock::time_point minNextTriggerTimepoint = boost::chrono::steady_clock::now() + boost::chrono::milliseconds(100U);

            for (auto it = serverWorkers_.begin(), itEnd = serverWorkers_.end(); itEnd != it; ++it)
            {
                const auto& svrWk = (*it);
                const auto wkName = svrWk->getWorkerName();
                bool shouldReconnect = false;
                try
                {
                    svrWk->checkToPerform();
                }
                catch (const rpos::robot_platforms::ConnectionLostException& excp)
                {
                    shouldReconnect = true;
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::robot_platforms::ConnectionTimeOutException& excp)
                {
                    shouldReconnect = true;
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::robot_platforms::ConnectionFailException& excp)
                {
                    shouldReconnect = true;
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::robot_platforms::OperationFailException& excp)
                {
                    RCLCPP_WARN(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const rpos::system::detail::ExceptionBase& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (const std::exception& excp)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, exception: %s.", wkName.c_str(), excp.what());
                }
                catch (...)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "worker name: %s, unknown exception.", wkName.c_str());
                }

                if (shouldReconnect)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "it should reconnect to slamware.");
                    return;
                }

                const auto tmpNextTp = svrWk->getNextTimepointToTrigger();
                if (tmpNextTp < minNextTriggerTimepoint)
                    minNextTriggerTimepoint = tmpNextTp;
            }

            auto tpNow = boost::chrono::steady_clock::now();
            if (tpNow <= minNextTriggerTimepoint)
            {
                const auto durSleep = boost::chrono::duration_cast<boost::chrono::milliseconds>(minNextTriggerTimepoint - tpNow);
                boost::this_thread::sleep_for(durSleep);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template<class MsgT>
    void SlamwareRosSdkServer::msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
        , const std::string& msgTopic
        ,  typename msg_cb_help_t<MsgT>::msg_shared_ptr  msg
        )
    {
        BOOST_ASSERT(nullptr != mfpCbPerform);
        auto pltfm = safeGetSlamwarePlatform_();
        if (!pltfm)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "process msgTopic: %s, not connected.", msgTopic.c_str());
            return;
        }

        try
        {
            (this->*mfpCbPerform)(pltfm, msg);
        }
        catch (const std::exception& excp)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "process msgTopic: %s, exception: %s.", msgTopic.c_str(), excp.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "process msgTopic: %s, unknown exception.", msgTopic.c_str());
        }
    }

    template<class MsgT>
    typename rclcpp::Subscription<MsgT>::SharedPtr SlamwareRosSdkServer::subscribe_T_(const std::string& msgTopic
        , std::uint32_t queueSize
        , typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
        )
    {
        typedef msg_cb_help_t<MsgT>     TheMsgCbHelpT;

        typename TheMsgCbHelpT::ros_cb_fun_t rosCbFun(
            boost::bind(&SlamwareRosSdkServer::msgCbWrapperFun_T_<MsgT>, this, mfpCbPerform, msgTopic, _1)
            );
        return nh_->create_subscription<MsgT>(msgTopic, queueSize, rosCbFun);
    }

    void SlamwareRosSdkServer::msgCbSyncMap_(slamware_platform_t& /*pltfm*/, const slamware_ros_sdk::msg::SyncMapRequest::SharedPtr /*msg*/)
    {
        requestSyncMap();
    }

    void SlamwareRosSdkServer::msgCbClearMap_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::ClearMapRequest::SharedPtr msg)
    {
        rpos::features::location_provider::MapKind kind;
        rosMsgToSltc(msg->kind, kind);

        pltfm.clearMap(kind);
    }

    void SlamwareRosSdkServer::msgCbSetMapUpdate_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::SetMapUpdateRequest::SharedPtr msg)
    {
        rpos::features::location_provider::MapKind kind;
        rosMsgToSltc(msg->kind, kind);

        pltfm.setMapUpdate(msg->enabled, kind);

        {
            boost::lock_guard<boost::mutex> lkGuard(auroraSdkLock_);
            auroraSdk_->controller.requireMappingMode();
        }
    }

    void SlamwareRosSdkServer::msgCbSetMapLocalization_(slamware_platform_t& pltfm, const slamware_ros_sdk::msg::SetMapLocalizationRequest::SharedPtr msg)
    {
        pltfm.setMapLocalization(msg->enabled);
        {
            //boost::lock_guard<boost::mutex> lkGuard(auroraSdkLock_);

            //default : disable always mapping 
            //auroraSdk_->controller.requirePureLocalizationMode();
        }
    }

    //////////////////////////////////////////////////////////////////////////

    template<class SrvMsgT>
    bool SlamwareRosSdkServer::srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
        , const std::string& srvMsgTopic
        , typename srv_cb_help_t<SrvMsgT>::request_t req
        , typename srv_cb_help_t<SrvMsgT>::response_t resp
        )
    {
        BOOST_ASSERT(nullptr != mfpCbPerform);
        auto pltfm = safeGetSlamwarePlatform_();
        if (!pltfm)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "process request: %s, not connected.", srvMsgTopic.c_str());
            return false;
        }

        try
        {
            return (this->*mfpCbPerform)(req, resp);
        }
        catch (const std::exception& excp)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "process request: %s, exception: %s.", srvMsgTopic.c_str(), excp.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamware ros sdk server"), "process request: %s, unknown exception.", srvMsgTopic.c_str());
        }
        return false;
    }

    template<class SrvMsgT>
    typename rclcpp::Service<SrvMsgT>::SharedPtr SlamwareRosSdkServer::advertiseService_T_(const std::string& srvMsgTopic
        , typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
        )
    {
        typedef srv_cb_help_t<SrvMsgT>     TheSrvCbHelpT;

        typename TheSrvCbHelpT::ros_cb_fun_t rosCbFun(
            boost::bind(&SlamwareRosSdkServer::srvCbWrapperFun_T_<SrvMsgT>, this, mfpCbPerform, srvMsgTopic, _1, _2)
            );
        return nh_->create_service<SrvMsgT>(srvMsgTopic, rosCbFun); 
    }

    bool SlamwareRosSdkServer::srvCbSyncGetStcm_(slamware_ros_sdk::srv::SyncGetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncGetStcm::Response::SharedPtr resp)
    {
        const char *mapfile = req->mapfile.c_str();         
        auto aurora = safeGetAuroraSdk();
        std::promise<int> resultPromise;
        auto resultFuture = resultPromise.get_future();

        auto resultCallback = [](void *userData, int isOK)
        {
            auto promise = reinterpret_cast<std::promise<bool> *>(userData);
            promise->set_value(isOK != 0);
        };

        if (!aurora->mapManager.startDownloadSession(mapfile, resultCallback, &resultPromise))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to start map storage session");
            resp->success = false;
            resp->message = "Failed to start map storage session";
            return false;
        }

        while (resultFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
        {
            slamtec_aurora_sdk_mapstorage_session_status_t status;
            if (!aurora->mapManager.querySessionStatus(status))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to query storage status");
                resp->success = false;
                resp->message = "Failed to query storage status";
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map download progress: %f", status.progress);
            std::this_thread::yield();
        }

        int result = resultFuture.get();
        if (result == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save map");
            resp->success = false;
            resp->message = "Failed to save map";
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map saved successfully");
        resp->success = true;
        resp->message = "Map saved successfully";
        return true;
    }

    bool SlamwareRosSdkServer::srvCbSyncSetStcm_(slamware_ros_sdk::srv::SyncSetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncSetStcm::Response::SharedPtr resp)
    {
        auto aurora = safeGetAuroraSdk();
        {
            std::promise<bool> resultPromise;
            auto resultFuture = resultPromise.get_future();

            auto resultCallback = [](void *userData, int isOK)
            {
                auto promise = reinterpret_cast<std::promise<bool> *>(userData);
                promise->set_value(isOK != 0);
            };
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting map upload %s", req->mapfile.c_str());
            if (!aurora->mapManager.startUploadSession(req->mapfile.c_str(), resultCallback, &resultPromise))
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to start map upload session");
                resp->success = false;
                resp->message = "Failed to start map upload session";
                return false;
            }

            while (resultFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
            {
                slamtec_aurora_sdk_mapstorage_session_status_t status;
                if (!aurora->mapManager.querySessionStatus(status))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to query upload status");
                    resp->success = false;
                    resp->message = "Failed to query upload status";
                    return false;
                }

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map upload progress: %f", status.progress);
                std::this_thread::yield();
            }

            int result = resultFuture.get();
            if (result == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to upload map");
                resp->success = false;
                resp->message = "Failed to upload map";
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map uploaded successfully");
            resp->success = true;
            resp->message = "Map uploaded successfully";
        }
        requestSyncMap();
        aurora->controller.requireMappingMode();
        return true;
    }

    //////////////////////////////////////////////////////////////////////////
    bool SlamwareRosSdkServer::srvCbRelocalizationRequest_(slamware_ros_sdk::srv::RelocalizationRequest::Request::SharedPtr req,
                                                           slamware_ros_sdk::srv::RelocalizationRequest::Response::SharedPtr resp)
    {
        if(relocalization_active_.load())
        {
            RCLCPP_ERROR(this->get_logger(), "Relocalization already in progress");
            resp->success = false;
            return false;
        }
        // Handle relocalization request using Aurora SDK
        auto aurora = safeGetAuroraSdk();
        aurora->controller.requireRelocalization();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Relocalization requested");
        relocalization_active_.store(true);
        cancel_requested_.store(false);


        relocalization_future_ = std::async(std::launch::async, [this]() {
            while (true)
            {
                checkRelocalizationStatus();
                if (cancel_requested_.load() || !relocalization_active_.load())
                {
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        resp->success = true;
        return true;
    }

    void SlamwareRosSdkServer::checkRelocalizationStatus()
    {
        slamtec_aurora_sdk_relocalization_status_t result;
        auto aurora = safeGetAuroraSdk();
        aurora->dataProvider.peekRelocalizationStatus(result);
        if (result.status == 2)
        {
            RCLCPP_INFO(this->get_logger(), "Relocalization succeeded");
            relocalization_active_.store(false);
        }
        else if (result.status == 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Relocalization failed");
            relocalization_active_.store(false);
        }
        else if (result.status == 4)
        {
            RCLCPP_INFO(this->get_logger(), "Relocalization canceled by system");
            relocalization_active_.store(false);
        }
    }

    void SlamwareRosSdkServer::msgCbRelocalizationCancel_(slamware_platform_t &pltfm, const slamware_ros_sdk::msg::RelocalizationCancelRequest::SharedPtr msg)
    {
        // Handle relocalization cancel using Aurora SDK
        if (relocalization_active_.load())
        {
            auto aurora = safeGetAuroraSdk();
            slamtec_aurora_sdk_errorcode_t error_code = aurora->controller.cancelRelocalization();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Relocalization cancel requested");

            cancel_requested_.store(true);
            relocalization_active_.store(false);
            if (error_code != SLAMTEC_AURORA_SDK_ERRORCODE_OK)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to cancel relocalization: %d", error_code);
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Relocalization not active");
        }
    }
}
