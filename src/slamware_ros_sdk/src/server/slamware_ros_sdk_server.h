#pragma once

#include "server_workers.h"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <slamware_ros_sdk/srv/sync_get_stcm.hpp>
#include <slamware_ros_sdk/srv/sync_set_stcm.hpp>
#include <slamware_ros_sdk/srv/relocalization_request.hpp>
#include <slamware_ros_sdk/msg/relocalization_cancel_request.hpp>

#include <message_filters/subscriber.h>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <boost/function.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <aurora_pubsdk_inc.h>
#include <fstream>

namespace slamware_ros_sdk
{
    class SlamwareRosSdkServer : public rclcpp::Node
    {
    private:
        friend class ServerWorkerBase;

    public:
        SlamwareRosSdkServer();
        ~SlamwareRosSdkServer();

        bool startRun(std::string& errMsg);
        void requestStop();
        void waitUntilStopped(); // not thread-safe

        const std::shared_ptr<rclcpp::Node> getRosNodeHandle() const { return nh_; }

    public:
        void requestSyncMap();

        typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

        slamware_platform_t safeGetSlamwarePlatform() 
        {
            boost::lock_guard<boost::mutex> lkGuard(slamwarePltfmLock_);
            return slamwarePltfm_;
        }

        rp::standalone::aurora::RemoteSDK* safeGetAuroraSdk() 
        {
            boost::lock_guard<boost::mutex> lkGuard(auroraSdkLock_);
            return auroraSdk_;
        }

    private:
        enum ServerState
        {
            ServerStateNotInit
            , ServerStateRunning
            , ServerStateStopped
        };


        template<class MsgT>
        struct msg_cb_help_t
        {
            typedef MsgT                                msg_t;
            typedef typename msg_t::SharedPtr           msg_shared_ptr;
            typedef void (SlamwareRosSdkServer::*msg_cb_perform_fun_t)(slamware_platform_t&,  msg_shared_ptr);
            typedef boost::function< void( msg_shared_ptr) >        ros_cb_fun_t; // callback function for ROS.
        };

        template<class SrvMsgT>
        struct srv_cb_help_t
        {
            typedef SrvMsgT                             srv_msg_t;
            typedef typename srv_msg_t::Request::SharedPtr         request_t;
            typedef typename srv_msg_t::Response::SharedPtr        response_t;
            typedef bool (SlamwareRosSdkServer::*srv_cb_perform_fun_t)(request_t, response_t);
            typedef boost::function< bool(request_t, response_t) >            ros_cb_fun_t; // callback function for ROS.
        };

    private:
        static boost::chrono::milliseconds sfConvFloatSecToBoostMs_(float fSec);

        bool isRunning_() const { return ServerStateRunning == state_.load(); }
        bool shouldContinueRunning_() const;

        const std::shared_ptr<rclcpp::Node> rosNodeHandle_() const { return nh_; }
        std::shared_ptr<rclcpp::Node> rosNodeHandle_() { return nh_; }

        const ServerParams& serverParams_() const { return params_; }
        
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster_() const { return tfBrdcstr_; }
        std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster_() { return tfBrdcstr_; }

        ServerWorkData_ConstPtr safeGetWorkData_() const;
        ServerWorkData_Ptr safeGetMutableWorkData_();

        bool safeIsSlamwarePlatformConnected_() const;
        slamware_platform_t safeGetSlamwarePlatform_() const;
        void safeSetSlamwarePlatform_(const slamware_platform_t& pltfm);
        void safeReleaseSlamwarePlatform_();
        slamware_platform_t connectSlamwarePlatform_(const std::string& ip, int port) const;
        void disconnectSlamwarePlatform_(slamware_platform_t& pltfm) const;

        bool init_(std::string& errMsg);
        void cleanup_();

        void workThreadFun_();

        void roughSleepWait_(std::uint32_t maxSleepMs, std::uint32_t onceSleepMs);
        void loopTryConnectToSlamwarePlatform_();

        bool reinitWorkLoop_();
        void loopWork_();

        void loopTryConnectToAuroraSdk_();

        void connectAuroraSdk_();
        void disconnectAuroraSdk_();
        bool discoverAndSelectAuroraDevice(rp::standalone::aurora::RemoteSDK *sdk, rp::standalone::aurora::SDKServerConnectionDesc &selectedDeviceDesc);


        //////////////////////////////////////////////////////////////////////////
        // subscribed messages
        //////////////////////////////////////////////////////////////////////////

        template<class MsgT>
        void msgCbWrapperFun_T_(typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
            , const std::string& msgTopic
            ,  typename msg_cb_help_t<MsgT>::msg_shared_ptr  msg
            );
        template<class MsgT>
        typename rclcpp::Subscription<MsgT>::SharedPtr subscribe_T_(const std::string& msgTopic
            , std::uint32_t queueSize
            , typename msg_cb_help_t<MsgT>::msg_cb_perform_fun_t mfpCbPerform
            );

        
        /////////////////////////////////////slamware & aurora /////////////////////////////////////
        void msgCbSyncMap_(slamware_platform_t &pltfm, const slamware_ros_sdk::msg::SyncMapRequest::SharedPtr msg);
        void msgCbClearMap_(slamware_platform_t &pltfm, const slamware_ros_sdk::msg::ClearMapRequest::SharedPtr msg);
        void msgCbSetMapUpdate_(slamware_platform_t &pltfm, const slamware_ros_sdk::msg::SetMapUpdateRequest::SharedPtr msg);
        void msgCbSetMapLocalization_(slamware_platform_t &pltfm, const slamware_ros_sdk::msg::SetMapLocalizationRequest::SharedPtr msg);
        
        //////////////////////////////////////////////////////////////////////////////////////////

        // Relocalization cancel
        void msgCbRelocalizationCancel_(slamware_platform_t &pltfm, const slamware_ros_sdk::msg::RelocalizationCancelRequest::SharedPtr msg);

        //////////////////////////////////////////////////////////////////////////
        // advertised services
        //////////////////////////////////////////////////////////////////////////

        template<class SrvMsgT>
        bool srvCbWrapperFun_T_(typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform
            , const std::string& srvMsgTopic
            , typename srv_cb_help_t<SrvMsgT>::request_t req
            , typename srv_cb_help_t<SrvMsgT>::response_t resp
            );
        template <class SrvMsgT>
        typename rclcpp::Service<SrvMsgT>::SharedPtr advertiseService_T_(const std::string &srvMsgTopic, typename srv_cb_help_t<SrvMsgT>::srv_cb_perform_fun_t mfpCbPerform);

        bool srvCbSyncGetStcm_(slamware_ros_sdk::srv::SyncGetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncGetStcm::Response::SharedPtr resp);
        bool srvCbSyncSetStcm_(slamware_ros_sdk::srv::SyncSetStcm::Request::SharedPtr req, slamware_ros_sdk::srv::SyncSetStcm::Response::SharedPtr resp);

        // Relocalization request
        bool srvCbRelocalizationRequest_(slamware_ros_sdk::srv::RelocalizationRequest::Request::SharedPtr req,
                                         slamware_ros_sdk::srv::RelocalizationRequest::Response::SharedPtr resp);
        rclcpp::Service<slamware_ros_sdk::srv::RelocalizationRequest>::SharedPtr relocalization_request_srv_;
        void checkRelocalizationStatus();

    private:     
        boost::atomic<ServerState> state_;
        boost::atomic<bool> isStopRequested_;

        std::shared_ptr<rclcpp::Node> nh_;
        ServerParams params_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBrdcstr_;

        mutable boost::mutex workDatLock_;
        ServerWorkData_Ptr workDat_;

        std::vector<ServerWorkerBase_Ptr> serverWorkers_;

        // subscriptions  
        rclcpp::Subscription<slamware_ros_sdk::msg::SyncMapRequest>::SharedPtr subSyncMap_;

        rclcpp::Subscription<slamware_ros_sdk::msg::ClearMapRequest>::SharedPtr subClearMap_;
        rclcpp::Subscription<slamware_ros_sdk::msg::SetMapUpdateRequest>::SharedPtr subSetMapUpdate_;
        rclcpp::Subscription<slamware_ros_sdk::msg::SetMapLocalizationRequest>::SharedPtr subSetMapLocalization_;
        rclcpp::Subscription<slamware_ros_sdk::msg::RelocalizationCancelRequest>::SharedPtr subRelocalizationCancel_;

        
        // services
        rclcpp::Service<slamware_ros_sdk::srv::SyncGetStcm>::SharedPtr srvSyncGetStcm_;
        rclcpp::Service<slamware_ros_sdk::srv::SyncSetStcm>::SharedPtr srvSyncSetStcm_;


        boost::thread workThread_;

        mutable boost::mutex slamwarePltfmLock_;
        slamware_platform_t slamwarePltfm_;
        rp::standalone::aurora::RemoteSDK *auroraSdk_;

        boost::mutex auroraSdkLock_;
        boost::atomic_bool auroraSdkConnected_;
        boost::atomic_bool relocalization_active_;
        std::future<void> relocalization_future_;
        std::atomic<bool> cancel_requested_;
    };

}
