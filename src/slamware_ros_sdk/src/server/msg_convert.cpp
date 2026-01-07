
#include "msg_convert.h"

#include <stdexcept>

namespace slamware_ros_sdk {

	//////////////////////////////////////////////////////////////////////////

    void MsgConvert<slamware_ros_sdk::msg::MapKind, rpos::features::location_provider::MapKind>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        switch (sltcVal)
        {
        case rpos::features::location_provider::EXPLORERMAP:
            rosMsg.kind = ros_msg_t::EXPLORERMAP;
            break;
        case rpos::features::location_provider::SWEEPERMAP:
            rosMsg.kind = ros_msg_t::SWEEPERMAP;
            break;
        case rpos::features::location_provider::UWBMAP:
            rosMsg.kind = ros_msg_t::UWBMAP;
            break;
        case rpos::features::location_provider::SLAMMAP:
            rosMsg.kind = ros_msg_t::SLAMMAP;
            break;
        case rpos::features::location_provider::LOCALSLAMMAP:
            rosMsg.kind = ros_msg_t::LOCALSLAMMAP;
            break;
        default:
            rosMsg.kind = ros_msg_t::UNKNOWN;
            break;
        }
    }

    void MsgConvert<slamware_ros_sdk::msg::MapKind, rpos::features::location_provider::MapKind>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        switch (rosMsg.kind)
        {
        case ros_msg_t::EXPLORERMAP:
            sltcVal = rpos::features::location_provider::EXPLORERMAP;
            break;
        case ros_msg_t::SWEEPERMAP:
            sltcVal = rpos::features::location_provider::SWEEPERMAP;
            break;
        case ros_msg_t::UWBMAP:
            sltcVal = rpos::features::location_provider::UWBMAP;
            break;
        case ros_msg_t::SLAMMAP:
            sltcVal = rpos::features::location_provider::SLAMMAP;
            break;
        case ros_msg_t::LOCALSLAMMAP:
            sltcVal = rpos::features::location_provider::LOCALSLAMMAP;
            break;
        default:
            //sltcVal = ((rpos::features::location_provider::MapKind)-1);
            throw std::runtime_error("no MapKind::UNKNOWN in RPOS.");
            break;
        }
    }

    void MsgConvert<geometry_msgs::msg::Point, rpos::core::Location>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
    	rosMsg.x = sltcVal.x();
    	rosMsg.y = sltcVal.y();
    	rosMsg.z = sltcVal.z();
    }

    void MsgConvert<geometry_msgs::msg::Point, rpos::core::Location>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
    	sltcVal.x() = rosMsg.x;
    	sltcVal.y() = rosMsg.y;
    	sltcVal.z() = rosMsg.z;
    }

    void MsgConvert<tf2::Quaternion, rpos::core::Rotation>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        // rosMsg = tf2_ros::createQuaternionMsgFromRollPitchYaw(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());
        tf2::Quaternion tf2Quat;
        tf2Quat.setRPY(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());
        rosMsg = tf2Quat;
    }

    void MsgConvert<tf2::Quaternion, rpos::core::Rotation>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        if (0.0 == rosMsg.x() && 0.0 == rosMsg.y() && 0.0 == rosMsg.z() && 0.0 == rosMsg.w())
        {
            sltcVal = sltc_val_t(0.0, 0.0, 0.0);
            return;
        }

        const tf2::Quaternion tq(rosMsg.x(), rosMsg.y(), rosMsg.z(), rosMsg.w());
        const tf2::Matrix3x3 tMat(tq);
        tMat.getRPY(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());

        if (std::isnan(sltcVal.roll()))
        {
            RCLCPP_WARN(rclcpp::get_logger("msg convert"), "Quaternion to RPY, roll is nan, set to zero.");
            sltcVal.roll() = 0.0;
        }
        if (std::isnan(sltcVal.pitch()))
        {
            RCLCPP_WARN(rclcpp::get_logger("msg convert"), "Quaternion to RPY, pitch is nan, set to zero.");
            sltcVal.pitch() = 0.0;
        }
        if (std::isnan(sltcVal.yaw()))
        {
            RCLCPP_WARN(rclcpp::get_logger("msg convert"), "Quaternion to RPY, yaw is nan, set to zero.");
            sltcVal.yaw() = 0.0;
        }
    }

    void MsgConvert<geometry_msgs::msg::Quaternion, rpos::core::Rotation>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        tf2::Quaternion tf2Quat;
        tf2Quat.setRPY(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());

        geometry_msgs::msg::Quaternion geoQuat = tf2::toMsg(tf2Quat);
        rosMsg = geoQuat;
    }

    void MsgConvert<geometry_msgs::msg::Quaternion, rpos::core::Rotation>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        tf2::Quaternion tf2Quat = tf2::impl::toQuaternion(rosMsg);
        if (0.0 == tf2Quat.x() && 0.0 == tf2Quat.y() && 0.0 == tf2Quat.z() && 0.0 == tf2Quat.w())
        {
            sltcVal = sltc_val_t(0.0, 0.0, 0.0);
            return;
        }

        const tf2::Quaternion tq(tf2Quat.x(), tf2Quat.y(), tf2Quat.z(), tf2Quat.w());
        const tf2::Matrix3x3 tMat(tq);
        tMat.getRPY(sltcVal.roll(), sltcVal.pitch(), sltcVal.yaw());

        if (std::isnan(sltcVal.roll()))
        {
            RCLCPP_WARN(rclcpp::get_logger("msg convert"), "geometry_msgs::msg::Quaternion to RPY, roll is nan, set to zero.");
            sltcVal.roll() = 0.0;
        }
        if (std::isnan(sltcVal.pitch()))
        {
            RCLCPP_WARN(rclcpp::get_logger("msg convert"), "geometry_msgs::msg::Quaternion to RPY, pitch is nan, set to zero.");
            sltcVal.pitch() = 0.0;
        }
        if (std::isnan(sltcVal.yaw()))
        {
            RCLCPP_WARN(rclcpp::get_logger("msg convert"), "geometry_msgs::msg::Quaternion to RPY, yaw is nan, set to zero.");
            sltcVal.yaw() = 0.0;
        }
    }

    void MsgConvert<geometry_msgs::msg::Pose, rpos::core::Pose>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        sltcToRosMsg(sltcVal.location(), rosMsg.position);
        sltcToRosMsg(sltcVal.rotation(), rosMsg.orientation);
    }

    void MsgConvert<geometry_msgs::msg::Pose, rpos::core::Pose>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        rosMsgToSltc(rosMsg.position, sltcVal.location());
        rosMsgToSltc(rosMsg.orientation, sltcVal.rotation());
    }

    void MsgConvert<slamware_ros_sdk::msg::Vec2DInt32, rpos::core::Vector2i>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
    }

    void MsgConvert<slamware_ros_sdk::msg::Vec2DInt32, rpos::core::Vector2i>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y);
    }

    void MsgConvert<slamware_ros_sdk::msg::Vec2DFlt32, rpos::core::Vector2f>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
    }

    void MsgConvert<slamware_ros_sdk::msg::Vec2DFlt32, rpos::core::Vector2f>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y);
    }

    void MsgConvert<slamware_ros_sdk::msg::Vec2DFlt32, rpos::core::Point>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
    }

    void MsgConvert<slamware_ros_sdk::msg::Vec2DFlt32, rpos::core::Point>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y);
    }

    void MsgConvert<slamware_ros_sdk::msg::RectInt32, rpos::core::RectangleI>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
        rosMsg.w = sltcVal.width();
        rosMsg.h = sltcVal.height();
    }

    void MsgConvert<slamware_ros_sdk::msg::RectInt32, rpos::core::RectangleI>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y, rosMsg.w, rosMsg.h);
    }

    void MsgConvert<slamware_ros_sdk::msg::RectFlt32, rpos::core::RectangleF>::toRos(const sltc_val_t& sltcVal, ros_msg_t& rosMsg)
    {
        rosMsg.x = sltcVal.x();
        rosMsg.y = sltcVal.y();
        rosMsg.w = sltcVal.width();
        rosMsg.h = sltcVal.height();
    }

    void MsgConvert<slamware_ros_sdk::msg::RectFlt32, rpos::core::RectangleF>::toSltc(const ros_msg_t& rosMsg, sltc_val_t& sltcVal)
    {
        sltcVal = sltc_val_t(rosMsg.x, rosMsg.y, rosMsg.w, rosMsg.h);
    }


    //////////////////////////////////////////////////////////////////////////
    
}
