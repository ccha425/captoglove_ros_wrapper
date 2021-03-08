#include <captoglove_ros/captoglove_ros.h>
#include <signal.h>
#include <iostream>
#include <string>


namespace captoglove_ros {

    ros_translate::ros_translate() {
    }

    ros_translate::~ros_translate() {
    }

// Fingers
    captoglove_ros_msgs::FingerFeedbackMsg ros_translate::FingerFeedbackMsg_PB2ROS(captoglove_v1::FingerFeedbackMsg pb_msg) {

        captoglove_ros_msgs::FingerFeedbackMsg ros_msg;

        ros_msg.ThumbFinger = pb_msg.thumb_finger();
        ros_msg.IndexFinger = pb_msg.index_finger();
        ros_msg.MiddleFinger = pb_msg.middle_finger();
        ros_msg.RingFinger = pb_msg.ring_finger();
        ros_msg.LittleFinger = pb_msg.little_finger();

        return ros_msg;

    }

    captoglove_ros_wrapper::BatteryLevelMsg ros_translate::BatteryLevelMsg_PB2ROS(captoglove_v1::BatteryLevelMsg pb_msg) {

        captoglove_ros_wrapper::BatteryLevelMsg ros_msg;

        ros_msg.BatteryLevel = pb_msg.level();

        return ros_msg;

    }

    captoglove_ros_wrapper::DeviceInformationMsg ros_translate::DeviceInformationMsg_PB2ROS(captoglove_v1::DeviceInformationMsg pb_msg) {

        captoglove_ros_wrapper::DeviceInformationMsg ros_msg;

        ros_msg.DeviceName = pb_msg.device_name();
        //ros_msg.DeviceAddress = pb_msg.device_address();
        // TODO: Maybe add preffered connection parameters

        return ros_msg;

    }

}

