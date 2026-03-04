#include "edie_localization_manager/utility/info_parser.hpp"

namespace info_parser
{

ParsedPoseInfo ParsePoseInfo(const edie_msgs::msg::PoseWithInfoStamped::SharedPtr msg)
{
    ParsedPoseInfo result;

    if (!msg || msg->info.size() < 1) {
        RCLCPP_WARN(rclcpp::get_logger("info_parser"), "Invalid pose info message");
        result.pose_mode = PoseMode::UNKNOWN;
        result.is_pose_mode = false;
        return result;
    }

    // Parse info[0] - pose mode
    if (msg->info[0].key == "set_pose") {
        result.pose_mode = PoseMode::SET_POSE;
        result.is_pose_mode = (msg->info[0].value == "true");

        if (msg->info[1].key == "robocup") {
            result.localization_scene = LocalizationScene::ROBOCUP;

            // 문자열로 파싱
            if (msg->info[1].value == "init") {
                result.pose_set_type = RoboCupPoseSetType::INIT;
            } else if (msg->info[1].value == "penalty") {
                result.pose_set_type = RoboCupPoseSetType::PENALTY;
            } else if (msg->info[1].value == "corner_kick") {
                result.pose_set_type = RoboCupPoseSetType::CORNER_KICK;
            } else if (msg->info[1].value == "kidnap") {
                result.pose_set_type = RoboCupPoseSetType::KIDNAP;
            } else {
                result.pose_set_type = RoboCupPoseSetType::UNKNOWN;
                RCLCPP_WARN(rclcpp::get_logger("info_parser"),
                    "Unknown pose set type: %s", msg->info[1].value.c_str());
            }
        }
    }
    else if (msg->info[0].key == "fusion_pose") {
        result.pose_mode = PoseMode::FUSION_POSE;
        result.is_pose_mode = (msg->info[0].value == "true");

    }

    return result;
}
} // namespace info_parser
