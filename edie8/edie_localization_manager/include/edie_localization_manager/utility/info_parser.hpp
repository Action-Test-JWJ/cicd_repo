#ifndef INFO_PARSER_HPP
#define INFO_PARSER_HPP

#include "edie_msgs/msg/pose_with_info_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace info_parser
{

enum class LocalizationScene : uint8_t {
    ROBOCUP = 1,
    LIBRARY = 2, //예시
    TEST = 3, //예시 입니당
    UNKNOWN = 0
};
enum class PoseMode : uint8_t {
    SET_POSE = 1,
    FUSION_POSE = 2, //??? 이름은
    UNKNOWN = 0
};
enum class RoboCupPoseSetType : uint8_t {
    INIT = 1,
    PENALTY = 2,
    CORNER_KICK = 3,
    KIDNAP = 4,
    UNKNOWN = 0
};

struct ParsedPoseInfo {
    PoseMode pose_mode = PoseMode::UNKNOWN;
    bool is_pose_mode = false;
    LocalizationScene localization_scene = LocalizationScene::UNKNOWN;
    RoboCupPoseSetType pose_set_type = RoboCupPoseSetType::UNKNOWN;
};


// 메인 파싱 함수
ParsedPoseInfo ParsePoseInfo(const edie_msgs::msg::PoseWithInfoStamped::SharedPtr msg);


} // namespace info_parser

#endif // info_parser_HPP
