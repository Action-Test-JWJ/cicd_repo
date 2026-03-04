#ifndef POSE_SET_HPP
#define POSE_SET_HPP

// C++ system files
#include <chrono>
#include <cmath>

// ROS 관련 헤더
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 다른 프로젝트 헤더
#include "edie_msgs/msg/pose_with_info_stamped.hpp"

// 현재 프로젝트 헤더
#include "edie_localization_manager/utility/types.hpp"

// 구조체들
struct GameParam
{
    int robot_number = -1;
    int team_number = -1;
};

struct RobotInfo
{
    bool goalKeeper = false;
    uint8_t penalty = 0;
    uint8_t yellowCardCount = 0;
    uint8_t redCardCount = 0;
};

struct GameInfo {
    uint8_t team_index;
    uint8_t robot_index;
    uint8_t kickoff_team;
    uint8_t state;
    uint8_t secondary_state;
    uint8_t secondary_state_info;
};

struct PosParam
{
    std::vector<double> init_left;
    std::vector<double> init_right;
    std::vector<double> ready_range_left;
    std::vector<double> ready_range_right;
    std::vector<double> field_range_x;
    std::vector<double> field_range_y;
};

struct PenaltyInfo {
    bool is_penalized = false;
    std::chrono::steady_clock::time_point penalty_last_time;
    static constexpr double PENALTY_END_DELAY_SECONDS = 3.0;
};

struct HoldingInfo {
    bool is_holding = false;
    bool holding_check = false;
    bool holding_completed = false;
    std::chrono::steady_clock::time_point holding_start_time;
    std::chrono::steady_clock::time_point holding_end_time;
    static constexpr double HOLDING_CHECK_DURATION_MS = 1000.0;
    static constexpr double HOLDING_COMPLETE_DURATION_MS = 3000.0;
};

namespace pose_set
{

    // 파라미터 설정 함수들
    void SetPenaltyInfo(const PenaltyInfo& penalty_info);
    void SetHoldingInfo(const HoldingInfo& holding_info);

    // 메인 함수
    edie_msgs::msg::PoseWithInfoStamped SetPose(bool game_data_received);
    bool DeciderGameStatePose(GameInfo& game, RobotInfo& robot);



    void UpdateStatusPenalty(const RobotInfo& robot);
    void UpdateStatusHolding(bool is_lifting_now);

    bool SetPosePenalty(const GameInfo& game);
    bool SetPoseInit(GameInfo&  game);

    // 초기화 함수
    void Initialize();
}

#endif // POSE_SET_HPP
