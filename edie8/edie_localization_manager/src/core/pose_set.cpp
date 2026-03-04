#include "edie_localization_manager/core/pose_set.hpp"

namespace pose_set
{

static GameParam game_param;
static PosParam pos_param;
static PenaltyInfo penalty_info;
static HoldingInfo holding_info;

static GameInfo game;
static RobotInfo robot;

static edie_msgs::msg::PoseWithInfoStamped set_pose;
static bool ready_check = false;
// 미사용 변수 제거
// static double field_range_x_[2];
// static double field_range_y_[2];

// 파라미터 설정 함수들
void SetPenaltyInfo(const PenaltyInfo& info) { penalty_info = info; }
void SetHoldingInfo(const HoldingInfo& info) { holding_info = info; }

edie_msgs::msg::PoseWithInfoStamped SetPose(bool game_data_received)
{
    edie_msgs::msg::PoseWithInfoStamped result;
    if (!game_data_received)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("pose_set"), "No game data received");

        diagnostic_msgs::msg::KeyValue info;
        info.key = "set_pose";
        info.value = "false";
        result.info.push_back(info);

        result.pose.x = 0.0;
        result.pose.y = 0.0;
        result.pose.theta = 0.0;
        return result;
    }

    return result;
}
bool DeciderGameStatePose(GameInfo& game, RobotInfo& robot)
{
    // 1. 패널티 상황 체크
    UpdateStatusPenalty(robot);

    if (penalty_info.is_penalized)
    {
        return SetPosePenalty(game);
    }

    // 2. 게임 상태별 set pose 판단
    switch (game.state)
    {
        case 0: // INIT

            return SetPoseInit(game);

        case 2: // SET
            return false;

        case 1: // READY
            ready_check = true;
            return false; // READY에서는 set pose 불필요

        case 3: // PLAY

        default:
            return false;
    }
}

void UpdateStatusHolding(bool is_lifting_now)
{
    auto now = std::chrono::steady_clock::now();

    // 로봇 들기 시작 (이전엔 안들려있다가)
    if (is_lifting_now && !holding_info.is_holding)
    {
        holding_info.holding_start_time = now;
        holding_info.holding_check = false;
        holding_info.is_holding = false;  // 아직 확정 아님
        holding_info.holding_completed = false;

        RCLCPP_INFO(rclcpp::get_logger("pose_set"), "Lifting started (checking 1s...)");
        return;
    }

    // 로봇 계속 들고 있음 → 1초 넘으면 lift 인정
    if (is_lifting_now)
    {
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - holding_info.holding_start_time).count();

        if (!holding_info.is_holding && duration >= HoldingInfo::HOLDING_CHECK_DURATION_MS)
        {
            holding_info.is_holding = true;
            holding_info.holding_check = true;
            RCLCPP_INFO(rclcpp::get_logger("pose_set"),
                "Lifting confirmed after %.1f ms", duration);
        }
        return;
    }

    // 들었던 로봇 내려놓음 (false로 변경됨)
    if (!is_lifting_now && holding_info.is_holding)
    {
        auto holding_end_time = now;
        double certified_lift_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            holding_end_time - holding_info.holding_start_time).count();

        if (certified_lift_duration >= HoldingInfo::HOLDING_COMPLETE_DURATION_MS)
        {
            holding_info.holding_completed = true;
            RCLCPP_INFO(rclcpp::get_logger("pose_set"),
                "Holding completed after %.1f ms - Reposition triggered", certified_lift_duration);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("pose_set"),
                "Holding ended too soon (%.1f ms) - Ignored", certified_lift_duration);
        }

        // 상태 초기화
        holding_info.is_holding = false;
        holding_info.holding_check = false;
        return;
    }

    // 4. 들다가 1초 안돼서 바로 놓은 경우
    if (!is_lifting_now && !holding_info.is_holding && holding_info.holding_check)
    {
        holding_info.holding_check = false;
        RCLCPP_INFO(rclcpp::get_logger("pose_set"),
            "Lift cancelled before 1s");
    }
}

void UpdateStatusPenalty(const RobotInfo& robot)
{

    if (robot.penalty)
    {
        // 패널티 시작
        penalty_info.is_penalized = true;
        penalty_info.penalty_last_time = std::chrono::steady_clock::now(); //페널티먹고 있는 시간 계속 업데이트
    }
    else if(!robot.penalty && penalty_info.is_penalized)
    {
        // 패널티 종료 - 즉시 false로 하지 않고 시간 3초 대기
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - penalty_info.penalty_last_time).count() / 1000.0;

        if (elapsed >= PenaltyInfo::PENALTY_END_DELAY_SECONDS)
        {
            penalty_info.is_penalized = false;
            RCLCPP_INFO(rclcpp::get_logger("pose_set"),
                "Penalty status reset after %.1f seconds", elapsed);
        }
    }
}

bool SetPosePenalty(const GameInfo& game)
{
    RCLCPP_INFO(rclcpp::get_logger("pose_set"), "SetPosePenalty");
    if (game.team_index == 1) // 오른쪽 팀
    {
        set_pose.pose.x = pos_param.init_right[0];
        set_pose.pose.y = pos_param.init_right[1];
        set_pose.pose.theta = pos_param.init_right[2];

    }
    else // 왼쪽 팀
    {
        set_pose.pose.x = pos_param.init_left[0];
        set_pose.pose.y = pos_param.init_left[1];
        set_pose.pose.theta = pos_param.init_left[2];
    }
    set_pose.info.clear();
    set_pose.info.resize(2);  // 크기를 2로 설정

    set_pose.info[0].key = "set_pose";
    set_pose.info[0].value = "true";
    set_pose.info[1].key = "robocup";
    set_pose.info[1].value = "penalty";


    return true;
}

bool SetPoseInit(GameInfo& game)
{
    // if (!ready_check) //아직 ready 전
    // {
        RCLCPP_INFO(rclcpp::get_logger("pose_set"), "INIT REPOSITION");
        set_pose.info.clear();



        if (game.team_index == 1) //오른쪽팀
        {
            set_pose.pose.x = pos_param.init_right[0];
            set_pose.pose.y = pos_param.init_right[1];
            set_pose.pose.theta = pos_param.init_right[2];
        }
        else //왼쪽팀
        {
            set_pose.pose.x = pos_param.init_left[0];
            set_pose.pose.y = pos_param.init_left[1];
            set_pose.pose.theta = pos_param.init_left[2];

        }
        diagnostic_msgs::msg::KeyValue info;
        // info[0].key = "set_pose";
        // info.value = "true";
        // set_pose.info.push_back(info);
        // info.key = "robocup";
        // info.value = "init";
        // set_pose.info.push_back(info);
        set_pose.info.clear();
        set_pose.info.resize(2);  // 크기를 2로 설정

        set_pose.info[0].key = "set_pose";
        set_pose.info[0].value = "true";
        set_pose.info[1].key = "robocup";
        set_pose.info[1].value = "init";

        return true;
    // }
    //비교하는거 그거 꼭 해야함 나중에!! set_pose 확인해보세용
    // return false;
}

} // namespace pose_set
