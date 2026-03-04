
// ROS2 액션 서버의 콜백 함수들이 구현된 파일
// 모션 목표(goal)를 수신하고, 유효성 검사, 목표 실행, 취소 요청 처리 등을 담당
// 모션 실행의 제어 흐름과 액션 서버 인터페이스를 담당
// `ExecuteGoal` 함수에서 실제 모션 데이터를 로드하고, 모션 단계별로 다리와 귀의 위치를 계산하여 퍼블리시



#include "edie_motion/edie_motion_handler.hpp"

rclcpp_action::GoalResponse EdieMotionHandler::HandleGoal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const EdieMotion::Goal> goal)
{
    // 경고문구 지우기용
    (void)uuid;
    
    RCLCPP_INFO(this->get_logger(), "Received Motion Index: %u", goal->motion_index);

    // 예외: 실제 존재하는 키 기준으로 유효성 검사
    if (!config[std::to_string(goal->motion_index)]) {
        RCLCPP_ERROR(this->get_logger(), "Invalid motion index: %d. Please Send goal in: ", goal->motion_index);
        std::string key_list;
        for (auto it = config.begin(); it != config.end(); ++it) {
            key_list += it->first.as<std::string>() + " ";
        }
        RCLCPP_ERROR(this->get_logger(), "Available keys: %s", key_list.c_str());
        PubMotionDone(true);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 기존 목표가 실행 중이라면 취소 요청
    if (motion_goal_handle_ && motion_goal_handle_->is_active())
    {
        RCLCPP_INFO(this->get_logger(), "Canceling previous goal...");
        motion_goal_handle_->abort(std::make_shared<EdieMotion::Result>());
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EdieMotionHandler::HandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotion>> goal_handle)
{
    // 목표가 활성 상태인지 확인
    if (goal_handle->is_active()) {
        RCLCPP_INFO(this->get_logger(), "Canceling active goal...");

        auto result = std::make_shared<EdieMotion::Result>();
        result->motion_done = true;
        goal_handle->canceled(result);

        return rclcpp_action::CancelResponse::ACCEPT;
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "Goal is not active and cannot be canceled.");
        return rclcpp_action::CancelResponse::REJECT;
    }
}

void EdieMotionHandler::ExecuteGoal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotion>> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Received Motion Index: %u", goal->motion_index);

    goal_start_time_ = this->now();

    // 모션 데이터 로드
    LoadMotionData(goal->motion_index);

    // 예외 : 모션데이터 없음 처리
    if (motion_data_sequence.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No motion data found for index: %u", goal->motion_index);
        auto result = std::make_shared<EdieMotion::Result>();
        result->motion_done = false;
        goal_handle->abort(result);
        PubMotionDone(true);
        return;
    }

    // 전체 모션 시간
    double total_motion_time = 0.0;
    for (const auto &motion_data : motion_data_sequence)
    {
        total_motion_time += motion_data.motion_time;
    }

    remaining_motion_step_ = motion_data_sequence.size();
    auto feedback = std::make_shared<EdieMotion::Feedback>();
    rclcpp::Rate rate(100.0);  // 100 Hz

    // 모션 시작 시각
    goal_start_time_ = this->now();

    for (std::size_t step = 0; step < motion_data_sequence.size() && rclcpp::ok(); ++step)
    {
    const auto &md = motion_data_sequence[step];

    // ★ 이 스텝의 시작 시각
    const rclcpp::Time step_start = this->now();

    // ★★★ Leg Trajectory Goal 전송
    auto leg_goal = EdieMotionTrajectory::Goal();
    leg_goal.position = md.legs;  // [left_leg, right_leg]
    leg_goal.traj_type = "quintic";
    leg_goal.time = md.motion_time;
    
    if (!leg_traj_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Leg trajectory action server not available");
    } else {
        auto leg_send_goal_options = rclcpp_action::Client<EdieMotionTrajectory>::SendGoalOptions();
        leg_traj_action_client_->async_send_goal(leg_goal, leg_send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sent leg trajectory goal: L=%.2f, R=%.2f, time=%.2f", 
                    md.legs[0], md.legs[1], md.motion_time);
    }

    // ★★★ Ear Trajectory Goal 전송
    auto ear_goal = EdieMotionTrajectory::Goal();
    ear_goal.position = md.ears;  // [left ear, right ear]
    ear_goal.traj_type = "quintic";
    ear_goal.time = md.motion_time;

    if (!ear_traj_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Ear trajectory action server not available");
    } else {
        auto ear_send_goal_options = rclcpp_action::Client<EdieMotionTrajectory>::SendGoalOptions();
        ear_traj_action_client_->async_send_goal(ear_goal, ear_send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sent ear trajectory goal: L=%.2f, R=%.2f, time=%.2f", 
                    md.ears[0], md.ears[1], md.motion_time);
    }

    while (rclcpp::ok())
    {
        // 경과 시간 계산
        const rclcpp::Time now = this->now();
        const double elapsed_total = (now - goal_start_time_).seconds();       // 전체 기준
        const double elapsed_step  = (now - step_start).seconds();              // 스텝 기준
        const double remaining_step = std::max(md.motion_time - elapsed_step, 0.0);
        const double remaining_total = std::max(total_motion_time - elapsed_total, 0.0);

        // 피드백 채우기
        feedback->elapsed_time = elapsed_total;
        feedback->remaining_time = remaining_total;
        feedback->current_motion_step = step;
        feedback->remaining_motion_step = remaining_motion_step_ - step;
        feedback->l_leg_current_position = legs_joint_states_[0].position;
        feedback->r_leg_current_position = legs_joint_states_[1].position;
        feedback->l_ear_current_position = ears_joint_states_[0].position;
        feedback->r_ear_current_position = ears_joint_states_[1].position;
        goal_handle->publish_feedback(feedback);

        // 보기용 로그 (원하면 throttle)
        RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500, // 0.5초에 한 번
        "Step %zu | goal legs(L,R)=(%.2f, %.2f) ears(L,R)=(%.2f, %.2f) dt=%.2fs | "
        "Elapsed=%.2f Rem=%.2f (step rem=%.2f)",
        step, md.legs[0], md.legs[1], md.ears[0], md.ears[1],
        md.motion_time, elapsed_total, remaining_total, remaining_step
        );

        // 취소 전파
        if (goal_handle->is_canceling())
        {
        RCLCPP_INFO(this->get_logger(), "Goal canceled by client.");
        auto result = std::make_shared<EdieMotion::Result>();
        result->motion_done = true;
        goal_handle->canceled(result);
        return;
        }

        // ★★★ 스텝 종료 조건: 이 스텝의 지정 시간이 지나면 다음 스텝으로
        if (elapsed_step >= md.motion_time - 1e-6) {
        break;  // for 루프의 다음 step으로 진행
        }

        rate.sleep();
    }
    }

    // 목표 완료 처리
    auto result = std::make_shared<EdieMotion::Result>();
    result->motion_done = true;
    goal_handle->succeed(result);
    PubMotionDone(true);

    current_motion_step_ = 0;
    remaining_motion_step_ = 0;
    elapsed_time_ = 0.0;
    remaining_time_ = 0.0;
}