
// ROS2 토픽 콜백 함수들이 구현된 파일
// 토픽 기반 입력 처리와 액션 클라이언트 호출을 담당
// `/joint_states` 토픽을 구독하여 다리와 귀의 현재 관절 상태를 업데이트
// 파라미터 변경 콜백을 처리
// `/edie8/emotion/motion_index` 토픽을 구독하여 모션 인덱스 메시지를 받으면 액션 클라이언트로 목표를 전송하는 역할


#include "edie_motion/edie_motion_handler.hpp"

void EdieMotionHandler::JointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    legs_joint_states_[0].position = msg->position[2];
    legs_joint_states_[1].position = msg->position[3];
    ears_joint_states_[0].position = msg->position[4];
    ears_joint_states_[1].position = msg->position[5];
}

// rcl_interfaces::msg::SetParametersResult EdieMotionHandler::ParameterCallback(const std::vector<rclcpp::Parameter> &parameters)
// {
//     rcl_interfaces::msg::SetParametersResult result;
//     result.successful = true;

//     for (const auto &param : parameters)
//     {
//         if (param.get_name() == "trajectory_type")
//         {
//             // 새 파라미터 값 처리
//             trajectory_type_ = param.as_string();
//             RCLCPP_INFO(this->get_logger(), "trajectory_type updated to: %s", trajectory_type_.c_str());
//         }
//         else
//         {
//             // 처리하지 않는 파라미터에 대한 실패 처리 예제
//             result.successful = false;
//             result.reason = "Unsupported parameter.";
//         }
//     }

//     return result;
// }

void EdieMotionHandler::MotionIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    auto goal_msg = EdieMotion::Goal();
    goal_msg.motion_index = msg->data;

    motion_action_client_->async_send_goal(goal_msg);
}