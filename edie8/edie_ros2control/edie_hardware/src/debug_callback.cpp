#include "edie_hardware/edie_hardware.hpp"

namespace edie_hardware
{

void EdieHardware::DebugModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Debug Mode: %d", msg->data);
  debug_mode_ = msg->data;
}

void EdieHardware::ResetOdometryCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req; // 요청이 필요하지 않을 경우 사용되지 않음을 명시
    RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Reset Odometry");

    // 응답 설정
    res->success = true;
    res->message = "Odometry reset";

    // 휠 위치 초기화
    joints_[0].state.position = 0.0;
    joints_[1].state.position = 0.0;
}

}