#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus SwitchVoraMode::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    auto mode = getInput<std::string>("mode");
    if (!mode)
    {
      return BT::NodeStatus::FAILURE;
    }

    // 이미 요청한 모드와 같으면 중복 발행 방지
    if (edie->vora_mode_state == mode.value()) {
      RCLCPP_INFO(edie->ros_manager->get_logger(), 
                  "[SwitchVoraMode] Already in '%s' mode. Skip.", mode.value().c_str());
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(edie->ros_manager->get_logger(), 
                "[SwitchVoraMode] Switching to '%s' mode...", mode.value().c_str());
    edie->PubVoraModeSwitchRequest(mode.value());
    return BT::NodeStatus::SUCCESS;
  }
} // namespace aeirobot
