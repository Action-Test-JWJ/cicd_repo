#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus IsLLMState::tick()
  {
    if (Edie::GetInstance()->is_llm_state)
    {
      return BT::NodeStatus::SUCCESS; // LLM이 활성 상태이면 SUCCESS
      // RCLCPP_INFO(rclcpp::get_logger("IsLLMState"), "[IsLLMState] LLM is active.");
    }
    // RCLCPP_INFO(rclcpp::get_logger("IsLLMState"), "[IsLLMState] LLM is inactive.");
    return BT::NodeStatus::FAILURE; // LLM이 비활성 상태이면 FAILURE
  }
} // namespace aeirobot
