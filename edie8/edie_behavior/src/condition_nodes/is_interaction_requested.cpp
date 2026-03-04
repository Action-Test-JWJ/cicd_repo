#include "edie_behavior/condition_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus IsInteractionRequested::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    // 원샷 소비: 이전 값이 true면 SUCCESS 반환 + 즉시 false로 클리어
    if (edie->is_ready_emotion) {
      // RCLCPP_INFO(rclcpp::get_logger("IsInteractionRequested"), "[IsInteractionRequested] is_ready_emotion: %d", edie->is_ready_emotion);
      edie->is_ready_emotion = false;
      // RCLCPP_INFO(rclcpp::get_logger("IsInteractionRequested"), "[IsInteractionRequested] is_ready_emotion: %d", edie->is_ready_emotion);
      // RCLCPP_INFO(rclcpp::get_logger("IsInteractionRequested"), "[IsInteractionRequested] 111111111111111111111111111111111111111111111");
      return BT::NodeStatus::SUCCESS;
    }
    // RCLCPP_INFO(rclcpp::get_logger("IsInteractionRequested"), "[IsInteractionRequested] is_ready_emotion: false %d", edie->is_ready_emotion);
    // RCLCPP_INFO(rclcpp::get_logger("IsInteractionRequested"), "[IsInteractionRequested] 222222222222222222222222222222222222222222222");
    return BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot