#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{
  ResetWaitingStatus::ResetWaitingStatus(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus ResetWaitingStatus::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) {
      return BT::NodeStatus::FAILURE;
    }

    if (edie->infront_waiting_stage_ != 0) {
      RCLCPP_INFO(edie->ros_manager->get_logger(), "[ResetWaitingStatus] Resetting wait stage from %d to 0.", edie->infront_waiting_stage_);
      edie->infront_waiting_stage_ = 0;
    }

    return BT::NodeStatus::SUCCESS;
  }
} // namespace aeirobot
