#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{
  ResetAngryRunaway::ResetAngryRunaway(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus ResetAngryRunaway::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;
    
    edie->is_surprised_runaway = false;
    RCLCPP_INFO(rclcpp::get_logger("ResetAngryRunaway"), 
                "[ResetAngryRunaway] Runaway complete. Resetting flag.");
    return BT::NodeStatus::SUCCESS;
  }
} // namespace aeirobot

