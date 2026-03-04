#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  IsWaitingForTouch::IsWaitingForTouch(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList IsWaitingForTouch::providedPorts()
  {
    return { BT::InputPort<int>("stage") };
  }

  BT::NodeStatus IsWaitingForTouch::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) {
      return BT::NodeStatus::FAILURE;
    }

    auto expected_stage = getInput<int>("stage");
    if (!expected_stage) {
      return BT::NodeStatus::FAILURE;
    }

    if (edie->infront_waiting_stage_ == expected_stage.value()) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
} // namespace aeirobot
