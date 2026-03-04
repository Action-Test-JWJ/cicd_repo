#include "edie_behavior/action_nodes.hpp"
#include <chrono>

namespace aeirobot
{
  WaitAction::WaitAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {}

  BT::PortsList WaitAction::providedPorts()
  {
    return{ BT::InputPort<int>("msec") };
  }

  BT::NodeStatus WaitAction::onStart()
  {
    int msec;
    if (!getInput("msec", msec)) {
      return BT::NodeStatus::FAILURE;
    }
    
    deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(msec);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus WaitAction::onRunning()
  {
    if (std::chrono::steady_clock::now() < deadline_)
    {
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      return BT::NodeStatus::SUCCESS;
    }
  }

  void WaitAction::onHalted()
  {
  }
} // namespace aeirobot
