#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{
  SetWaitDuration::SetWaitDuration(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

  BT::PortsList SetWaitDuration::providedPorts()
  {
    return { 
      BT::InputPort<int>("stage", "The waiting stage to configure"),
      BT::InputPort<double>("duration_sec", "The duration for the specified stage")
    };
  }

  BT::NodeStatus SetWaitDuration::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    int stage;
    double duration;

    if (!getInput<int>("stage", stage)) return BT::NodeStatus::FAILURE;
    if (!getInput<double>("duration_sec", duration)) return BT::NodeStatus::FAILURE;

    edie->wait_stage_durations_sec_[stage] = duration;
    // This node always succeeds after setting the value.
    return BT::NodeStatus::SUCCESS;
  }

} // namespace aeirobot
