#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{

IsRandomWalkNeeded::IsRandomWalkNeeded(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList IsRandomWalkNeeded::providedPorts()
{
  return {};
}

BT::NodeStatus IsRandomWalkNeeded::tick()
{
  auto edie = Edie::GetInstance();
  if (!edie) {
    return BT::NodeStatus::FAILURE;
  }
  
  return edie->is_random_walk_needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace aeirobot
