#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{

IsHumanVisible::IsHumanVisible(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

BT::PortsList IsHumanVisible::providedPorts()
{
  return {};
}

BT::NodeStatus IsHumanVisible::tick()
{
  auto edie = Edie::GetInstance();
  if (!edie) {
    return BT::NodeStatus::FAILURE;
  }
  
  return edie->is_human_visible_now ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace aeirobot
