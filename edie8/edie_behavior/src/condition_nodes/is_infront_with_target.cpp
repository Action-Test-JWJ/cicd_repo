#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus IsInfrontWithTarget::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;
    return edie->debounced_is_infront_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
} // namespace aeirobot
