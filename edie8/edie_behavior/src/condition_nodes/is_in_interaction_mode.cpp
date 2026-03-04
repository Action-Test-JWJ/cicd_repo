#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus IsInInteractionMode::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    // interaction mode가 활성화되어 있으면 SUCCESS
    if (edie->is_in_interaction_mode) 
    {
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
  }
} // namespace aeirobot

