#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus IsAngryRunaway::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    // 놀라서 도망가기 플래그가 true이면 SUCCESS
    if (edie->is_surprised_runaway) 
    {
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
  }
} // namespace aeirobot

