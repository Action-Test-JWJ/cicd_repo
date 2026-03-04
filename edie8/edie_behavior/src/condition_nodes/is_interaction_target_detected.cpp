#include "edie_behavior/condition_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus IsInteractionTargetDetected::tick()
  {
    return BT::NodeStatus::SUCCESS;
  }

} // namespace aeirobot