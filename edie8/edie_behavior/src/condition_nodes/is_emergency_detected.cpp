#include "edie_behavior/condition_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus IsEmergencyDetected::tick()
  {
    return BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot