#include "edie_behavior/action_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus TryCharge::onStart()
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus TryCharge::onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void TryCharge::onHalted()
  {
  }

} // namespace aeirobot