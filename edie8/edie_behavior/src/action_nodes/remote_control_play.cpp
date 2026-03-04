#include "edie_behavior/action_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus RemoteControlPlay::onStart()
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus RemoteControlPlay::onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void RemoteControlPlay::onHalted()
  {
  }

} // namespace aeirobot