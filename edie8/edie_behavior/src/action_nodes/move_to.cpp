#include "edie_behavior/action_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::PortsList MoveTo::providedPorts()
  {
    return {
      BT::InputPort<string>("location")
    };
  }
  
  BT::NodeStatus MoveTo::onStart()
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus MoveTo::onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void MoveTo::onHalted()
  {
  }

} // namespace aeirobot