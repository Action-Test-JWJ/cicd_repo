#include "edie_behavior/action_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus SendEmergencySignal::onStart()
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus SendEmergencySignal::onRunning()
  {
    return BT::NodeStatus::SUCCESS;
  }

  void SendEmergencySignal::onHalted()
  {
  }

} // namespace aeirobot