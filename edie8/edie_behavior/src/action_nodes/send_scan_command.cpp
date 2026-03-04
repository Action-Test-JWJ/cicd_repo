#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{

BT::NodeStatus SendScanCommand::tick()
{
  bool command = true;
  getInput("command", command);

  Edie::GetInstance()->PubScanCommand(command);

  return BT::NodeStatus::SUCCESS;
}

} // namespace aeirobot
