#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/edie.hpp"

namespace aeirobot
{

BT::NodeStatus IsStationOutValue::tick()
{
  auto edie = Edie::GetInstance();
  
  // Get expected value from port
  int expected_value;
  if (!getInput<int>("value", expected_value))
  {
    throw BT::RuntimeError("IsStationOutValue: missing required input [value]");
  }
  
  // Check if station_out_command matches expected value
  if (edie->station_out_command == static_cast<uint8_t>(expected_value))
  {
    // Reset the command after checking (consume it)
    edie->station_out_command = 0;
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot

