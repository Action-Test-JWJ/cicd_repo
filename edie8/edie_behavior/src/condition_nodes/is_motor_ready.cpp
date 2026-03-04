#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/singleton.hpp"

namespace aeirobot
{

  BT::NodeStatus IsMotorReady::tick()
  {
    auto edie = Edie::GetInstance();

    return (edie->is_motor_enable_state) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot