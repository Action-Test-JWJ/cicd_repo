#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/singleton.hpp"

namespace aeirobot
{

  BT::NodeStatus IsReadyToAction::tick()
  {
    auto edie = Edie::GetInstance();

    return (edie->is_action_ready) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot