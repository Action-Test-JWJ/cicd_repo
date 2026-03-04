#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus IsVoraMode::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    auto mode = getInput<std::string>("mode");
    if (!mode)
    {
      return BT::NodeStatus::FAILURE;
    }

    if (edie->vora_mode_state == mode.value()) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
} // namespace aeirobot
