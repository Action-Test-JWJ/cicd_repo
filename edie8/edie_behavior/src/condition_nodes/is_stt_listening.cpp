#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  BT::NodeStatus IsSTTListening::tick()
  {
    if (Edie::GetInstance()->is_stt_listening)
    {
      return BT::NodeStatus::SUCCESS; // 듣고 있으면 SUCCESS
    }
    return BT::NodeStatus::FAILURE; // 안 듣고 있으면 FAILURE
  }
} // namespace aeirobot
