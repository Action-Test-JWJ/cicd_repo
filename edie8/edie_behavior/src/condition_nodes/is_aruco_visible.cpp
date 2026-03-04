#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/singleton.hpp"

namespace aeirobot
{

  BT::NodeStatus IsArucoVisible::tick()
  {
    auto edie = Edie::GetInstance();

    return (edie->is_aruco_visible) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot