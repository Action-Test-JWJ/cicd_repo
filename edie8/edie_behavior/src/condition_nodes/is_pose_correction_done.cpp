#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/singleton.hpp"

namespace aeirobot
{

  BT::NodeStatus IsPoseCorrectionDone::tick()
  {
    auto edie = Edie::GetInstance();

    return (edie->is_pose_correction_done) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot