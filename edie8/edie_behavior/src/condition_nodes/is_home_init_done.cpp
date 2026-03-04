#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/edie.hpp"

namespace aeirobot
{

BT::NodeStatus IsHomeInitDone::tick()
{
  auto edie = Edie::GetInstance();
  if (edie->is_home_init_done)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot
