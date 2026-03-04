#include "edie_behavior/action_nodes.hpp"
#include "edie_behavior/edie.hpp"

namespace aeirobot
{

BT::NodeStatus SetHomeInitDone::tick()
{
  auto edie = Edie::GetInstance();
  edie->is_home_init_done = true;
  return BT::NodeStatus::SUCCESS;
}

} // namespace aeirobot
