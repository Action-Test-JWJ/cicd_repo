#include "edie_behavior/action_nodes/set_state.hpp"
#include "edie_behavior/edie_behavior_node.hpp"
#include "edie_behavior/singleton.hpp"

SetState::SetState(const string &name, const BT::NodeConfig &config)
 : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetState::providedPorts()
{
  return {
    BT::InputPort<int>("num")
  };
}

BT::NodeStatus SetState::tick()
{
  int n;
  getInput<int>("num", n);
  auto edie = *Singleton<shared_ptr<EdieBehaviorTree>>().GetInstance();

  edie->SetState(n);

  return BT::NodeStatus::SUCCESS;
}