#include "edie_behavior/action_nodes/set_action.hpp"
#include "edie_behavior/edie_behavior_node.hpp"
#include "edie_behavior/singleton.hpp"

SetAction::SetAction(const string &name, const BT::NodeConfig &config)
 : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetAction::providedPorts()
{
  return {
    BT::InputPort<int>("num")
  };
}

BT::NodeStatus SetAction::tick()
{
  int n;
  getInput<int>("num", n);
  auto edie = *Singleton<shared_ptr<EdieBehaviorTree>>().GetInstance();

  edie->SetAction(n);

  return BT::NodeStatus::SUCCESS;
}