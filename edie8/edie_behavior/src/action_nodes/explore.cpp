#include "edie_behavior/action_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus Explore::onStart()
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus Explore::onRunning()
  {
    auto edie = Edie::GetInstance();

    // manual 모드인데 follow가 아니면 Explore 종료
    if (edie->control_mode == Edie::ControlMode::k_manual && 
        edie->manual_mode != Edie::ManualMode::k_follow)
    {
      // Mode changed, stop exploring and return SUCCESS to allow BT to re-evaluate
      return BT::NodeStatus::SUCCESS;
    }

    if (edie->vora_mode_state == "emotion") {
      return BT::NodeStatus::FAILURE;
    }

    // 1) Continuously execute the follow/search logic.
    // This function will make the robot rotate to find a person,
    // or approach if a person is found.
    edie->FollowTarget();

    return BT::NodeStatus::RUNNING;
  }

  void Explore::onHalted()
  {
    // Stop the robot if this action is preempted.
    auto edie = Edie::GetInstance();
    edie->PubRemote(0.0, 0.0);
  }

} // namespace aeirobot