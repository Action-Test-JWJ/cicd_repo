#include "edie_behavior/condition_nodes.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::PortsList IsRobotMode::providedPorts()
  {
    return {
      BT::InputPort<string>("mode")
    };
  }
  
  BT::NodeStatus IsRobotMode::tick()
  {
    string mode;
    getInput<string>("mode", mode);
    auto edie = Edie::GetInstance();

    switch (edie->control_mode)
    {
    case Edie::ControlMode::k_auto:
      if(mode == "auto")
        return BT::NodeStatus::SUCCESS;
      break;
    case Edie::ControlMode::k_manual:
      if(mode == "manual")
        return BT::NodeStatus::SUCCESS;
      // 추가: manual_mode 세부 체크
      else if(mode == "remote" && edie->manual_mode == Edie::ManualMode::k_remote)
        return BT::NodeStatus::SUCCESS;
      else if(mode == "follow" && edie->manual_mode == Edie::ManualMode::k_follow)
        return BT::NodeStatus::SUCCESS;
      else if(mode == "home" && edie->manual_mode == Edie::ManualMode::k_home)
        return BT::NodeStatus::SUCCESS;
      else if(mode == "llm" && edie->manual_mode == Edie::ManualMode::k_llm)
        return BT::NodeStatus::SUCCESS;
      break;

    case Edie::ControlMode::k_sim:
      if(mode == "sim")
        return BT::NodeStatus::SUCCESS;
      break;
    }
    
    return BT::NodeStatus::FAILURE;
  }

} // namespace aeirobot