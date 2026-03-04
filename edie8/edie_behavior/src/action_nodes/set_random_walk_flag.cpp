#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{

SetRandomWalkFlag::SetRandomWalkFlag(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList SetRandomWalkFlag::providedPorts()
{
  // "value" 라는 이름의 bool 타입 입력 포트를 정의합니다.
  return { BT::InputPort<bool>("value", "The value to set the flag to") };
}

BT::NodeStatus SetRandomWalkFlag::tick()
{
  auto edie = Edie::GetInstance();
  if (!edie) {
    return BT::NodeStatus::FAILURE;
  }
  
  // 입력 포트에서 "value" 값을 읽어옵니다.
  auto value_to_set = getInput<bool>("value");
  if (!value_to_set) {
    // 포트가 제공되지 않았거나 타입이 잘못된 경우 에러 처리
    return BT::NodeStatus::FAILURE;
  }

  edie->is_random_walk_needed = value_to_set.value();
  RCLCPP_INFO(edie->ros_manager->get_logger(), "[SetRandomWalkFlag] Set is_random_walk_needed to: %s", 
              value_to_set.value() ? "true" : "false");
  
  return BT::NodeStatus::SUCCESS;
}

} // namespace aeirobot
