#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{

StareAtTarget::StareAtTarget(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

BT::PortsList StareAtTarget::providedPorts()
{
  return{ BT::InputPort<double>("duration", 2.0, "Seconds to stare at the target") };
}

BT::NodeStatus StareAtTarget::onStart()
{
  auto edie = Edie::GetInstance();
  if (!edie) return BT::NodeStatus::FAILURE;

  if (!getInput<double>("duration", duration_sec_)) {
    return BT::NodeStatus::FAILURE;
  }
  
  start_time_ = edie->ros_manager->now();
  RCLCPP_INFO(edie->ros_manager->get_logger(), "[StareAtTarget] Starting confirmation. Will stare for %.1f seconds.", duration_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StareAtTarget::onRunning()
{
  auto edie = Edie::GetInstance();
  if (!edie) return BT::NodeStatus::FAILURE;

  // 1. 타겟을 놓치면 즉시 실패
  if (!edie->is_human_visible_now) {
    RCLCPP_INFO(edie->ros_manager->get_logger(), "[StareAtTarget] Target lost during confirmation. Failing.");
    return BT::NodeStatus::FAILURE;
  }
  
  // 2. 시간이 다 되면 성공
  if ((edie->ros_manager->now() - start_time_).seconds() > duration_sec_) {
    RCLCPP_INFO(edie->ros_manager->get_logger(), "[StareAtTarget] Confirmation successful!");
    edie->PubEmotion(9);//찾았다 내사랑
    return BT::NodeStatus::SUCCESS;
  }

  // 3. 아직 시간이 남았으면 계속 바라봄
  auto twist = edie->GetTwistForStaring();
  edie->PubRemote(0.0, twist.angular.z);
  
  return BT::NodeStatus::RUNNING;
}

void StareAtTarget::onHalted()
{
  auto edie = Edie::GetInstance();
  if (edie) {
    RCLCPP_INFO(edie->ros_manager->get_logger(), "[StareAtTarget] Halted. Stopping robot.");
    edie->PubRemote(0.0, 0.0);
  }
}

} // namespace aeirobot
