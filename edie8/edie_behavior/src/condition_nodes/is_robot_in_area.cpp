#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/edie.hpp"

namespace aeirobot
{

BT::NodeStatus IsRobotInArea::tick()
{
  auto edie = Edie::GetInstance();

  // 입력 포트에서 area 파라미터 읽기
  std::string area;
  if (!getInput<std::string>("area", area))
  {
    return BT::NodeStatus::FAILURE;
  }

  // 평균 ArUco 포즈 확인 (CollectArucoPoseData에서 계산된 값)
  if (!edie->averaged_aruco_pose.has_value())
  {
    RCLCPP_WARN(edie->ros_manager->get_logger(), 
                "IsRobotInArea: No averaged ArUco pose available");
    return BT::NodeStatus::FAILURE;
  }

  const auto& aruco_pose = edie->averaged_aruco_pose.value();
  const double x = aruco_pose.position.x;
  const double y = aruco_pose.position.y;

  RCLCPP_INFO(edie->ros_manager->get_logger(), 
              "IsRobotInArea: Checking area '%s' with averaged pose (x: %.3f, y: %.3f)", 
              area.c_str(), x, y);

  // 영역 판단
  if (area == "left")
  {
    // y > 0.1이면 left
    bool result = (y > 0.1);
    if (result) {
      RCLCPP_INFO(edie->ros_manager->get_logger(), 
                  "✓ Robot is in LEFT area (y: %.3f > 0.1)", y);
    } else {
      RCLCPP_DEBUG(edie->ros_manager->get_logger(), 
                   "✗ Robot is NOT in left area (y: %.3f <= 0.1)", y);
    }
    return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  else if (area == "right")
  {
    // y < -0.1이면 right
    bool result = (y < -0.1);
    if (result) {
      RCLCPP_INFO(edie->ros_manager->get_logger(), 
                  "✓ Robot is in RIGHT area (y: %.3f < -0.1)", y);
    } else {
      RCLCPP_DEBUG(edie->ros_manager->get_logger(), 
                   "✗ Robot is NOT in right area (y: %.3f >= -0.1)", y);
    }
    return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(edie->ros_manager->get_logger(), 
               "IsRobotInArea: Unknown area '%s'", area.c_str());
  return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot

