#include "edie_behavior/condition_nodes.hpp"
#include <cmath>
#include "geometry_msgs/msg/quaternion.hpp"

namespace aeirobot
{

static double yaw_from_quat(const geometry_msgs::msg::Quaternion& q)
{
  // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  return std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
}

BT::NodeStatus IsRealMiddle::tick()
{
  auto edie = Edie::GetInstance();

  // 입력 포트 파라미터 읽기
  const double y_thresh   = getInput<double>("y_thresh").value_or(0.10);
  const double yaw_thresh = getInput<double>("yaw_thresh").value_or(0.15);
  const double recent_sec = getInput<double>("recent_sec").value_or(0.5);

  // 최신 ArUco 포즈가 없으면 실패
  if (!edie->last_aruco_pose.has_value())
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto& pose = edie->last_aruco_pose.value();

  // 최신성 확인
  const rclcpp::Time now = edie->ros_manager->now();
  const rclcpp::Time stamp = pose.header.stamp;
  if ((now - stamp).seconds() > recent_sec)
  {
    return BT::NodeStatus::FAILURE;
  }

  // 중앙(y≈0) 및 각도(yaw≈0) 확인
  const double y = pose.pose.position.y;
  const double yaw = yaw_from_quat(pose.pose.orientation);

  if (std::abs(y) <= y_thresh && std::abs(yaw) <= yaw_thresh)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot
