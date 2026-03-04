#include "edie_behavior/action_nodes.hpp"
#include "edie_behavior/edie.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace aeirobot
{

BT::NodeStatus CollectArucoPoseData::onStart()
{
  auto edie = Edie::GetInstance();
  
  // 목표 샘플 개수 읽기 (기본값: 100)
  target_count_ = getInput<int>("sample_count").value_or(100);
  
  // 수집 데이터 초기화
  collected_poses_.clear();
  
  RCLCPP_INFO(edie->ros_manager->get_logger(), 
              "CollectArucoPoseData: Starting to collect %d pose samples", target_count_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CollectArucoPoseData::onRunning()
{
  auto edie = Edie::GetInstance();
  
  // ArUco 포즈가 있는지 확인
  if (!edie->last_aruco_pose.has_value())
  {
    // ArUco 포즈가 없으면 계속 대기
    return BT::NodeStatus::RUNNING;
  }
  
  // 현재 ArUco 포즈 수집
  const auto& pose_stamped = edie->last_aruco_pose.value();
  collected_poses_.push_back(pose_stamped.pose);
  
  RCLCPP_DEBUG(edie->ros_manager->get_logger(), 
               "CollectArucoPoseData: Collected %zu/%d samples", 
               collected_poses_.size(), target_count_);
  
  // 목표 개수만큼 수집했는지 확인
  if (collected_poses_.size() >= static_cast<size_t>(target_count_))
  {
    // 이상치 제거 및 평균 계산
    geometry_msgs::msg::Pose averaged_pose = computeAveragedPose();
    
    // Edie 싱글톤에 평균 pose 저장
    edie->averaged_aruco_pose = averaged_pose;
    
    RCLCPP_INFO(edie->ros_manager->get_logger(), 
                "CollectArucoPoseData: Completed. Averaged pose: x=%.3f, y=%.3f, z=%.3f",
                averaged_pose.position.x, averaged_pose.position.y, averaged_pose.position.z);
    
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

void CollectArucoPoseData::onHalted()
{
  auto edie = Edie::GetInstance();
  RCLCPP_WARN(edie->ros_manager->get_logger(), "CollectArucoPoseData: Halted");
  collected_poses_.clear();
}

// 이상치 제거 및 평균 계산 함수
geometry_msgs::msg::Pose CollectArucoPoseData::computeAveragedPose()
{
  auto edie = Edie::GetInstance();
  
  // x, y, z 좌표 분리
  std::vector<double> x_values, y_values, z_values;
  for (const auto& pose : collected_poses_)
  {
    x_values.push_back(pose.position.x);
    y_values.push_back(pose.position.y);
    z_values.push_back(pose.position.z);
  }
  
  // 각 축에 대해 이상치 제거
  auto filtered_x = removeOutliers(x_values);
  auto filtered_y = removeOutliers(y_values);
  auto filtered_z = removeOutliers(z_values);
  
  RCLCPP_INFO(edie->ros_manager->get_logger(), 
              "CollectArucoPoseData: After outlier removal - x: %zu/%zu, y: %zu/%zu, z: %zu/%zu",
              filtered_x.size(), x_values.size(),
              filtered_y.size(), y_values.size(),
              filtered_z.size(), z_values.size());
  
  // 평균 계산
  geometry_msgs::msg::Pose result;
  result.position.x = computeMean(filtered_x);
  result.position.y = computeMean(filtered_y);
  result.position.z = computeMean(filtered_z);
  
  // orientation은 첫 번째 값 사용 (또는 평균 계산 가능)
  if (!collected_poses_.empty())
  {
    result.orientation = collected_poses_[0].orientation;
  }
  
  return result;
}

// 이상치 제거 (평균에서 2σ 벗어난 값 제거)
std::vector<double> CollectArucoPoseData::removeOutliers(const std::vector<double>& data)
{
  if (data.size() < 3)
    return data;
  
  // 평균 계산
  double mean = computeMean(data);
  
  // 표준편차 계산
  double variance = 0.0;
  for (const auto& val : data)
  {
    variance += (val - mean) * (val - mean);
  }
  variance /= data.size();
  double std_dev = std::sqrt(variance);
  
  // 이상치 제거 (평균 ± 2σ 범위 내의 값만 유지)
  std::vector<double> filtered;
  for (const auto& val : data)
  {
    if (std::abs(val - mean) <= 2.0 * std_dev)
    {
      filtered.push_back(val);
    }
  }
  
  // 너무 많이 제거된 경우 원본 반환
  if (filtered.size() < data.size() / 2)
    return data;
  
  return filtered;
}

// 평균 계산
double CollectArucoPoseData::computeMean(const std::vector<double>& data)
{
  if (data.empty())
    return 0.0;
  
  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  return sum / data.size();
}

} // namespace aeirobot

