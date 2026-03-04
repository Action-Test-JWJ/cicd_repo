#ifndef EDIE_LOCALIZATION_MANAGER_POSE_PREDICTION_HPP_
#define EDIE_LOCALIZATION_MANAGER_POSE_PREDICTION_HPP_

// C++ system files
#include <tf2/utils.h>

// ROS 관련 헤더
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 현재 프로젝트 헤더
#include "edie_localization_manager/utility/types.hpp"

namespace pose_prediction
{


edie_msgs::msg::PoseWithInfoStamped UpdatePoseWithPrediction(
    bool odom_initialized,
    const std::deque<edie_msgs::msg::PoseWithInfoStamped>& pose_history,
    const tf2::Transform& odom_delta_sum,
    const rclcpp::Clock::SharedPtr& clock);

// 예측 결과 생성: 이전 fused pose + odom delta → predicted pose
geometry_msgs::msg::Pose2D PredictPoseFromOdometry(
    const geometry_msgs::msg::Pose2D& last_fused_pose,
    const tf2::Transform& odom_delta);

// odometry 메시지를 tf2::Transform으로 변환
tf2::Transform OdometryMsgToTF(const nav_msgs::msg::Odometry& odom_msg);

// pose2d를 tf2::Transform으로 변환
tf2::Transform Pose2DToTF(const geometry_msgs::msg::Pose2D& pose);

// tf2::Transform을 pose2d로 변환
geometry_msgs::msg::Pose2D TFToPose2D(const tf2::Transform& tf);

// 이전 odometry pose와 현재 odometry pose로부터 이동 delta 계산
tf2::Transform ComputeOdomDelta(
    const tf2::Transform& prev_odom_tf,
    const tf2::Transform& curr_odom_tf);
} // namespace pose_prediction

#endif // EDIE_LOCALIZATION_MANAGER_POSE_PREDICTION_HPP_
