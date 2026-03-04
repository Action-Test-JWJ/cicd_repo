#ifndef EDIE_LOCALIZATION_MANAGER_TYPES_HPP_
#define EDIE_LOCALIZATION_MANAGER_TYPES_HPP_

#include <deque>
#include <utility>  // for std::pair
#include <vector>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/time.hpp"

#include "edie_msgs/msg/pose_with_info_stamped.hpp"

namespace edie_localization_manager
{
  // A pair of a pose message and the time it was received by the manager.
  using PoseStampedWithTime = std::pair<edie_msgs::msg::PoseWithInfoStamped, rclcpp::Time>;

  // A deque to store a time-series of pose data.
  using PoseBuffer = std::deque<PoseStampedWithTime>;
  using OdomBuffer = std::deque<nav_msgs::msg::Odometry::SharedPtr>;

  struct EvaluationParameters
  {
    double max_allowed_age_sec;
    int trend_window_size;
    double max_linear_velocity_mps;
    double max_angular_velocity_rps;
    double max_allowed_trend_deviation_m;
    double max_allowed_trend_deviation_rad;
    double max_position_diff_m;
    double max_angle_diff_rad;
    double intermittent_pos_diff_multiplier;
    double intermittent_angle_diff_multiplier;
    std::vector<std::string> continuous_sources;
    std::vector<std::string> intermittent_sources;
    double acceptable_compute_ratio;  // 메시지 처리 시간의 목표 비율 (주기 대비)
    double weight_regularity;         // 규칙성 점수의 가중치
    double weight_computation;        // 계산 효율성 점수의 가중치
    double initial_quality_score;

    struct Weights
    {
      double latency;
      double timing;
      double continuity;
      double smoothness;
      double conformity;
    };

    Weights continuous_weights;
    Weights intermittent_weights;
    double continuous_staleness_threshold;
    double intermittent_staleness_threshold;

    // ArUco 신뢰성 평가 파라미터
    int aruco_history_size;
    double aruco_pos_x_stddev_thresh;
    double aruco_pos_y_stddev_thresh;
    double aruco_yaw_stddev_thresh;
    double aruco_reproj_err_thresh;
    double aruco_min_area_thresh;
    double aruco_min_update_distance_m;
    int aruco_zupt_true_threshold;

    // Parameters for the manager's final, stricter check
    double final_check_pos_x_stddev_thresh;
    double final_check_pos_y_stddev_thresh;
    double final_check_yaw_stddev_thresh;
  };

  struct FusionParameters
  {
    double angle_weight;
    int max_window_size;
    double distance_threshold;
    double max_alpha;
    double min_alpha;
    double decay_rate;
    double outlier_threshold;
    double outlier_penalty;
  };

  struct ManagerParameters
  {
    double pose_buffer_seconds;
    int odom_buffer_size;
    std::string manager_param_path;
    std::string evaluation_param_path;
    std::string set_position_param_path;
    std::string fusion_param_path;
    std::string game_param_path;
    double loop_rate;
    int pose_history_size;
    std::vector<std::string> fusion_strategies_str;
    EvaluationParameters eval_params;
    FusionParameters fusion_params;
  };

  struct LocalizationSourceInfo
  {
    edie_msgs::msg::PoseWithInfoStamped latest_pose;
    float quality_score = 0.0f;    // 종합 품질 점수
    bool is_valid = false;         // 유효한 소스인지 여부

    // 세부 품질 점수
    float latency_score = 1.0f;     // 지연 시간 점수 (신선도)
    float timing_score = 1.0f;      // 타이밍 점수 (주기 및 처리 시간)
    float continuity_score = 1.0f;  // 연속성 점수
    float smoothness_score = 1.0f;  // 부드러움 점수
    float conformity_score = 1.0f;  // 일치도 점수 (예측값과의 일치도)
  };
} // namespace edie_localization_manager

#endif  // EDIE_LOCALIZATION_MANAGER_TYPES_HPP_
