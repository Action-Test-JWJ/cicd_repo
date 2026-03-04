#include "edie_localization_manager/utility/parameter_loader.hpp"

namespace edie_localization_manager
{
namespace parameter_loader
{

void DeclareAndLoadParameters(
    rclcpp::Node* node,
    edie_localization_manager::ManagerParameters& params,
    std::map<std::string, double>& topic_hz)
{
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.read_only = true;

    // ===== 파라미터 선언 및 로드 =====
    params.pose_buffer_seconds = node->declare_parameter("buffer.pose_buffer_seconds", 5.0, param_desc);
    params.odom_buffer_size = node->declare_parameter("buffer.odom_buffer_size", 10, param_desc);

    params.manager_param_path = node->declare_parameter("paths.manager_param", "/src/edie8_parameters/config/manager_param.yaml", param_desc);
    params.evaluation_param_path = node->declare_parameter("paths.evaluation_param", "/src/edie8_parameters/config/manager_eval_param.yaml", param_desc);
    params.set_position_param_path = node->declare_parameter("paths.set_position_param", "/src/edie8_parameters/config/setposition_param.yaml", param_desc);
    params.fusion_param_path = node->declare_parameter("paths.fusion_param", "/src/edie8_parameters/config/manager_fusion_param.yaml", param_desc);

    params.loop_rate = node->declare_parameter("logic.loop_rate", 60.0, param_desc);
    params.pose_history_size = node->declare_parameter("logic.pose_history_size", 10, param_desc);
    params.fusion_strategies_str = node->declare_parameter("logic.fusion_strategies", std::vector<std::string>{"OutlierRejection", "SimpleWeightedAverage", "Smoothing"}, param_desc);

    topic_hz["ekf_odom"] = node->declare_parameter("topic_hz.ekf_odom", 20.0, param_desc);
    topic_hz["raw_odom"] = node->declare_parameter("topic_hz.raw_odom", 20.0, param_desc);
    topic_hz["openvins"] = node->declare_parameter("topic_hz.openvins", 200.0, param_desc);


    /////////////////////
    // EVALUATION
    /////////////////////

    // ===== 소스 타입 분류 =====
    params.eval_params.continuous_sources = node->declare_parameter("evaluation.source_types.continuous_sources", std::vector<std::string>{"monte_carlo", "visual_slam"}, param_desc);
    params.eval_params.intermittent_sources = node->declare_parameter("evaluation.source_types.intermittent_sources", std::vector<std::string>{"section", "vision", "vision_dir", "trilateration"}, param_desc);
    params.eval_params.continuous_staleness_threshold = node->declare_parameter("evaluation.source_types.staleness_threshold.continuous", 1.0, param_desc);
    params.eval_params.intermittent_staleness_threshold = node->declare_parameter("evaluation.source_types.staleness_threshold.intermittent", 2.0, param_desc);
    
    // ===== ArUco 신뢰성 평가 파라미터 =====
    params.eval_params.aruco_history_size = node->declare_parameter("evaluation.aruco_reliability.history_size", 10, param_desc);
    params.eval_params.aruco_pos_x_stddev_thresh = node->declare_parameter("evaluation.aruco_reliability.pos_x_stddev_thresh", 0.05, param_desc);
    params.eval_params.aruco_pos_y_stddev_thresh = node->declare_parameter("evaluation.aruco_reliability.pos_y_stddev_thresh", 0.05, param_desc);
    params.eval_params.aruco_yaw_stddev_thresh = node->declare_parameter("evaluation.aruco_reliability.yaw_stddev_thresh", 0.1, param_desc);
    params.eval_params.aruco_reproj_err_thresh = node->declare_parameter("evaluation.aruco_reliability.reproj_err_thresh", 1.0, param_desc);
    params.eval_params.aruco_min_area_thresh = node->declare_parameter("evaluation.aruco_reliability.min_area_thresh", 500.0, param_desc);
    params.eval_params.aruco_min_update_distance_m = node->declare_parameter("evaluation.aruco_reliability.min_update_distance_m", 0.05, param_desc);
    params.eval_params.aruco_zupt_true_threshold = node->declare_parameter("evaluation.aruco_reliability.zupt_true_threshold", 5, param_desc);

    // Load final check parameters
    params.eval_params.final_check_pos_x_stddev_thresh = node->declare_parameter("evaluation.aruco_reliability.final_check_pos_x_stddev_thresh", 0.01, param_desc);
    params.eval_params.final_check_pos_y_stddev_thresh = node->declare_parameter("evaluation.aruco_reliability.final_check_pos_y_stddev_thresh", 0.01, param_desc);
    params.eval_params.final_check_yaw_stddev_thresh = node->declare_parameter("evaluation.aruco_reliability.final_check_yaw_stddev_thresh", 0.05, param_desc);

    // ===== 1. Latency(지연 시간) 평가 파라미터 =====
    // 데이터의 신선도를 평가하는 데 사용됨
    params.eval_params.max_allowed_age_sec = node->declare_parameter("evaluation.latency.max_allowed_age_sec", 1.0, param_desc);

    // ===== 2. Timing(타이밍) 평가 파라미터 =====
    // 메시지 도착 간격과 처리 시간을 평가하는 데 사용됨
    params.eval_params.acceptable_compute_ratio = node->declare_parameter("evaluation.timing.acceptable_compute_ratio", 0.5, param_desc);
    params.eval_params.weight_regularity = node->declare_parameter("evaluation.timing.weight_regularity", 0.7, param_desc);
    params.eval_params.weight_computation = node->declare_parameter("evaluation.timing.weight_computation", 0.3, param_desc);

    // ===== 3. Continuity(연속성) 평가 파라미터 =====
    // 위치 변화의 속도가 합리적인 범위 내에 있는지 평가하는 데 사용됨
    params.eval_params.max_linear_velocity_mps = node->declare_parameter("evaluation.max_linear_velocity_mps", 2.0, param_desc);
    params.eval_params.max_angular_velocity_rps = node->declare_parameter("evaluation.max_angular_velocity_rps", 1.57, param_desc);

    // ===== 4. Smoothness(부드럽기) 평가 파라미터 =====
    // 이전 위치 변화의 추세와 현재 변화의 일관성을 평가하는 데 사용됨
    params.eval_params.trend_window_size = node->declare_parameter("evaluation.trend_window_size", 5, param_desc);
    params.eval_params.max_allowed_trend_deviation_m = node->declare_parameter("evaluation.max_allowed_trend_deviation_m", 0.5, param_desc);
    params.eval_params.max_allowed_trend_deviation_rad = node->declare_parameter("evaluation.max_allowed_trend_deviation_rad", 0.52, param_desc);

    // ===== 5. Conformity(적합성) 평가 파라미터 =====
    // 예측된 위치와 실제 측정값의 일치도를 평가하는 데 사용됨
    params.eval_params.max_position_diff_m = node->declare_parameter("evaluation.max_position_diff_m", 0.5, param_desc);
    params.eval_params.max_angle_diff_rad = node->declare_parameter("evaluation.max_angle_diff_rad", 0.52, param_desc);
    params.eval_params.intermittent_pos_diff_multiplier = node->declare_parameter("evaluation.intermittent_pos_diff_multiplier", 1.5, param_desc);
    params.eval_params.intermittent_angle_diff_multiplier = node->declare_parameter("evaluation.intermittent_angle_diff_multiplier", 1.5, param_desc);

    // ===== 연속 소스 가중치 파라미터 =====
    params.eval_params.continuous_weights.latency = node->declare_parameter("evaluation.weights.continuous.latency", 0.1, param_desc);
    params.eval_params.continuous_weights.timing = node->declare_parameter("evaluation.weights.continuous.timing", 0.2, param_desc);
    params.eval_params.continuous_weights.continuity = node->declare_parameter("evaluation.weights.continuous.continuity", 0.3, param_desc);
    params.eval_params.continuous_weights.smoothness = node->declare_parameter("evaluation.weights.continuous.smoothness", 0.2, param_desc);
    params.eval_params.continuous_weights.conformity = node->declare_parameter("evaluation.weights.continuous.conformity", 0.2, param_desc);

    // ===== 간헐 소스 가중치 파라미터 =====
    params.eval_params.intermittent_weights.latency = node->declare_parameter("evaluation.weights.intermittent.latency", 0.1, param_desc);
    params.eval_params.intermittent_weights.timing = node->declare_parameter("evaluation.weights.intermittent.timing", 0.1, param_desc);
    params.eval_params.intermittent_weights.continuity = node->declare_parameter("evaluation.weights.intermittent.continuity", 0.1, param_desc);
    params.eval_params.intermittent_weights.smoothness = node->declare_parameter("evaluation.weights.intermittent.smoothness", 0.1, param_desc);
    params.eval_params.intermittent_weights.conformity = node->declare_parameter("evaluation.weights.intermittent.conformity", 0.6, param_desc);

    // ===== 정체 임계값 파라미터 =====
    params.eval_params.continuous_staleness_threshold = node->declare_parameter("evaluation.staleness.continuous_threshold", 0.5, param_desc);
    params.eval_params.intermittent_staleness_threshold = node->declare_parameter("evaluation.staleness.intermittent_threshold", 2.0, param_desc);

    /////////////////////
    // FUSION
    /////////////////////

    // ===== 포즈 퓨전 파라미터 =====
    params.fusion_params.angle_weight = node->declare_parameter("fusion.calculation.angle_weight", 0.5, param_desc);
    params.fusion_params.max_window_size = node->declare_parameter("fusion.smoothing.max_window_size", 20, param_desc);
    params.fusion_params.distance_threshold = node->declare_parameter("fusion.smoothing.distance_threshold", 1.0, param_desc);
    params.fusion_params.max_alpha = node->declare_parameter("fusion.smoothing.max_alpha", 0.15, param_desc);
    params.fusion_params.min_alpha = node->declare_parameter("fusion.smoothing.min_alpha", 0.02, param_desc);
    params.fusion_params.decay_rate = node->declare_parameter("fusion.smoothing.decay_rate", 0.3, param_desc);
    params.fusion_params.outlier_threshold = node->declare_parameter("fusion.outlier.threshold", 2.0, param_desc);
    params.fusion_params.outlier_penalty = node->declare_parameter("fusion.outlier.penalty", 0.1, param_desc);
}



} // namespace parameter_loader
} // namespace edie_localization_manager
