#include "edie_localization_manager/core/pose_evaluation.hpp"
#include <optional> // Add this include for std::optional

namespace
{
// 각도를 [-PI, PI] 범위로 정규화하는 헬퍼 함수
double NormalizeAngle(double angle)
{
    return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

    // --- 개별 품질 점수 계산 함수 ---
    /**
     * @brief 지연 시간(Latency) 평가 함수
     *
     * @details
     * - ROS2 자체의 지연시간 체크 방식 적용
     * - 시간 오프셋 자동 계산 방식 적용
     *
     * @param latest_arrival_time
     * @param latest_msg_stamp
     * @param params
     * @return double
     */
    double CalculateLatencyScore(
        const rclcpp::Time& latest_arrival_time,
        const builtin_interfaces::msg::Time& latest_msg_stamp,
        const edie_localization_manager::EvaluationParameters& params)
    {
        // 시간 오프셋 자동 계산 방식 적용
        static double time_offset = 0.0;
        static bool first_msg = true;

        double msg_time = static_cast<double>(latest_msg_stamp.sec) + static_cast<double>(latest_msg_stamp.nanosec) / 1e9;

        if (first_msg) {
            // 첫 메시지로 시간 오프셋 계산
            time_offset = latest_arrival_time.seconds() - msg_time;
            first_msg = false;

            auto logger = rclcpp::get_logger("pose_evaluation");
            RCLCPP_DEBUG_STREAM(logger, "============= LATENCY DEBUG ==============");
            RCLCPP_DEBUG_STREAM(logger, "첫 메시지 감지 - 시간 오프셋 계산");
            RCLCPP_DEBUG_STREAM(logger, "latest_arrival_time: " << latest_arrival_time.seconds() << "s");
            RCLCPP_DEBUG_STREAM(logger, "latest_msg_stamp: " << msg_time << "s");
            RCLCPP_DEBUG_STREAM(logger, "계산된 시간 오프셋: " << time_offset << "s");
            RCLCPP_DEBUG_STREAM(logger, "===========================================");

            return 1.0;  // 첫 메시지는 기준점으로 최고 점수 부여
        }

        // 오프셋 적용한 메시지 시간
        double adjusted_msg_time = msg_time + time_offset;

        // 조정된 지연 시간
        double adjusted_latency = latest_arrival_time.seconds() - adjusted_msg_time;
        double score = 1.0 - std::min(1.0, std::max(0.0, adjusted_latency) / params.max_allowed_age_sec);

        // 디버깅 정보 추가
        auto logger = rclcpp::get_logger("pose_evaluation");
        RCLCPP_DEBUG_STREAM(logger, "============= LATENCY DEBUG ==============");
        RCLCPP_DEBUG_STREAM(logger, "latest_arrival_time: " << latest_arrival_time.seconds() << "s");
        RCLCPP_DEBUG_STREAM(logger, "원본 msg_stamp: " << msg_time << "s");
        RCLCPP_DEBUG_STREAM(logger, "시간 오프셋: " << time_offset << "s");
        RCLCPP_DEBUG_STREAM(logger, "조정된 msg_time: " << adjusted_msg_time << "s");
        RCLCPP_DEBUG_STREAM(logger, "조정된 Latency: " << adjusted_latency << "s");
        RCLCPP_DEBUG_STREAM(logger, "max_allowed_age_sec: " << params.max_allowed_age_sec);
        RCLCPP_DEBUG_STREAM(logger, "Calculation: 1.0 - min(1.0, max(0.0, " << adjusted_latency << ") / " << params.max_allowed_age_sec << ") = " << score);
        RCLCPP_DEBUG_STREAM(logger, "===========================================");

        return score;
    }

    /**
     * @brief 메시지 처리의 시간적 품질 점수를 계산
     *
     * @details
     * - Regularity: 메시지 도착 간격이 목표 주기와 얼마나 일치하는가 (jitter)
     * - Computation: 메시지를 처리하는 데 걸린 시간이 목표 주기의 몇 %를 초과했는가
     * 가중 평균으로 계산
     *
     * @param arrival_interval 직전 메시지와의 도착 시간 간격 (초)
     * @param delta_time 메시지를 처리하는 데 걸린 시간 (초)
     * @param target_hz 기대하는 메시지 주기 (Hz)
     * @param acceptable_compute_ratio 처리 시간의 목표 비율 (0.5 = 주기의 50%)
     */
    double CalculateTimingScore(
        const std::string& source_name,
        double arrival_interval,
        float delta_time,
        double target_hz,
        const edie_localization_manager::EvaluationParameters& params)
    {
        double regularity = 1.0;
        double computation = 1.0;
        double expected_interval = 0.0;
        double interval_error = 0.0;
        double expected_compute_time = 0.0;

        pose_evaluation::SourceType source_type = pose_evaluation::GetSourceType(source_name, params);

        // 규칙성(Regularity) 점수는 주기적 소스에 대해서만 계산
        if (source_type == pose_evaluation::SourceType::CONTINUOUS && target_hz > 0)
        {
            expected_interval = 1.0 / target_hz;
            interval_error = std::abs(arrival_interval - expected_interval) / expected_interval;
            regularity = 1.0 - std::min(1.0, interval_error);
        }

        // 처리 효율성(Computation) 점수는 모든 소스에 대해 각자의 target_hz를 기준으로 계산
        if (target_hz > 0)
        {
            if (expected_interval == 0.0) { // 중복 계산 방지
                expected_interval = 1.0 / target_hz;
            }
            expected_compute_time = expected_interval * params.acceptable_compute_ratio;
            if (delta_time > expected_compute_time) {
                const double overtime = delta_time - expected_compute_time;
                const double penalty = std::min(1.0, overtime / expected_compute_time);
                computation = 1.0 - penalty;
            }
        }

        double final_score = params.weight_regularity * regularity + params.weight_computation * computation;

        // 디버깅 정보 추가
        auto logger = rclcpp::get_logger("pose_evaluation");
        RCLCPP_DEBUG_STREAM(logger, "============= TIMING DEBUG (" << source_name << ") ==============");
        RCLCPP_DEBUG_STREAM(logger, "Source Type: " << (source_type == pose_evaluation::SourceType::CONTINUOUS ? "CONTINUOUS" : "INTERMITTENT"));
        RCLCPP_DEBUG_STREAM(logger, "Inputs: arrival_interval=" << arrival_interval << "s, delta_time=" << delta_time << "s, target_hz=" << target_hz << "Hz");

        if (source_type == pose_evaluation::SourceType::CONTINUOUS && target_hz > 0) {
            RCLCPP_DEBUG_STREAM(logger, "Regularity: expected_interval=" << expected_interval << "s, interval_error=" << interval_error << ", score=" << regularity);
        } else {
            RCLCPP_DEBUG_STREAM(logger, "Regularity: (skipped for INTERMITTENT source), score=" << regularity);
        }

        if (target_hz > 0) {
            RCLCPP_DEBUG_STREAM(logger, "Computation: expected_compute_time=" << expected_compute_time << "s, score=" << computation);
        } else {
            RCLCPP_DEBUG_STREAM(logger, "Computation: (skipped for target_hz <= 0), score=" << computation);
        }

        RCLCPP_DEBUG_STREAM(logger, "Final Score: (" << params.weight_regularity << " * " << regularity << ") + (" << params.weight_computation << " * " << computation << ") = " << final_score);
        RCLCPP_DEBUG_STREAM(logger, "==================================================");

        return final_score;
    }

    /**
     * @brief 연속성 점수를 계산함
     * @details 선형 및 각도 이동 속도가 합리적인 범위 안에 있는지 평가
     */
    double CalculateContinuityScore(
        [[maybe_unused]] const std::string& source_name,
        double dx,
        double dy,
        double dtheta,
        double arrival_interval,
        const edie_localization_manager::EvaluationParameters& params)
    {
        const double time_delta = arrival_interval > 1e-6 ? arrival_interval : 1e-6;
        const double distance = std::sqrt(dx * dx + dy * dy);
        const double linear_velocity = distance / time_delta;
        const double angular_velocity = std::abs(dtheta) / time_delta;

        const double quality_continuity_linear = 1.0 - std::min(1.0, linear_velocity / params.max_linear_velocity_mps);
        const double quality_continuity_angular = 1.0 - std::min(1.0, angular_velocity / params.max_angular_velocity_rps);
        double final_score = quality_continuity_linear * quality_continuity_angular;

        // 디버깅 정보 추가
        auto logger = rclcpp::get_logger("pose_evaluation");
        RCLCPP_DEBUG_STREAM(logger, "============= CONTINUITY DEBUG (" << source_name << ") ==============");
        RCLCPP_DEBUG_STREAM(logger, "Inputs: dx=" << dx << ", dy=" << dy << ", dtheta=" << dtheta << ", interval=" << arrival_interval << "s");
        RCLCPP_DEBUG_STREAM(logger, "Calculated Velocities: linear=" << linear_velocity << " m/s, angular=" << angular_velocity << " rad/s");
        RCLCPP_DEBUG_STREAM(logger, "Params: max_linear=" << params.max_linear_velocity_mps << " m/s, max_angular=" << params.max_angular_velocity_rps << " rad/s");
        RCLCPP_DEBUG_STREAM(logger, "Linear Score: " << quality_continuity_linear);
        RCLCPP_DEBUG_STREAM(logger, "Angular Score: " << quality_continuity_angular);
        RCLCPP_DEBUG_STREAM(logger, "Final Score: " << final_score);
        RCLCPP_DEBUG_STREAM(logger, "====================================================");

        return final_score;
    }

    /**
     * @brief 부드럽게 이어지는 점수를 계산함
     * @details 이전 위치 변화의 추세와 현재 변화가 일관성이 있는지 평가
     */
    double CalculateSmoothnessScore(
        [[maybe_unused]] const std::string& source_name,
        const edie_localization_manager::PoseBuffer& buffer,
        double dx,
        double dy,
        double dtheta,
        const edie_localization_manager::EvaluationParameters& params)
    {
        if (buffer.size() < 3)
        {
            return 0.3; // 추세 분석을 위한 데이터가 충분하지 않음
        }

        double sum_dx = 0.0, sum_dy = 0.0, sum_dtheta = 0.0;
        const size_t num_intervals = std::min(buffer.size() - 1, static_cast<size_t>(params.trend_window_size));

        for (size_t i = 0; i < num_intervals; ++i)
        {
            const auto& current_pose = buffer[buffer.size() - 1 - i].first.pose;
            const auto& prev_pose = buffer[buffer.size() - 2 - i].first.pose;
            sum_dx += current_pose.x - prev_pose.x;
            sum_dy += current_pose.y - prev_pose.y;
            sum_dtheta += NormalizeAngle(current_pose.theta - prev_pose.theta);
        }

        const double avg_dx = sum_dx / num_intervals;
        const double avg_dy = sum_dy / num_intervals;
        const double avg_dtheta = sum_dtheta / num_intervals;

        const double deviation_dist = std::sqrt(std::pow(dx - avg_dx, 2) + std::pow(dy - avg_dy, 2));
        const double deviation_theta = std::abs(dtheta - avg_dtheta);

        const double quality_smoothness_linear = 1.0 - std::min(1.0, deviation_dist / params.max_allowed_trend_deviation_m);
        const double quality_smoothness_angular = 1.0 - std::min(1.0, deviation_theta / params.max_allowed_trend_deviation_rad);
        double final_score = quality_smoothness_linear * quality_smoothness_angular;

        // 디버깅 정보 추가
        auto logger = rclcpp::get_logger("pose_evaluation");
        RCLCPP_DEBUG_STREAM(logger, "============= SMOOTHNESS DEBUG (" << source_name << ") ==============");
        RCLCPP_DEBUG_STREAM(logger, "Inputs: dx=" << dx << ", dy=" << dy << ", dtheta=" << dtheta);
        RCLCPP_DEBUG_STREAM(logger, "Trend: avg_dx=" << avg_dx << ", avg_dy=" << avg_dy << ", avg_dtheta=" << avg_dtheta);
        RCLCPP_DEBUG_STREAM(logger, "Deviations: dist=" << deviation_dist << ", theta=" << deviation_theta);
        RCLCPP_DEBUG_STREAM(logger, "Params: max_dev_m=" << params.max_allowed_trend_deviation_m << ", max_dev_rad=" << params.max_allowed_trend_deviation_rad);
        RCLCPP_DEBUG_STREAM(logger, "Linear Score: " << quality_smoothness_linear);
        RCLCPP_DEBUG_STREAM(logger, "Angular Score: " << quality_smoothness_angular);
        RCLCPP_DEBUG_STREAM(logger, "Final Score: " << final_score);
        RCLCPP_DEBUG_STREAM(logger, "=====================================================");

        return final_score;
    }

    double CalculateConformityScore(
        const bool& odom_initialized,
        const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
        const edie_msgs::msg::PoseWithInfoStamped& latest_msg,
        const std::string& source_name,
        const edie_localization_manager::EvaluationParameters& params)
    {
        if(!odom_initialized) //의미 없는 값
        {
            return 1.0;
        }

        const double dx = latest_msg.pose.x - predicted_pose.pose.x;
        const double dy = latest_msg.pose.y - predicted_pose.pose.y;
        const double distance_diff = std::sqrt(dx * dx + dy * dy);

        // Pose2D에서는 orientation 대신 theta를 사용
        const double pred_theta = predicted_pose.pose.theta;
        const double curr_theta = latest_msg.pose.theta;
        const double angle_diff = std::abs(NormalizeAngle(curr_theta - pred_theta));

        // 최대 허용 차이
        double max_position_diff = params.max_position_diff_m;
        double max_angle_diff = params.max_angle_diff_rad;

        // 불연속적인 소스는 더 큰 차이를 허용
        if (pose_evaluation::GetSourceType(source_name, params) == pose_evaluation::SourceType::INTERMITTENT)
        {
            max_position_diff *= params.intermittent_pos_diff_multiplier;
            max_angle_diff *= params.intermittent_angle_diff_multiplier;
        }  // 20% 더 엄격하게

        const double quality_position = 1.0 - std::min(1.0, distance_diff / max_position_diff);
        const double quality_angle = 1.0 - std::min(1.0, angle_diff / max_angle_diff);

        // 불연속적인 소스의 경우, 위치와 각도의 일치도가 너무 낮으면 가중치 감소
        double conformity_score = quality_position * quality_angle;

        if (pose_evaluation::GetSourceType(source_name, params) == pose_evaluation::SourceType::INTERMITTENT && conformity_score < 0.3)
        {
            conformity_score *= 0.5;  // 낮은 적합성 점수에 추가 페널티
        }

        // 디버깅 정보 추가
        auto logger = rclcpp::get_logger("pose_evaluation");
        RCLCPP_DEBUG_STREAM(logger, "============= CONFORMITY DEBUG (" << source_name << ") ==============");
        RCLCPP_DEBUG_STREAM(logger, "Predicted Pose: x=" << predicted_pose.pose.x << ", y=" << predicted_pose.pose.y << ", theta=" << pred_theta);
        RCLCPP_DEBUG_STREAM(logger, "Latest Pose:    x=" << latest_msg.pose.x << ", y=" << latest_msg.pose.y << ", theta=" << curr_theta);
        RCLCPP_DEBUG_STREAM(logger, "Differences: dist=" << distance_diff << ", angle=" << angle_diff);
        RCLCPP_DEBUG_STREAM(logger, "Params: max_pos_diff=" << max_position_diff << ", max_angle_diff=" << max_angle_diff);
        RCLCPP_DEBUG_STREAM(logger, "Position Score: " << quality_position);
        RCLCPP_DEBUG_STREAM(logger, "Angle Score: " << quality_angle);
        RCLCPP_DEBUG_STREAM(logger, "Final Score: " << conformity_score);
        RCLCPP_DEBUG_STREAM(logger, "====================================================");

        return conformity_score;
    }

} // namespace anonymous

namespace pose_evaluation
{
    /**
     * @brief 소스 이름에 따른 소스 유형을 반환
     * @param source_name 소스 이름
     * @return 소스 유형 (연속적/불연속적)
     */
    SourceType GetSourceType(const std::string& source_name, const edie_localization_manager::EvaluationParameters& params)
    {
        // 연속적인 소스 목록에 있는지 확인
        if (std::find(params.continuous_sources.begin(), params.continuous_sources.end(), source_name) != params.continuous_sources.end())
        {
            return SourceType::CONTINUOUS;
        }
        // 불연속적인 소스 목록에 있는지 확인
        else if (std::find(params.intermittent_sources.begin(), params.intermittent_sources.end(), source_name) != params.intermittent_sources.end())
        {
            return SourceType::INTERMITTENT;
        }
        // 기본값은 CONTINUOUS (안전한 선택)
        else
        {
            return SourceType::CONTINUOUS;
        }
    }

    /**
     * @brief 소스 이름에 따른 평가 가중치를 반환
     * @param source_name 소스 이름
     * @return 소스별 품질 평가 가중치
     */
    QualityWeights GetSourceWeights(const std::string& source_name, const edie_localization_manager::EvaluationParameters& params)
    {
        SourceType source_type = GetSourceType(source_name, params);
        QualityWeights weights;

        // 소스 유형에 따라 다른 가중치 반환 및 타입 변환
        if (source_type == SourceType::CONTINUOUS)
        {
            // edie_localization_manager::EvaluationParameters::Weights에서 pose_evaluation::QualityWeights로 변환
            weights.latency = static_cast<float>(params.continuous_weights.latency);
            weights.timing = static_cast<float>(params.continuous_weights.timing);
            weights.continuity = static_cast<float>(params.continuous_weights.continuity);
            weights.smoothness = static_cast<float>(params.continuous_weights.smoothness);
            weights.conformity = static_cast<float>(params.continuous_weights.conformity);
        }
        else
        {
            // edie_localization_manager::EvaluationParameters::Weights에서 pose_evaluation::QualityWeights로 변환
            weights.latency = static_cast<float>(params.intermittent_weights.latency);
            weights.timing = static_cast<float>(params.intermittent_weights.timing);
            weights.continuity = static_cast<float>(params.intermittent_weights.continuity);
            weights.smoothness = static_cast<float>(params.intermittent_weights.smoothness);
            weights.conformity = static_cast<float>(params.intermittent_weights.conformity);
        }

        return weights;
    }

    /**
     * @brief 소스 이름에 따른 데이터 오래됨 기준을 반환
     * @param source_name 소스 이름
     * @return 오래됨 기준 시간 (초)
     */
    double GetStalenessThreshold(const std::string& source_name, const edie_localization_manager::EvaluationParameters& params)
    {
        SourceType source_type = GetSourceType(source_name, params);

        // 소스 유형에 따라 다른 오래됨 기준 반환
        if (source_type == SourceType::CONTINUOUS)
        {
            return params.continuous_staleness_threshold;
        }
        else
        {
            return params.intermittent_staleness_threshold;
        }
    }

    /**
     * @brief 위치 소스의 품질 점수를 일반적으로 평가함
     * @details 다양한 품질 지표를 계산하고 가중치를 적용하여 종합 점수를 산출함
     * @param source_name 평가할 위치 소스의 이름
     * @param buffer 해당 소스의 데이터 버퍼
     * @param now 현재 시간 (현재 미사용)
     * @param target_hz 목표 주파수 (Hz)
     * @return 품질 점수를 포함하는 객체
     */
    pose_evaluation::QualityScores EvalGeneral(
        const bool& odom_initialized,
        const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
        const std::string & source_name,
        const edie_localization_manager::PoseBuffer & buffer,
        const rclcpp::Time & now,
        double target_hz,
        const edie_localization_manager::EvaluationParameters& params)
    {
        (void)now; // 현재는 사용하지 않음, buffer의 arrival_time 사용
        pose_evaluation::QualityScores scores;

        // 불연속 소스의 경우 다른 평가 기준 적용
        SourceType source_type = GetSourceType(source_name, params);

        // 최소 데이터 포인트 필요
        if (buffer.size() < 2)
        {
            // 불연속 소스는 데이터가 충분하지 않아도 conformity만 중요함
            if (source_type == SourceType::INTERMITTENT)
            {
                scores.latency = 1.0f;
                scores.timing = 0.5f;  // 불연속 소스의 타이밍은 정확하게 평가하기 어려움
                scores.continuity = 0.5f;  // 데이터 부족으로 연속성 평가 불가
                scores.smoothness = 0.5f;  // 데이터 부족으로 부드럽기 평가 불가
                scores.conformity = CalculateConformityScore(odom_initialized, predicted_pose, buffer.back().first, source_name, params);

                // 가중치 계산
                QualityWeights weights = GetSourceWeights(source_name, params);
                scores.score = weights.latency * scores.latency +
                               weights.timing * scores.timing +
                               weights.continuity * scores.continuity +
                               weights.smoothness * scores.smoothness +
                               weights.conformity * scores.conformity;
                return scores;
            }
            else
            {
                // 연속 소스의 경우 데이터가 부족할 때는 신뢰할 수 없음
                scores.latency = 0.5f;
                scores.timing = 0.5f;
                scores.continuity = 0.0f;
                scores.smoothness = 0.0f;
                scores.conformity = 0.0f;
                scores.score = 0.2f;  // 데이터가 충분하지 않으면 낮은 점수
                return scores;
            }
        }

        // 최신 및 이전 데이터 포인트 추출
        const auto& latest_data = buffer.back();
        const auto& previous_data = buffer[buffer.size() - 2];

        const auto& latest_msg = latest_data.first;
        const auto& latest_arrival_time = latest_data.second;
        const auto& previous_msg = previous_data.first;
        const auto& previous_arrival_time = previous_data.second;

        // --- 여러 지표에서 필요한 중간값 계산 ---
        const double arrival_interval = (latest_arrival_time - previous_arrival_time).seconds();
        const double dx = latest_msg.pose.x - previous_msg.pose.x;
        const double dy = latest_msg.pose.y - previous_msg.pose.y;
        const double dtheta = NormalizeAngle(latest_msg.pose.theta - previous_msg.pose.theta);

        // --- 헬퍼 함수를 호출하여 개별 품질 점수 계산 ---
        scores.latency = CalculateLatencyScore(latest_arrival_time, latest_msg.header.stamp, params);
        scores.timing = CalculateTimingScore(source_name, arrival_interval, latest_msg.delta_time, target_hz, params);
        scores.continuity = CalculateContinuityScore(source_name, dx, dy, dtheta, arrival_interval, params);
        scores.smoothness = CalculateSmoothnessScore(source_name, buffer, dx, dy, dtheta, params);
        scores.conformity = CalculateConformityScore(odom_initialized, predicted_pose, latest_msg, source_name, params);
        // 소스별 특성에 맞는 가중치 사용
        QualityWeights weights = GetSourceWeights(source_name, params);
        scores.score = weights.latency * scores.latency +
                       weights.timing * scores.timing +
                       weights.continuity * scores.continuity +
                       weights.smoothness * scores.smoothness +
                       weights.conformity * scores.conformity;

        // RCLCPP_INFO(rclcpp::get_logger("pose_evaluation"),
        //     "[%s] Quality | \n Latency: %.2f, Compute: %.2f, Regularity: %.2f, Continuity: %.2f, Smoothness: %.2f | Score: %.3f",
        //     source_name.c_str(),
        //     scores.latency, scores.computation, scores.regularity, scores.continuity, scores.smoothness,
        //     scores.score);

        scores.score = std::max(0.0f, scores.score);
        return scores;
    }
    // ===== 데이터 품질 평가 함수들 (구현 예정) =====
    // - 최근 업데이트 빈도
    // - 이전 pose와의 일관성은 기본이고 각 데이터의 특성들

    ArucoEvaluationResult EvaluateArucoReliability(
        const std::deque<edie_msgs::msg::PoseWithInfoStamped>& aruco_pose_history,
        const edie_localization_manager::EvaluationParameters& params,
        const rclcpp::Logger& logger)
    {
        // This static variable will hold the last pose that was deemed reliable.
        // It persists across multiple calls to this function.

        ArucoEvaluationResult result;

 
        if (aruco_pose_history.size() < static_cast<size_t>(params.aruco_history_size)) {
            RCLCPP_DEBUG(logger, "Not enough ArUco samples for reliability check. Have %zu, need %d",
                        aruco_pose_history.size(), params.aruco_history_size);
            return result; // is_reliable is false
        }

        // Check 2: Positional and rotational stability 
        double sum_x = 0, sum_y = 0, sum_theta = 0;
        for (const auto& pose_info : aruco_pose_history) {
            sum_x += pose_info.pose.x;
            sum_y += pose_info.pose.y;
            sum_theta += pose_info.pose.theta;
        }
        double mean_x = sum_x / aruco_pose_history.size();
        double mean_y = sum_y / aruco_pose_history.size();
        double mean_theta = sum_theta / aruco_pose_history.size();

        double sq_sum_x = 0, sq_sum_y = 0, sq_sum_theta = 0;
        for (const auto& pose_info : aruco_pose_history) {
            sq_sum_x += std::pow(pose_info.pose.x - mean_x, 2);
            sq_sum_y += std::pow(pose_info.pose.y - mean_y, 2);
            double d_theta = pose_info.pose.theta - mean_theta;
            d_theta = std::atan2(std::sin(d_theta), std::cos(d_theta));
            sq_sum_theta += std::pow(d_theta, 2);
        }
        result.stddev_x = std::sqrt(sq_sum_x / aruco_pose_history.size());
        result.stddev_y = std::sqrt(sq_sum_y / aruco_pose_history.size());
        result.stddev_yaw = std::sqrt(sq_sum_theta / aruco_pose_history.size());

        if (result.stddev_x <= params.aruco_pos_x_stddev_thresh && result.stddev_y <= params.aruco_pos_y_stddev_thresh) {
            result.is_stable_pos = true;
        }
        if (result.stddev_yaw <= params.aruco_yaw_stddev_thresh) {
            result.is_stable_yaw = true;
        }
        
        // Quality metrics evaluation removed - only using positional and rotational stability

        // Preliminary reliability decision
        bool is_preliminary_reliable = result.is_stable_pos && result.is_stable_yaw;

        if (!is_preliminary_reliable) {
            // RCLCPP_INFO(logger, "ArUco pose is not reliable. is_stable_pos: %d, is_stable_yaw: %d",
            //             result.is_stable_pos, result.is_stable_yaw);
            result.is_reliable = false;
            return result;
        }


        // All checks passed! This pose is officially reliable.
        result.is_reliable = true;
        // Update the static variable for the next call.
        // last_accepted_pose = current_pose;

        return result;
    }

    /**
     * @brief 몬테카를로 로컬라이제이션 특화 평가 함수
     * @details 파티클 분산도 등을 고려하여 품질 평가
     */
    void EvalMontecarlo(const edie_localization_manager::PoseBuffer& buffer)
    {
        (void)buffer; // 현재 미사용
        // TODO: 몬테카를로 특화 평가 로직 구현 필요
    }

    /**
     * @brief 섹션 기반 로컬라이제이션 특화 평가 함수
     */
    void EvalSection(const edie_localization_manager::PoseBuffer& buffer)
    {
        (void)buffer; // 현재 미사용
        // TODO: 섹션 특화 평가 로직 구현 필요
    }

    /**
     * @brief 비전 기반 로컬라이제이션 특화 평가 함수
     */
    void EvalVision(const edie_localization_manager::PoseBuffer& buffer)
    {
        (void)buffer; // 현재 미사용
        // TODO: 비전 특화 평가 로직 구현 필요
    }

    /**
     * @brief 비전 방향성 특화 평가 함수
     */
    void EvalVisionDir(const edie_localization_manager::PoseBuffer& buffer)
    {
        (void)buffer; // 현재 미사용
        // TODO: 비전 방향성 특화 평가 로직 구현 필요
    }

    /**
     * @brief 데이터 소스의 데이터가 오래되었는지 확인
     * @param source_name 소스 이름
     * @param last_data_time 마지막 데이터 수신 시간
     * @param current_time 현재 시간
     * @param node 로깅용 노드 (선택적)
     * @return 데이터가 오래되었는지 여부
     */
    bool IsDataStale(
        const std::string& source_name,
        const rclcpp::Time& last_data_time,
        const rclcpp::Time& current_time,
        const edie_localization_manager::EvaluationParameters& params,
        rclcpp::Node* node)
    {
        const double elapsed_since_last_data = (current_time - last_data_time).seconds();
        // 소스 유형에 따른 오래됨 기준 사용
        const double staleness_threshold = GetStalenessThreshold(source_name, params);
        const bool is_stale = (elapsed_since_last_data > staleness_threshold);

        if (is_stale && node != nullptr)
        {
            RCLCPP_DEBUG(node->get_logger(), "[%s] Data too stale (%.2f sec old, threshold=%.2f). Marked as invalid.",
                        source_name.c_str(), elapsed_since_last_data, staleness_threshold);
        }

        return is_stale;
    }

    /**
     * @brief 이전에 오래된 데이터 소스의 품질 점수를 초기화
     * @param source_name 소스 이름
     * @param initial_source_info 초기 소스 정보
     * @param source_info 초기화할 소스 정보
     * @param node 로깅용 노드 (선택적)
     */
    void ResetQualityScore(
        const std::string& source_name,
        const std::map<std::string, edie_localization_manager::LocalizationSourceInfo>& initial_source_info,
        edie_localization_manager::LocalizationSourceInfo& source_info,
        const edie_localization_manager::EvaluationParameters& params,
        rclcpp::Node* node)
    {
        if (initial_source_info.find(source_name) != initial_source_info.end())
        {
            // 소스 유형에 따라 다른 초기 품질 점수 사용
            float initial_score = params.initial_quality_score;

            // 불연속적인 소스는 더 낮은 초기 점수를 가짐
            if (GetSourceType(source_name, params) == SourceType::INTERMITTENT)
            {
                initial_score = params.initial_quality_score * 0.7f;  // 70%로 감소
            }

            source_info.quality_score = initial_score;
            source_info.is_valid = true;

            if (node != nullptr)
            {
                RCLCPP_INFO(node->get_logger(), "[%s] New data after stale period. Resetting quality to %.2f",
                          source_name.c_str(), initial_score);
            }
        }
    }

    /**
     * @brief 소스의 목표 주파수 가져오기
     * @param source_name 소스 이름
     * @param topic_hz 주파수 맵
     * @param default_hz 기본 주파수
     * @param node 로깅용 노드 (선택적)
     * @return 목표 주파수
     */
    double GetTargetHz(
        const std::string& source_name,
        const std::map<std::string, double>& topic_hz,
        double default_hz,
        rclcpp::Node* node)
    {
        if (!topic_hz.empty())
        {
            auto it = topic_hz.find(source_name);
            if (it != topic_hz.end())
            {
                double target_hz = static_cast<double>(it->second);
                if (node != nullptr)
                {
                    RCLCPP_DEBUG(node->get_logger(), "[%s] Using source-specific target Hz: %.1f",
                            source_name.c_str(), target_hz);
                }
                return target_hz;
            }
        }
        return default_hz;
    }

    /**
     * @brief 단일 소스에 대한 품질 평가 수행
     * @param source_name 소스 이름
     * @param buffer 데이터 버퍼
     * @param source_info 품질 정보를 업데이트할 소스 정보 객체
     * @param stale_sources 오래된 소스 추적용 맵
     * @param initial_source_info 초기 소스 정보
     * @param topic_hz 주파수 맵
     * @param current_time 현재 시간
     * @param node ROS 노드 (타임스태프 및 로깅용)
     * @return 평가 성공 여부
     */
    bool EvaluateSourceQuality(
        const bool& odom_initialized,
        const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
        const std::string& source_name,
        const edie_localization_manager::PoseBuffer& buffer,
        edie_localization_manager::LocalizationSourceInfo& source_info,
        std::map<std::string, bool>& stale_sources,
        const edie_localization_manager::EvaluationParameters& params,
        const std::map<std::string, edie_localization_manager::LocalizationSourceInfo>& initial_source_info,
        const std::map<std::string, double>& topic_hz,
        const rclcpp::Time& current_time,
        rclcpp::Node* node)
    {
        if (buffer.empty())
        {
            source_info.is_valid = false;
            return false;
        }

        // 소스 유형 확인 (연속적/불연속적)
        SourceType source_type = GetSourceType(source_name, params);

        // 마지막 데이터 수신 시간 확인
        const rclcpp::Time& last_data_time = buffer.back().second;
        // elapsed_since_last_data는 stale 체크에 사용됨
        // IsDataStale 함수 내부에서 동일한 계산을 하므로 여기서는 제거

        // 마지막 데이터 수신 후 경과 시간 계산
        double elapsed_since_last_data = (current_time - last_data_time).seconds();
        double staleness_threshold = GetStalenessThreshold(source_name, params);

        // 데이터가 오래됨에 따라 품질 점수를 감소시킬 감쇠 계수 계산
        double decay_factor = 1.0;
        if (elapsed_since_last_data > 0)
        {
            // 경과 시간이 임계값에 가까워질수록 점수가 0에 가깝게 감소 (지수적 감쇠)
            double ratio = std::min(1.0, elapsed_since_last_data / staleness_threshold);
            decay_factor = 1.0 - ratio * ratio; // (1 - (t/T)^2) 형태의 감쇠
        }

        // 데이터가 임계 시간보다 오래되었으면 유효하지 않음
        if ((current_time - last_data_time).seconds() > GetStalenessThreshold(source_name, params))
        {
            // 경고 메시지는 EvaluateSourceQuality에서 이미 출력하므로 여기서는 중복 출력하지 않음
            source_info.is_valid = false;
            source_info.latest_pose = buffer.back().first;
            stale_sources[source_name] = true;
            return false;
        }
        // 이전에 오래된 데이터였다가 새로 들어온 경우 품질 점수 초기화 - 소스 유형에 따라 다른 초기값 적용
        if (stale_sources.find(source_name) != stale_sources.end() && stale_sources[source_name])
        {
            ResetQualityScore(source_name, initial_source_info, source_info, params, node);
            stale_sources[source_name] = false;
        }

        // 소스별 목표 주기 가져오기
        const double target_hz = GetTargetHz(source_name, topic_hz, 30.0, node); // Default 30Hz

        // 품질 점수 계산 - 소스별 특성에 맞는 가중치 사용
        QualityScores scores = EvalGeneral(odom_initialized, predicted_pose, source_name, buffer, current_time, target_hz, params);

        // 데이터 유효성 판단 - 소스 유형에 따라 다른 임계값 적용
        float validity_threshold = 0.3f;

        // 불연속적 소스는 더 높은 임계값 적용 (더 엄격하게 판단)
        if (source_type == SourceType::INTERMITTENT)
        {
            validity_threshold = 0.4f;  // 연속적 소스보다 더 높은 임계값

            // 적합성(conformity) 점수가 너무 낮으면 유효하지 않은 것으로 판단
            // 불연속적 소스는 적합성이 특히 중요함
            if (scores.conformity < 0.2f)
            {
                if (node != nullptr)
                {
                    RCLCPP_WARN(node->get_logger(), "[%s] Very low conformity score (%.2f). Data considered invalid.",
                        source_name.c_str(), scores.conformity);
                }
                source_info.is_valid = false;
                source_info.quality_score = scores.score * decay_factor;
                source_info.latest_pose = buffer.back().first;
                return false;
            }
        }

        const bool is_valid = (scores.score >= validity_threshold);

        // 품질 점수 및 유효성 저장
        // 세부 품질 점수 저장
        source_info.is_valid = is_valid;
        source_info.quality_score = scores.score * decay_factor;
        source_info.latest_pose = buffer.back().first;
        source_info.latency_score = scores.latency;
        source_info.timing_score = scores.timing;
        source_info.continuity_score = scores.continuity;
        source_info.smoothness_score = scores.smoothness;
        source_info.conformity_score = scores.conformity;

        // 디버그 로깅 추가
        if (node != nullptr)
        {
            if (is_valid)
            {
                // 디버그 로깅: 품질 점수 기록
                // RCLCPP_INFO(node->get_logger(),
                //     "[%s] Quality: %.3f (L:%.2f T:%.2f C:%.2f S:%.2f CF:%.2f) Type=%s Valid=%s",
                    // source_name.c_str(),
                    // scores.score,
                    // scores.latency,
                    // scores.timing,
                    // scores.continuity,
                    // scores.smoothness,
                    // scores.conformity,
                    // (source_type == SourceType::CONTINUOUS) ? "CONTINUOUS" : "INTERMITTENT",
                    // is_valid ? "true" : "false");
            }
            else
            {
                // 유효하지 않은 데이터의 경우 경고 로깅
                // RCLCPP_WARN(node->get_logger(),
                //     "[%s] Low quality: %.3f (L:%.2f T:%.2f C:%.2f S:%.2f CF:%.2f) Type=%s Threshold=%.2f. Marked as invalid.",
                    // source_name.c_str(),
                    // scores.score,
                    // scores.latency,
                    // scores.timing,
                    // scores.continuity,
                    // scores.smoothness,
                    // scores.conformity,
                    // (source_type == SourceType::CONTINUOUS) ? "CONTINUOUS" : "INTERMITTENT",
                    // validity_threshold);
            }
        }

        return true;
    }

    /**
     * @brief 모든 데이터 소스의 품질을 평가하는 종합 함수
     * @param data_buffers 각 소스별 데이터 버퍼
     * @param initial_source_info 초기 소스 정보 (선택적)
     * @param topic_hz 각 소스별 목표 주기 (Hz)
     * @param node ROS 노드 (타임스태프 및 로깅용)
     * @return 평가 결과를 포함하는 객체
     */
    QualityEvaluationResult Evaluate(
        const bool& odom_initialized,
        const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
        const std::map<std::string, edie_localization_manager::PoseBuffer>& data_buffers,
        const edie_localization_manager::EvaluationParameters& params,
        const std::map<std::string, edie_localization_manager::LocalizationSourceInfo>& initial_source_info,
        const std::map<std::string, double>& topic_hz,
        rclcpp::Node* node,
        const std::map<std::string, std::shared_ptr<rclcpp::Publisher<edie_msgs::msg::PoseEvaluation>>>* quality_publishers)
    {
        // 결과 객체 생성
        QualityEvaluationResult result;

        // 초기값이 있으면 복사
        if (!initial_source_info.empty())
        {
            result.source_info = initial_source_info;
        }

        // 현재 시간 가져오기
        rclcpp::Time current_time = (node != nullptr) ?
            node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);

        // 오래된 데이터 소스를 추적하기 위한 맵
        static std::map<std::string, bool> stale_data_sources;

        for (const auto& [source_name, buffer] : data_buffers)
        {
            // 품질 평가 수행
            EvaluateSourceQuality(
                odom_initialized,
                predicted_pose,
                source_name,
                buffer,
                result.source_info[source_name],
                stale_data_sources,
                params,
                initial_source_info,
                topic_hz,
                current_time,
                node
            );

            // 퍼블리셔가 제공되었고 품질 평가가 수행된 경우 메시지 발행
            if (quality_publishers != nullptr && quality_publishers->count(source_name) > 0)
            {
                // 품질 평가 결과 메시지 생성
                edie_msgs::msg::PoseEvaluation eval_msg;

                // 헤더 설정
                eval_msg.header.stamp = current_time;
                eval_msg.header.frame_id = source_name;

                // 품질 점수 설정
                const auto& source_info = result.source_info[source_name];
                eval_msg.score = source_info.quality_score;
                eval_msg.latency = source_info.latency_score;
                eval_msg.timing = source_info.timing_score;
                eval_msg.continuity = source_info.continuity_score;
                eval_msg.smoothness = source_info.smoothness_score;
                eval_msg.conformity = source_info.conformity_score;

                // 메시지 발행
                quality_publishers->at(source_name)->publish(eval_msg);

                if (node != nullptr)
                {
                    RCLCPP_DEBUG(node->get_logger(),
                        "[%s] Published quality evaluation: score=%.3f valid=%s",
                        source_name.c_str(),
                        eval_msg.score,
                        source_info.is_valid ? "true" : "false");
                }
            }
        }

        return result;
    }
} // namespace pose_evaluation
