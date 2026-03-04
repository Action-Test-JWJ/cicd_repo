#include "edie_localization_manager/core/pose_fusion.hpp"

// 익명 네임스페이스 (파일 내부에서만 사용되는 헬퍼 함수들)
namespace
{
/**
 * @brief 두 각도 간의 최단 거리를 계산함
 * @details 두 각도 사이의 최단 각도 차이를 계산하며, 결과는 0~PI 사이의 값으로 나타냄
 * @param from 시작 각도 (rad)
 * @param to 대상 각도 (rad)
 * @return 최단 각도 차이의 절대값 (rad)
 */
double ShortestAngleDist(double from, double to)
{
    double result = std::fmod(to - from, 2.0 * M_PI);
    if (result > M_PI)
    {
        result -= 2.0 * M_PI;
    }
    else if (result < -M_PI)
    {
        result += 2.0 * M_PI;
    }
    return std::abs(result);
}

/**
 * @brief 두 포즈 간의 거리를 계산함
 * @details 위치 차이와 각도 차이를 모두 고려하여 두 포즈 간의 거리를 계산함
 * @param a 첫 번째 포즈
 * @param b 두 번째 포즈
 * @return 두 포즈 간의 거리 (위치 및 각도 차이를 포함)
 */
double CalculateDistance(const geometry_msgs::msg::Pose2D& a, const geometry_msgs::msg::Pose2D& b, const edie_localization_manager::FusionParameters& fusion_params)
{
    double pos_dist = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    double angle_dist = ShortestAngleDist(a.theta, b.theta);
    return pos_dist + fusion_params.angle_weight * angle_dist;
}

/**
 * @brief 포즈 스무딩을 위한 가중 이동 평균 필터 적용
 * @details 여러 프레임에 걸쳐 가중치를 적용하여 더 부드러운 스무딩 제공. 각도는 사인, 코사인 평균을 통해 계산함.
 * @param fused_pose 현재 포즈
 * @param pose_history 이전 포즈 이력
 * @param node 로깅용 노드
 * @return 스무딩된 포즈
 */
edie_msgs::msg::PoseWithInfoStamped ApplySmoothing(
    const edie_msgs::msg::PoseWithInfoStamped& fused_pose,
    const std::deque<edie_msgs::msg::PoseWithInfoStamped>& pose_history,
    rclcpp::Node* node,
    const edie_localization_manager::FusionParameters& fusion_params)
{
    if (pose_history.empty())
    {
        return fused_pose; // 이력이 없으면 스무딩 생략
    }

    // 최대 사용할 프레임 수 (설정 파라미터 사용)
    const size_t max_window_size = static_cast<size_t>(fusion_params.max_window_size);

    // 실제 사용할 프레임 수 (이력을 초과하지 않도록)
    const size_t window_size = std::min(max_window_size, pose_history.size() + 1);

    // 이전 스무딩 결과
    const auto& prev_smoothed = pose_history.front();

    // 현재 포즈와 이전 포즈 간의 차이 계산 (포즈 변화의 크기 측정)
    double dx = std::abs(fused_pose.pose.x - prev_smoothed.pose.x);
    double dy = std::abs(fused_pose.pose.y - prev_smoothed.pose.y);
    double dtheta = std::abs(fused_pose.pose.theta - prev_smoothed.pose.theta);

    // 위치 차이에 따라 스무딩 강도 자동 조절
    // 차이가 클수록 현재 포즈의 가중치를 높임 (더 빠르게 전환)
    double pose_distance = std::sqrt(dx*dx + dy*dy + dtheta*dtheta);

    // 파라미터로 설정된 스무딩 값 사용
    const double distance_threshold = fusion_params.distance_threshold; // 포즈 변화의 임계값
    const double max_alpha = fusion_params.max_alpha;                 // 최대 가중치
    const double min_alpha = fusion_params.min_alpha;                // 최소 가중치

    // 포즈 변화에 따른 가중치 조절
    double current_alpha;
    if (pose_distance > distance_threshold) {
        // 변화가 큼 때 현재 포즈에 더 높은 가중치 부여
        current_alpha = max_alpha;
    } else {
        // 변화에 비례하여 스무딩 강도 조절
        current_alpha = min_alpha + (max_alpha - min_alpha) * (pose_distance / distance_threshold);
    }

    // 윈도우 기반 가중 평균 계산을 위한 변수 초기화
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    double sum_weights = 0.0;

    // 가중치 배열 계산 (가장 최근 데이터에 가장 높은 가중치)
    std::vector<double> weights(window_size);
    for (size_t i = 0; i < window_size; ++i) {
        // 가우시안과 유사한 가중치 배분 사용 (0번째가 현재 프레임)
        weights[i] = std::exp(-fusion_params.decay_rate * i); // 설정된 감케율 사용
        sum_weights += weights[i];
    }

    // 가중치 정규화
    for (size_t i = 0; i < window_size; ++i) {
        weights[i] /= sum_weights;
    }

    // 현재 프레임 처리 (가장 최근 데이터)
    sum_x += weights[0] * fused_pose.pose.x;
    sum_y += weights[0] * fused_pose.pose.y;
    sum_sin += weights[0] * std::sin(fused_pose.pose.theta);
    sum_cos += weights[0] * std::cos(fused_pose.pose.theta);

    // 이전 프레임들 처리
    size_t history_size = std::min(window_size - 1, pose_history.size());
    for (size_t i = 0; i < history_size; ++i) {
        const auto& past_pose = pose_history[i];
        sum_x += weights[i + 1] * past_pose.pose.x;
        sum_y += weights[i + 1] * past_pose.pose.y;
        sum_sin += weights[i + 1] * std::sin(past_pose.pose.theta);
        sum_cos += weights[i + 1] * std::cos(past_pose.pose.theta);
    }

    // 최종 가중 평균을 사용하여 스무딩된 포즈 계산
    auto smoothed_pose = fused_pose;
    smoothed_pose.pose.x = sum_x;
    smoothed_pose.pose.y = sum_y;
    double theta = std::atan2(sum_sin, sum_cos);
    // 각도 범위를 0~2π로 조정
    if (theta < 0.0)
    {
        theta += 2.0 * M_PI;
    }
    smoothed_pose.pose.theta = theta;

    RCLCPP_DEBUG(node->get_logger(), "스무딩 적용함: 현재 가중치=%.3f, 윈도우 크기=%zu, 변화량=%.3f",
                current_alpha, window_size, pose_distance);

    return smoothed_pose;
}

/**
 * @brief 품질 점수를 가중치로 사용하여 여러 소스로부터 포즈를 융합함
 * @details 각 소스의 품질 점수를 가중치로 사용하여 각 소스의 포즈를 가중평균함. 각도는 사인, 코사인 가중평균을 통해 계산함.
 * @param sources 위치 정보 소스들의 맵
 * @param node 로깅 및 시간 정보를 위한 ROS 노드
 * @return 융합된 포즈 정보
 */
edie_msgs::msg::PoseWithInfoStamped FuseBySimpleWeightedAverage(
    const pose_fusion::SourceInfoMap& sources,
    rclcpp::Node* node)
{
    double total_weight = 0.0;
    double weighted_sum_x = 0.0;
    double weighted_sum_y = 0.0;
    double weighted_sum_cos = 0.0;
    double weighted_sum_sin = 0.0;

    // 각 소스의 품질 점수를 가중치로 사용하여 가중합 계산
    for (const auto& pair : sources)
    {
        if (pair.second.is_valid)
        {
            total_weight += pair.second.quality_score;
            weighted_sum_x += pair.second.latest_pose.pose.x * pair.second.quality_score;
            weighted_sum_y += pair.second.latest_pose.pose.y * pair.second.quality_score;
            weighted_sum_cos += std::cos(pair.second.latest_pose.pose.theta) * pair.second.quality_score;
            weighted_sum_sin += std::sin(pair.second.latest_pose.pose.theta) * pair.second.quality_score;
        }
    }

    // 가중평균을 통해 융합된 포즈 계산
    edie_msgs::msg::PoseWithInfoStamped fused_pose;
    fused_pose.pose.x = weighted_sum_x / total_weight;
    fused_pose.pose.y = weighted_sum_y / total_weight;
    double theta = std::atan2(weighted_sum_sin / total_weight, weighted_sum_cos / total_weight);
    // 각도 범위를 0~2π로 조정
    if (theta < 0.0)
    {
        theta += 2.0 * M_PI;
    }
    fused_pose.pose.theta = theta;
    fused_pose.header.stamp = node->get_clock()->now();
    fused_pose.header.frame_id = "map";

    return fused_pose;
}

/**
 * @brief 다른 소스들과의 평균 거리를 기반으로 이상치를 제거함
 * @details 각 소스의 포즈가 다른 소스들과 얼마나 배치적으로 유사한지 평가하고, 멀리 떨어져 있는 포즈에 대해 불신을 증가시킴
 * @param sources 위치 정보 소스들의 맵
 * @param node 로깅용 노드
 * @return 이상치가 제거된 소스 정보 맵
 */
pose_fusion::SourceInfoMap FuseByOutlierRejection(
    const pose_fusion::SourceInfoMap& sources,
    [[maybe_unused]] rclcpp::Node* node,
    const edie_localization_manager::FusionParameters& fusion_params)
{
    if (sources.size() < 2)
    {
        return sources; // 소스가 2개 미만이면 이상치 제거 불가
    }

    // 1. 모든 포즈 쌍 간의 거리 계산
    std::map<std::string, std::vector<std::pair<std::string, double>>> distances;
    for (auto it1 = sources.begin(); it1 != sources.end(); ++it1)
    {
        if (!it1->second.is_valid) continue;
        for (auto it2 = std::next(it1); it2 != sources.end(); ++it2)
        {
            if (!it2->second.is_valid) continue;
            double dist = CalculateDistance(it1->second.latest_pose.pose, it2->second.latest_pose.pose, fusion_params);
            distances[it1->first].push_back({it2->first, dist});
            distances[it2->first].push_back({it1->first, dist});
        }
    }

    // 2. 각 포즈의 평균 거리를 기반으로 이상치 점수 계산
    std::map<std::string, double> outlier_scores;
    for (const auto& pair : distances)
    {
        double total_dist = 0.0;
        for (const auto& dist_pair : pair.second)
        {
            total_dist += dist_pair.second;
        }
        outlier_scores[pair.first] = total_dist / pair.second.size();
    }

    // 3. 이상치 점수가 임계값을 초과하면 해당 소스의 신뢰도를 낮춤
    // 파라미터로 불러온 값 사용
    const double outlier_threshold = fusion_params.outlier_threshold; // 이상치로 판단할 평균 거리 임계값
    const double outlier_penalty = fusion_params.outlier_penalty; // 이상치에 적용할 신뢰도 페널티
    auto updated_sources = sources;
    for (const auto& score_pair : outlier_scores)
    {
        if (score_pair.second > outlier_threshold)
        {
            updated_sources[score_pair.first].quality_score *= outlier_penalty;
            // RCLCPP_WARN(node->get_logger(), "이상치 의심: 소스 '%s'의 신뢰도를 낮춥니다 (평균 거리: %.2f, 신규 신뢰도: %.3f)",
            //             score_pair.first.c_str(), score_pair.second, updated_sources[score_pair.first].quality_score);
        }
    }

    return updated_sources;
}
} // anonymous namespace

namespace pose_fusion
{
/**
 * @brief 여러 위치 정보 소스를 융합하여 하나의 포즈를 생성함
 * @details 이 함수는 set_pose가 있는지 먼저 확인한 후, 있다면 해당 값을 사용하고 없다면 지정된 전략에 따라 포즈를 융합함
 * @param set_pose_result SetPose 요청에 의해 세팅된 포즈 결과
 * @param strategies 적용할 융합 전략 목록
 * @param initial_source_info 초기 위치 정보 소스들
 * @param pose_history 이전 포즈 이력
 * @param node 로깅용 노드 포인터
 * @return 융합된 포즈 정보 또는 SetPose에 의해 강제 지정된 포즈
 */
edie_msgs::msg::PoseWithInfoStamped FusePoses(
    const edie_msgs::msg::PoseWithInfoStamped& set_pose_result,
    const std::vector<Strategy>& strategies,
    SourceInfoMap& initial_source_info,
    const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
    std::deque<edie_msgs::msg::PoseWithInfoStamped>& pose_history,
    rclcpp::Node* node,
    const edie_localization_manager::FusionParameters& fusion_params)
{
    // 1. set_pose_result에 특별한 값이 있는지 확인
    bool use_set_pose = false;
    if (!set_pose_result.info.empty())
    {
        for (const auto& info : set_pose_result.info)
        {
            if (info.key == "set_pose" && info.value == "true")
            {
                use_set_pose = true;
                break;
            }
        }
    }

    // 2. 특별한 값이 있다면, 해당 값을 즉시 반환하고 다른 데이터 버퍼 초기화함
    if (use_set_pose)
    {
        RCLCPP_WARN(node->get_logger(), "SetPose에 의해 위치가 강제 지정되었습니다.");

        // 다른 센서 데이터 버퍼 초기화함
        for (auto& pair : initial_source_info)
        {
            pair.second.is_valid = false;
        }

        // 포즈 히스토리를 초기화함
        pose_history.clear();

        // SetPose가 제공한 포즈를 히스토리에 추가함
        pose_history.push_back(set_pose_result);

        // SetPose 결과를 반환함
        return set_pose_result;
    }

    // 3. 특별한 값이 없다면, 기존 융합 로직을 수행함
    // 유효한 소스가 하나라도 있는지 확인함
    bool has_valid_source = std::any_of(initial_source_info.begin(), initial_source_info.end(),
                                         [](const auto& pair){ return pair.second.is_valid; });

    // 유효한 소스가 없다면 이전 히스토리를 반환함
    if (!has_valid_source)
    {
        // RCLCPP_WARN(node->get_logger(), "모든 위치 소스가 유효하지 않아 이전 위치를 반환함");
        RCLCPP_DEBUG(node->get_logger(), "모든 위치 소스가 유효하지 않아 예측된 포즈를 사용합니다.");
        pose_history.push_back(predicted_pose);
        return predicted_pose;
    }

    // 융합을 위한 변수들을 초기화함
    SourceInfoMap current_source_info = initial_source_info;
    edie_msgs::msg::PoseWithInfoStamped intermediate_pose;
    bool pose_fused = false;

    for (auto strategy : strategies)
    {
        switch (strategy)
        {
            case Strategy::kOutlierRejection:
            {
                current_source_info = FuseByOutlierRejection(current_source_info, node, fusion_params);
                bool any_valid = std::any_of(current_source_info.begin(), current_source_info.end(),
                                             [](const auto& pair) { return pair.second.is_valid; });

                // 모든 소스가 이상치로 판단되었는지 확인함
                if (!any_valid && !initial_source_info.empty())
                {
                    // 최고 품질 점수를 가진 소스를 찾음
                    auto best_source_it = std::max_element(initial_source_info.begin(), initial_source_info.end(),
                                                               [](const auto& a, const auto& b) {
                                                                   return a.second.quality_score < b.second.quality_score;
                                                               });

                    if (best_source_it != initial_source_info.end())
                    {
                        current_source_info[best_source_it->first].is_valid = true;
                        RCLCPP_WARN(node->get_logger(),
                                    "모든 소스가 이상치로 판단됨. 최고 품질의 소스로 폴백함: '%s'",
                                    best_source_it->first.c_str());
                    }
                }
                break;
            }
            case Strategy::kSimpleWeightedAverage:
                {
                    intermediate_pose = FuseBySimpleWeightedAverage(current_source_info, node);
                    pose_fused = true;
                    break;
                }
            case Strategy::kSmoothing:
            {
                if (pose_fused)
                {
                    // 포즈가 퓨전되었을 경우 스무딩을 적용함
                    intermediate_pose = ApplySmoothing(intermediate_pose, pose_history, node, fusion_params);
                }
                else
                {
                    RCLCPP_WARN(node->get_logger(), "스무딩은 포즈가 융합된 후에만 적용할 수 있습니다.");
                }
                break;
            }
        }
    }

    // 성공적으로 포즈가 융합되었다면 결과를 반환함
    if (pose_fused)
    {
        // RCLCPP_INFO(node->get_logger(), "포즈 융합 성공");
        intermediate_pose.info.clear();
        intermediate_pose.info.resize(2);  // 크기를 2로 설정

        intermediate_pose.info[0].key = "fusion_pose";
        intermediate_pose.info[0].value = "true";
        return intermediate_pose;
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "어떤 융합 전략도 성공적으로 적용되지 않았습니다.");

        // 이전 포즈 히스토리가 있다면 마지막 포즈를 반환함
        if (!pose_history.empty())
        {
            return pose_history.back();
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "융합 전략이 실패했고 이전 포즈도 없습니다!");
            return edie_msgs::msg::PoseWithInfoStamped();
        }
    }

} // namespace pose_fusion
}
