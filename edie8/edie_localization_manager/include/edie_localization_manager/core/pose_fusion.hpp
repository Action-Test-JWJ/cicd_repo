#ifndef EDIE_LOCALIZATION_MANAGER_POSE_FUSION_HPP_
#define EDIE_LOCALIZATION_MANAGER_POSE_FUSION_HPP_

// C++ system files
#include <map>
#include <string>
#include <deque>
#include <cmath>
#include <algorithm>
#include <vector>
#include <initializer_list>

// ROS 관련 헤더
#include "rclcpp/rclcpp.hpp"

// 다른 프로젝트 헤더
#include "edie_msgs/msg/pose_with_info_stamped.hpp"

// 현재 프로젝트 헤더
#include "edie_localization_manager/utility/types.hpp"

namespace pose_fusion
{

    using SourceInfoMap = std::map<std::string, edie_localization_manager::LocalizationSourceInfo>;

    /**
     * 여러 소스의 위치 정보를 융합하여 최종 위치를 결정하는 함수
     *
     * @param source_info 각 소스별 위치 정보와 품질 점수
     * @param node ROS 노드 포인터 (로깅용)
     * @param previous_pose 이전 프레임의 위치 정보 (급격한 변화 방지용)
     * @return 융합된 최종 위치 정보
     */
    // 융합에 사용할 전략을 정의하는 열거형
    enum class Strategy
    {
    kSimpleWeightedAverage, // 가중 평균
    kOutlierRejection,      // 이상치 제거
    kSmoothing              // 스무딩
    };

    /**
     * @brief 여러 위치 소스를 융합하여 최종 포즈를 계산함.
     *
     * @param strategies 사용할 융합 전략 목록.
     * @param source_info 융합에 사용할 소스 정보 맵.
     * @param pose_history 이전 프레임의 최종 포즈 목록 (스무딩에 사용됨).
     * @param node 로깅을 위한 ROS 2 노드 포인터.
     * @return edie_msgs::msg::PoseWithInfoStamped 융합된 최종 포즈.
     */
    edie_msgs::msg::PoseWithInfoStamped FusePoses(
    const edie_msgs::msg::PoseWithInfoStamped& set_pose_result,
    const std::vector<Strategy>& strategies,
    SourceInfoMap& initial_source_info,
    const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
    std::deque<edie_msgs::msg::PoseWithInfoStamped>& pose_history,
    rclcpp::Node *node,
    const edie_localization_manager::FusionParameters& fusion_params);

    /**
     * 두 위치 간의 거리를 계산하는 함수
     */
    double CalculateDistance(
        const geometry_msgs::msg::Pose2D &pose1,
        const geometry_msgs::msg::Pose2D &pose2,
        const edie_localization_manager::FusionParameters& fusion_params);

    /**
     * 두 각도 간의 차이를 계산하는 함수 (-PI ~ PI 범위로 정규화)
     */
    double NormalizeAngleDifference(double angle1, double angle2);

    /**
     * 위치 품질 점수를 기반으로 가중치 계산하는 함수
     */
    std::map<std::string, double> CalculateWeights(
        const std::map<std::string, edie_localization_manager::LocalizationSourceInfo> &source_info);

} // namespace pose_fusion

#endif // EDIE_LOCALIZATION_MANAGER_POSE_FUSION_HPP_
