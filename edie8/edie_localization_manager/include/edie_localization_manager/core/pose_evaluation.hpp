#ifndef POSE_EVALUATION_HPP_
#define POSE_EVALUATION_HPP_

// C++ system files
#include <map>
#include <set>
#include <string>
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs, std::sqrt, M_PI, std::floor
#include <chrono>    // For std::chrono

// ROS 관련 헤더
#include "rclcpp/rclcpp.hpp"

// 다른 프로젝트 헤더
#include "edie_msgs/msg/pose_with_info_stamped.hpp"
#include "edie_msgs/msg/pose_evaluation.hpp"

// 현재 프로젝트 헤더
#include "edie_localization_manager/utility/types.hpp"

namespace edie_localization_manager
{
    // PoseBuffer와 PoseStampedWithTime은 types.hpp에 정의되어 있음
    // EvaluationParameters 구조체는 types.hpp에 정의되어 있음
} // namespace edie_localization_manager

namespace pose_evaluation
{
    /**
     * @brief ArUco 신뢰성 평가의 상세 결과를 담는 구조체
     */
    struct ArucoEvaluationResult
    {
        bool is_reliable = false;
        bool is_stable_pos = false;
        bool is_stable_yaw = false;
        double stddev_x = 0.0;
        double stddev_y = 0.0;
        double stddev_yaw = 0.0;
    };

    // 데이터 소스의 유형 정의
    enum class SourceType
    {
        CONTINUOUS,  // 연속적인 데이터를 제공하는 소스 (monte_carlo, visual_slam)
        INTERMITTENT // 불연속적인 데이터를 제공하는 소스 (section, vision, vision_dir, trilateration)
    };

    // 소스별 평가 가중치 구조체
    struct QualityWeights
    {
        float latency = 0.20f;     // 지연성 가중치
        float timing = 0.20f;      // 타이밍 가중치
        float continuity = 0.20f;  // 연속성 가중치
        float smoothness = 0.20f;  // 부드러움 가중치
        float conformity = 0.20f;  // 적합성 가중치
    };

    // A struct to hold all the components of a quality score.
    struct QualityScores
    {
        float score = 0.0f;
        float latency = 1.0f;
        float timing = 1.0f;
        float continuity = 1.0f;
        float smoothness = 1.0f;
        float conformity = 1.0f;
    };

    // A struct to hold all quality evaluation results for multiple sources.
    struct QualityEvaluationResult
    {
        std::map<std::string, edie_localization_manager::LocalizationSourceInfo> source_info;

        // 평가 결과에서 특정 소스의 품질 점수 가져오기
        float GetQualityScore(const std::string& source_name) const
        {
            auto it = source_info.find(source_name);
            return (it != source_info.end()) ? it->second.quality_score : 0.0f;
        }

        // 평가 결과에서 특정 소스가 유효한지 확인
        bool IsSourceValid(const std::string& source_name) const
        {
            auto it = source_info.find(source_name);
            return (it != source_info.end()) && it->second.is_valid;
        }
    };



    /**
     * @brief ArUco 포즈의 신뢰성을 평가합니다 (안정성 + 품질).
     * @param aruco_pose_history ArUco 포즈 이력
     * @param params ArUco 신뢰성 평가 파라미터가 포함된 EvaluationParameters
     * @param logger 로깅을 위한 rclcpp::Logger
     * @return bool 신뢰할 수 있으면 true
     */
    bool IsArucoPoseReliable(
        const std::deque<edie_msgs::msg::PoseWithInfoStamped>& aruco_pose_history,
        const edie_localization_manager::EvaluationParameters& params,
        const rclcpp::Logger& logger);


    /**
     * @brief 소스 이름에 따른 소스 유형을 반환
     * @param source_name 소스 이름
     * @return 소스 유형 (연속적/불연속적)
     */
    SourceType GetSourceType(const std::string& source_name, const edie_localization_manager::EvaluationParameters& params);


    /**
     * @brief 소스 이름에 따른 평가 가중치를 반환
     * @param source_name 소스 이름
     * @return 소스별 품질 평가 가중치
     */
    QualityWeights GetSourceWeights(const std::string& source_name, const edie_localization_manager::EvaluationParameters& params);

    /**
     * @brief 소스 이름에 따른 데이터 오래됨 기준을 반환
     * @param source_name 소스 이름
     * @return 오래됨 기준 시간 (초)
     */
    double GetStalenessThreshold(const std::string& source_name, const edie_localization_manager::EvaluationParameters& params);


    QualityScores EvalGeneral(
        const bool& odom_initialized,
        const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
        const std::string & source_name,
        const edie_localization_manager::PoseBuffer & buffer,
        const rclcpp::Time & now,
        double target_hz,
        const edie_localization_manager::EvaluationParameters& params);

    void EvalMontecarlo(const edie_localization_manager::PoseBuffer& buffer);
    void EvalSection(const edie_localization_manager::PoseBuffer& buffer);
    void EvalVision(const edie_localization_manager::PoseBuffer& buffer);
    void EvalVisionDir(const edie_localization_manager::PoseBuffer& buffer);

    /**
     * @brief ArUco 포즈의 신뢰성을 평가합니다 (안정성 + 품질).
     * @param aruco_pose_history ArUco 포즈 이력
     * @param params ArUco 신뢰성 평가 파라미터가 포함된 EvaluationParameters
     * @param logger 로깅을 위한 rclcpp::Logger
     * @return ArucoEvaluationResult 평가의 상세 결과를 담은 구조체
     */
    ArucoEvaluationResult EvaluateArucoReliability(
        const std::deque<edie_msgs::msg::PoseWithInfoStamped>& aruco_pose_history,
        const edie_localization_manager::EvaluationParameters& params,
        const rclcpp::Logger& logger);

    /**
     * @brief 모든 데이터 소스의 품질을 평가합니다.
     * @param data_buffers 각 소스별 데이터 버퍼
     * @param initial_source_info 초기 소스 정보 (선택적)
     * @param topic_hz 각 소스별 목표 주기 (Hz)
     * @param node ROS 노드 (타임스태프 및 로깅용)
     * @return QualityEvaluationResult 평가 결과를 포함하는 객체
     */
    /**
     * @brief 모든 데이터 소스의 품질을 평가하고 결과를 발행합니다.
     * @param odom_initialized 오도메트리 초기화 여부
     * @param predicted_pose 예측된 포즈
     * @param data_buffers 각 소스별 데이터 버퍼
     * @param initial_source_info 초기 소스 정보 (선택적)
     * @param topic_hz 각 소스별 목표 주기 (Hz)
     * @param node ROS 노드 (타임스태프 및 로깅용)
     * @param quality_publishers 품질 평가 결과 발행용 퍼블리셔 맵 (선택적)
     * @return QualityEvaluationResult 평가 결과를 포함하는 객체
     */
    QualityEvaluationResult Evaluate(
        const bool& odom_initialized,
        const edie_msgs::msg::PoseWithInfoStamped& predicted_pose,
        const std::map<std::string, edie_localization_manager::PoseBuffer>& data_buffers,
        const edie_localization_manager::EvaluationParameters& params,
        const std::map<std::string, edie_localization_manager::LocalizationSourceInfo>& initial_source_info,
        const std::map<std::string, double>& topic_hz,
        rclcpp::Node* node,
        const std::map<std::string, std::shared_ptr<rclcpp::Publisher<edie_msgs::msg::PoseEvaluation>>>* quality_publishers = nullptr
    );  // ROS 타임스태프 및 로깅 용도
}   // namespace pose_evaluation

#endif  // POSE_EVALUATION_HPP_
