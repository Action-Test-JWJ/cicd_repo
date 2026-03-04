#include "edie_localization_manager/utility/tools.hpp"

namespace tools
{

bool ReadYamlManager(const std::string &path,
                             ManagerParameters &params,
                             std::map<std::string, double>& topic_hz,
                             rclcpp::Logger logger)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(path.c_str());
        std::cout << "yaml_node: " << yaml_node << std::endl;
        RCLCPP_INFO(logger, "성공: manager_param.yaml 로드함");

        // 기본 파라미터 읽기
        if (yaml_node["buffer"] && yaml_node["buffer"]["pose_buffer_seconds"])
            params.pose_buffer_seconds = yaml_node["buffer"]["pose_buffer_seconds"].as<double>();

        if (yaml_node["buffer"] && yaml_node["buffer"]["odom_buffer_size"])
            params.odom_buffer_size = yaml_node["buffer"]["odom_buffer_size"].as<int>();

        if (yaml_node["logic"] && yaml_node["logic"]["loop_rate"])
            params.loop_rate = yaml_node["logic"]["loop_rate"].as<double>();

        if (yaml_node["logic"] && yaml_node["logic"]["pose_history_size"])
            params.pose_history_size = yaml_node["logic"]["pose_history_size"].as<int>();

        // 퓨전 전략 읽기
        if (yaml_node["logic"] && yaml_node["logic"]["fusion_strategies"])
        {
            params.fusion_strategies_str.clear();
            const YAML::Node& strategies_node = yaml_node["logic"]["fusion_strategies"];
            for (YAML::const_iterator it = strategies_node.begin(); it != strategies_node.end(); ++it)
            {
                params.fusion_strategies_str.push_back(it->as<std::string>());
            }
        }
        if (yaml_node["topic_hz"]) 
        {
            topic_hz.clear();
            const YAML::Node& topic_hz_node = yaml_node["topic_hz"];
            if (topic_hz_node.IsMap()) {
                for (auto it = topic_hz_node.begin(); it != topic_hz_node.end(); ++it) {
                std::string key   = it->first.as<std::string>();
                double      value = it->second.as<double>();
                topic_hz[key] = value;
                }
            }

            // ─────── 여기서 로드된 내용 확인 ───────
            RCLCPP_INFO(logger, "Loaded topic_hz entries:");
            for (const auto &p : topic_hz) 
            {
                RCLCPP_INFO(logger, "  %s: %.2f", p.first.c_str(), p.second);
            }
            // ─────────────────────────────────────
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger, "YAML 관리자 파라미터 파일 읽기 실패: %s", e.what());
        return false;
    }

    return true;
}

bool ReadYamlEvaluation(const std::string& filename, edie_localization_manager::EvaluationParameters& eval_params, rclcpp::Logger logger)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(filename.c_str());
        RCLCPP_INFO(logger, "성공: manager_eval_param.yaml 로드함");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger, "평가 YAML 파일 읽기 실패: %s", e.what());
        return false;
    }

    try
    {
        // 중첩 구조로 평가 노드에 접근
        const YAML::Node& eval_node = yaml_node["edie_localization_manager"]["ros__parameters"]["evaluation"];
        if (!eval_node) {
            RCLCPP_ERROR(logger, "평가 매개변수 노드를 찾을 수 없음");
            return false;
        }

        // ===== 지연 시간(Latency) 평가 파라미터 =====
        eval_params.max_allowed_age_sec = eval_node["latency"]["max_allowed_age_sec"].as<double>();

        // ===== 타이밍(Timing) 평가 파라미터 =====
        const YAML::Node& timing_node = eval_node["timing"];
        eval_params.acceptable_compute_ratio = timing_node["acceptable_compute_ratio"].as<double>();
        eval_params.weight_regularity = timing_node["weight_regularity"].as<double>();
        eval_params.weight_computation = timing_node["weight_computation"].as<double>();

        // ===== 연속성(Continuity) 평가 파라미터 =====
        const YAML::Node& continuity_node = eval_node["continuity"];
        eval_params.max_linear_velocity_mps = continuity_node["max_linear_velocity_mps"].as<double>();
        eval_params.max_angular_velocity_rps = continuity_node["max_angular_velocity_rps"].as<double>();

        // ===== 부드러움(Smoothness) 평가 파라미터 =====
        const YAML::Node& smoothness_node = eval_node["smoothness"];
        eval_params.trend_window_size = smoothness_node["trend_window_size"].as<int>();
        eval_params.max_allowed_trend_deviation_m = smoothness_node["max_allowed_trend_deviation_m"].as<double>();
        eval_params.max_allowed_trend_deviation_rad = smoothness_node["max_allowed_trend_deviation_rad"].as<double>();

        // ===== 적합성(Conformity) 평가 파라미터 =====
        const YAML::Node& conformity_node = eval_node["conformity"];
        eval_params.max_position_diff_m = conformity_node["max_position_diff_m"].as<double>();
        eval_params.max_angle_diff_rad = conformity_node["max_angle_diff_rad"].as<double>();
        eval_params.intermittent_pos_diff_multiplier = conformity_node["intermittent_pos_diff_multiplier"].as<double>();
        eval_params.intermittent_angle_diff_multiplier = conformity_node["intermittent_angle_diff_multiplier"].as<double>();

        // ===== 기타 파라미터 =====
        eval_params.initial_quality_score = eval_node["misc"]["initial_quality_score"].as<double>();

        // ===== ArUco 신뢰성 평가 파라미터 =====
        const YAML::Node& aruco_node = eval_node["aruco_reliability"];
        eval_params.aruco_history_size = aruco_node["history_size"].as<int>();
        eval_params.aruco_pos_x_stddev_thresh = aruco_node["pos_x_stddev_thresh"].as<double>();
        eval_params.aruco_pos_y_stddev_thresh = aruco_node["pos_y_stddev_thresh"].as<double>();
        eval_params.aruco_yaw_stddev_thresh = aruco_node["yaw_stddev_thresh"].as<double>();
        eval_params.aruco_reproj_err_thresh = aruco_node["reproj_err_thresh"].as<double>();
        eval_params.aruco_min_area_thresh = aruco_node["min_area_thresh"].as<double>();
        eval_params.aruco_min_update_distance_m = aruco_node["min_update_distance_m"].as<double>();
        eval_params.aruco_zupt_true_threshold = aruco_node["zupt_true_threshold"].as<int>();
        eval_params.final_check_pos_x_stddev_thresh = aruco_node["final_check_pos_x_stddev_thresh"].as<double>();
        eval_params.final_check_pos_y_stddev_thresh = aruco_node["final_check_pos_y_stddev_thresh"].as<double>();
        eval_params.final_check_yaw_stddev_thresh = aruco_node["final_check_yaw_stddev_thresh"].as<double>();

        // ===== 소스 타입 분류 =====
        const YAML::Node& source_types_node = eval_node["source_types"];
        eval_params.continuous_sources.clear();
        eval_params.intermittent_sources.clear();

        const YAML::Node& continuous_sources_node = source_types_node["continuous_sources"];
        for (YAML::const_iterator it = continuous_sources_node.begin(); it != continuous_sources_node.end(); ++it) {
            eval_params.continuous_sources.push_back(it->as<std::string>());
        }

        const YAML::Node& intermittent_sources_node = source_types_node["intermittent_sources"];
        for (YAML::const_iterator it = intermittent_sources_node.begin(); it != intermittent_sources_node.end(); ++it) {
            eval_params.intermittent_sources.push_back(it->as<std::string>());
        }

        // 임계값 읽기
        eval_params.continuous_staleness_threshold = source_types_node["staleness_threshold"]["continuous"].as<double>();
        eval_params.intermittent_staleness_threshold = source_types_node["staleness_threshold"]["intermittent"].as<double>();

        // ===== 평가 가중치 =====
        const YAML::Node& weights_node = eval_node["weights"];
        const YAML::Node& continuous_weights = weights_node["continuous"];
        eval_params.continuous_weights.latency = continuous_weights["latency"].as<double>();
        eval_params.continuous_weights.timing = continuous_weights["timing"].as<double>();
        eval_params.continuous_weights.continuity = continuous_weights["continuity"].as<double>();
        eval_params.continuous_weights.smoothness = continuous_weights["smoothness"].as<double>();
        eval_params.continuous_weights.conformity = continuous_weights["conformity"].as<double>();

        const YAML::Node& intermittent_weights = weights_node["intermittent"];
        eval_params.intermittent_weights.latency = intermittent_weights["latency"].as<double>();
        eval_params.intermittent_weights.timing = intermittent_weights["timing"].as<double>();
        eval_params.intermittent_weights.continuity = intermittent_weights["continuity"].as<double>();
        eval_params.intermittent_weights.smoothness = intermittent_weights["smoothness"].as<double>();
        eval_params.intermittent_weights.conformity = intermittent_weights["conformity"].as<double>();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger, "평가 매개변수 파싱 실패: %s", e.what());
        return false;
    }

    return true;
}

bool ReadYamlFusion(const std::string& filename, edie_localization_manager::FusionParameters& fusion_params, rclcpp::Logger logger)
{
    YAML::Node yaml_node;
    try
    {
        yaml_node = YAML::LoadFile(filename.c_str());
        RCLCPP_INFO(logger, "성공: manager_fusion_param.yaml 로드함");
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger, "퓨전 YAML 파일 읽기 실패: %s", e.what());
        return false;
    }

    try
    {
        // 중첩 구조로 퓨전 노드에 접근
        const YAML::Node& fusion_node = yaml_node["edie_localization_manager"]["ros__parameters"]["fusion"];
        if (!fusion_node) {
            RCLCPP_ERROR(logger, "퓨전 매개변수 노드를 찾을 수 없음");
            return false;
        }

        // ===== 거리 계산 관련 파라미터 =====
        const YAML::Node& calculation_node = fusion_node["calculation"];
        fusion_params.angle_weight = calculation_node["angle_weight"].as<double>();

        // ===== 스무딩 관련 파라미터 =====
        const YAML::Node& smoothing_node = fusion_node["smoothing"];
        fusion_params.max_window_size = smoothing_node["max_window_size"].as<int>();
        fusion_params.distance_threshold = smoothing_node["distance_threshold"].as<double>();
        fusion_params.max_alpha = smoothing_node["max_alpha"].as<double>();
        fusion_params.min_alpha = smoothing_node["min_alpha"].as<double>();
        fusion_params.decay_rate = smoothing_node["decay_rate"].as<double>();

        // ===== 이상치 처리 관련 파라미터 =====
        const YAML::Node& outlier_node = fusion_node["outlier"];
        fusion_params.outlier_threshold = outlier_node["threshold"].as<double>();
        fusion_params.outlier_penalty = outlier_node["penalty"].as<double>();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger, "퓨전 매개변수 파싱 실패: %s", e.what());
        return false;
    }

    return true;
}

} // namespace tools
