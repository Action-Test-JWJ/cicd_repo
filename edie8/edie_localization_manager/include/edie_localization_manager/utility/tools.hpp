#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include <string>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "edie_localization_manager/core/pose_set.hpp"
#include "edie_localization_manager/utility/types.hpp"

using edie_localization_manager::ManagerParameters;
using edie_localization_manager::EvaluationParameters;
using edie_localization_manager::FusionParameters;

namespace tools
{
    /**
     * manager_param.yaml 파일을 읽어 ManagerParameters 구조체에 설정함
     * @param path YAML 파일 경로
     * @param params 관리자 파라미터 구조체
     * @param logger 로깅을 위한 ROS 로거
     * @return 성공 여부
     */
    bool ReadYamlManager(
        const std::string& file_path, 
        ManagerParameters& params, 
        std::map<std::string, double>& topic_hz,
        rclcpp::Logger logger);

    /**
     * @brief manager_eval_param.yaml 파일을 읽어 EvaluationParameters 구조체에 설정함
     *
     * @param filename YAML 파일 경로
     * @param eval_params 평가 파라미터 구조체
     * @param logger 로깅을 위한 ROS 로거
     * @return 성공 여부
     */
    bool ReadYamlEvaluation(const std::string& filename, EvaluationParameters& eval_params, rclcpp::Logger logger);

    /**
     * @brief manager_fusion_param.yaml 파일을 읽어 FusionParameters 구조체에 설정함
     *
     * @param filename YAML 파일 경로
     * @param fusion_params 퓨전 파라미터 구조체
     * @param logger 로깅을 위한 ROS 로거
     * @return 성공 여부
     */
    bool ReadYamlFusion(const std::string& filename, FusionParameters& fusion_params, rclcpp::Logger logger);
} // namespace tools

#endif // TOOLS_HPP_
