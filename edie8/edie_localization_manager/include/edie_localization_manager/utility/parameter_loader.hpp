#ifndef EDIE_LOCALIZATION_MANAGER_PARAMETER_LOADER_HPP_
#define EDIE_LOCALIZATION_MANAGER_PARAMETER_LOADER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "edie_localization_manager/utility/types.hpp"
#include <string>
#include <vector>
#include <map>

namespace edie_localization_manager
{
namespace parameter_loader
{

struct ManagerParameters {
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

void DeclareAndLoadParameters(
    rclcpp::Node* node,
    edie_localization_manager::ManagerParameters& params,
    std::map<std::string, double>& topic_hz);

} // namespace parameter_loader
} // namespace edie_localization_manager

#endif // EDIE_LOCALIZATION_MANAGER_PARAMETER_LOADER_HPP_
