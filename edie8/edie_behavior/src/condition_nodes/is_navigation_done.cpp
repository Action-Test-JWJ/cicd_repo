#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{

BT::NodeStatus IsNavigationDone::tick()
{
    auto edie = Edie::GetInstance();

    auto key = getInput<std::string>("state_key");
    auto status = getInput<std::string>("state_value");

    if (!key || !status) {
        return BT::NodeStatus::FAILURE;
    }

    std::string current_status;

    auto it = edie->navigation_statuses.find(key.value());
    if (it != edie->navigation_statuses.end()) {
        current_status = it->second;
    }

    if (current_status == status.value()) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot