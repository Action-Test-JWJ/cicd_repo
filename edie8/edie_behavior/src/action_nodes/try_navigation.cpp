#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{

BT::NodeStatus TryNavigation::onStart()
{
    auto edie = Edie::GetInstance();

    auto command = getInput<uint8_t>("command");
    if (!command)
    {
        return BT::NodeStatus::FAILURE;
    }

    edie->is_bt_idle_ = false;
    // 이전 상태를 지워서 새로운 업데이트를 기다리도록 함
    auto status_key = getInput<std::string>("status_key");
    if (status_key) {
        edie->navigation_statuses.erase(status_key.value());
    }

    edie->PubNavigation(command.value());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TryNavigation::onRunning()
{
    auto edie = Edie::GetInstance();

    auto status_key_opt = getInput<std::string>("status_key");
    if (!status_key_opt) {
        return BT::NodeStatus::FAILURE;
    }
    std::string status_key = status_key_opt.value();

    std::string status_value;
    {
        auto it = edie->navigation_statuses.find(status_key);
        if (it != edie->navigation_statuses.end()) {
            status_value = it->second;
        }
    }

    if (status_value == "0") {
        edie->is_bt_idle_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TryNavigation::onHalted()
{
    // 중단되었을 때, idle(0) 명령을 보내서 현재 동작을 멈춤
    auto edie = Edie::GetInstance();
    edie->is_bt_idle_ = true;
    edie->PubNavigation(0);
}

} // namespace aeirobot