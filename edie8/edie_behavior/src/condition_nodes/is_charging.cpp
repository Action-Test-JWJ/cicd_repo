#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/edie.hpp"

namespace aeirobot
{

IsCharging::IsCharging(const std::string& name, const BT::NodeConfig& config)
  : BT::ConditionNode(name, config)
{
}

BT::PortsList IsCharging::providedPorts()
{
    return { 
        BT::InputPort<int>("status", 0, "Expected charging status")
    };
}

BT::NodeStatus IsCharging::tick()
{
    auto edie = Edie::GetInstance();
    if (!edie) {
        return BT::NodeStatus::FAILURE;
    }

    auto expected_status_opt = getInput<int>("status");
    if (!expected_status_opt)
    {
        return BT::NodeStatus::FAILURE;
    }
    uint8_t expected_status = static_cast<uint8_t>(expected_status_opt.value());


    if (edie->charging_status == expected_status)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot
