#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{

BT::NodeStatus SetTargetMarkerId::tick()
{
    auto edie = Edie::GetInstance();
    if (!edie) {
        RCLCPP_ERROR(rclcpp::get_logger("SetTargetMarkerId"), "Failed to get Edie instance!");
        return BT::NodeStatus::FAILURE;
    }

    // Get marker_id from input port
    std::string marker_id;
    if (!getInput("marker_id", marker_id)) {
        RCLCPP_ERROR(edie->ros_manager->get_logger(), "SetTargetMarkerId: Failed to get marker_id input!");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(edie->ros_manager->get_logger(), "SetTargetMarkerId: Setting target marker ID to '%s'", marker_id.c_str());

    // Publish the marker ID via topic
    edie->PubTargetMarkerId(marker_id);

    return BT::NodeStatus::SUCCESS;
}

} // namespace aeirobot

