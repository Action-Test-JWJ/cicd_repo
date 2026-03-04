#include "edie_behavior/condition_nodes.hpp"
#include <rclcpp/time.hpp>
#include <cmath>

namespace aeirobot
{

BT::NodeStatus IsScanResultReceived::tick()
{
  auto edie = Edie::GetInstance();
  
  if (edie->is_scan_result_received)
  {
    RCLCPP_INFO(edie->ros_manager->get_logger(), "Scan result received and consumed by BT.");
    
    // Reset flag for next time
    edie->is_scan_result_received = false;
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_DEBUG(edie->ros_manager->get_logger(), "No scan result available yet");
  return BT::NodeStatus::FAILURE;
}

} // namespace aeirobot
