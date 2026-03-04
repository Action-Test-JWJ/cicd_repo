#include "edie_behavior/condition_nodes.hpp"

namespace aeirobot
{
  IsFaceDetectionActive::IsFaceDetectionActive(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
  }

  BT::NodeStatus IsFaceDetectionActive::onStart()
  {
    auto edie = Edie::GetInstance();
    if (!edie) 
    {
      return BT::NodeStatus::FAILURE;
    }
    start_time_ = edie->ros_manager->now();
    
    // Check the condition immediately on the first tick.
    return onRunning();
  }

  BT::NodeStatus IsFaceDetectionActive::onRunning()
  {
    auto edie = Edie::GetInstance();
    if (!edie) 
    {
      return BT::NodeStatus::FAILURE;
    }
    
    double timeout_sec = 10.0;
    getInput("timeout_sec", timeout_sec);
    
    auto now = edie->ros_manager->now();

    // Case 1: A face has been detected at some point in the past.
    if (edie->last_face_detection_stamp != 0.0) 
    {
      double elapsed_since_detection = now.seconds() - edie->last_face_detection_stamp;
      if (elapsed_since_detection <= timeout_sec) {
        return BT::NodeStatus::SUCCESS; // Active: seen recently.
      } else {
        return BT::NodeStatus::FAILURE; // Inactive: seen too long ago.
      }
    }
    // Case 2: No face has EVER been detected by the system.
    else 
    {
      double elapsed_since_start = (now - start_time_).seconds();
      // If this node has been ticking for longer than the timeout without any face ever being detected.
      if (elapsed_since_start > timeout_sec) {
          RCLCPP_INFO(edie->ros_manager->get_logger(), "No face has been detected for %.1f seconds. Failing.", timeout_sec);
          return BT::NodeStatus::FAILURE;
      } else {
          // Still in the initial grace period, waiting for the first face.
          return BT::NodeStatus::RUNNING;
      }
    }
  }

  void IsFaceDetectionActive::onHalted()
  {
    // You can add cleanup logic here if needed.
  }

} // namespace aeirobot
