#include "edie_behavior/action_nodes.hpp"
#include <limits>

namespace aeirobot
{

BT::NodeStatus FindClosestMarker::tick()
{
    auto edie = Edie::GetInstance();
    if (!edie) {
        RCLCPP_ERROR(rclcpp::get_logger("FindClosestMarker"), "Failed to get Edie instance!");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(edie->ros_manager->get_logger(), "=== FindClosestMarker: Starting ===");

    // 1. Check if scan result is available
    if (edie->aruco_marker_position.empty()) {
        RCLCPP_WARN(edie->ros_manager->get_logger(), "FindClosestMarker: No markers in last scan result.");
    } else {
        RCLCPP_INFO(edie->ros_manager->get_logger(), "Scan result timestamp: %d.%09u",
                    edie->last_scan_result.header.stamp.sec,
                    edie->last_scan_result.header.stamp.nanosec);
    }
    
    // 1. Get robot pose from scan result
    auto robot_pose = edie->last_scan_result.pose;
    RCLCPP_INFO(edie->ros_manager->get_logger(), "Robot pose from scan: x=%.3f, y=%.3f, z=%.3f", 
                robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);

    // 2. Check loaded marker poses
    RCLCPP_INFO(edie->ros_manager->get_logger(), "Number of loaded marker poses: %zu", 
                edie->aruco_marker_position.size());
    
    if (edie->aruco_marker_position.empty()) {
        RCLCPP_ERROR(edie->ros_manager->get_logger(), "No localization markers loaded!");
        RCLCPP_INFO(edie->ros_manager->get_logger(), "Available parameters in Edie:");
        for (const auto& pair : edie->aruco_marker_position) {
            RCLCPP_INFO(edie->ros_manager->get_logger(), "  - %s: [%zu elements]", 
                        pair.first.c_str(), pair.second.size());
        }
        return BT::NodeStatus::FAILURE;
    }

    // 3. Print all loaded markers for debugging
    RCLCPP_INFO(edie->ros_manager->get_logger(), "Loaded markers:");
    for (const auto& pair : edie->aruco_marker_position) {
        const auto& marker_pose_vec = pair.second;
        RCLCPP_INFO(edie->ros_manager->get_logger(), "  Marker %s: [%zu elements]", 
                    pair.first.c_str(), marker_pose_vec.size());
        if (marker_pose_vec.size() >= 2) {
            RCLCPP_INFO(edie->ros_manager->get_logger(), "    Position: x=%.3f, y=%.3f", 
                        marker_pose_vec[0], marker_pose_vec[1]);
        }
    }

    // 4. Find the closest marker
    double min_dist_sq = std::numeric_limits<double>::max();
    std::string closest_marker_id = "none";
    geometry_msgs::msg::Pose closest_marker_pose;
    
    RCLCPP_INFO(edie->ros_manager->get_logger(), "Calculating distances to markers:");
    
    for (const auto& pair : edie->aruco_marker_position) {
        const auto& marker_pose_vec = pair.second;
        if (marker_pose_vec.size() < 2) {
            RCLCPP_WARN(edie->ros_manager->get_logger(), "Marker %s has insufficient data (%zu elements)", 
                        pair.first.c_str(), marker_pose_vec.size());
            continue; // Needs at least x, y
        }

        double dx = marker_pose_vec[0] - robot_pose.position.x;
        double dy = marker_pose_vec[1] - robot_pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        double distance = std::sqrt(dist_sq);

        RCLCPP_INFO(edie->ros_manager->get_logger(), "  Marker %s: distance=%.3f m (dx=%.3f, dy=%.3f)", 
                    pair.first.c_str(), distance, dx, dy);

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            // Parameter name is like "aruco_marker_position.0", so extract the ID "0"
            size_t last_dot = pair.first.find_last_of('.');
            if (last_dot != std::string::npos) {
                closest_marker_id = pair.first.substr(last_dot + 1);
            } else {
                closest_marker_id = pair.first;
            }
            closest_marker_pose.position.x = marker_pose_vec[0];
            closest_marker_pose.position.y = marker_pose_vec[1];
            
            RCLCPP_INFO(edie->ros_manager->get_logger(), "    -> New closest marker: %s", 
                        closest_marker_id.c_str());
        }
    }
    
    if (closest_marker_id != "none") {
        RCLCPP_INFO(edie->ros_manager->get_logger(), 
            "=== RESULT: Closest marker is ID '%s' at (%.2f, %.2f). Distance: %.2f m ===",
            closest_marker_id.c_str(), 
            closest_marker_pose.position.x, 
            closest_marker_pose.position.y,
            std::sqrt(min_dist_sq));
        
        // Store the found ID in the blackboard for other nodes to use
        setOutput("found_marker_id", closest_marker_id);
        
        // Publish the marker ID via topic
        edie->PubTargetMarkerId(closest_marker_id);

        return BT::NodeStatus::SUCCESS;
    } 
    else {
        RCLCPP_WARN(edie->ros_manager->get_logger(), "=== RESULT: Could not find any closest marker ===");
        RCLCPP_INFO(edie->ros_manager->get_logger(), "This might be because:");
        RCLCPP_INFO(edie->ros_manager->get_logger(), "1. No markers were loaded from parameters");
        RCLCPP_INFO(edie->ros_manager->get_logger(), "2. All markers have insufficient position data");
        RCLCPP_INFO(edie->ros_manager->get_logger(), "3. Parameter names don't match expected format");
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace aeirobot
