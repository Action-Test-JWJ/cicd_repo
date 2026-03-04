#ifndef EDIE_PATH_TRACKING_NODE_HPP
#define EDIE_PATH_TRACKING_NODE_HPP

#include <vector>
#include <cmath>
#include <random>
#include <functional>
#include <yaml-cpp/yaml.h>
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aeirobot_toolbox/aeirobot_signal_processing.h"
#include "tf2_ros/transform_broadcaster.h"  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

class EdiePathTrackingNode : public rclcpp::Node
{
public:
    EdiePathTrackingNode();
    ~EdiePathTrackingNode();
    // void PubPath();
    // void DeclareParams();
    // void InitializeParams();
    void RawOdomCallback(const nav_msgs::msg::Odometry &);
    void EkfOdomCallback(const nav_msgs::msg::Odometry &);
private:

    nav_msgs::msg::Odometry raw_odom_sub_;
    nav_msgs::msg::Odometry ekf_odom_sub_;
    nav_msgs::msg::Path raw_path_pub_;
    nav_msgs::msg::Path ekf_path_pub_;
    // Subscriber   
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_raw_odom;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ekf_odom;
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_raw_path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ekf_path;
};

#endif // EDIE_PATH_TRACKING_NODE_HPP