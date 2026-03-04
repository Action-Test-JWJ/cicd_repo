#ifndef ODOM_UTILS_HPP
#define ODOM_UTILS_HPP

#include <vector>
#include <cmath>
#include <random>
#include <filesystem>
#include <functional>
#include <yaml-cpp/yaml.h>
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aeirobot_toolbox/aeirobot_signal_processing.h"
#include "tf2_ros/transform_broadcaster.h"  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

class OdomUtils
{
public:
    OdomUtils();
    ~OdomUtils();
    static double NormalizeAngleToPi(double z);
    double GetAngleDiff(double a, double b);
    size_t FindMinErrorIdx(geometry_msgs::msg::PoseArray samples, double weight_pos, double weight_yaw);
    static double yawFromQuat(const geometry_msgs::msg::Quaternion& q);
    geometry_msgs::msg::Pose GetPose(double x, double y, double z, double yaw_rad);
    void SetPubOdomInfo(const nav_msgs::msg::Odometry &in, nav_msgs::msg::Odometry &out, const geometry_msgs::msg::Pose &pose_filtered, const std::string &child_frame_id);
    void SetPubPoseInfo(const nav_msgs::msg::Odometry &in, geometry_msgs::msg::PoseStamped &out, const geometry_msgs::msg::Pose &pose_filtered);
private:
};

#endif // ODOM_UTILS_HPP