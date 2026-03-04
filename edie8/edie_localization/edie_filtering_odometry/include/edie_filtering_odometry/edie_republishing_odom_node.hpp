#ifndef EDIE_REPUBLISHING_ODOM_NODE_HPP
#define EDIE_REPUBLISHING_ODOM_NODE_HPP

#include <vector>
#include <cmath>
#include <random>
#include <functional>
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include "edie_filtering_odometry/odom_utils.hpp"

class EdieRepublishingOdomNode : public rclcpp::Node
{
public:
    EdieRepublishingOdomNode();
    ~EdieRepublishingOdomNode();
    void OdometryCallback(const nav_msgs::msg::Odometry &odom);
    bool HandleFirstOdom();
    void BroadcastTF(geometry_msgs::msg::TransformStamped &t, const nav_msgs::msg::Odometry &target_odom, std::string child_frame_id);
    void PubRawOdom();
private:
    // 클래스 변수 선언
    bool is_first_odom_;

    // 인스턴스 선언
    OdomUtils odom_utils_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Sub.
    nav_msgs::msg::Odometry odom_sub_;
    // Pub.
    nav_msgs::msg::Odometry raw_odom_pub_;

    // Subscriber 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_raw_odom;
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_raw_odom;
};

#endif // EDIE_REPUBLISHING_ODOM_NODE_HPP