#include "edie_filtering_odometry/edie_path_tracking_node.hpp"

EdiePathTrackingNode::EdiePathTrackingNode()
    : Node("edie_path_tracking_node")
    //   is_first_odom_(true),
{   
    rclcpp::QoS sub_qos(rclcpp::KeepLast(10)); // 1000 -> 10
    sub_qos.best_effort(); // 패킷 유실 허용, 지연 ↓
    sub_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    rclcpp::QoS pub_qos(rclcpp::KeepLast(1)); // 50 -> 10
    pub_qos.reliable();   // 패킷 유실 없음, 지연 있음 ->  신뢰성 있는 QoS로 설정 
    pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

    // Subscriber
    sub_raw_odom = create_subscription<nav_msgs::msg::Odometry>("/edie8/localization/raw_odom", sub_qos,
        std::bind(&EdiePathTrackingNode::RawOdomCallback, this, std::placeholders::_1));
    sub_ekf_odom = create_subscription<nav_msgs::msg::Odometry>("/edie8/localization/ekf_odom", sub_qos,
        std::bind(&EdiePathTrackingNode::EkfOdomCallback, this, std::placeholders::_1));

    // Publisher
    pub_raw_path = create_publisher<nav_msgs::msg::Path>("/edie8/localization/raw_path", pub_qos);
    pub_ekf_path = create_publisher<nav_msgs::msg::Path>("/edie8/localization/ekf_path", pub_qos);
}

EdiePathTrackingNode::~EdiePathTrackingNode()
{
}

void EdiePathTrackingNode::RawOdomCallback(const nav_msgs::msg::Odometry &raw_odom_)
{
    raw_odom_sub_ = raw_odom_;

    // 첫 수신 시에만 frame_id 설정
    if (raw_path_pub_.header.frame_id.empty()) {
        raw_path_pub_.header.frame_id = raw_odom_.header.frame_id;  // 보통 "odom" 등
    }

    // Append to path
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = raw_odom_.header;
    stamped.pose = raw_odom_.pose.pose;
    raw_path_pub_.header.stamp = stamped.header.stamp;
    raw_path_pub_.poses.push_back(stamped);

    pub_raw_path->publish(raw_path_pub_);
}

void EdiePathTrackingNode::EkfOdomCallback(const nav_msgs::msg::Odometry &ekf_odom_)
{
    ekf_odom_sub_ = ekf_odom_;

    if (ekf_path_pub_.header.frame_id.empty()) {
        ekf_path_pub_.header.frame_id = ekf_odom_.header.frame_id;
    }
    
    // Append to path
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = ekf_odom_.header;
    stamped.pose = ekf_odom_.pose.pose;
    ekf_path_pub_.header.stamp = stamped.header.stamp;
    ekf_path_pub_.poses.push_back(stamped);
    
    pub_ekf_path->publish(ekf_path_pub_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdiePathTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}