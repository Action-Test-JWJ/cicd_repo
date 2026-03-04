#ifndef EDIE_CORRECTING_ODOM_NODE_HPP
#define EDIE_CORRECTING_ODOM_NODE_HPP

#include <vector>
#include <cmath>
#include <random>
#include <mutex>
#include <functional>
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "edie_msgs/msg/bool_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_broadcaster.h"  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include "edie_filtering_odometry/odom_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// --- SE2 유틸 ---
struct SE2 { double x{0}, y{0}, yaw{0}; };

static inline SE2 Identity() { 
    return SE2{0, 0, 0}; 
}

static inline SE2 compose(const SE2& A, const SE2& B) {
  SE2 R;
  
  const double c = std::cos(A.yaw), s = std::sin(A.yaw);
  R.x = A.x + c*B.x - s*B.y;
  R.y = A.y + s*B.x + c*B.y;
  R.yaw = OdomUtils::NormalizeAngleToPi(A.yaw + B.yaw);

  return R;
}

static inline SE2 inverse(const SE2& A) {
  const double c = std::cos(A.yaw), s = std::sin(A.yaw);
  SE2 R;
  
  R.x = -( c*A.x + s*A.y);
  R.y = -(-s*A.x + c*A.y);
  R.yaw = -A.yaw;
  
  return R;
}

class EdieCorrectingOdomNode : public rclcpp::Node
{
public:
    EdieCorrectingOdomNode();
    ~EdieCorrectingOdomNode();
    void DeclareParameters();
    void InitializeParameters();
    void EkfOdometryCallback(const nav_msgs::msg::Odometry &odom);
    void ResetRequestCallback(const std_msgs::msg::Bool &msg);
    void BroadcastTF(geometry_msgs::msg::TransformStamped &t, const nav_msgs::msg::Odometry &target_odom, std::string child_frame_id);
    void PubEkfOdom();
    void PubResetDoneBoolStamped();
private:
    // 클래스 변수 선언
    bool is_reset_requested_;
    bool is_reset_done_;
    bool last_reset_request_;
    bool cur_stop_;
    bool prev_stop_;
    bool below_active_;
    bool stop_state_initialized_;
    double lin_stop_th_;
    double ang_stop_th_;
    double stop_hold_time_th_;
    SE2 C_;
    rclcpp::Time below_since_;
    rclcpp::Time stop_enter_time_;
    // 인스턴스 선언
    OdomUtils odom_utils_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // 멀티스레딩 문제 방지를 위한 뮤텍스 선언
    std::mutex mtx_;

    // Sub.
    nav_msgs::msg::Odometry ekf_odom_sub_;
    std_msgs::msg::Bool reset_bool_sub_;
    // Pub.
    nav_msgs::msg::Odometry ekf_odom_pub_;
    edie_msgs::msg::BoolStamped reset_done_bool_stamped_pub_;
    std_msgs::msg::Bool stop_bool_pub_;

    // Subscriber 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ekf_odom;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset_bool;
    
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ekf_odom;    
    rclcpp::Publisher<edie_msgs::msg::BoolStamped>::SharedPtr pub_reset_done_bool_stamped;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_bool;
};

#endif // EDIE_CORRECTING_ODOM_NODE_HPP