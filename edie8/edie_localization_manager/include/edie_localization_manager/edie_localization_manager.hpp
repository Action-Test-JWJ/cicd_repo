#ifndef EDIE_LOCALIZATION_MANAGER_EDIE_LOCALIZATION_MANAGER_HPP_
#define EDIE_LOCALIZATION_MANAGER_EDIE_LOCALIZATION_MANAGER_HPP_

// C++ system files
#include <chrono>
#include <deque>
#include <map>
#include <string>
#include <vector>
#include <utility> // for std::pair
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

// ROS 2 .h files
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Other libraries' .h files
#include "aeirobot_toolbox/qos_profiles.hpp"
#include "edie_msgs/msg/pose_with_info_stamped.hpp"
#include "edie_msgs/msg/pose_evaluation.hpp"
#include "edie_msgs/msg/bool_stamped.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Current project's .h files
#include "edie_localization_manager/core/pose_prediction.hpp"
#include "edie_localization_manager/core/pose_evaluation.hpp"
#include "edie_localization_manager/core/pose_fusion.hpp"
#include "edie_localization_manager/utility/types.hpp"
#include "edie_localization_manager/utility/info_parser.hpp"
#include "edie_localization_manager/utility/odom_tools.hpp"
#include "edie_localization_manager/utility/tools.hpp"
#include "edie_localization_manager/utility/parameter_loader.hpp"


class EdieLocalizationManager : public rclcpp::Node
{
public:
    EdieLocalizationManager();
    ~EdieLocalizationManager();

    // Main processing function
    void main();

    // Getters
    double GetLoopRate() const;

private:
    // Initialization
    void Init();

    bool ReadYamlManager(const std::string &path);

    // New callbacks for state estimator and RTAP
    void EkfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void RawOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void OpenVinsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Absolute pose from ArUco marker
    void ArucoPoseInfoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void ZuptStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void EkfStopStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void ScanCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void BtIdleCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // TF Broadcasting
    void BroadcastMapToOdom();

    // Buffer management
    template<typename T>
    void AddBuffer(std::deque<T>& buffer, const T& data, size_t max_size);
    template<typename T>
    void CleanData(std::deque<T>& buffer);

    // Core logic
    // 모듈화로 인해 더 이상 사용되지 않음, 코드 참조용으로 유지
    // void EvaluateDataQuality(const std::map<std::string, edie_localization_manager::PoseBuffer>& buffers);

    bool IsPoseValid(const edie_msgs::msg::PoseWithInfoStamped& pose);
    bool IsOdomValid(const nav_msgs::msg::Odometry& odom);
    void CreateQualityPublishers(const std::string& source_name);
    void ProcessScanData();

    // ROS Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ekf_odom;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_raw_odom;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_openvins;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_aruco_pose_info;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_zupt_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ekf_stop_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_scan_command_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bt_idle_;

    // ROS Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_reset_request_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_robot_pose_state_;
    // rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_result_pose;
    rclcpp::Publisher<edie_msgs::msg::PoseWithInfoStamped>::SharedPtr pub_result_pose_info;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_result_odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_robot_pose;
    rclcpp::Publisher<edie_msgs::msg::PoseWithInfoStamped>::SharedPtr pub_aruco_eval_;
    rclcpp::Publisher<edie_msgs::msg::PoseWithInfoStamped>::SharedPtr pub_aruco_accepted_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_stationary_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_scan_result_;

    // Data buffers
    std::map<std::string, edie_localization_manager::PoseBuffer> pose_data_buffers_; // 이름을 pose_data_buffers_로 명확히 함
    std::map<std::string, edie_localization_manager::OdomBuffer> odom_data_buffers_; // Odometry 버퍼 추가
    std::map<std::string, edie_localization_manager::LocalizationSourceInfo> source_info_;
    std::map<std::string, rclcpp::Publisher<edie_msgs::msg::PoseEvaluation>::SharedPtr> quality_publishers_;

    // Configuration & Parameters
    edie_localization_manager::ManagerParameters params_;
    std::vector<pose_fusion::Strategy> fusion_strategies;

    std::map<std::string, double> topic_hz_; // 이건 tools에서 사용해서 남겨둠
    bool game_data_received = false;
    // Odometry Prediction
    edie_msgs::msg::PoseWithInfoStamped predicted_pose_;
    tf2::Transform current_odom_tf_;
    tf2::Transform last_odom_tf_;
    bool odom_initialized_ = false;
    bool allow_prediction_ = true;
    std::mutex odom_mutex_;
    std::mutex callback_mutex;
    tf2::Transform odom_delta_sum_;

    // Localization Results & History
    std::deque<edie_msgs::msg::PoseWithInfoStamped> pose_history_;

    edie_msgs::msg::PoseWithInfoStamped result_pose;
    nav_msgs::msg::Odometry result_odom;
    nav_msgs::msg::Odometry result_odom_to_pelvis;

    geometry_msgs::msg::Pose robot_base;

    // ArUco pose callback (marker-based absolute pose)
    void ArucoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // Latest ArUco absolute pose (overrides fusion when available)
    std::atomic<bool> has_new_aruco_pose_{false};
    std::deque<edie_msgs::msg::PoseWithInfoStamped> aruco_pose_history_;
    
    // Odometry Reset Handling & TF Broadcasting
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_;
    tf2::Transform map_to_odom_transform_;
    std::atomic<bool> is_waiting_for_reset_done_{false};

    // Stationary status from ZUPT
    std::atomic<bool> is_zupt_stopped_{false};
    int consecutive_zupt_true_count_ = 0;

    // Stationary status from EKF-based stop detector
    std::atomic<bool> is_ekf_stopped_{false};
    
    // BT is idle status
    std::atomic<bool> is_bt_idle_{true};
    std::atomic<uint8_t> robot_pose_state_{0}; // 0: unstable, 1: stable
    
    // Add this trigger flag
    bool reset_request_triggered_ = false;
    
    // Scan mode variables
    std::atomic<bool> is_scanning_{false};
    std::vector<edie_msgs::msg::PoseWithInfoStamped> scan_poses_;
};
#endif // EDIE_LOCALIZATION_MANAGER_EDIE_LOCALIZATION_MANAGER_HPP_
