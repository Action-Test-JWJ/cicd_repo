#ifndef EDIE_FILTERING_ODOM_NODE_HPP
#define EDIE_FILTERING_ODOM_NODE_HPP

#include <vector>
#include <cmath>
#include <random>
#include <functional>
#include <yaml-cpp/yaml.h>
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aeirobot_toolbox/aeirobot_signal_processing.h"
#include "tf2_ros/transform_broadcaster.h"  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include "edie_filtering_odometry/gaussian_modeling_filter.hpp"
#include "edie_filtering_odometry/odom_utils.hpp"

class EdieFilteringOdomNode : public rclcpp::Node
{
public:
    EdieFilteringOdomNode();
    ~EdieFilteringOdomNode();
    void PubFilteredModeledOdom();
    void DeclareParams();
    void InitializeParams();
    void LoadConfigFromFile(const std::string& filename);
    void OdometryCallback(const nav_msgs::msg::Odometry &);
    void LPFOdometry(double position_x, double position_y, double yaw);
    bool HandleFirstOdom(const nav_msgs::msg::Odometry &odometry);
    void BroadcastTF(geometry_msgs::msg::TransformStamped &t, const nav_msgs::msg::Odometry &target_odom, std::string child_frame_id);

    // void UpdateSamples(const nav_msgs::msg::Odometry &odometry);
    // void NoiseGaussianModeling(double transl_std_x, double transl_std_y, double rot1_std, double rot2_std);
private:
    // 클래스 변수 선언
    bool is_first_odom_;
    // 파라미터 선언 
    bool use_modeled_odom_;
    double delta_transl;
    double delta_rot1;
    double delta_rot2;
    double alpha1_;
    double alpha2_;
    double alpha3_x_;
    double alpha3_y_;
    double alpha3_;
    double alpha4_;
    int64_t nr_samples_;
    double weight_pos_;
    double weight_yaw_;
    double translation_threshold_;
    double rotation_threshold_;
    double high_odom_yaw_noise_;
    double low_odom_yaw_noise_;
    double high_imu_yaw_noise_;
    double low_imu_yaw_noise_;

    // 측정한 오도메트리 데이터 저장
    double yaw_meas_;
    // LPF 입력으로 사용할 데이터 저장
    double lpf_in_x_;
    double lpf_in_y_;
    double lpf_in_yaw_;
    // 마지막 오도메트리 데이터 저장
    double last_odom_x_;
    double last_odom_y_;
    double last_odom_yaw_;
    double last_rep_sample_x_;
    double last_rep_sample_y_;
    double last_rep_sample_yaw_;

    // 인스턴스 선언
    // std::default_random_engine noise_generator_;
    GaussianModelingFilter gaussian_filter_;
    OdomUtils odom_utils_;
    std::function<double(double, double)> angle_diff_;     // OdomUtils::GetAngleDiff를 대신 호출
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    bool is_rotating_only_;
    bool slip_hold_;
    rclcpp::Time     last_rot_end_time_;
    rclcpp::Duration rot_hold_duration_{rclcpp::Duration::from_seconds(0.5)};  // 0.5초 홀드

    geometry_msgs::msg::PoseArray samples_;
    geometry_msgs::msg::Pose rep_sample_;
    nav_msgs::msg::Odometry odom_msg_sub_;
    nav_msgs::msg::Odometry lpf_odom_;
    nav_msgs::msg::Odometry modeled_odom_;

    // Subscriber   
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_array;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lpf_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_modeled_odom;

    // LPF 관련 멤버 변수 선언
    // 작아질 수록 노이즈 제거 효과 증가
    double lpf_odom_x;
    double lpf_odom_y;
    double lpf_odom_yaw;

    double lpf_x_cutoff_;
    double lpf_y_cutoff_;
    double lpf_yaw_cutoff_;

    aeirobot::LowPassFilter lpf_position_x;
    aeirobot::LowPassFilter lpf_position_y;
    aeirobot::LowPassFilter lpf_yaw;
};

#endif // EDIE_FILTERING_ODOM_NODE_HPP