#ifndef GAUSSIAN_MODELING_FILTER_HPP
#define GAUSSIAN_MODELING_FILTER_HPP

#include <vector>
#include <cmath>
#include <random>
#include <functional>
#include <yaml-cpp/yaml.h>
#include "tf2/utils.h"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "aeirobot_toolbox/aeirobot_signal_processing.h"
#include "tf2_ros/transform_broadcaster.h"  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include "edie_filtering_odometry/odom_utils.hpp"

class GaussianModelingFilter
{
public:
    GaussianModelingFilter();
    ~GaussianModelingFilter();
    void DeclareParams();
    void InitializeParams();
    void LoadConfigFromFile(const std::string& filename);
    void CopyParams(double lx, double ly, double lyaw, double myaw); 
    void UpdateSamples(const nav_msgs::msg::Odometry &odometry, geometry_msgs::msg::PoseArray &samples);
    void NoiseGaussianModeling(double transl_std_x, double transl_std_y, double rot1_std, double rot2_std, geometry_msgs::msg::PoseArray &samples);
private:
    // 파라미터 선언 
    double alpha1_;
    double alpha2_;
    double alpha3_x_;
    double alpha3_y_;
    double alpha3_;
    double alpha4_;
    double delta_transl;
    double delta_rot1;
    double delta_rot2;
    double translation_threshold_;
    double weight_pos_;
    double weight_yaw_;
    // 마지막 오도메트리 데이터 저장
    double last_odom_x_;
    double last_odom_y_;
    double last_odom_yaw_;
    // 측정한 오도메트리 데이터 저장
    double yaw_meas_;

    OdomUtils odom_utils_;
    // alias를 위한 함수 객체 (OdomUtils::GetAngleDiff를 대신 호출)
    std::function<double(double, double)> angle_diff_;

    std::default_random_engine noise_generator_;
};

#endif // GAUSSIAN_MODELING_FILTER_HPP