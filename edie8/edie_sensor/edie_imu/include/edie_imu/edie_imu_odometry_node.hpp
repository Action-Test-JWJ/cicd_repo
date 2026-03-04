#ifndef EDIE_IMU_ODOMETRY_NODE_HPP
#define EDIE_IMU_ODOMETRY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cstdlib>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <cmath> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_math/math_tool.hpp"
#include "aeirobot_toolbox/aeirobot_signal_processing.h"

class EdieImuOdomNode : public rclcpp::Node
{
public:
    EdieImuOdomNode();
    ~EdieImuOdomNode();

    // 타이머 함수
    void TimerCallback();
    void ImuCallback(const sensor_msgs::msg::Imu &);
    // 파라미터 초기화 및 로드 함수
    // 정지상태 판별 함수
    bool IsStationary(const tf2::Vector3& accl, const std::array<double, 3>& gyro);
    // 오디멘트리 예측 함수
    void PredictOdometry(double t, std::array<double, 3>& accl, std::array<double, 3>& gyro);
    // 오디멘트리 발행 함수
    void pubLatestOdometry(double t, std::array<double, 3>& P, std::array<double, 3>& V, tf2::Quaternion& Q);
    
    // (sub) raw or offset 받은 값
    std::array<double, 3> accl_sub, gyro_sub;
    
    // 오디멘트리 예측 변수
    std::array<double, 3> latest_P{0.0, 0.0, 0.0};
    std::array<double, 3> last_moving_P{0.0, 0.0, 0.0};
    std::array<double, 3> latest_V{0.0, 0.0, 0.0};
    tf2::Quaternion latest_Q{0,0,0,1};
    tf2::Quaternion last_moving_Q{0,0,0,1};

private:
    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_data;
    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // 인스턴스 생성
    sensor_msgs::msg::Imu imu_sub;
    nav_msgs::msg::Odometry odom_pub;

    // 클래스 멤버 변수 선언
    bool is_data_ready;
    bool ready_to_publish;
    bool was_stationary;
    double g_to_m_sec_sqrd;
    double sample_freq_;
    double v_forward_;
    
    double base_time{-1.0};
    double prev_time{0.0};
    double latest_time{0.0};

    // Zero Velocity Detection 관련 멤버 변수 선언
    double zero_vel_accel_threshold_;
    double zero_vel_gyro_threshold_;
};

#endif // EDIE_IMU_ODOMETRY_NODE_HPP
