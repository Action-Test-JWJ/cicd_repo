#ifndef EDIE_IMU_NODE_HPP
#define EDIE_IMU_NODE_HPP

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
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_math/math_tool.hpp"
#include "edie_imu/kalman_filter.hpp"
#include "edie_imu/low_pass_filter.hpp"
#include "edie_imu/one_euro_filter.hpp"
#include "aeirobot_toolbox/aeirobot_signal_processing.h"

class EdieImuNode : public rclcpp::Node
{
public:
    EdieImuNode();
    ~EdieImuNode();
    void CalibrateTimeOffset();
    void ReadImuData();
    // 필터링 관련 함수
    void CheckFiltering();
    std::pair<std::array<double, 3>, std::array<double, 3>> ProcessLpfImu(std::array<double, 3>& accel, std::array<double, 3>& gyro, double timestamp);
    // 멀티쓰레딩 관련 함수 추가
    void ProcessingLoop();
    // Publish 함수
    void PubImuData();
    // 타이머 함수
    void TimerCallback();
    void ImuCallback(const sensor_msgs::msg::Imu &);
    void OffsetInitCallback(const std_msgs::msg::Bool &);
    // 파라미터 초기화 및 로드 함수
    void DeclareParams();
    void InitializeParams();
    void LoadConfigFromFile(const std::string& filename);
    // 정지상태 판별 함수
    bool IsStationary(const tf2::Vector3& accl, const std::array<double, 3>& gyro);
    // 오디멘트리 예측 함수
    void PredictOdometry(double t, std::array<double, 3>& accl, std::array<double, 3>& gyro);

    std::vector<std::array<double,3>> gyro_buffer;
    std::vector<std::array<double, 3>> accel_buffer;
    
    // (sub) raw or offset 받은 값
    std::array<double, 3> accl_sub, gyro_sub;
    // (pub) LPF 반환값
    std::array<double, 3> accl_lpf, gyro_lpf;
    // (input) 오디멘트리 예측 입력 변수
    std::array<double, 3> accl_odom_pred_in, gyro_odom_pred_in;
    
    // 오디멘트리 예측 변수
    tf2::Quaternion latest_Q{0,0,0,1};
    tf2::Quaternion last_moving_Q{0,0,0,1};

    double base_time{-1.0};
    double prev_time{0.0};
    double latest_time{0.0};
private:
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_lpf;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_kf;
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_data;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_offset_done;
    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // 인스턴스 생성
    sensor_msgs::msg::Imu imu_msg_sub;
    sensor_msgs::msg::Imu imu_msg_lpf;
    sensor_msgs::msg::Imu imu_msg_kf;
    KalmanFilter kalman_filter_; 

    // 1. Butterworth LPF 인스턴스 생성
    LowPassFilter lpf_accel_x;
    LowPassFilter lpf_accel_y;
    LowPassFilter lpf_accel_z;
    LowPassFilter lpf_gyro_x;
    LowPassFilter lpf_gyro_y;
    LowPassFilter lpf_gyro_z;

    // 2. OneEuroFilter 인스턴스 생성
    OneEuroFilter oef_accel_x;
    OneEuroFilter oef_accel_y;
    OneEuroFilter oef_accel_z;
    OneEuroFilter oef_gyro_x;
    OneEuroFilter oef_gyro_y;
    OneEuroFilter oef_gyro_z;

    std::string imu_topic;
    
    // 클래스 멤버 변수 선언
    bool is_first_sub;
    bool is_data_ready;
    bool ready_to_publish;
    bool was_stationary;
    double g_to_m_sec_sqrd;

    // 커널 타임스탬프
    struct timespec ts_start; // 읽기 시작 시점
    struct timespec ts_end; // 읽기 종료 시점

    using ns = std::chrono::nanoseconds;
    using mono_clock = std::chrono::steady_clock;
    using sys_clock  = std::chrono::system_clock;
    std::chrono::nanoseconds steady_to_system_offset_{};

    // LPF 관련 멤버 변수 선언
    double sample_freq_;
    bool use_low_pass_filter_;
    // 작아질 수록 노이즈 제거 효과 증가
    double lpf_accel_cutoff_;
    double lpf_gyro_cutoff_;

    // One Euro Filter 관련 멤버 변수 선언
    double min_cutoff_;
    double beta_;
    double dcutoff_;

    // 멀티쓰레딩 관련 멤버 변수 선언
    std::thread worker_thread_;
    std::mutex worker_mutex_;
    std::condition_variable worker_cv_;
    std::queue<rclcpp::Time> imu_time_queue_;
    bool stop_worker_;

    // Kalman filter 관련 멤버 변수 선언
    bool use_kalman_filter_;
    bool kf_accel_updated_;
    bool kf_gyro_updated_;
    // 추정치
    double accl_x_est_; 
    double accl_y_est_;
    double accl_z_est_;
    double gyro_x_est_;
    double gyro_y_est_;
    double gyro_z_est_;
    // 프로세스 노이즈 공분산
    double Q_cov_;
    // 측정 노이즈 공분산
    double R_cov_;

    // Zero Velocity Detection 관련 멤버 변수 선언
    double zero_vel_accel_threshold_;
    double zero_vel_gyro_threshold_;
    double accel_norm_;
    double gyro_norm_;
};

#endif // EDIE_IMU_NODE_HPP
