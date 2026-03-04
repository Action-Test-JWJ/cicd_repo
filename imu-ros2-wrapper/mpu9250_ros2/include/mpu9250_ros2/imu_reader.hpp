#ifndef IMU_READ_HPP
#define IMU_READ_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/bool.hpp>

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
#include <memory>

#include "driver_mpu9250_basic.h"
#include "imu-ros2-wrapper/imu_utils.hpp"
#include "aeirobot_math/math_tool.hpp"

class ImuReadNode: public rclcpp::Node
{
public:
    ImuReadNode();
    ~ImuReadNode();

    bool InitializeImu();
    void ReadRawImuData();
    void PubImuData();
    void TimerCallback();
    void CheckCalibration();
    void CheckMovingState();
    void ApplyOffsets();
    void OffsetInitCallback(const std_msgs::msg::Bool &);

    // Raw IMU 측정값
    std::array<double, 3> accel_measurement, gyro_measurement;
    // offset 반환값
    std::array<double, 3> accl_offset, gyro_offset;
    
    std::vector<std::array<double, 3>> dynamic_accel_buffer;
    std::vector<std::array<double, 3>> dynamic_gyro_buffer;

    std::vector<std::array<double,3>> gyro_buffer;
    std::vector<std::array<double, 3>> accel_buffer;
    std::vector<double> dt_buffer;
private:
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_raw;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_offset;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_offset_done;

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_req_offset_init;

    // 인스턴스 생성
    sensor_msgs::msg::Imu imu_msg_raw;
    sensor_msgs::msg::Imu imu_msg_offset;
    ImuUtils imu_utils_;
    std_msgs::msg::Bool new_offset_init;
    std_msgs::msg::Bool offset_done;

    // Timer
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time base_time;
    rclcpp::Time prev_stamp; // 이전 샘플의 타임스탬프 저장용

    // 클래스 멤버 변수 선언
    bool offset_init_done;
    bool ready_to_publish;
    bool is_gyro_cali_done;
    bool is_accel_cali_done;
    bool is_saved;
    bool is_loaded;
    bool is_stop;
    bool is_first_offset;
    bool is_first_sample;
    int buffer_size;
    int dynamic_buffer_size;  // 예: 500개 이상의 연속 정지 데이터 샘플이 모이면 재보정 실행
    double target_interval_sec;
    double gravity_to_meter_per_second_squared;
    double stop_state_threshold;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
};

#endif // IMU_READ_HPP

