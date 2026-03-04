#ifndef ICM20948_READ_HPP
#define ICM20948_READ_HPP

#include <Python.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/bool.hpp>
#include <time.h>
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
#include <cmath>

#include "imu-ros2-wrapper/imu_utils.hpp"
#include "aeirobot_math/math_tool.hpp"

// I²C 주소 및 풀스케일 상수
static constexpr int I2C_ADDR = 0x68;
static constexpr int GPM4      = 0x01;  // qwiic_icm20948.gpm4
static constexpr int DPS500    = 0x01;  // qwiic_icm20948.dps500

class Icm20948ReadNode: public rclcpp::Node
{
public:
    Icm20948ReadNode();
    ~Icm20948ReadNode();

    /**
    * @brief 파이썬 임베딩 환경 구축, ICM-20948 Python 드라이버 초기화 및 I2C 통신을 통한 센서 장치 인스턴스 설정
    * SparkFun 제조사의 외부 코드 참조
    */
    void InitializePython();

    /**
    * @brief Python 드라이버 연결 체크 및 센서 장치 초기화 함수 호출
    * SparkFun 제조사의 외부 코드 참조
    */
    bool InitializeImu();
    
    /**
    * @brief ICM-20948 센서 장치로부터 원시 데이터 읽기
    */
    void ReadRawImuData();

    /**
    * @brief ROS2 통신 이용해 센서 데이터 발행
    * Raw: 디버깅용. Offset: 실제 사용 데이터.
    */    
    void PubImuData();

    /**
    * @brief 센서 데이터 읽기부터 데이터 발행까지의 타이머 콜백 함수
    * 주기: 500[Hz](= 2ms).
    */
    void TimerCallback();

    /**
    * @brief 모노토닉 타임을 실시간 시간으로 변환하기 위한 오프셋을 1회 캘리브레이션(계산)하는 함수
    * 이후 IMU 읽기 시점(모노토닉 기준)을 실제 절대 시간(ns)으로 정확히 변환하기 위한 오프셋을 구하는 함수
    */
    void CalibrateTimeOffset();

    /**
    * @brief 캘리브레이션 여부에 따라 IMU 오프셋(바이어스)을 전 lifecycle 동안 관리하는 핵심 함수
    * 초기에는 정적 가속도/자이로 데이터를 모아 캘리브레이션을 수행하고 그 결과(오프셋·회전 행렬)를 파일로 저장/로드.
    * 이후에는 충분히 정지해 있을 때 최근 가속도 데이터를 이용해 가속도 바이어스를 다시 추정한 뒤
    * 최종 보정값이 실제 퍼블리시되는 IMU 메시지에 적용. <- 로봇 다리 들리고 내림으로 인한 가속도 축 변화 반영.
    */
    void CheckCalibration();

    /**
    * @brief 보정된 IMU 데이터를 계산 및 이를 퍼블리시용 IMU 메시지의 각 필드에 채워 넣는 함수
    */
    void ApplyOffsets();

    // Raw IMU 측정값
    std::array<double, 3> accel_measurement, gyro_measurement;
    // offset 반환값
    std::array<double, 3> accl_offset, gyro_offset;

    // 초기 보정을 위한 데이터 수집용 버퍼
    std::vector<std::array<double,3>> gyro_buffer;
    std::vector<std::array<double, 3>> accel_buffer;
private:
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_raw;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_offset;

    // 인스턴스 생성
    /// 메시지 클래스 객체
    sensor_msgs::msg::Imu imu_msg_raw;
    sensor_msgs::msg::Imu imu_msg_offset;
    /// 유틸리티 클래스 객체
    ImuUtils imu_utils_;
    /// Python 임베딩 객체
    PyObject* pI2cMod_;      // qwiic_i2c 모듈
    PyObject * pModule_;    // qwiic_icm20948.py 모듈
    PyObject * pInstance_;  // QwiicIcm20948 클래스 인스턴스
    PyObject * pDriver_;    // qwiic_i2c 드라이버
    PyObject * pGetDriver_; // get_i2c_driver 함수
    
    /// Timer
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time base_time;
    rclcpp::Time prev_stamp; // 이전 샘플의 타임스탬프 저장용

    /// 커널 타임스탬프
    struct timespec ts_start; // 읽기 시작 시점
    struct timespec ts_end; // 읽기 종료 시점

    using ns = std::chrono::nanoseconds;
    using mono_clock = std::chrono::steady_clock;
    using sys_clock  = std::chrono::system_clock;
    std::chrono::nanoseconds steady_to_system_offset_{};
    
    /// 클래스 멤버 변수 선언
    bool is_gyro_cali_done;
    bool is_accel_cali_done;
    bool is_saved;
    bool is_loaded;
    bool is_stop;
    int buffer_size;                            // 초기 보정을 위한 데이터 수집용 버퍼 크기
    double gravity_to_meter_per_second_squared; // [g] -> [m/s^2] 변환 계수
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
};

#endif // ICM20948_READ_HPP

