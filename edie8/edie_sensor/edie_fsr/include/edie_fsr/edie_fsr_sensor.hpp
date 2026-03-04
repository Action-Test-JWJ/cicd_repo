#ifndef EDIE_FSR_SENSOR_HPP
#define EDIE_FSR_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <string>
#include <algorithm>
#include <fstream>
#include <vector>
#include <filesystem>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <array>
#include <deque> 
#include <cmath>
#include <limits>
#include "aeirobot_toolbox/aeirobot_signal_processing.h"

enum class TouchPhase : uint8_t {
    NoTouch   = 0,
    WeakTouch = 1,
    StrongTouch = 2,
};

class EdieFsrSensorNode : public rclcpp::Node
{
public:
    EdieFsrSensorNode();
    ~EdieFsrSensorNode();

private:
    void InitializeParams();
    // Publisher
    void PubFsrSkinState(std_msgs::msg::UInt8MultiArray msg);
    void PubFsrOEFiltered(std_msgs::msg::Int16MultiArray msg);
    void PubFsrInterpolated(std_msgs::msg::Int16MultiArray msg);
    // Subscriber
    void FsrSensorCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    
    // ROS2 인스턴스
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_fsr_sensor_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_fsr_skin_state_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_fsr_sensor_oe_filtered_;  // I) raw, prev-filtered
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_fsr_sensor_interpolated_; // I) current-filtered, prev-filtered
    // 시각화용
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_fsr_repr_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_fsr_weak_lower_step_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_fsr_weak_upper_step_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_fsr_strong_lower_step_;

    // OneEuroFilter 인스턴스
    std::array<aeirobot::OneEuroFilter, 12> oe_filters_; // 12개 FSR 각각에 대한 인스턴스
    
    // 이전 프레임의 센서 데이터 보관
    std::shared_ptr<std_msgs::msg::Int16MultiArray> prev_fsr_sensor_values_;

    // OneEuroFilter 필터링 관련 파라미터
    double fsr_hz_;
    double min_cutoff_;
    double beta_;
    double dcutoff_;

    // 상태머신 관련 파라미터
    std::array<std::deque<double>, 12> fsr_history_;      // 각 채널별 히스토리 버퍼 (필터링된 힘 값 저장)
    std::array<TouchPhase, 12> touch_phase_{};            // 각 채널별 터치 상태
    std::array<double, 12> last_state_change_time_sec_{}; // 각 채널별 상태가 마지막으로 바뀐 시각 (초)
    std::size_t fsr_buffer_size_;                         // 버퍼 최대 길이. 예: 50샘플 ≒ 1초(50Hz)
    double noise_threshold_;
    // StrongTouch -> WeakTouch
    double strong_to_weak_hold_time_sec_;                 // StrongTouch → WeakTouch 로 내려가는 데 필요한 유지 시간
    double strong_to_weak_band_ratio_;                    // [StrongTouch -> WeakTouch] 대표값의 ±x*100 %
    double strong_to_weak_min_;                           // [StrongTouch -> WeakTouch] 최소 margin
    // WeakTouch -> StrongTouch
    double weak_to_strong_hold_time_sec_;                 // WeakTouch → StrongTouch 로 올라가는 데 필요한 유지 시간(짧게, ex. 0.02~0.05초)
    double weak_to_strong_band_ratio_;                    // [WeakTouch -> StrongTouch] 대표값의 ±x*100 %
    double weak_to_strong_min_;                           // [WeakTouch -> StrongTouch] 최소 margin
    double weak_to_strong_max_;                           // [WeakTouch -> StrongTouch] 최대 margin    
    std::array<int, 12> weak_episode_total_samples_{};    // WeakTouch 구간에서의 샘플 개수
    std::array<int, 12> weak_episode_strong_samples_{};    // WeakTouch 구간에서의 Weak 샘플 개수

    // Interpolation 관련 파라미터
    // 현재 센서 값이 올라갈 때 / 내려갈 때의 "보간 속도"를 정하는 시간 [초]
    double rise_time_sec_;   // 값이 커질 때(위로 갈 때) 반응 속도
    double fall_time_sec_;   // 값이 작아질 때(아래로 갈 때) 반응 속도

    // 위 시간과 fsr_hz_로부터 계산된 보간 비율
    double rise_alpha_;      // 올라갈 때 보간 비율
    double fall_alpha_;      // 내려갈 때 보간 비율
    std::array<double, 12> interpolated_{};      // 각 채널별 보간 결과 Y[k]
    std::array<bool,   12> interpolated_initialized_{};   // 각 채널별 초기화 여부

    // [시각화용 파라미터] Weak/Strong 구간별 스냅샷 값 (전환 시점 기준)
    std::array<int16_t, 12> weak_lower_ref_{};     // Weak band lower (repr - s2w_margin)
    std::array<int16_t, 12> weak_upper_ref_{};     // Weak band upper (repr + s2w_margin)
    std::array<int16_t, 12> strong_lower_ref_{};   // Strong band "하한선" (repr - w2s_margin)
};

#endif // EDIE_FSR_SENSOR_HPP