#include "edie_fsr/edie_fsr_sensor.hpp"
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>

EdieFsrSensorNode::EdieFsrSensorNode()
    : Node("edie_fsr_sensor_node")
{
    sub_fsr_sensor_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr", rclcpp::QoS(1), std::bind(&EdieFsrSensorNode::FsrSensorCallback, this, std::placeholders::_1));
    pub_fsr_skin_state_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/edie8/sensor/skin_state", rclcpp::QoS(1));
    pub_fsr_sensor_oe_filtered_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr_oe_filtered", rclcpp::QoS(1));
    pub_fsr_sensor_interpolated_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr_interpolated", rclcpp::QoS(1));
    // 시각화용
    pub_fsr_repr_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr_repr", rclcpp::QoS(1));
    pub_fsr_weak_lower_step_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr_weak_lower_step", rclcpp::QoS(1));
    pub_fsr_weak_upper_step_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr_weak_upper_step", rclcpp::QoS(1));
    pub_fsr_strong_lower_step_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
        "/edie8/sensor/fsr_strong_lower_step", rclcpp::QoS(1));
    InitializeParams();
    
    // 이전 프레임의 센서 데이터 보관
    prev_fsr_sensor_values_ = std::make_shared<std_msgs::msg::Int16MultiArray>();
    prev_fsr_sensor_values_->data.resize(12, 0);

    // OneEuroFilter 파라미터 초기화
    const double control_time_ = 1.0 / fsr_hz_;   // 50[Hz] -> 0.02[s]
    for (auto& f : oe_filters_) {
        f.Initialize(min_cutoff_, beta_, dcutoff_, control_time_);
    }
    // Interpolation 파라미터 초기화
    rise_alpha_ = control_time_ / (rise_time_sec_ + control_time_);         // α = dt / (τ + dt) 
    fall_alpha_ = control_time_ / (fall_time_sec_ + control_time_);         // α = dt / (τ + dt) 

    // 상태 초기화
    for (int i = 0; i < 12; ++i) {
        fsr_history_[i].clear();
        touch_phase_[i] = TouchPhase::NoTouch;
        last_state_change_time_sec_[i] = 0.0;
    }
}

EdieFsrSensorNode::~EdieFsrSensorNode()
{
}

void EdieFsrSensorNode::InitializeParams()
{
    this->declare_parameter<double>("min_cutoff", min_cutoff_);
    this->declare_parameter<double>("beta", beta_);
    this->declare_parameter<double>("dcutoff", dcutoff_);
    this->declare_parameter<double>("fsr_hz", fsr_hz_);
    this->declare_parameter<double>("rise_time", rise_time_sec_);
    this->declare_parameter<double>("fall_time", fall_time_sec_);
    this->declare_parameter<double>("noise_threshold", noise_threshold_);
    this->declare_parameter<int>("fsr_buffer_size", fsr_buffer_size_);
    this->declare_parameter<double>("strong_to_weak_hold_time_sec", strong_to_weak_hold_time_sec_);
    this->declare_parameter<double>("strong_to_weak_band_ratio",    strong_to_weak_band_ratio_);
    this->declare_parameter<double>("strong_to_weak_min",           strong_to_weak_min_);
    this->declare_parameter<double>("weak_to_strong_hold_time_sec", weak_to_strong_hold_time_sec_);
    this->declare_parameter<double>("weak_to_strong_band_ratio",    weak_to_strong_band_ratio_);
    this->declare_parameter<double>("weak_to_strong_min",           weak_to_strong_min_);
    this->declare_parameter<double>("weak_to_strong_max",           weak_to_strong_max_);

    this->get_parameter("min_cutoff", min_cutoff_);
    this->get_parameter("beta", beta_);
    this->get_parameter("dcutoff", dcutoff_);
    this->get_parameter("fsr_hz", fsr_hz_);
    this->get_parameter("rise_time", rise_time_sec_);
    this->get_parameter("fall_time", fall_time_sec_);
    this->get_parameter("noise_threshold", noise_threshold_);
    this->get_parameter("fsr_buffer_size", fsr_buffer_size_);
    this->get_parameter("strong_to_weak_hold_time_sec", strong_to_weak_hold_time_sec_);
    this->get_parameter("strong_to_weak_band_ratio",    strong_to_weak_band_ratio_);
    this->get_parameter("strong_to_weak_min",           strong_to_weak_min_);
    this->get_parameter("weak_to_strong_hold_time_sec", weak_to_strong_hold_time_sec_);
    this->get_parameter("weak_to_strong_band_ratio",    weak_to_strong_band_ratio_);
    this->get_parameter("weak_to_strong_min",           weak_to_strong_min_);
    this->get_parameter("weak_to_strong_max",           weak_to_strong_max_);

    RCLCPP_INFO(this->get_logger(), "[init] min_cutoff: %f, beta: %f, dcutoff: %f, fsr_hz: %f, rise_time: %f, fall_time: %f, noise_threshold: %f", min_cutoff_, beta_, dcutoff_, fsr_hz_, rise_time_sec_, fall_time_sec_, noise_threshold_);
    RCLCPP_INFO(this->get_logger(), "[init] fsr_buffer_size: %zu, strong_to_weak_hold_time_sec: %.3f, weak_to_strong_hold_time_sec: %.3f", fsr_buffer_size_, strong_to_weak_hold_time_sec_, weak_to_strong_hold_time_sec_);
    RCLCPP_INFO(this->get_logger(), "[init] strong_to_weak_band_ratio: %.3f, strong_to_weak_min: %.3f, weak_to_strong_band_ratio: %.3f, weak_to_strong_min: %.3f, weak_to_strong_max: %.3f", strong_to_weak_band_ratio_, strong_to_weak_min_, weak_to_strong_band_ratio_, weak_to_strong_min_, weak_to_strong_max_);
    RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------------------------");
}

void EdieFsrSensorNode::FsrSensorCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    std_msgs::msg::UInt8MultiArray skin_state_msg;
    skin_state_msg.data.resize(int(msg->data.size()), 0u);
    std_msgs::msg::Int16MultiArray oe_filtered_msg;
    oe_filtered_msg.data.resize(int(msg->data.size()), 0);
    std_msgs::msg::Int16MultiArray interpolated_msg;
    interpolated_msg.data.resize(int(msg->data.size()), 0);
    // 시각화용 
    const int16_t nan_value = std::numeric_limits<int16_t>::quiet_NaN();
    std_msgs::msg::Int16MultiArray repr_msg;
    std_msgs::msg::Int16MultiArray weak_lower_step_msg;
    std_msgs::msg::Int16MultiArray weak_upper_step_msg;
    std_msgs::msg::Int16MultiArray strong_lower_step_msg;
    repr_msg.data.resize(int(msg->data.size()), 0);
    weak_lower_step_msg.data.resize(int(msg->data.size()), 0);
    weak_upper_step_msg.data.resize(int(msg->data.size()), 0);
    strong_lower_step_msg.data.resize(int(msg->data.size()), 0);
    const double now_sec = this->get_clock()->now().seconds();

    for (size_t i = 0; i < msg->data.size(); ++i)
    {        
         // 1) 필터링
        const double raw_value = static_cast<double>(msg->data[i]);
        const double cur_filtered = oe_filters_[i].GetFilteredOutput(raw_value);
        // 🔹 Interpolation 적용
        double &y = interpolated_[i];
        bool &initialized = interpolated_initialized_[i];
        if (!initialized)
        {
            // 첫 샘플은 그냥 현재값으로 초기화
            y = cur_filtered;
            initialized = true;
        }
        else
        {
            if (cur_filtered > y)
            {
                // 값이 올라갈 때: 빠르게 따라가기 (rise_alpha_)
                y = rise_alpha_ * cur_filtered + (1.0 - rise_alpha_) * y;
            }
            else
            {
                // 값이 내려갈 때: 천천히 내려가기 (fall_alpha_)
                y = fall_alpha_ * cur_filtered + (1.0 - fall_alpha_) * y;
            }
        }

        // 2) 버퍼 업데이트 (구 데이터 out, 신 데이터 in)
        auto &buf = fsr_history_[i]; // 길이 fsr_buffer_size_ 의 버퍼
        // buf.push_back(cur_filtered);
        buf.push_back(y);
        if (buf.size() > fsr_buffer_size_) 
        {
            buf.pop_front();
        }

        // 3) 버퍼에서 대표값 계산 (예: min & max 평균)
        double repr = 0; // 버퍼가 비어있다면 그냥 현재값(필터링된 값)
        if (!buf.empty()) 
        {
            double sum = 0;
            for (double v : buf) 
            {
                sum += v;
            }
            repr = sum / buf.size();
        }

        // 4) 현재 값 A와 대표값의 차이
        // const double A = cur_filtered;
        const double A = y;
        const bool is_touch = (A >= noise_threshold_);  // "터치 유무" 판단
        const double delta = A - repr;

        // 대표값 크기에 비례한 margin 계산
        const double s2w_margin = std::max(
            strong_to_weak_min_,                      // 최소 margin
            repr * strong_to_weak_band_ratio_        // 대표값 기준으로 “비슷한 힘”이라고 볼 구간. 대표값의 ±x% 이내면 WeakTouch
        );
        const double weak_upper_band = repr + s2w_margin; // WeakTouch 위 기준선
        const double weak_lower_band = repr - s2w_margin; // WeakTouch 아래 기준선

        // 대표값 크기에 비례한 margin 계산
        const double w2s_margin = std::min(
            weak_to_strong_max_,
            std::max(weak_to_strong_min_, repr * weak_to_strong_band_ratio_) // 최소 margin, 대표값보다 x% 이상 크면 StrongTouch
        );
        // 🔹 지금 시점의 Strong 기준선 (Weak→Strong 진입 기준)
        const double strong_lower_band = repr - w2s_margin; // StrongTouch 아래 기준선

        TouchPhase prev_phase = touch_phase_[i];
        TouchPhase new_phase  = prev_phase;

        // 5) 상태머신 구현
        if (!is_touch) // 비접촉 상태
        {
            // 완전 손 떼면 그냥 NoTouch
            if (prev_phase != TouchPhase::NoTouch) 
            {
                new_phase = TouchPhase::NoTouch;
                last_state_change_time_sec_[i] = now_sec;
            }
        } 
        else { // 접촉 상태
            // RCLCPP_INFO(this->get_logger(), "[FSR Sensor] 접촉 상태");
            // RCLCPP_INFO(this->get_logger(), "[FSR Sensor] channel: %d, delta: %f, std::fabs(repr): %f | band: %f, strong_to_weak_min_: %f, std::fabs(repr) * strong_to_weak_band_ratio_: %f", int(i), delta, std::fabs(repr), band, strong_to_weak_min_, std::fabs(repr) * strong_to_weak_band_ratio_);
            // RCLCPP_INFO(this->get_logger(), "[FSR Sensor] w2s_margin: %f, weak_to_strong_min_: %f, std::fabs(repr) * weak_to_strong_band_ratio_: %f", w2s_margin, weak_to_strong_min_, std::fabs(repr) * weak_to_strong_band_ratio_);
            // RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------------------------");
            switch (prev_phase) 
            {
                // 처음 힘 들어올 때 → StrongTouch 로 시작
                case TouchPhase::NoTouch:
                {
                    new_phase = TouchPhase::StrongTouch;
                    last_state_change_time_sec_[i] = now_sec;
                    break;
                }
                // 대표값 기준으로 “비슷한 힘”이 '일정 시간' 유지되면 WeakTouch
                case TouchPhase::StrongTouch:
                {
                    const bool is_similar_to_repr = std::fabs(delta) <= s2w_margin;
                    if (is_similar_to_repr) 
                    {
                        const double held_time = now_sec - last_state_change_time_sec_[i];
                        if (held_time >= strong_to_weak_hold_time_sec_) 
                        {
                            new_phase = TouchPhase::WeakTouch;
                            last_state_change_time_sec_[i] = now_sec;

                            // 🔹 새 Weak 에피소드 시작 → 카운터 리셋
                            weak_episode_total_samples_[i] = 0;
                            weak_episode_strong_samples_[i]  = 0;
                        }
                    }
                    else // 비슷한 힘이 아니라면 타이머 리셋
                    {
                        last_state_change_time_sec_[i] = now_sec;
                    }
                    break;
                }
                // 바뀐 대표값 기준으로 더 큰 현재값이면 다시 StrongTouch
                case TouchPhase::WeakTouch:
                {                    
                    // 🔹 에피소드 카운터 업데이트
                    weak_episode_total_samples_[i]++;
                    if (delta >= w2s_margin)
                    {
                        weak_episode_strong_samples_[i]++;
                    }

                    double w2s_ratio = 1.0;
                    if (weak_episode_total_samples_[i] > 0)
                    {
                        w2s_ratio = static_cast<double>(weak_episode_strong_samples_[i]) /
                                    static_cast<double>(weak_episode_total_samples_[i]);
                    }

                    // 이번 Weak 에피소드에서 strong 수준 샘플이 10% 이상 섞이기 시작하면 Strong으로 전환
                    constexpr double W2S_CONVERSION_THRESHOLD = 0.1;
                    if (w2s_ratio >= W2S_CONVERSION_THRESHOLD)
                    {
                        RCLCPP_INFO(this->get_logger(), "[W->S] ch:%d w2s_ratio:%f (strong:%d / total:%d) | delta:%f, w2s_margin:%f",
                                    int(i), w2s_ratio, weak_episode_strong_samples_[i], weak_episode_total_samples_[i], delta, w2s_margin)
                        ;
                        new_phase = TouchPhase::StrongTouch;
                        last_state_change_time_sec_[i] = now_sec;
                    }

                    break;
                }
            }
        }

        // 6) 퍼블리시 값 결정
        //    여기서는 일단 상태를 0/1/2로 encode:
        //    0: NoTouch, 1: WeakTouch, 2: StrongTouch
        uint8_t out = 0;
        if (!is_touch) {
            out = 0;  // NoTouch
        } else {
            if (new_phase == TouchPhase::StrongTouch)      out = 2; // StrongTouch
            else if (new_phase == TouchPhase::WeakTouch)   out = 1; // WeakTouch
            else                                       out = 0; // NoTouch
        }
        skin_state_msg.data[i] = out;
        repr_msg.data[i] = static_cast<int16_t>(repr);
        oe_filtered_msg.data[i] = static_cast<int16_t>(cur_filtered);
        interpolated_msg.data[i] = static_cast<int16_t>(y);
        touch_phase_[i] = new_phase;

        // [옵션] 시각화용 step 토픽 publish
        if (!is_touch)
        {
            // 손 뗀 상태 → 전부 NaN (그래프에서 끊기게)
            weak_lower_step_msg.data[i]  = nan_value;
            weak_upper_step_msg.data[i]  = nan_value;
            strong_lower_step_msg.data[i]= nan_value;
        }
        else
        {
            if (new_phase == TouchPhase::WeakTouch)
            {
                // Weak 구간: Weak band + Weak 기준 repr 만 보여줌
                weak_lower_step_msg.data[i]  = static_cast<int16_t>(weak_lower_band);
                weak_upper_step_msg.data[i]  = static_cast<int16_t>(weak_upper_band);

                // Strong band는 이 구간에서는 끊어둔다
                strong_lower_step_msg.data[i]= nan_value;
            }
            else if (new_phase == TouchPhase::StrongTouch)
            {
                // Strong 구간: Strong 기준 repr + band 표시
                strong_lower_step_msg.data[i] = static_cast<int16_t>(strong_lower_band);

                // Weak band는 이 구간에서는 끊어둔다
                weak_lower_step_msg.data[i]  = nan_value;
                weak_upper_step_msg.data[i]  = nan_value;
            }
            else
            {
                // 이론상 NoTouch지만 is_touch == true 인 특이 상황 방어 (혹시 모를 경우용)
                weak_lower_step_msg.data[i]  = nan_value;
                weak_upper_step_msg.data[i]  = nan_value;
                strong_lower_step_msg.data[i]= nan_value;
            }
        }
    }

    PubFsrSkinState(skin_state_msg);
    PubFsrOEFiltered(oe_filtered_msg);
    PubFsrInterpolated(interpolated_msg);
    // 시각화용 step 토픽 publish
    pub_fsr_repr_->publish(repr_msg);
    pub_fsr_weak_lower_step_->publish(weak_lower_step_msg);
    pub_fsr_weak_upper_step_->publish(weak_upper_step_msg);
    pub_fsr_strong_lower_step_->publish(strong_lower_step_msg);
    *prev_fsr_sensor_values_ = *msg;
}

void EdieFsrSensorNode::PubFsrSkinState(std_msgs::msg::UInt8MultiArray msg)
{
    pub_fsr_skin_state_->publish(msg);
}

void EdieFsrSensorNode::PubFsrOEFiltered(std_msgs::msg::Int16MultiArray msg)
{
    pub_fsr_sensor_oe_filtered_->publish(msg);
}

void EdieFsrSensorNode::PubFsrInterpolated(std_msgs::msg::Int16MultiArray msg)
{
    pub_fsr_sensor_interpolated_->publish(msg);
}