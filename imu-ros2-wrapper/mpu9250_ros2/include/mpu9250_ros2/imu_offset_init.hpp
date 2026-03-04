#ifndef IMU_OFFSET_INIT_HPP
#define IMU_OFFSET_INIT_HPP
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class ImuOffsetInit : public rclcpp::Node
{
public:
    ImuOffsetInit();
    ~ImuOffsetInit();

    void PubOffetInit();
    void TimerCallback();

private:
    // Publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_req_offset_init;

    // 인스턴스 생성
    std_msgs::msg::Bool new_offset_init;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
    rclcpp::TimerBase::SharedPtr timer;
    rcl_interfaces::msg::SetParametersResult ParamChangeCallback(const std::vector<rclcpp::Parameter> &parameters);

    // 클래스 멤버 변수 선언
    bool current_bool;
};

#endif // IMU_OFFSET_INIT_HPP