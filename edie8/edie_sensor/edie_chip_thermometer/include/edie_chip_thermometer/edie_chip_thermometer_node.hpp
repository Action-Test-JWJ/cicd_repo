#ifndef EDIE_CHIP_THERMOMETER_HPP
#define EDIE_CHIP_THERMOMETER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <fstream>
#include <string>

class EdieChipThermometerNode : public rclcpp::Node
{
public:
    EdieChipThermometerNode();
    ~EdieChipThermometerNode();

    float readChipTemperature();
    void pubChipTemperature();

private:

    void timerCallback();
    
    std::string temperature_path_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_temperature_;
};

#endif // EDIE_CHIP_THERMOMETER_HPP