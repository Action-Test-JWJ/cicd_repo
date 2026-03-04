#ifndef EDIE_LASER_VL53L0X_NODE
#define EDIE_LASER_VL53L0X_NODE

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"

#include "LaserSensor.hpp"
#include "kalman_filter.hpp"

using namespace std::chrono_literals;

class EdieLaserNode : public rclcpp::Node
{
  public:
    EdieLaserNode();
    ~EdieLaserNode();
    void PubLaserVal();

  private:
    LaserSensor laser_sensor_l;
    LaserSensor laser_sensor_r;
    KalmanFilter kalman_filter_l;
    KalmanFilter kalman_filter_r;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_laser_value_;
};

#endif // EDIE_LASER_VL53L0X_NODE