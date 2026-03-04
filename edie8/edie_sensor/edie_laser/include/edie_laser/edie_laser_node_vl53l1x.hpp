#ifndef EDIE_LASER_VL53L1X_NODE
#define EDIE_LASER_VL53L1X_NODE

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "vl53l1x.h"  // Library for the range sensor

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "kalman_filter.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EdieLaserNode : public rclcpp::Node
{
  public:
    EdieLaserNode();
    ~EdieLaserNode();
    void PubLaserVal();
    void timer_callback();

  private:
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_laser_value_;
    Vl53l1X laser_sensor_l;
    Vl53l1X laser_sensor_r;
    KalmanFilter kalman_filter_l;
    KalmanFilter kalman_filter_r;
    unsigned int timeout_;
    unsigned int timing_budget_;
    double freq_;
};

#endif // EDIE_LASER_VL53L1X_NODE