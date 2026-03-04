#ifndef IMU_JITTER_MONITOR_HPP
#define IMU_JITTER_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <numeric>
#include <time.h>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <sched.h>        // sched_setscheduler, SCHED_FIFO
#include <pthread.h>      // pthread_setschedparam
#include <cerrno>         // errno
#include <cstring>        // strerror
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class ImuJitterMonitor: public rclcpp::Node
{
public:
    ImuJitterMonitor();
    ~ImuJitterMonitor();

    void ImuJitterCallback(const sensor_msgs::msg::Imu &);
    void ReportJitter();
    void ShowTimeSeriesPlot(const std::vector<double>& v);
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    // Timer
    rclcpp::TimerBase::SharedPtr timer;
    
    // 클래스 멤버 변수 선언
    struct timespec prev_time;
    bool first_time;
    bool is_plotted;
    std::vector<double> buffer_delta_t;
    static constexpr size_t report_threshold = 10000;
};

#endif // IMU_JITTER_MONITOR_HPP