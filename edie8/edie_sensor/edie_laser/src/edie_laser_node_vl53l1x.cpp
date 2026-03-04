#include "edie_laser/edie_laser_node_vl53l1x.hpp"

EdieLaserNode::EdieLaserNode() : Node("edie_laser_vl53l1x_node"),
                                laser_sensor_l("/dev/i2c-9"),
                                laser_sensor_r("/dev/i2c-11"),
                                kalman_filter_l(10.0, 1.0, 1.0, 0.0),
                                kalman_filter_r(10.0, 1.0, 1.0, 0.0)
{
    // Declare and get Parameters
    timeout_ = this->declare_parameter("timeout", 500);
    timing_budget_ = this->declare_parameter("timing_budget", 50000);
    freq_ = this->declare_parameter("frequency", 25.0);

    // Setup the publisher
    pub_laser_value_ = this->create_publisher<sensor_msgs::msg::Range>("/edie8/sensor/front/laser", 5);
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)), 
    std::bind(&EdieLaserNode::timer_callback, this));

    // Set a 500ms timeout on the sensor. (Stop waiting and respond with an error)
    laser_sensor_l.setTimeout(timeout_);
    laser_sensor_r.setTimeout(timeout_);

    if (!laser_sensor_l.init() || !laser_sensor_r.init()) 
    {
        RCLCPP_ERROR(this->get_logger(), "Sensor offline!");
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    laser_sensor_l.setDistanceMode(Vl53l1X::Long);
    laser_sensor_l.setMeasurementTimingBudget(timing_budget_);
    laser_sensor_r.setDistanceMode(Vl53l1X::Long);
    laser_sensor_r.setMeasurementTimingBudget(timing_budget_);

    // Start continuous readings at a rate of one measurement every 100 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    laser_sensor_l.startContinuous( static_cast<int>(1000.0 / freq_) );  // Hardcode testcase 100
    laser_sensor_r.startContinuous( static_cast<int>(1000.0 / freq_) );  // Hardcode testcase 100
}

EdieLaserNode::~EdieLaserNode()
{
    laser_sensor_l.stopContinuous();
    laser_sensor_r.stopContinuous();
}

void EdieLaserNode::PubLaserVal()
{
    int16_t raw_data_l = laser_sensor_l.read_range();
    if(laser_sensor_l.timeoutOccurred()) {
        RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
        raw_data_l = 0;
    }

    int16_t raw_data_r = laser_sensor_r.read_range();
    if(laser_sensor_r.timeoutOccurred()) {
        RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
        raw_data_r = 0;
    }

    kalman_filter_l.Update(raw_data_l);
    kalman_filter_r.Update(raw_data_r);

    rclcpp::Time now = this->get_clock()->now();
    auto message = sensor_msgs::msg::Range();
    message.header.frame_id = "vl53l1x";
    message.header.stamp = now;
    message.radiation_type = sensor_msgs::msg::Range::INFRARED;
    message.field_of_view = 0.47;              // Typically 27 degrees or 0,471239 radians
    message.min_range = 0.02;                  // 140 mm.  (It is actully much less, but this makes sense in the context
    message.max_range = 3.00;                  // 3.6 m. in the dark, down to 73cm in bright light

    message.range = (float) kalman_filter_l.k_f_data / 1000.0; // range in meters

    if((message.range >= message.min_range) && (message.range <= message.max_range)) {
        pub_laser_value_->publish(message);
    }
}

void EdieLaserNode::timer_callback()
{
    PubLaserVal();
}