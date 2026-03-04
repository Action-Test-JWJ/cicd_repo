#include "edie_laser/edie_laser_node_vl53l0x.hpp"

EdieLaserNode::EdieLaserNode() : Node("edie_laser_vl53l0x_node"),
                                laser_sensor_l("/dev/i2c-11"),
                                laser_sensor_r("/dev/i2c-13"),
                                kalman_filter_l(10.0, 1.0, 1.0, 0.0),
                                kalman_filter_r(10.0, 1.0, 1.0, 0.0)
{
    pub_laser_value_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/edie8/sensor/front/laser", 10);
    
    laser_sensor_l.SensorSetup();
    laser_sensor_l.SensorCalibration();
    laser_sensor_r.SensorSetup();
    laser_sensor_r.SensorCalibration();
    ROS_GREEN_STREAM("Laser Sensor(VL53L0X) Node Init Successfully.");
}

EdieLaserNode::~EdieLaserNode()
{
}

void EdieLaserNode::PubLaserVal()
{
    int16_t raw_data_l = laser_sensor_l.RangingData();
    int16_t raw_data_r = laser_sensor_r.RangingData();
    kalman_filter_l.Update(raw_data_l);
    kalman_filter_r.Update(raw_data_r);
    auto msg = std_msgs::msg::Int16MultiArray();
    msg.data.push_back(kalman_filter_l.k_f_data);
    msg.data.push_back(kalman_filter_r.k_f_data);
    pub_laser_value_->publish(msg);
}