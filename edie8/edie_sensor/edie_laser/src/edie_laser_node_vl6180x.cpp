#include "edie_laser/edie_laser_node_vl6180x.hpp"

EdieLaserNode::EdieLaserNode() : Node("edie_laser_vl6180x_node"),  
                                laser_sensor_l("/dev/i2c-10"),
                                laser_sensor_r("/dev/i2c-12"),
                                kalman_filter_l(10.0, 1.0, 1.0, 0.0),
                                kalman_filter_r(10.0, 1.0, 1.0, 0.0)
{
    pub_laser_value_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/edie8/sensor/bottom/laser_values", 10);
    ROS_GREEN_STREAM("Laser Sensor(VL6180X) Node Init Successfully.");

    sensor_handle_l = laser_sensor_l.vl6180_initialise();
    sensor_handle_r = laser_sensor_r.vl6180_initialise();
}

EdieLaserNode::~EdieLaserNode()
{
}

void EdieLaserNode::PubLaserVal()
{
    int16_t raw_data_l, raw_data_r;
    std_msgs::msg::Int16MultiArray dist_msg;

    raw_data_l = laser_sensor_l.RangingData(sensor_handle_l);
    raw_data_r = laser_sensor_r.RangingData(sensor_handle_r);

    kalman_filter_l.Update(raw_data_l);
    kalman_filter_r.Update(raw_data_r);

    dist_msg.data.push_back(kalman_filter_l.k_f_data);
    dist_msg.data.push_back(kalman_filter_r.k_f_data);

    pub_laser_value_->publish(dist_msg);
}