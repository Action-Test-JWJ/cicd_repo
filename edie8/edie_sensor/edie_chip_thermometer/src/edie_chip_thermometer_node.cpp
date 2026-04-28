#include "edie_chip_thermometer/edie_chip_thermometer_node.hpp"


EdieChipThermometerNode::EdieChipThermometerNode()
    : Node("edie_chip_thermometer_node")
    , temperature_path_("/sys/class/hwmon/hwmon0/temp1_input")
{
    pub_temperature_ = this->create_publisher<std_msgs::msg::Float32>("/edie8/sensor/cpu_temperature", 1);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&EdieChipThermometerNode::timerCallback, this));
}

EdieChipThermometerNode::~EdieChipThermometerNode()
{
}


float EdieChipThermometerNode::readChipTemperature(int precision)
{
    (void)precision;
    std::ifstream temp_file(temperature_path_);
    float temperature = -1.0;

    if (temp_file.is_open()) {
        std::string line;
        std::getline(temp_file, line);
        temperature = std::stof(line) / 1000.0;
        temp_file.close();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open temperature file: %s", temperature_path_.c_str());
    }

    return temperature;
}


void EdieChipThermometerNode::pubChipTemperature()
{
    float temperature = readChipTemperature(2);

    if (temperature >= 0.0)
    {
        std_msgs::msg::Float32 msg;
        msg.data = temperature;

        pub_temperature_->publish(msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read temperature");
    }
}

void EdieChipThermometerNode::timerCallback()
{
    RCLCPP_DEBUG(this->get_logger(), "tick: temp=%f", readChipTemperature());
    pubChipTemperature();
}
