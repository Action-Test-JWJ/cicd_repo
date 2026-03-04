#include "edie_fsr/edie_fsr_sensor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieFsrSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}