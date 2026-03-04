#include "edie_chip_thermometer/edie_chip_thermometer_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieChipThermometerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}