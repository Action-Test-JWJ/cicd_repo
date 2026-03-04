#include "edie_laser/edie_laser_node_vl53l1x.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdieLaserNode>());
  rclcpp::shutdown();
  return 0;
}