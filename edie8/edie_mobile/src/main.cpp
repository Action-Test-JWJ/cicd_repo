#include "edie_mobile/edie_mobile_main.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EdieMobileNode>();
  rclcpp::WallRate loop_rate(100); // 10Hz

  while (rclcpp::ok())
  {
    node->RunProcess();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  } 

  rclcpp::shutdown();
  return 0;
}