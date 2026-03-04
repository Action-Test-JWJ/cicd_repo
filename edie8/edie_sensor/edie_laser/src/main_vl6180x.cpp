#include "edie_laser/edie_laser_node_vl6180x.hpp"
#include <csignal>

volatile sig_atomic_t exitFlag = 0;

void ExitSignalHandler(int) 
{
  exitFlag = 1;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EdieLaserNode>();
  // rclcpp::Rate loop_rate(10); // 10Hz
  rclcpp::WallRate loop_rate(20); // 10Hz

  if (exitFlag) { return 0; }

  signal(SIGINT, ExitSignalHandler);

  while (rclcpp::ok() && !exitFlag) 
  {
    node->PubLaserVal();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  } 

  rclcpp::shutdown();
  return 0;
}