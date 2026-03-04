#include "edie_motion/edie_motion_handler.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EdieMotionHandler>();
  size_t thread_count = 2;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_count);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}