#include "edie_camera/edie_camera_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EdieCameraNode>();
  node->ProcessRun(); 
  rclcpp::shutdown();
  return 0;
}
