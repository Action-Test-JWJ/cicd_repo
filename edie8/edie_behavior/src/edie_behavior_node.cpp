// #include "edie_behavior/edie_behavior_node.hpp"
// #include "edie_behavior/singleton.hpp"

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <aeirobot_toolbox/ros_manager.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "edie_behavior/joy_manager.hpp"

#include "edie_behavior/edie.hpp"

using namespace std;
using namespace chrono_literals;

using namespace aeirobot;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Node 생성하여 parameter 받기
  auto node = rclcpp::Node::make_shared("edie_behavior_node");
  
  auto edie = Edie::GetInstance();
  edie->Init(node);
  edie->Run();

  rclcpp::shutdown();

  return 0;
}
