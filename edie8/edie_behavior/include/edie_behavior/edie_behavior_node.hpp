#ifndef EDIE_BEHAVIOR_NODE_HPP
#define EDIE_BEHAVIOR_NODE_HPP

#include <behaviortree_cpp/bt_factory.h>
#include <edie_msgs/srv/check_ready.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "edie_behavior/register_nodes.hpp"
// #include "edie_behavior/singleton.hpp"

using namespace std;

class EdieBehaviorTree : public rclcpp::Node
{
public:
  EdieBehaviorTree();

  void InitBehaviorTree();

  void SetState(int input);
  void SetAction(int input);

  void PubState(int input);
  void PubAction(int input);

  void SubFsrRaw(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void SubFsrState(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
  void SubEmotionDone(const std_msgs::msg::Bool::SharedPtr msg);

private:
  void BehaviorTreeUpdate();
  void EdieUpdate();

  rclcpp::TimerBase::SharedPtr behavior_tree_timer;
  rclcpp::TimerBase::SharedPtr action_timer;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_state;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_action;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_fsr_raw;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr sub_fsr_state;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emotion_done;

  rclcpp::Client<edie_msgs::srv::CheckReady>::SharedPtr client_check_ready;
  void ResponseCheckReady(rclcpp::Client<edie_msgs::srv::CheckReady>::SharedFuture future);

  BT::Tree tree;
  bool tree_initialized = false;

  bool responsed;

public:
  int cur_state = -1;
  int cur_action = -1;
  int16_t fsr_raw[5];  // fure fsr sensor data
  int8_t fsr_state[5]; // 0 : no touch, 1 : touched, 2 : hit
  // bool emotion_done;
  bool is_action_ready;
};

#endif // EDIE_BEHAVIOR_NODE_HPP