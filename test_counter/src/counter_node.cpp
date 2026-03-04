#include <rclcpp/rclcpp.hpp>

class CounterNode : public rclcpp::Node
{
public:
  CounterNode() : Node("counter_node"), count_(0)
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CounterNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    count_++;
    RCLCPP_INFO(this->get_logger(), "%d", count_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CounterNode>());
  rclcpp::shutdown();
  return 0;
}
