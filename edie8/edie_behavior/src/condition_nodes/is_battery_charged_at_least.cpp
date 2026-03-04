#include "edie_behavior/condition_nodes.hpp"
#include "edie_behavior/singleton.hpp"

using namespace std;

namespace aeirobot
{
  
  BT::PortsList IsBatteryChargedAtLeast::providedPorts()
  {
    return {
      BT::InputPort<double>("percent", 0.2, "배터리 최소 잔량 (0.0 ~ 1.0)")
    };
  }
  
  BT::NodeStatus IsBatteryChargedAtLeast::tick()
  {
    auto edie = Edie::GetInstance();
    
    // 포트에서 임계값 읽기
    double min_percent = 0.2;
    getInput("percent", min_percent);
    
    // 배터리 잔량 확인
    if (edie->battery_percentage >= min_percent) {
      return BT::NodeStatus::SUCCESS;  // 충분한 배터리
    } else {
      return BT::NodeStatus::FAILURE;  // 배터리 부족
    }
  }

} // namespace aeirobot