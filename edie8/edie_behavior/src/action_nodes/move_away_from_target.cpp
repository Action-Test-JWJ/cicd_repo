#include "edie_behavior/action_nodes.hpp"

namespace aeirobot
{
  MoveAwayFromTarget::MoveAwayFromTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  BT::PortsList MoveAwayFromTarget::providedPorts()
  {
    // 포트에 turn_velocity와 forward_velocity 추가
    return { 
      BT::InputPort<double>("turn_duration", 2.0, "Duration of the turning maneuver"),
      BT::InputPort<double>("forward_duration", 3.0, "Duration of the forward movement"),
      BT::InputPort<double>("turn_velocity", 1.5, "Angular velocity for turning"),
      BT::InputPort<double>("forward_velocity", 0.25, "Linear velocity for moving forward")
    };
  }

  BT::NodeStatus MoveAwayFromTarget::onStart()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    // 포트에서 값 읽어오기
    if (!getInput<double>("turn_duration", turn_duration_sec_)) return BT::NodeStatus::FAILURE;
    if (!getInput<double>("forward_duration", forward_duration_sec_)) return BT::NodeStatus::FAILURE;
    // 새로 추가된 포트에서 값 읽어오기
    double desired_turn_velocity;
    double desired_forward_velocity;
    if (!getInput<double>("turn_velocity", desired_turn_velocity)) return BT::NodeStatus::FAILURE;
    if (!getInput<double>("forward_velocity", desired_forward_velocity)) return BT::NodeStatus::FAILURE;

    forward_velocity_ = desired_forward_velocity; // 멤버 변수에 저장
    
    // --- 회전 방향 결정 ---
    // GetTwistForStaring()은 사람을 중앙에 맞추려고 회전함.
    // 이 회전 방향의 반대로 돌아야 등을 돌리게 됨.
    auto initial_twist = edie->GetTwistForStaring();
    double turn_direction = -1.0; // 기본적으로 왼쪽으로 회전
    if (initial_twist.angular.z > 0) { // 로봇이 왼쪽으로 돌려고 하면 (사람이 오른쪽에 있으면)
      turn_direction = 1.0; // 오른쪽으로 돌아 등을 보이게 함
    }
    
    turn_velocity_ = turn_direction * desired_turn_velocity; // 읽어온 값으로 회전 속도 설정

    // 첫 단계를 '회전'으로 설정
    current_phase_ = Phase::TURNING;
    start_time_ = edie->ros_manager->now();
    RCLCPP_INFO(edie->ros_manager->get_logger(), "[MoveAway] Phase 1: Turning back for %.1f seconds.", turn_duration_sec_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus MoveAwayFromTarget::onRunning()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    if (current_phase_ == Phase::TURNING)
    {
      // 회전 시간이 다 되었는지 확인
      if ((edie->ros_manager->now() - start_time_).seconds() >= turn_duration_sec_) {
        // 회전 완료, '전진' 단계로 전환
        current_phase_ = Phase::MOVING_FORWARD;
        start_time_ = edie->ros_manager->now(); // 타이머 리셋
        RCLCPP_INFO(edie->ros_manager->get_logger(), "[MoveAway] Phase 2: Moving forward for %.1f seconds.", forward_duration_sec_);
      } else {
        // 계속 회전
        edie->PubRemote(0.0, turn_velocity_);
      }
    }
    
    if (current_phase_ == Phase::MOVING_FORWARD)
    {
      // 전진 시간이 다 되었는지 확인
      if ((edie->ros_manager->now() - start_time_).seconds() >= forward_duration_sec_) {
        RCLCPP_INFO(edie->ros_manager->get_logger(), "[MoveAway] Maneuver complete.");
        edie->PubRemote(0.0, 0.0); // 로봇 정지
        return BT::NodeStatus::SUCCESS;
      } else {
        // 계속 전진 (하드코딩된 값 대신 멤버 변수 사용)
        edie->PubRemote(forward_velocity_, 0.0);
      }
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void MoveAwayFromTarget::onHalted()
  {
    auto edie = Edie::GetInstance();
    if (edie) {
      edie->PubRemote(0.0, 0.0); // 중단 시 로봇 정지
      RCLCPP_WARN(edie->ros_manager->get_logger(), "[MoveAway] Halted. Stopping robot.");
    }
  }

} // namespace aeirobot
