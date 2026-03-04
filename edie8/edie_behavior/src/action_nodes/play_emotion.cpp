#include "edie_behavior/action_nodes.hpp"
    
namespace aeirobot
{
  PlayEmotion::PlayEmotion(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  BT::PortsList PlayEmotion::providedPorts()
  {
    return{ BT::InputPort<int>("index", "Emotion action index to play") };
  }

  BT::NodeStatus PlayEmotion::onStart()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    // [수정] 현재 대기 단계의 감정을 이미 표현했다면, 즉시 SUCCESS를 반환하여 반복을 막음
    if (edie->infront_waiting_stage_ != 0 && edie->infront_waiting_stage_ == edie->emotion_played_for_stage_) {
      return BT::NodeStatus::SUCCESS;
    }

    // If robot is busy, wait.
    if (!edie->is_action_ready) {
      return BT::NodeStatus::RUNNING;
    }

    auto index_input = getInput<int>("index");
    if (!index_input) {
      return BT::NodeStatus::FAILURE;
    }

    edie->PubEmotion(static_cast<uint8_t>(index_input.value()));
    
    // [수정] 감정을 표현했음을 현재 단계에 기록
    if (edie->infront_waiting_stage_ != 0) {
      edie->emotion_played_for_stage_ = edie->infront_waiting_stage_;
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus PlayEmotion::onRunning()
  {
    auto edie = Edie::GetInstance();
    if (!edie) return BT::NodeStatus::FAILURE;

    return (edie->is_action_ready) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void PlayEmotion::onHalted()
  {
  }

} // namespace aeirobot
