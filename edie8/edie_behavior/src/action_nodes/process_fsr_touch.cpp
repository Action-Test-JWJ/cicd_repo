#include "edie_behavior/action_nodes.hpp"
#include <algorithm>
#include <vector>
#include <map>

namespace aeirobot
{
  ProcessFsrTouch::ProcessFsrTouch(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  BT::PortsList ProcessFsrTouch::providedPorts()
  {
    return { 
        BT::InputPort<int>("threshold", 2000, "FSR value threshold for single touch reaction")
    };
  }

  BT::NodeStatus ProcessFsrTouch::tick()
  {
    auto edie = Edie::GetInstance();
    if (!edie) { return BT::NodeStatus::FAILURE; }

    const auto& skin_state_values = edie->skin_state_values;
    if (skin_state_values.size() != 12) {
        return BT::NodeStatus::FAILURE;
    }
    // If another emotion is playing, we can't start a new one.
    if (!edie->is_action_ready) {
        return BT::NodeStatus::FAILURE;
    }

    // skin_state_values 벡터에서 값이 2인 센서를 찾는다.
    // 여러 개가 2인 경우, 가장 앞(인덱스가 가장 작은) 센서를 사용한다.
    auto tgt_itr = std::find(skin_state_values.begin(), skin_state_values.end(), 2); // StrongTouch(2) 찾기
    if (tgt_itr == skin_state_values.end()) {
        return BT::NodeStatus::FAILURE; // StrongTouch(2) 가 하나도 없으면 실패
    }
    int tgt_idx = std::distance(skin_state_values.begin(), tgt_itr);

    TouchedArea current_area = TouchedArea::NONE;
    if (tgt_idx >= 4 && tgt_idx <= 6) { // 정수리
        current_area = TouchedArea::HEAD;
    } else if (tgt_idx >= 0 && tgt_idx <= 3) { // 오른쪽 궁뎅이
        current_area = TouchedArea::RIGHT_BUTT;
    } else if (tgt_idx >= 7 && tgt_idx <= 11) { // 쓰담쓰담 (양 볼 포함)
        current_area = TouchedArea::SIDE;
    } else {
        return BT::NodeStatus::FAILURE;
    }

    if (current_area != _last_touched_area) {
        _current_sequence_index = 0;
    }
    _last_touched_area = current_area;

    const std::map<TouchedArea, std::vector<int>> sequences = {
        {TouchedArea::HEAD,       {4, 3}},
        {TouchedArea::RIGHT_BUTT, {6, 7, 5}},
        {TouchedArea::SIDE,       {9}}
    };

    const auto& current_sequence = sequences.at(current_area);

    if (_current_sequence_index >= current_sequence.size()) {
        _current_sequence_index = 0;
    }

    int action_to_publish = current_sequence[_current_sequence_index];
    edie->PubEmotion(action_to_publish);

    _current_sequence_index++;

    return BT::NodeStatus::SUCCESS;
  }

} // namespace aeirobot
