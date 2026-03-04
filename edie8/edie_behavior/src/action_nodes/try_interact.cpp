#include "edie_behavior/action_nodes.hpp"
#include <random> // For std::random_device, std::mt19937, std::uniform_int_distribution
#include <vector> // For std::vector

using namespace std;

namespace aeirobot
{
  
  BT::NodeStatus TryInteract::onStart()
  {
    auto edie = Edie::GetInstance();

    // 1) 진행 중이면 퍼블리시 금지 (동일 틱 재발행 방지)
    if (!edie->is_action_ready) {
      return BT::NodeStatus::RUNNING;
    }

    // 2) 새 메시지가 없으면 퍼블리시 금지 (값이 같든 다르든)
    //   is_new_emo_msg_received: /edie8/vision/emotion_state 콜백에서 true로 set,
    //   PubEmotion(emo) 직후 false로 reset
    if (!edie->is_new_emo_msg_received) {
      return BT::NodeStatus::RUNNING;
    }

    uint8_t emo = 1; // 기본값
    // "Neutral"(0), "Anger"(1), "Happiness"(2), "Surprise"(3), "Disgust"(4), "Sadness"(5), "Fear"(6), "Unknown"(7)
    /*
    # action_index
    # 0 : -
    # 1 : CURIOUS
    # 2 : SLEEPY
    # 3 : AMUSED
    # 4 : HOPEFUL
    # 5 : CRYING
    # 6 : SURPRISED
    # 7 : SURPRISED_2
    # 8 : DISAPPOINTED
    # 9 : LOVE
    # 10: DIZZY
    # 11: DIZZY_2
    */
    // C++11 방식의 표준 난수 생성기
    static std::random_device rd;
    static std::mt19937 gen(rd());

    switch (edie->emotion_state)
    {
      case 0: // Neutral
      {
        // "Neutral"일 때는 아무 감정도 표현하지 않고 즉시 반환합니다.
        // is_new_emo_msg_received 플래그를 소비하여 중복 호출을 방지합니다.
        edie->is_new_emo_msg_received = false;
        return BT::NodeStatus::RUNNING;
      }
      case 1: // Anger
        emo = 8;
        break;
      case 2: // Happiness
      {
        std::vector<uint8_t> options = {3, 4, 9};
        std::uniform_int_distribution<> distrib(0, options.size() - 1);
        emo = options[distrib(gen)];
        if (emo == 9) {
          edie->is_mobile_for_emotion = true;  // 회전 중 플래그 설정
          edie->PubEmotion(9);
          edie->PubNavigation(2);  // SpinOnce 명령 (CMD_SPIN_ONCE = 2)
          RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                      "[TryInteract] LOVE(9)! Spinning with joy!");
          return BT::NodeStatus::RUNNING;
        }
        break;
      }
      
      case 3: // Surprise
      {
        std::vector<uint8_t> options = {6, 7, 8};
        std::uniform_int_distribution<> distrib(0, options.size() - 1);
        emo = options[distrib(gen)];
        
        // Surprise 카운트 먼저 증가
        edie->surprise_count++;
        RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                    "[TryInteract] Surprise detected! Count: %d/3", edie->surprise_count);
        
        // 3번째면 Interaction Mode 끄고 도망가기!
        if (edie->surprise_count >= 3) {
          edie->surprise_count = 0;  // 카운터 리셋
          edie->is_in_interaction_mode = false;  // Interaction Mode OFF
          edie->is_fsr_touch_triggered_ = false;  // 트리거 플래그 리셋!
          edie->is_doing_interaction_approach_ = false;  // 접근 플래그 리셋!
          edie->is_surprised_runaway = true;  
          edie->PubEmotion(8); 
          RCLCPP_WARN(rclcpp::get_logger("TryInteract"), 
                      "[TryInteract] Too much surprise (3/3)! Running away!");
          return BT::NodeStatus::RUNNING;
        }
        
        // 1~2번: 7번(SURPRISED_2)이면 뒤로 가기 (Interaction Mode 유지)
        if (emo == 7) {
          edie->is_mobile_for_emotion = true;
          edie->PubEmotion(emo);
          edie->PubNavigation(6);  // CMD_GO_BACK
          RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                      "[TryInteract] SURPRISED_2! Going back (count: %d/3)", edie->surprise_count);
          return BT::NodeStatus::RUNNING;
        }
        // 6번(SURPRISED)은 그냥 감정만 표현 (Interaction Mode 유지)
        break;
      }
      case 4: // Disgust
      {
        std::vector<uint8_t> options = {5, 8};
        std::uniform_int_distribution<> distrib(0, options.size() - 1);
        emo = options[distrib(gen)];
        break;
      }
      case 5: // Sadness
        emo = 5;
        if (emo == 5) {
          edie->is_mobile_for_emotion = true;  
          edie->PubEmotion(emo);
          edie->PubNavigation(16);  // CMD_GO_front 명령 (CMD_GO_FRONT = 16)
          RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                      "[TryInteract] SADNESS(%d)! go front with sadness!", emo);
          return BT::NodeStatus::RUNNING;
        }
        break;
      case 6: // Fear
      {
        std::vector<uint8_t> options = {10, 11};
        std::uniform_int_distribution<> distrib(0, options.size() - 1);
        emo = options[distrib(gen)];
        break;
      }
      case 7: // Unknown - FSR 부위별 반응
      {
        RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                    "[TryInteract] Unknown emotion. Using FSR-based reaction.");
        
        const auto& skin_state_values = edie->skin_state_values;
        if (skin_state_values.size() != 12) {
          emo = 1;  // 기본값: CURIOUS
          break;
        }
        
        auto max_it = std::max_element(skin_state_values.begin(), skin_state_values.end());
        int max_idx = std::distance(skin_state_values.begin(), max_it);
        
        // 부위별 감정 매핑 (ProcessFsrTouch와 유사)
        if (max_idx >= 4 && max_idx <= 6) { 
          // 정수리: HOPEFUL
          emo = 4;
          RCLCPP_INFO(rclcpp::get_logger("TryInteract"), "[TryInteract] Head touched → HOPEFUL(4)");
        } else if (max_idx >= 0 && max_idx <= 3) { 
          // 오른쪽 궁뎅이: SURPRISED
          emo = 6;
          RCLCPP_INFO(rclcpp::get_logger("TryInteract"), "[TryInteract] Right butt touched → SURPRISED(6)");
        } else if (max_idx >= 7 && max_idx <= 11) { 
          // 쓰담쓰담 (양 볼): LOVE
          emo = 9;
          RCLCPP_INFO(rclcpp::get_logger("TryInteract"), "[TryInteract] Side touched → LOVE(9)");
        } else {
          emo = 1;  // 기본값: CURIOUS
        }
        break;
      }
      default:
        // 완전히 정의되지 않은 emotion_state의 경우 기본값(1)을 사용합니다.
        emo = 1;
        break;
    }
    
    // RCLCPP_INFO(rclcpp::get_logger("TryInteract"), "[TryInteract::onStart] emo: %d", emo);
    // RCLCPP_INFO(rclcpp::get_logger("TryInteract"), "[TryInteract::onStart] --------------------------------------------------------");

    edie->PubEmotion(emo);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus TryInteract::onRunning()
  {
    auto edie = Edie::GetInstance();

    if (edie->is_mobile_for_emotion) {
      // SpinOnce 완료 체크
      auto spin_it = edie->navigation_statuses.find("SpinOnce");
      if (spin_it != edie->navigation_statuses.end() && spin_it->second == "0") {
        // 회전 완료!
        edie->is_mobile_for_emotion = false;
        RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                    "[TryInteract] Spin completed!");
        return BT::NodeStatus::SUCCESS;
      }
      
      // GoBack 완료 체크
      auto goback_it = edie->navigation_statuses.find("GoBack");
      if (goback_it != edie->navigation_statuses.end() && goback_it->second == "0") {
        // 뒤로 가기 완료!
        edie->is_mobile_for_emotion = false;
        RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                    "[TryInteract] Go back completed!");
        return BT::NodeStatus::SUCCESS;
      }
      
      // GoToFront 완료 체크
      auto front_it = edie->navigation_statuses.find("GoToFront");
      if (front_it != edie->navigation_statuses.end() && front_it->second == "0") {
        // 앞으로 가기 완료!
        edie->is_mobile_for_emotion = false;
        RCLCPP_INFO(rclcpp::get_logger("TryInteract"), 
                    "[TryInteract] Go front completed!");
        return BT::NodeStatus::SUCCESS;
      }
      
      return BT::NodeStatus::RUNNING;  // 아직 모바일 동작 중
    }

    // 일반 감정 표현 완료 체크
    return (edie->is_action_ready) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void TryInteract::onHalted()
  {
  }

} // namespace aeirobot