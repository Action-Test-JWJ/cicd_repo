#include "edie_action_handler/edie_action_handler_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionHandlerNode>();
    uint8_t idle_count = 0;
    uint8_t idle_cycle = 0;
    uint8_t hit_count = 0;
    std_msgs::msg::Bool action_done_msg;
    std_msgs::msg::UInt8 display_index_msg;
    std_msgs::msg::UInt8 sound_index_msg;
    std_msgs::msg::UInt8 motion_index_msg;
    
    rclcpp::Time last_published_time_ = node->get_clock()->now();
    rclcpp::Rate loop_rate(50);

    action_done_msg.data = true;
    while (rclcpp::ok())
    {
        // 감정표현 후 Flag 리셋
        if(node->CheckActionDone() && !node->GetActionDone())
        {
            node->SetActionDone(true);
            node->PubActionDone(action_done_msg);
            node->SetActionReceived(false);
            node->SetEdieAction(0);
        }
        
        // Normal (Idle)
        if (node->GetEdieAction() == 0)
        {
            // RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] Idle");
            // RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] +++++++++++++++++++++++++++++++++++++++");
            auto current_time = node->get_clock()->now();

            if (idle_cycle < 4) {
                if ((current_time - last_published_time_).seconds() >= 5.0 && idle_count < 4) {
                    display_index_msg.data = 0;
                    node->PubDisplayIndex(display_index_msg);
                    node->SetSoundDone(true);
                    idle_count++;
                    last_published_time_ = current_time;
                }
                else if ((current_time - last_published_time_).seconds() >= 5.0 && idle_count == 4) {
                    display_index_msg.data = 1;
                    sound_index_msg.data = 0; 
                    node->PubDisplayIndex(display_index_msg);
                    node->PubSoundIndex(sound_index_msg);
                    last_published_time_ = current_time;
                    idle_count = 0;
                    idle_cycle++;
                }
            }
            else if (idle_cycle == 4 && (current_time - last_published_time_).seconds() >= 5.0) {
                display_index_msg.data = 2;
                node->PubDisplayIndex(display_index_msg);
                node->SetSoundDone(true);
                last_published_time_ = current_time;
            }
        }
        else if (node->GetActionReceived())
        {
            RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] Action Received");
            uint8_t current_action = node->GetEdieAction();
            RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] current_action: %d", current_action);

            // 예외처리 : 없는 액션 인덱스 받으면 무시
            if (node->action_data.find(current_action) == node->action_data.end())
            {   
                node->SetDisplayDone(true);
                node->SetSoundDone(true);
                node->SetMotionDone(true);
                node->SetActionReceived(false);
                RCLCPP_ERROR(node->get_logger(), "Index Error. Check Action Index.");
                continue;
            }
            
            // 무작위 선택 로직: 옵션 배열이 있으면 그중에서, 없으면 기본값 사용
            // 스레드별로 1개만 유지되는 엔진
            // 난수 생성
            // 이 스레드 안에서만 공유되는, 한 번만 초기화된 Mersenne Twister 난수 엔진을 만든다
            static thread_local std::mt19937 rng{std::random_device{}()};

            const auto &tmp_data = node->action_data[current_action];
            uint8_t selected_display_index;
            uint8_t selected_sound_index;

            // display_index 결정
            if (!tmp_data.display_options.empty()) 
            {
                std::uniform_int_distribution<size_t> dist(0, tmp_data.display_options.size() - 1);
                selected_display_index = static_cast<uint8_t>(tmp_data.display_options[dist(rng)]);
            } 
            else 
            {
                selected_display_index = static_cast<uint8_t>(tmp_data.display_index);
            }

            // sound_index 결정
            if (!tmp_data.sound_options.empty()) 
            {
                std::uniform_int_distribution<size_t> dist(0, tmp_data.sound_options.size() - 1);
                selected_sound_index = static_cast<uint8_t>(tmp_data.sound_options[dist(rng)]);
            } 
            else 
            {
                selected_sound_index = static_cast<uint8_t>(tmp_data.sound_index);
            }

            display_index_msg.data = selected_display_index;
            sound_index_msg.data   = selected_sound_index;
            motion_index_msg.data  = node->action_data[current_action].motion_index;

            node->StoreLastDisplayMsg(display_index_msg);

            node->PubDisplayIndex(display_index_msg);
            node->PubSoundIndex(sound_index_msg);
            node->PubMotionIndex(motion_index_msg);
            node->SetActionReceived(false);
            idle_count = 0;
            idle_cycle = 0;
            RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] display_index_msg: %d", display_index_msg.data);
            RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] sound_index_msg: %d", sound_index_msg.data);
            RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] motion_index_msg: %d", motion_index_msg.data);
            RCLCPP_INFO(node->get_logger(), "[Action Handler::main.cpp] $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");

        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}