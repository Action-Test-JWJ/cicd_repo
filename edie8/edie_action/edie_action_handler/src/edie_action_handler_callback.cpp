#include "edie_action_handler/edie_action_handler_node.hpp"

void ActionHandlerNode::SoundDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    sound_done_ = msg->data;
    return;
}

void ActionHandlerNode::DisplayDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // display_done = true 신호를 받았을 때만 로직을 실행합니다.
    if (msg->data == true)
    {
        // 모션이 아직 실행 중이라면...
        if (!motion_done_)
        {
            // ...기억해둔 마지막 display 명령을 다시 보내서 표정을 계속 유지(반복)시킵니다.
            pub_display_->publish(last_display_index_msg_);
            
            // 아직 display가 완전히 끝난 것으로 간주하지 않고 함수를 종료합니다.
            // 이렇게 하면 display_done_ 플래그는 false로 유지됩니다.
            return;
        }
        else
        {
            // 모션이 끝났다면, 비로소 display도 끝난 것으로 처리합니다.
            display_done_ = true;
        }
    }
    else
    {
        // display_done = false 신호 (예: 디스플레이 시작)를 받으면 상태를 업데이트합니다.
        display_done_ = false;
    }
    return;
}

void ActionHandlerNode::MotionDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    motion_done_ = msg->data;
    // motion_done_이 true가 되어도 다른 플래그를 강제로 변경하지 않습니다.
    // 완료 여부는 main 루프의 CheckActionDone()이 종합적으로 판단합니다.
    // if (motion_done_) {
    //     action_done_ = true;
    //     display_done_ = true;
    //     sound_done_ = true;
    // }
    return;
}

void ActionHandlerNode::EdieActionCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if(now_edie_action_ == 0)
    {
        now_edie_action_ = msg->data;
        action_received_ = true;
        display_done_ = false;
        sound_done_ = false;
        motion_done_ = false;
        action_done_ = false;
        RCLCPP_INFO(this->get_logger(), "Received Edie Action : %d", msg->data);
    }
    else if (!action_done_) // Idle이 아닌 상태에서 emotion이 끝나지 않았을때 다른 action이 들어왔을 때
    {
        if (now_edie_action_ == msg->data) {
            // 진행 중에 같은 인덱스면 조용히 무시 (로그 폭주 방지)
            return;
        }
        RCLCPP_ERROR(this->get_logger(), "Cannot update action because emotion is not done yet.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Out of the logic.");
    }
}

void ActionHandlerNode::SrvActionDoneCallback(const std::shared_ptr<edie_msgs::srv::CheckReady::Request>,
            std::shared_ptr<edie_msgs::srv::CheckReady::Response> response)
{
    if (action_done_ == true)
    {
        response->ready = true;
        // RCLCPP_INFO(this->get_logger(), "NOW : [%d]", (int)response->ready);
    }
    else
    {
        response->ready = false;
        // RCLCPP_INFO(this->get_logger(), "NOW : [%d]", (int)response->ready);
    }
}
