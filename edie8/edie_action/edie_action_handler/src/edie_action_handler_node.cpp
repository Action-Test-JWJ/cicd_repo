#include "edie_action_handler/edie_action_handler_node.hpp"

ActionHandlerNode::ActionHandlerNode() : Node("edie_action_handler_node"), 
        display_done_(true), 
        sound_done_(true),
        motion_done_(true),
        action_done_(true),
        action_received_(false),
        now_edie_action_(0),
        parser("predefined_action.yaml")
{
    action_data = parser.GetActionData();

    sub_display_done_ = this->create_subscription<std_msgs::msg::Bool>(
        "/edie8/emotion/display_done", 10, std::bind(&ActionHandlerNode::DisplayDoneCallback, this, std::placeholders::_1));      // 디스플레이 동작 마침

    sub_sound_done_ = this->create_subscription<std_msgs::msg::Bool>(
        "/edie8/emotion/sound_done", 10, std::bind(&ActionHandlerNode::SoundDoneCallback, this, std::placeholders::_1));          // 사운드 동작 마침

    sub_motion_done_ = this->create_subscription<std_msgs::msg::Bool>(
        "/edie8/emotion/motion_done", 10, std::bind(&ActionHandlerNode::MotionDoneCallback, this, std::placeholders::_1));          // 모션 동작 마침

    sub_edie_emotion_index_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/edie8/emotion/action_index", aeirobot::qos_topic_profile, std::bind(&ActionHandlerNode::EdieActionCallback, this, std::placeholders::_1));              // 현재 상태에 따른 액션

    pub_action_done_ = this->create_publisher<std_msgs::msg::Bool>("/edie8/emotion/emotion_done", 10);                               // 감정표현 마침
    pub_display_ = this->create_publisher<std_msgs::msg::UInt8>("/edie8/emotion/display_index", 10);                                   // 디스플레이 명령
    pub_sound_ = this->create_publisher<std_msgs::msg::UInt8>("/edie8/emotion/sound_index", 10);                                       // 사운드 명령
    pub_motion_ = this->create_publisher<std_msgs::msg::UInt8>("/edie8/emotion/motion_index", 10);                                   // 모션 명령

    serv_action_done_ = this->create_service<edie_msgs::srv::CheckReady>("/edie8/emotion/action_ready",
                                                                    std::bind(&ActionHandlerNode::SrvActionDoneCallback, this, std::placeholders::_1, std::placeholders::_2));  
}

ActionHandlerNode::~ActionHandlerNode()
{}

bool ActionHandlerNode::GetDisplayDone()
{
    return display_done_;
}

bool ActionHandlerNode::GetSoundDone()
{
    return sound_done_;
}

bool ActionHandlerNode::GetMotionDone()
{
    return motion_done_;
}

bool ActionHandlerNode::GetActionDone()
{
    return action_done_;
}

uint8_t ActionHandlerNode::GetEdieAction()
{
    return now_edie_action_;
}

bool ActionHandlerNode::GetActionReceived()
{
    return action_received_;
}

void ActionHandlerNode::SetEdieAction(uint8_t input)
{
    now_edie_action_ = input;
}

void ActionHandlerNode::SetDisplayDone(bool val)
{
    display_done_ = val;
}

void ActionHandlerNode::SetSoundDone(bool val)
{
    sound_done_ = val;
}

void ActionHandlerNode::SetMotionDone(bool val)
{
    motion_done_ = val;
}

void ActionHandlerNode::SetActionDone(bool val)
{
    action_done_ = val;
}

void ActionHandlerNode::SetActionReceived(bool val)
{
    action_received_ = val;
}

void ActionHandlerNode::StoreLastDisplayMsg(const std_msgs::msg::UInt8 &msg)
{
    last_display_index_msg_ = msg;
}

void ActionHandlerNode::PubActionDone(const std_msgs::msg::Bool &msg)
{
    pub_action_done_->publish(msg);
}

void ActionHandlerNode::PubDisplayIndex(const std_msgs::msg::UInt8 &msg)
{
    pub_display_->publish(msg);
}

void ActionHandlerNode::PubSoundIndex(const std_msgs::msg::UInt8 &msg)
{
    pub_sound_->publish(msg);
}

void ActionHandlerNode::PubMotionIndex(const std_msgs::msg::UInt8 &msg)
{
    pub_motion_->publish(msg);
}

bool ActionHandlerNode::CheckActionDone()
{
    // display, sound, motion이 "모두" 완료되었을 때만 true를 반환하도록 수정
    return (display_done_ && sound_done_ && motion_done_);
    
    // 아래는 기존의 잘못된 로직
    // if(display_done_ || sound_done_ || motion_done_) return true;
    
    // // motion이 완료가 되어야 action이 완료된 것으로 간주
    // // if(motion_done_) return true;
    // else return false;
}