#ifndef EDIE_ACTION_HANDLER_HPP
#define EDIE_ACTION_HANDLER_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include "edie_msgs/srv/check_ready.hpp"
#include <memory>
#include <random>
#include <aeirobot_toolbox/qos_profiles.hpp>
#include "action_yaml_parser.hpp"
#include "action_data.hpp"

class ActionHandlerNode : public rclcpp::Node
{
public:
    ActionHandlerNode();
    ~ActionHandlerNode();

    bool GetDisplayDone();
    bool GetSoundDone();
    bool GetMotionDone();
    bool GetActionDone();
    uint8_t GetEdieAction();
    bool GetActionReceived();
    void SetEdieAction(uint8_t input);
    void SetDisplayDone(bool val);
    void SetSoundDone(bool val);
    void SetMotionDone(bool val);
    void SetActionDone(bool val);
    void SetActionReceived(bool val);
    void StoreLastDisplayMsg(const std_msgs::msg::UInt8 &msg);
    void PubActionDone(const std_msgs::msg::Bool &msg);
    void PubDisplayIndex(const std_msgs::msg::UInt8 &msg);
    void PubSoundIndex(const std_msgs::msg::UInt8 &msg);
    void PubMotionIndex(const std_msgs::msg::UInt8 &msg);
    bool CheckActionDone();
    std::map<int, ActionData> action_data;

private:
    void SoundDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void DisplayDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void MotionDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void EdieActionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void SrvActionDoneCallback(const std::shared_ptr<edie_msgs::srv::CheckReady::Request>,
             std::shared_ptr<edie_msgs::srv::CheckReady::Response> response);

    bool display_done_, sound_done_, motion_done_, action_done_;
    bool action_received_;
    uint8_t now_edie_action_;
    std_msgs::msg::UInt8 last_display_index_msg_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_display_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_sound_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motion_done_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_edie_emotion_index_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_display_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_sound_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_motion_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_action_done_;
    rclcpp::Service<edie_msgs::srv::CheckReady>::SharedPtr serv_action_done_;

    ActionYamlParser parser;
};

#endif // EDIE_ACTION_HANDLER_HPP