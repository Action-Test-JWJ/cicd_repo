#include "aeirobot_toolbox/command_protocol.hpp"

namespace aeirobot
{
    std::string CmdConverter::FloatArrToString(const std::vector<double> &values)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
                oss << " , "; // 첫 번째 값이 아니라면 콤마로 구분
            oss << values[i];
        }
        return oss.str();
    }

    // 명령어에 해당하는 문자열 반환
    std::string CmdConverter::CommandToString(const aeirobot_msgs::msg::Command &msg)
    {
        switch (msg.command)
        {
        case Command::k_stop:
            return "stop";
        case Command::k_motion:
            return "motion";
        case Command::k_walking:
            return "walking";
        case Command::k_tracking:
            return "tracking";
        case Command::k_kick:
            return "kick";
        case Command::k_torque:
            return "torque";
        case Command::k_setmode:
            return "setmode";
        case Command::k_tuning:
            return "tuning";
        default:
            return "unknown_command";
        }
    }

    std::string CmdConverter::StyleToString(const aeirobot_msgs::msg::Command &msg)
    {
        // 처리할 명령어 유형에 따라 스타일을 분류
        switch (msg.command)
        {
        case Command::k_motion: // motion 명령어에 대한 스타일 처리
            switch (msg.style)
            {
            case Style::Motion::k_wholebody:
                return "wholebody";
            case Style::Motion::k_torso:
                return "torso";
            case Style::Motion::k_arms:
                return "arms";
            case Style::Motion::k_legs:
                return "legs";
            case Style::Motion::k_arms_torso:
                return "arms_torso";
            case Style::Motion::k_raim:
                return "raim";
            case Style::Motion::k_finish_1:
                return "wholebody";
            default:
                return "unknown_motion_style";
            }
        case Command::k_walking: // walking 명령어에 대한 스타일 처리
            switch (msg.style)
            {
            case Style::Walking::k_omni:
                return "omni";
            case Style::Walking::k_parallel:
                return "parallel";
            case Style::Walking::k_forward:
                return "forward";
            case Style::Walking::k_backward:
                return "backward";
            case Style::Walking::k_left:
                return "left";
            case Style::Walking::k_right:
                return "right";
            case Style::Walking::k_turn_left:
                return "turn_left";
            case Style::Walking::k_turn_right:
                return "turn_right";
            case Style::Walking::k_centered_left:
                return "centered_left";
            case Style::Walking::k_centered_right:
                return "centered_right";
            case Style::Walking::k_spot:
                return "spot";
            case Style::Walking::k_speed_position:
                return "speed_position";
            case Style::Walking::k_speed_velocity:
                return "speed_velocity";
            case Style::Walking::k_gymnastics:
                return "gymnastics";
            case Style::Walking::k_stop:
                return "stop";
            default:
                return "unknown_walking_style";
            }
        case Command::k_torque: // torque 명령어에 대한 스타일 처리
            switch (msg.style)
            {
            case Style::Torque::k_on:
                return "on";
            case Style::Torque::k_off:
                return "off";
            default:
                return "unknown_torque_style";
            }
        case Command::k_setmode: // setmode 명령어에 대한 스타일 처리
            switch (msg.style)
            {
            case Style::SetMode::k_whole_active:
                return "whole_active";
            case Style::SetMode::k_part_active:
                return "part_active";
            default:
                return "unknown_setmode_style";
            }
        case Command::k_kick: // kick 명령어에 대한 스타일 처리
            switch (msg.style)
            {
            case Style::Kick::k_pass:
                return "pass";
            case Style::Kick::k_left_kick:
                return "left_kick";
            case Style::Kick::k_right_kick:
                return "right_kick";
            case Style::Kick::k_left_elude_kick:
                return "left_elude_kick";
            case Style::Kick::k_right_elude_kick:
                return "right_elude_kick";
            case Style::Kick::k_left_elude_dribble:
                return "left_elude_dribble";
            case Style::Kick::k_right_elude_dribble:
                return "right_elude_dribble";
            case Style::Kick::k_squat:
                return "squat";
            case Style::Kick::k_squat_up:
                return "squat_up";
            case Style::Kick::k_squat_down:
                return "squat_down";
            default:
                return "unknown_kick_style";
            }
        // 다른 명령어 유형에 대한 추가적인 처리를 구현할 수 있음
        default:
            return "style_not_applicable_for_this_command";
        }
    }

    std::string CmdConverter::ValueToString(const aeirobot_msgs::msg::Command &msg)
    {
        // Motion 명령어에 대한 처리
        if (msg.command == Command::k_motion)
        {
            if (msg.style == Style::Motion::k_wholebody)
            {
                // WholeBodyMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::WholeBodyMotion::k_base)
                        return "base";
                    // WholeBodyMotion에 대한 다른 값 처리를 여기에 추가할 수 있음
                    else
                        return "unknown_whole_body_motion_value";
                }
            }
            else if (msg.style == Style::Motion::k_torso)
            {
                // TorsoMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::TorsoMotion::k_base)
                        return "base";
                    else if (msg.value[0] == Value::TorsoMotion::k_keeper_base)
                        return "keeper_base";
                    else if (msg.value[0] == Value::TorsoMotion::k_search)
                        return "search";
                    else if (msg.value[0] == Value::TorsoMotion::k_waistless_search)
                        return "waistless_search";
                    else
                        return "unknown_torso_motion_value";
                }
            }
            else if (msg.style == Style::Motion::k_arms)
            {
                // ArmsMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::ArmsMotion::k_base)
                        return "base";
                    else
                        return "unknown_arms_motion";
                }
            }
            else if (msg.style == Style::Motion::k_legs)
            {
                // TorsoMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::LegsMotion::k_base)
                        return "base";
                    else
                        return "unknown_torso_motion_value";
                }
            }
            else if (msg.style == Style::Motion::k_arms_torso)
            {
                // ArmsTorsoMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::ArmsTorsoMotion::k_base)
                        return "base";
                    else
                        return "unknown_arms_&_torso_motion";
                }
            }
            else if (msg.style == Style::Motion::k_raim)
            {
                // WholeBodyMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::WholeBodyMotion::k_base)
                        return "base";
                    // WholeBodyMotion에 대한 다른 값 처리를 여기에 추가할 수 있음
                    else
                        return "unknown_whole_body_motion_value";
                }
            }
            else if (msg.style == Style::Motion::k_finish_1)
            {
                // WholeBodyMotion 값 처리
                if (!msg.value.empty())
                {
                    if (msg.value[0] == Value::WholeBodyMotion::k_base)
                        return "base";
                    // WholeBodyMotion에 대한 다른 값 처리를 여기에 추가할 수 있음
                    else
                        return "unknown_whole_body_motion_value";
                }
            }
        }
        else
        {
            return FloatArrToString(msg.value);
        }
        return "value_not_applicable";
    }

    std::string CmdConverter::CommandToString(const aeirobot_msgs::msg::Command *msg)
    {
        return CommandToString(*msg);
    }
    std::string CmdConverter::StyleToString(const aeirobot_msgs::msg::Command *msg)
    {
        return StyleToString(*msg);
    }
    std::string CmdConverter::ValueToString(const aeirobot_msgs::msg::Command *msg)
    {
        return ValueToString(*msg);
    }

    std::string CmdConverter::CommandToString(const aeirobot_msgs::msg::Command::SharedPtr msg)
    {
        return CommandToString(*msg);
    }
    std::string CmdConverter::StyleToString(const aeirobot_msgs::msg::Command::SharedPtr msg)
    {
        return StyleToString(*msg);
    }
    std::string CmdConverter::ValueToString(const aeirobot_msgs::msg::Command::SharedPtr msg)
    {
        return ValueToString(*msg);
    }

    std::string CmdConverter::CommandToString(const aeirobot_msgs::action::Command::Goal &goal)
    {
        aeirobot_msgs::msg::Command goal_msg;
        goal_msg.command = goal.command;
        goal_msg.style = goal.style;
        goal_msg.value = goal.value;
        return CommandToString(goal_msg);
    }
    std::string CmdConverter::StyleToString(const aeirobot_msgs::action::Command::Goal &goal)
    {
        aeirobot_msgs::msg::Command goal_msg;
        goal_msg.command = goal.command;
        goal_msg.style = goal.style;
        goal_msg.value = goal.value;
        return StyleToString(goal_msg);
    }
    std::string CmdConverter::ValueToString(const aeirobot_msgs::action::Command::Goal &goal)
    {
        aeirobot_msgs::msg::Command goal_msg;
        goal_msg.command = goal.command;
        goal_msg.style = goal.style;
        goal_msg.value = goal.value;
        return ValueToString(goal_msg);
    }

    std::string CmdConverter::CommandToString(const std::shared_ptr<GoalHandleCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        aeirobot_msgs::msg::Command goal_msg;
        goal_msg.command = goal->command;
        goal_msg.style = goal->style;
        goal_msg.value = goal->value;
        return CommandToString(goal_msg);
    }
    std::string CmdConverter::StyleToString(const std::shared_ptr<GoalHandleCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        aeirobot_msgs::msg::Command goal_msg;
        goal_msg.command = goal->command;
        goal_msg.style = goal->style;
        goal_msg.value = goal->value;
        return StyleToString(goal_msg);
    }
    std::string CmdConverter::ValueToString(const std::shared_ptr<GoalHandleCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        aeirobot_msgs::msg::Command goal_msg;
        goal_msg.command = goal->command;
        goal_msg.style = goal->style;
        goal_msg.value = goal->value;
        return ValueToString(goal_msg);
    }

    uint8_t CmdConverter::StringToCommand(const std::string &command)
    {
        if (command == "stop")
            return Command::k_stop;
        else if (command == "motion")
            return Command::k_motion;
        else if (command == "walking")
            return Command::k_walking;
        else if (command == "tracking")
            return Command::k_tracking;
        else if (command == "kick")
            return Command::k_kick;
        else if (command == "torque")
            return Command::k_torque;
        else if (command == "setmode")
            return Command::k_setmode;
        else if (command == "tuning")
            return Command::k_tuning;
        else if (command == "arms")
            return Command::k_arms;
        else if (command == "hands")
            return Command::k_hands;
        else
            return Command::k_max_size;
    }
    uint8_t CmdConverter::StringToStyle(const std::string &style)
    {
        if (style == "wholebody")
            return Style::Motion::k_wholebody;
        else if (style == "torso")
            return Style::Motion::k_torso;
        else if (style == "arms")
            return Style::Motion::k_arms;
        else if (style == "legs")
            return Style::Motion::k_legs;
        else if (style == "arms_torso")
            return Style::Motion::k_arms_torso;
        else if (style == "raim")
            return Style::Motion::k_raim;
        else if (style == "finish_1")
            return Style::Motion::k_finish_1;
        else if (style == "omni")
            return Style::Walking::k_omni;
        else if (style == "parallel")
            return Style::Walking::k_parallel;
        else if (style == "forward")
            return Style::Walking::k_forward;
        else if (style == "backward")
            return Style::Walking::k_backward;
        else if (style == "left")
            return Style::Walking::k_left;
        else if (style == "right")
            return Style::Walking::k_right;
        else if (style == "turn_left")
            return Style::Walking::k_turn_left;
        else if (style == "turn_right")
            return Style::Walking::k_turn_right;
        else if (style == "k_centered_left")
            return Style::Walking::k_centered_left;
        else if (style == "k_centered_right")
            return Style::Walking::k_centered_right;
        else if (style == "spot")
            return Style::Walking::k_spot;
        else if (style == "dribble")
            return Style::Walking::k_dribble;
        else if (style == "stop")
            return Style::Walking::k_stop;
        else if (style == "speed_position")
            return Style::Walking::k_speed_position;
        else if (style == "speed_velocity")
            return Style::Walking::k_speed_velocity;
        else if (style == "gymnastics")
            return Style::Walking::k_gymnastics;
        else if (style == "squat")
            return Style::Kick::k_squat;
        else if (style == "squat_up")
            return Style::Kick::k_squat_up;
        else if (style == "squat_down")
            return Style::Kick::k_squat_down;
        else if (style == "ik")
            return Style::Arm::k_ik;
        else
            return -1;
    }
}
