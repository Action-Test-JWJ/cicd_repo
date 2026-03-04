#include "edie_hardware/edie_hardware.hpp"

namespace edie_hardware
{

void EdieHardware::MotorEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
{ 
    motor_enable_cmd_ = msg->data;
}

void EdieHardware::SetLeftWheelPidGainCallback(
    const edie_msgs::srv::SetPidGain::Request::SharedPtr req,
    edie_msgs::srv::SetPidGain::Response::SharedPtr res)
{
    bool set_success = command_sender_.SetLeftWheelPidGains(comms_, req->p, req->i, req->d);
    res->success = set_success;
    res->part_name = "left_wheel";
}

void EdieHardware::SetRightWheelPidGainCallback(
    const edie_msgs::srv::SetPidGain::Request::SharedPtr req,
    edie_msgs::srv::SetPidGain::Response::SharedPtr res)
{
    bool set_success = command_sender_.SetRightWheelPidGains(comms_, req->p, req->i, req->d);
    res->success = set_success;
    res->part_name = "right_wheel";
}

void EdieHardware::SetLeftLegPidGainCallback(
    const edie_msgs::srv::SetPidGain::Request::SharedPtr req,
    edie_msgs::srv::SetPidGain::Response::SharedPtr res)
{
    bool set_success = command_sender_.SetLeftLegsPidGains(comms_, req->p, req->i, req->d);
    res->success = set_success;
    res->part_name = "left_leg";
}

void EdieHardware::SetRightLegPidGainCallback(
    const edie_msgs::srv::SetPidGain::Request::SharedPtr req,
    edie_msgs::srv::SetPidGain::Response::SharedPtr res)
{
    bool set_success = command_sender_.SetRightLegsPidGains(comms_, req->p, req->i, req->d);
    res->success = set_success;
    res->part_name = "right_leg";
}

void EdieHardware::SetLeftEarPidGainCallback(
    const edie_msgs::srv::SetPidGain::Request::SharedPtr req,
    edie_msgs::srv::SetPidGain::Response::SharedPtr res)
{
    bool set_success = command_sender_.SetLeftEarsPidGains(comms_, req->p, req->i, req->d);
    res->success = set_success;
    res->part_name = "left_ear";
}

void EdieHardware::SetRightEarPidGainCallback(
    const edie_msgs::srv::SetPidGain::Request::SharedPtr req,
    edie_msgs::srv::SetPidGain::Response::SharedPtr res)
{
    bool set_success = command_sender_.SetRightEarsPidGains(comms_, req->p, req->i, req->d);
    res->success = set_success;
    res->part_name = "right_ear";
}

void EdieHardware::GetWheelsPidGainCallback(
    const edie_msgs::srv::GetPidGain::Request::SharedPtr req,
    edie_msgs::srv::GetPidGain::Response::SharedPtr res)
{
    (void)req;
    float left_wheel_k_p = 0.0f, left_wheel_k_i = 0.0f, left_wheel_k_d = 0.0f;
    float right_wheel_k_p = 0.0f, right_wheel_k_i = 0.0f, right_wheel_k_d = 0.0f;
    status_requester_.RequestWheelsPidGains(comms_, left_wheel_k_p, left_wheel_k_i, left_wheel_k_d, right_wheel_k_p, right_wheel_k_i, right_wheel_k_d);
    res->part_name = "wheels";
    res->left_p = left_wheel_k_p;
    res->left_i = left_wheel_k_i;
    res->left_d = left_wheel_k_d;
    res->right_p = right_wheel_k_p;
    res->right_i = right_wheel_k_i;
    res->right_d = right_wheel_k_d;
}

void EdieHardware::GetLegsPidGainCallback(
    const edie_msgs::srv::GetPidGain::Request::SharedPtr req,
    edie_msgs::srv::GetPidGain::Response::SharedPtr res)
{
    (void)req;
    float left_leg_k_p = 0.0f, left_leg_k_i = 0.0f, left_leg_k_d = 0.0f;
    float right_leg_k_p = 0.0f, right_leg_k_i = 0.0f, right_leg_k_d = 0.0f;
    status_requester_.RequestLegsPidGains(comms_, left_leg_k_p, left_leg_k_i, left_leg_k_d, right_leg_k_p, right_leg_k_i, right_leg_k_d);
    res->part_name = "legs";
    res->left_p = left_leg_k_p;
    res->left_i = left_leg_k_i;
    res->left_d = left_leg_k_d;
    res->right_p = right_leg_k_p;
    res->right_i = right_leg_k_i;
    res->right_d = right_leg_k_d;
}

void EdieHardware::GetEarsPidGainCallback(
    const edie_msgs::srv::GetPidGain::Request::SharedPtr req,
    edie_msgs::srv::GetPidGain::Response::SharedPtr res)
{
    (void)req;
    float left_ear_k_p = 0.0f, left_ear_k_i = 0.0f, left_ear_k_d = 0.0f;
    float right_ear_k_p = 0.0f, right_ear_k_i = 0.0f, right_ear_k_d = 0.0f;
    status_requester_.RequestEarsPidGains(comms_, left_ear_k_p, left_ear_k_i, left_ear_k_d, right_ear_k_p, right_ear_k_i, right_ear_k_d);
    res->part_name = "ears";
    res->left_p = left_ear_k_p;
    res->left_i = left_ear_k_i;
    res->left_d = left_ear_k_d;
    res->right_p = right_ear_k_p;
    res->right_i = right_ear_k_i;
    res->right_d = right_ear_k_d;
}

} // namespace edie_hardware