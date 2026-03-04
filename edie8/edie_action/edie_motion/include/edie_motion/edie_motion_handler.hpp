#ifndef EDIE_MOTION_HANDLER_HPP // This should be unique to this file
#define EDIE_MOTION_HANDLER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "edie_msgs/action/edie_motion_trajectory.hpp"
#include "edie_msgs/action/edie_motion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using EdieMotionTrajectory = edie_msgs::action::EdieMotionTrajectory;
using EdieMotion = edie_msgs::action::EdieMotion;

struct MotionData 
{
    std::vector<double> legs; // left leg, right leg
    std::vector<double> ears; // left ear, right ear
    float motion_time;
};

struct JointState 
{
    float position;
    float velocity;
    float acceleration;
};

class EdieMotionHandler : public rclcpp::Node
{
  public:
    EdieMotionHandler();
    ~EdieMotionHandler();
    uint8_t GetMotionSteps(const std::vector<MotionData> &motion_data);
  
  private:
    // rcl_interfaces::msg::SetParametersResult ParameterCallback(const std::vector<rclcpp::Parameter> &parameters);
    void JointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void MotionIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void PubMotionDone(bool motion_done);
    void LoadMotionData(const uint8_t motion_index);

    // Motion Goal 
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const EdieMotion::Goal> goal);
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotion>> goal_handle);
    void ExecuteGoal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotion>> goal_handle);

    std::vector<MotionData> motion_data_sequence;
    rclcpp::Time last_motion_time;
    YAML::Node config;
    bool motion_done_;
    uint8_t current_motion_step_;
    uint8_t remaining_motion_step_;
    float elapsed_time_;
    float remaining_time_;
    rclcpp::Time goal_start_time_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motion_index_sub;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_done_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr l_ear_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr r_ear_cmd_pub_;

    rclcpp_action::Client<EdieMotionTrajectory>::SharedPtr leg_traj_action_client_;
    rclcpp_action::Client<EdieMotionTrajectory>::SharedPtr ear_traj_action_client_;
    rclcpp_action::Client<EdieMotion>::SharedPtr motion_action_client_;
    rclcpp_action::Server<EdieMotion>::SharedPtr motion_action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotion>> motion_goal_handle_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotionTrajectory>> traj_goal_handle_;

    std::vector<JointState> legs_joint_states_; // left leg, right leg
    std::vector<JointState> ears_joint_states_; // left ear, right ear

    // rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif // EDIE_MOTION_HANDLER_HPP