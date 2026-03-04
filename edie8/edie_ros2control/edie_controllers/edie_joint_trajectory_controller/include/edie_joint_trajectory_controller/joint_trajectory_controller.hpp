#ifndef EDIE_JOINT_CONTROLLER_HPP_
#define EDIE_JOINT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// #include "controller_interface/controller_interface.hpp"
#include "controller_interface/chainable_controller_interface.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"
#include "edie_joint_trajectory_controller/edie_joint_trajectory_controller_parameters.hpp"

#include "edie_msgs/action/edie_motion_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_msgs/srv/query_trajectory_state.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"

#include "edie_joint_trajectory_controller/trajectory/fifth_order_trajectory_generate.hpp"
#include "edie_joint_trajectory_controller/trajectory/third_order_trajectory_generate.hpp"
#include "edie_joint_trajectory_controller/trajectory/linear_trajectory_generate.hpp"
#include "edie_joint_trajectory_controller/trajectory/lspb_trajectory_generate.hpp"
#include "edie_joint_trajectory_controller/trajectory/sine_trajectory_generate.hpp"

#include "visibility_control.h"

using rclcpp_action::ServerGoalHandle;

namespace edie_joint_trajectory_controller  
{
    class JointTrajectoryController : public controller_interface::ChainableControllerInterface
    {
        public:
            using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
            using EdieMotionTrajectory = edie_msgs::action::EdieMotionTrajectory;
            using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
            using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJointTrajectory>;
            using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
            using StatePublisherPtr = std::unique_ptr<StatePublisher>;
            
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            // controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_init() override;

            std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

            controller_interface::return_type update_reference_from_subscribers() override;

            controller_interface::return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            std::vector<std::string> reference_interface_names_;
            std::vector<std::string> command_interface_names_;

            rclcpp::Duration duration_ = rclcpp::Duration(0, 0);
            
        protected:
            // rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
            rclcpp_action::Server<EdieMotionTrajectory>::SharedPtr action_server_;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotionTrajectory>> jtc_goal_handle_;

            rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
            StatePublisherPtr state_publisher_;

            uint8_t left_lamp_state_;
            uint8_t right_lamp_state_;
            uint16_t range_sensor_state_;

            std::shared_ptr<edie_joint_trajectory_controller::ParamListener> param_listener_;
            edie_joint_trajectory_controller::Params params_;

        private:
            rclcpp_action::GoalResponse HandleGoalReceiveCallback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const EdieMotionTrajectory::Goal> goal);
            rclcpp_action::CancelResponse HandleGoalCancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotionTrajectory>> goal_handle);
            void HandleGoalAcceptCallback(std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotionTrajectory>> goal_handle);

            FifthOrderTrajectoryGenerator fifth_traj_gen[2];
            ThirdOrderTrajectoryGenerator third_traj_gen[2];
            LinearTrajectoryGenerator linear_traj_gen[2];
            LspbTrajectoryGenerator lspb_traj_gen[2];
            SineTrajectoryGenerator sine_traj_gen[2];

            std::string trajectory_type_;
            rclcpp::Time start_time_;
            bool motion_active_ = false;
    };
}

#endif // EDIE_JOINT_CONTROLLER_HPP_