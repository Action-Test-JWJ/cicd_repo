#include "edie_joint_trajectory_controller/joint_trajectory_controller.hpp"
#include <string>

// namespace
// {
// constexpr auto DEFAULT_COMMAND_ACTION = get_node()->get_name() + "/command";
// constexpr auto DEFAULT_COMMAND_TOPIC = get_node()->get_name() + "/command";
// constexpr auto DEFAULT_STATE_TOPIC = get_node()->get_name() + "/controller_state";
// }  // namespace

namespace edie_joint_trajectory_controller
{
    controller_interface::CallbackReturn JointTrajectoryController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();

            reference_interface_names_.clear();
            for (const auto & joint_name : params_.joints)
            {
                reference_interface_names_.emplace_back(joint_name + "/position");
            }
            reference_interfaces_.resize(reference_interface_names_.size(), 0.0);
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }


    // controller_interface::InterfaceConfiguration JointTrajectoryController::command_interface_configuration() const
    // {
    //     controller_interface::InterfaceConfiguration config;
    //     config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    //     config.names.push_back("left_leg_joint/position");
    //     config.names.push_back("right_leg_joint/position");
    //     config.names.push_back("left_ear_joint/position");
    //     config.names.push_back("right_ear_joint/position");

    //     // RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration...");
    //     return config;
    // }
    controller_interface::InterfaceConfiguration
    JointTrajectoryController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names = command_interface_names_;

        return command_interfaces_config;
    }

    // controller_interface::InterfaceConfiguration JointTrajectoryController::state_interface_configuration() const
    // {
    //     controller_interface::InterfaceConfiguration config;
    //     config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    //     config.names.push_back("left_leg_joint/position");
    //     config.names.push_back("right_leg_joint/position");
    //     config.names.push_back("left_ear_joint/position");
    //     config.names.push_back("right_ear_joint/position");

    //     // RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration...");
    //     return config;
    // }

    controller_interface::InterfaceConfiguration JointTrajectoryController::state_interface_configuration()
    const
    {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }

    // controller_interface::return_type JointTrajectoryController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    // {
        // enable_motor_state_ = static_cast<bool>(state_interfaces_[StateInterfaces::ENABLE_MOTOR_STATE].get_value());
        // left_lamp_state_ = static_cast<uint8_t>(state_interfaces_[StateInterfaces::LEFT_LAMP_STATE].get_value());
        // right_lamp_state_ = static_cast<uint8_t>(state_interfaces_[StateInterfaces::RIGHT_LAMP_STATE].get_value());
        // range_sensor_state_ = static_cast<uint16_t>(state_interfaces_[StateInterfaces::RANGE_SENSOR_STATE].get_value());

    //     return controller_interface::return_type::OK;
    // }

    controller_interface::CallbackReturn JointTrajectoryController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        std::string command_action_name = std::string(get_node()->get_name()) + "/command";
        std::string state_topic_name = std::string(get_node()->get_name()) + "/controller_state";

        command_interface_names_.clear();
        for (const auto & joint_name : params_.joints)
        {
            command_interface_names_.emplace_back(joint_name + "/position");
        }

        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<EdieMotionTrajectory>(
            get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
            get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
            command_action_name,
            std::bind(&JointTrajectoryController::HandleGoalReceiveCallback, this, _1, _2),
            std::bind(&JointTrajectoryController::HandleGoalCancelCallback, this, _1),
            std::bind(&JointTrajectoryController::HandleGoalAcceptCallback, this, _1));

        publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            state_topic_name, rclcpp::SystemDefaultsQoS());

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointTrajectoryController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // try
        // {
        //     // pub_robot_state_ = get_node()->create_publisher<minibot_interfaces::msg::RobotState>("~/robot_state", rclcpp::SystemDefaultsQoS());
        //     // sub_servo_joint_goal_angles_ = get_node()->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     //     "~/servo_joint/all/goal_angles", 10, std::bind(&JointTrajectoryController::callback_joint_angles_command, this, std::placeholders::_1));
        //     // sub_ears_joint_goal_angles_ = get_node()->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     //     "~/servo_joint/ears/goal_angles", 10, std::bind(&JointTrajectoryController::callback_ear_joint_angles_command, this, std::placeholders::_1));
        //     // sub_legs_joint_goal_angles_ = get_node()->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     //     "~/servo_joint/legs/goal_angles", 10, std::bind(&JointTrajectoryController::callback_leg_joint_angles_command, this, std::placeholders::_1));
        //     // pub_range_sensor_state_ = get_node()->create_publisher<sensor_msgs::msg::Range>("~/range", rclcpp::SystemDefaultsQoS());
        // }
        // catch (...)
        // {
        //     return LifecycleNodeInterface::CallbackReturn::ERROR;
        // }

        RCLCPP_INFO(get_node()->get_logger(), "on_activate...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointTrajectoryController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // try
        // {

        // }
        // catch (...)
        // {
        //     return LifecycleNodeInterface::CallbackReturn::ERROR;
        // }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_action::GoalResponse JointTrajectoryController::HandleGoalReceiveCallback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const EdieMotionTrajectory::Goal> goal)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Received Manual Goal l_position: [%f, %f] time: %f", goal->position[0], goal->position[1], goal->time);

        // 예외: 유효 모션인덱스 외 입력시 처리
        // if (goal->motion_index < 0 || static_cast<std::size_t>(goal->motion_index) >= config.size())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Invalid motion index: %d. Please Send goal 0~%ld.", goal->motion_index, config.size() - 1);
        //     PubMotionDone(true);
        //     return rclcpp_action::GoalResponse::REJECT;
        // }

        // 기존 목표가 실행 중이라면 취소 요청
        if (jtc_goal_handle_ && jtc_goal_handle_->is_active())
        {
            RCLCPP_INFO(get_node()->get_logger(), "Canceling previous goal...");
            jtc_goal_handle_->abort(std::make_shared<EdieMotionTrajectory::Result>());
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse JointTrajectoryController::HandleGoalCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotionTrajectory>> goal_handle)
    {
        // 목표가 활성 상태인지 확인
        if (goal_handle->is_active()) {
            RCLCPP_INFO(get_node()->get_logger(), "Canceling active goal...");

            auto result = std::make_shared<EdieMotionTrajectory::Result>();
            result->motion_done = true;
            goal_handle->canceled(result);

            return rclcpp_action::CancelResponse::ACCEPT;
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "Goal is not active and cannot be canceled.");
            return rclcpp_action::CancelResponse::REJECT;
        }
    }

    void JointTrajectoryController::HandleGoalAcceptCallback(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<EdieMotionTrajectory>> goal_handle)
    {
        jtc_goal_handle_ = goal_handle;

        const auto goal = goal_handle->get_goal();
        start_time_ = get_node()->now();  // 시작 시간 저장
        trajectory_type_ = "fifth";
        duration_ = rclcpp::Duration::from_seconds(goal->time);

        for (int i = 0; i < 2; i++)
        {
            if (trajectory_type_ == "fifth")
            {
                fifth_traj_gen[i].current_pose = fifth_traj_gen[i].trajectory_final_value;
                fifth_traj_gen[i].current_velocity = 0.0;
                fifth_traj_gen[i].current_acc = 0.0;
                fifth_traj_gen[i].DetectChangeFinalValue(goal->position[i], 0.0, goal->time);
            }
            else
            {
                // TODO: 추가하기
            }
        }

        motion_active_ = true;  // <- update에서 사용할 플래그
    }

    controller_interface::return_type JointTrajectoryController::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        if (!motion_active_) return controller_interface::return_type::OK;

        // 루프 시작 시간 기록
        auto loop_start = std::chrono::steady_clock::now();

        double l_leg_current_position = 0.0;
        double r_leg_current_position = 0.0;

        rclcpp::Duration elapsed = time - start_time_;
        double t = elapsed.seconds();

        if (trajectory_type_ == "fifth")
        {
            l_leg_current_position = fifth_traj_gen[0].GenerateFifthOderTrajectory(
            fifth_traj_gen[0].current_pose, fifth_traj_gen[0].final_pose,
            fifth_traj_gen[0].current_velocity, fifth_traj_gen[0].final_velocity,
            fifth_traj_gen[0].current_acc, fifth_traj_gen[0].final_acc,
            0.0, fifth_traj_gen[0].final_time);

            r_leg_current_position = fifth_traj_gen[1].GenerateFifthOderTrajectory(
            fifth_traj_gen[1].current_pose, fifth_traj_gen[1].final_pose,
            fifth_traj_gen[1].current_velocity, fifth_traj_gen[1].final_velocity,
            fifth_traj_gen[1].current_acc, fifth_traj_gen[1].final_acc,
            0.0, fifth_traj_gen[1].final_time);
        }
        else
        {
            // TODO: 추가하기
        }

        command_interfaces_[0].set_value(l_leg_current_position);
        command_interfaces_[1].set_value(r_leg_current_position);
        // RCLCPP_INFO_STREAM(
        //     this->get_node()->get_logger(),
        //     trajectory_type_ << " " <<
        //     "jtc command: [0]=" << l_leg_current_position
        //                                 << ", [1]=" << r_leg_current_position);

        // 완료 조건
        if (trajectory_type_ == "fifth" && !fifth_traj_gen[0].is_moving_traj && !fifth_traj_gen[1].is_moving_traj)
        {
            RCLCPP_INFO(get_node()->get_logger(), "Motion complete.");

            auto result = std::make_shared<EdieMotionTrajectory::Result>();
            result->motion_done = true;
            result->success = true;
            jtc_goal_handle_->succeed(result);
            motion_active_ = false;
        }
        else
        {
            // TODO:
        }

        // 루프 끝 시간 기록
        auto loop_end = std::chrono::steady_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();

        // RCLCPP_INFO_STREAM(
        //     this->get_node()->get_logger(),
        //     "Loop timing → start=" << std::chrono::duration_cast<std::chrono::microseconds>(loop_start.time_since_epoch()).count()
        //     << " [us], end=" << std::chrono::duration_cast<std::chrono::microseconds>(loop_end.time_since_epoch()).count()
        //     << " [us], elapsed=" << loop_duration << " [us]");

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type JointTrajectoryController::update_reference_from_subscribers()
    {
        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface>
    JointTrajectoryController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> exported_interfaces;

        for (size_t i = 0; i < reference_interface_names_.size(); ++i)
        {
            exported_interfaces.push_back(
                hardware_interface::CommandInterface(
                    get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
        }

        return exported_interfaces;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(edie_joint_trajectory_controller::JointTrajectoryController, controller_interface::ChainableControllerInterface)
