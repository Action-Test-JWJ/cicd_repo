
// `EdieMotionHandler` 클래스의 주요 멤버 함수와 생성자, 소멸자가 구현된 파일
// Publisher와 Subscriber 생성, 모션 데이터 로드, 모션 데이터 퍼블리시 함수 등이 포함
// 액션 서버와 클라이언트 생성
// 모션 데이터 구조체 관리 및 기본적인 유틸리티 함수가 구현
// 모션 핸들러의 핵심 기능과 데이터 관리를 담당



#include "edie_motion/edie_motion_handler.hpp"

EdieMotionHandler::EdieMotionHandler() : Node("edie_motion_handler_node"),
                                        motion_done_(false),
                                        current_motion_step_(0)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    rcl_action_server_options_t action_server_options = rcl_action_server_get_default_options();

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&EdieMotionHandler::JointStatesCallback, this, std::placeholders::_1), options);
    motion_index_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/edie8/emotion/motion_index", 10, std::bind(&EdieMotionHandler::MotionIndexCallback, this, std::placeholders::_1), options);

    motion_done_pub = this->create_publisher<std_msgs::msg::Bool>("/edie8/emotion/motion_done", 10);
    
    // l_ear_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    //   "/edie_l_ear_position_controller/commands", 10);
    
    // r_ear_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    //   "/edie_r_ear_position_controller/commands", 10);

    leg_traj_action_client_ = rclcpp_action::create_client<EdieMotionTrajectory>(
      this,
      "/edie_leg_trajectory_controller/command");

    ear_traj_action_client_ = rclcpp_action::create_client<EdieMotionTrajectory>(
      this,
      "/edie_ear_trajectory_controller/command");

    motion_action_client_ = rclcpp_action::create_client<EdieMotion>(
      this,
      "/edie8/emotion/motion_index");
    motion_action_server_ = rclcpp_action::create_server<EdieMotion>(
        this,
        "/edie8/emotion/motion_index",
        std::bind(&EdieMotionHandler::HandleGoal, this, _1, _2),
        std::bind(&EdieMotionHandler::HandleCancel, this, _1),
        std::bind(&EdieMotionHandler::ExecuteGoal, this, _1),
        action_server_options,
        callback_group_);

    std::string yaml_file_path = ament_index_cpp::get_package_share_directory("edie_motion") + "/data/motion.yaml";
    config = YAML::LoadFile(yaml_file_path);

    // 파라미터 변경 콜백 등록
    // parameter_callback_handle_ = this->add_on_set_parameters_callback(
    //     std::bind(&EdieMotionHandler::ParameterCallback, this, std::placeholders::_1));
        
    legs_joint_states_.resize(2);
    ears_joint_states_.resize(2);
}

EdieMotionHandler::~EdieMotionHandler()
{
}

void EdieMotionHandler::LoadMotionData(const uint8_t motion_index)
{
    // RCLCPP_INFO(this->get_logger(), "LoadMotionData called with motion_index: %d", motion_index);
    motion_data_sequence.clear();

    // RCLCPP_INFO(this->get_logger(), "Config size: %zu", config.size());
    // RCLCPP_INFO(this->get_logger(), "Config keys:");
    // for (auto it = config.begin(); it != config.end(); ++it) {
    //     RCLCPP_INFO(this->get_logger(), "  Key: %s", it->first.as<std::string>().c_str());
    // }
    
    if (!config[std::to_string(motion_index)]) {
        RCLCPP_ERROR(this->get_logger(), "Invalid motion index: %d", motion_index);
        return;
    }

    // motion.yaml에서 motion을 추가하면 수동으로 추가 할 필요 없이 selected_motion_data를 자동으로 추가
    auto selected_motion_data = config[std::to_string(motion_index)].as<std::vector<std::vector<std::vector<double>>>>();
    // RCLCPP_INFO(this->get_logger(), "Selected motion data size: %zu", selected_motion_data.size());

    for (const auto& step : selected_motion_data) 
    {
        if (step.size() != 2 || step[0].size() < 3 || step[1].size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Skipping invalid step data");
            continue;
        }

        MotionData motion_data;
        motion_data.legs = {step[0][0], step[0][1]};
        motion_data.ears = {step[1][0], step[1][1]};
        motion_data.motion_time = step[0][2];
        // RCLCPP_INFO(this->get_logger(), "Loaded step - legs: [%.2f, %.2f], ears: [%.2f, %.2f], time: %.2f", motion_data.legs[0], motion_data.legs[1], motion_data.ears[0], motion_data.ears[1], motion_data.motion_time);
        motion_data_sequence.push_back(motion_data);
    }
}

void EdieMotionHandler::PubMotionDone(bool motion_done)
{
    std_msgs::msg::Bool msg;
    msg.data = motion_done;
    motion_done_pub->publish(msg);
}

uint8_t EdieMotionHandler::GetMotionSteps(const std::vector<MotionData> &motion_data) 
{
    uint8_t valid_count = motion_data.size();
    RCLCPP_INFO(this->get_logger(), "Valid motion steps: %d", valid_count);
    return valid_count;
}