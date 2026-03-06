#include "mpu9250_ros2/imu_offset_init.hpp"

using namespace std::placeholders;

ImuOffsetInit::ImuOffsetInit()
    : Node("imu_offset_init_node")
{
    declare_parameter<bool>("imu_req_offset_init", true);
    param_callback_handle = add_on_set_parameters_callback(std::bind(&ImuOffsetInit::ParamChangeCallback, this, _1));

    pub_req_offset_init = this->create_publisher<std_msgs::msg::Bool>("/edie8/sensor/imu_req_offset_init", 10);
    // // Timer: 1k[Hz]
    // timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuOffsetInit::TimerCallback, this));
    // Timer: 2k[Hz]
    timer = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&ImuOffsetInit::TimerCallback, this));

    current_bool = get_parameter("imu_req_offset_init").as_bool();
}

ImuOffsetInit::~ImuOffsetInit()
{
}

void ImuOffsetInit::TimerCallback()
{
    PubOffetInit();
}

void ImuOffsetInit::PubOffetInit()
{
    new_offset_init.data = current_bool;
    pub_req_offset_init->publish(new_offset_init);
}

rcl_interfaces::msg::SetParametersResult ImuOffsetInit::ParamChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    for(const auto& param : parameters)
    {
        if(param.get_name() == "imu_req_offset_init" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        {
                RCLCPP_INFO_STREAM(get_logger(), "Param imu_req_offset_init changed! New value is " << param.as_bool());
            result.successful = true;
            current_bool = param.as_bool();
        }
    }
        
    return result;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuOffsetInit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}