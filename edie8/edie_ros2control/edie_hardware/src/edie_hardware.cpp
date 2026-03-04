#include "edie_hardware/edie_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace edie_hardware
{

CallbackReturn EdieHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger("EdieHardware"), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());

  for (uint i = 0; i < info_.joints.size(); i++) 
  {
    joints_[i].state.position = 0.0;
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
  }

  cfg_.port = info_.hardware_parameters["usb_port"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stof(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.l_wheel_coeff = std::stof(info_.hardware_parameters["l_wheel_coeff"]);
  cfg_.r_wheel_coeff = std::stof(info_.hardware_parameters["r_wheel_coeff"]);
  cfg_.leg_limit = std::stof(info_.hardware_parameters["leg_limit"]);
  cfg_.ear_limit = std::stof(info_.hardware_parameters["ear_limit"]);

  ROS_CYAN_STREAM("usb_port: " << cfg_.port.c_str());
  ROS_CYAN_STREAM("baud_rate: " << cfg_.baud_rate);

  // wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_pulse_per_rev);
  // wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_pulse_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
      // Wheel
      if (joint.name.find("wheel") != std::string::npos)
      {
          if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
          {
              RCLCPP_FATAL(
                  rclcpp::get_logger("EdieHardware"),
                  "Wheel joint '%s' must have exactly one 'velocity' command interface.", joint.name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
          if (joint.state_interfaces.size() != 2 ||
              joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
              joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
          {
              RCLCPP_FATAL(
                  rclcpp::get_logger("EdieHardware"),
                  "Wheel joint '%s' must have 'position' and 'velocity' as state interfaces.", joint.name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
      }
      // Legs Joint
      else if (joint.name.find("leg") != std::string::npos)
      {
          if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
          {
              RCLCPP_FATAL(
                  rclcpp::get_logger("EdieHardware"),
                  "Leg joint '%s' must have exactly one 'position' command interface.", joint.name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
          if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
          {
              RCLCPP_FATAL(
                  rclcpp::get_logger("EdieHardware"),
                  "Leg joint '%s' must have exactly one 'position' state interface.", joint.name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
      }
      // Ears Joint
      else if (joint.name.find("ear") != std::string::npos)
      {
          if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
          {
              RCLCPP_FATAL(
                  rclcpp::get_logger("EdieHardware"),
                  "Ear joint '%s' must have exactly one 'position' command interface.", joint.name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
          if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
          {
              RCLCPP_FATAL(
                  rclcpp::get_logger("EdieHardware"),
                  "Ear joint '%s' must have exactly one 'position' state interface.", joint.name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
      }
  }

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=edie_hardware"});
  node_ = rclcpp::Node::make_shared("_", options);
  fsr_values_pub = node_->create_publisher<std_msgs::msg::Int16MultiArray>("/edie8/sensor/fsr", rclcpp::QoS(1));
  battery_volt_pub = node_->create_publisher<std_msgs::msg::Float32>("/edie8/battery/voltage", rclcpp::QoS(1));
  battery_charging_status_pub = node_->create_publisher<std_msgs::msg::UInt8>("/edie8/battery/charging_status", rclcpp::QoS(1));
  motor_enable_state_pub = node_->create_publisher<std_msgs::msg::Bool>("/edie8/motor/enable/state", rclcpp::QoS(1));
  left_wheel_vel_pub = node_->create_publisher<std_msgs::msg::Int16>("/edie8/motor/left_wheel/velocity/rpm", rclcpp::QoS(10));
  right_wheel_vel_pub = node_->create_publisher<std_msgs::msg::Int16>("/edie8/motor/right_wheel/velocity/rpm", rclcpp::QoS(10));
  left_wheel_enc_pub = node_->create_publisher<std_msgs::msg::Int32>("/edie8/motor/left_wheel/encoder/counts", rclcpp::QoS(10));
  right_wheel_enc_pub = node_->create_publisher<std_msgs::msg::Int32>("/edie8/motor/right_wheel/encoder/counts", rclcpp::QoS(10));
  left_leg_limit_sw_pub = node_->create_publisher<std_msgs::msg::Bool>("/edie8/motor/left_leg/limit_switch", rclcpp::QoS(10));
  right_leg_limit_sw_pub = node_->create_publisher<std_msgs::msg::Bool>("/edie8/motor/right_leg/limit_switch", rclcpp::QoS(10));
  motor_enabled_cmd_sub = node_->create_subscription<std_msgs::msg::Bool>("/edie8/motor/enable/command", rclcpp::QoS(10), std::bind(&EdieHardware::MotorEnabledCallback, this, std::placeholders::_1));
  debug_mode_sub = node_->create_subscription<std_msgs::msg::Bool>("/edie8/debug_mode/enable", rclcpp::QoS(10), std::bind(&EdieHardware::DebugModeCallback, this, std::placeholders::_1));
  set_left_wheel_pid_gain_srv = node_->create_service<edie_msgs::srv::SetPidGain>("/edie8/pid_gain/set/left_wheel", std::bind(&EdieHardware::SetLeftWheelPidGainCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_right_wheel_pid_gain_srv = node_->create_service<edie_msgs::srv::SetPidGain>("/edie8/pid_gain/set/right_wheel", std::bind(&EdieHardware::SetRightWheelPidGainCallback, this, std::placeholders::_1, std::placeholders::_2)); 
  set_left_leg_pid_gain_srv = node_->create_service<edie_msgs::srv::SetPidGain>("/edie8/pid_gain/set/left_leg", std::bind(&EdieHardware::SetLeftLegPidGainCallback, this, std::placeholders::_1, std::placeholders::_2)); 
  set_right_leg_pid_gain_srv = node_->create_service<edie_msgs::srv::SetPidGain>("/edie8/pid_gain/set/right_leg", std::bind(&EdieHardware::SetRightLegPidGainCallback, this, std::placeholders::_1, std::placeholders::_2)); 
  set_left_ear_pid_gain_srv = node_->create_service<edie_msgs::srv::SetPidGain>("/edie8/pid_gain/set/left_ear", std::bind(&EdieHardware::SetLeftEarPidGainCallback, this, std::placeholders::_1, std::placeholders::_2)); 
  set_right_ear_pid_gain_srv = node_->create_service<edie_msgs::srv::SetPidGain>("/edie8/pid_gain/set/right_ear", std::bind(&EdieHardware::SetRightEarPidGainCallback, this, std::placeholders::_1, std::placeholders::_2)); 
  get_wheels_pid_gain_srv = node_->create_service<edie_msgs::srv::GetPidGain>("/edie8/pid_gain/get/wheels", std::bind(&EdieHardware::GetWheelsPidGainCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_legs_pid_gain_srv = node_->create_service<edie_msgs::srv::GetPidGain>("/edie8/pid_gain/get/legs", std::bind(&EdieHardware::GetLegsPidGainCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_ears_pid_gain_srv = node_->create_service<edie_msgs::srv::GetPidGain>("/edie8/pid_gain/get/ears", std::bind(&EdieHardware::GetEarsPidGainCallback, this, std::placeholders::_1, std::placeholders::_2));
  req_reset_odometry_srv = node_->create_service<std_srvs::srv::Trigger>("/edie8/diff_drive_controller/reset_odom", std::bind(&EdieHardware::ResetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));

  executor_.add_node(node_);

  motor_enable_cmd_ = false;
  motor_enable_state_ = false;

  fsr_values.resize(12, 0);
  l_wheel_rpm = 0.0;
  r_wheel_rpm = 0.0;
  l_wheel_last_enc = 0;
  r_wheel_last_enc = 0;
  l_leg_pos_enc = 0;
  r_leg_pos_enc = 0;
  l_ear_pos_enc = 0;
  r_ear_pos_enc = 0;

  debug_mode_ = false;

  is_plug_charging = false;
  is_station_charging = false;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EdieHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++) {
    //     if (info_.joints[i].name == "left_wheel_joint" || info_.joints[i].name == "right_wheel_joint") 
    //     {
    //         state_interfaces.emplace_back(hardware_interface::StateInterface(
    //             info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    //         state_interfaces.emplace_back(hardware_interface::StateInterface(
    //             info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    //     }
    //     else 
    //     {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
        }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EdieHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        if (info_.joints[i].name == "left_wheel_joint" || info_.joints[i].name == "right_wheel_joint") 
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
        }
        else
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
        }
    }
    return command_interfaces;
}

CallbackReturn EdieHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Configuring ...please wait...");
  if (comms_.IsConnected())
  {
    comms_.StopAsyncRx();
    comms_.Disconnect();
  }

  try
  {
    comms_.Connect(cfg_.port, cfg_.baud_rate);
    comms_.StartAsyncRx();
  }
  catch(LibSerial::OpenFailed &e)
  {
      ROS_RED_STREAM("Exceptions: \033[0;91m%s\033[0m" << e.what());
      ROS_RED_STREAM("\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]..." << cfg_.port.c_str());
      // exit(-1);
      rclcpp::shutdown();
  }

  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn EdieHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Cleaning up ...please wait...");

  if (comms_.IsConnected())
  {
    try
    {
      comms_.StopAsyncRx();
      comms_.Disconnect();
    }
    catch(...)
    {
      RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Something went wrong while closing the port.");
    }
  }

  stop_executor_ = true;
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn EdieHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "\033[1m\033[92mReal Robot Activating ...please wait...\033[0m");
  if (!comms_.IsConnected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // if (cfg_.pid_p > 0)
  // {
  //   comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  // }

  for (uint i = 0; i < joints_.size(); i++) 
  {
    // 관절 상태 초기화
    joints_[i].state.position = 0.0;
    joints_[i].state.velocity = 0.0;
    // 명령 초기화
    joints_[i].command.velocity = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "\033[1m\033[96m>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<\033[0m");
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "\033[1m\033[96m>>>>>>>>>>Real Robot Mode<<<<<<<<<<\033[0m");
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "\033[1m\033[96m>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<\033[0m");

  stop_executor_ = false;
  executor_thread_ = std::thread([this]() {
      while (!stop_executor_) {
          try {
              executor_.spin_some();
          } catch (const std::exception& e) {
              RCLCPP_ERROR(rclcpp::get_logger("EdieHardware"), "Exception in executor thread: %s", e.what());
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn EdieHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type EdieHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  StatusFrame s;
  if (!comms_.TryGetLatest(s)) {
    // 새 데이터 없으면 이전 상태 유지 (논블로킹 리턴)
    return return_type::OK;
  }

  // ---- 기존 변환/누적 로직 ----
  motor_enable_state_ = s.enabled;
  battery_voltage     = s.battery_voltage;

  int16_t l_wheel_pos_enc = s.l_wheel_pos_enc;
  int16_t r_wheel_pos_enc = s.r_wheel_pos_enc;
  l_wheel_rpm = s.l_wheel_rpm;
  r_wheel_rpm = s.r_wheel_rpm;

  l_leg_pos_enc = s.l_leg_pos_enc;
  r_leg_pos_enc = s.r_leg_pos_enc;
  l_ear_pos_enc = s.l_ear_pos_enc; // deg
  r_ear_pos_enc = s.r_ear_pos_enc; // deg

  l_leg_limit_sw = s.l_leg_limit_sw;
  r_leg_limit_sw = s.r_leg_limit_sw;

  fsr_values.assign(s.fsr.begin(), s.fsr.end());

  is_plug_charging = s.is_plug_charging;
  is_station_charging = s.is_station_charging;

  // 속도: RPM → rad/s
  joints_[0].state.velocity = static_cast<double>(l_wheel_rpm) * (2.0*M_PI/60.0) * -1.0;
  joints_[1].state.velocity = static_cast<double>(r_wheel_rpm) * (2.0*M_PI/60.0) * +1.0;

  // 위치 누적(랩 보정)
  if (l_wheel_last_enc == 0 && r_wheel_last_enc == 0) {
    l_wheel_last_enc = l_wheel_pos_enc;
    r_wheel_last_enc = r_wheel_pos_enc;
  }
  int32_t dl = int32_t(l_wheel_pos_enc - l_wheel_last_enc);
  if (dl >  32767) dl -= 65536;
  if (dl < -32768) dl += 65536;

  int32_t dr = int32_t(r_wheel_pos_enc - r_wheel_last_enc);
  if (dr >  32767) dr -= 65536;
  if (dr < -32768) dr += 65536;

  l_wheel_last_enc = l_wheel_pos_enc;
  r_wheel_last_enc = r_wheel_pos_enc;

  // Leg: ×100 스케일 → 실수 나눗셈
  joints_[0].state.position = l_leg_pos_enc;
  joints_[1].state.position = r_leg_pos_enc;

  // Ear: deg → rad
  joints_[2].state.position = l_ear_pos_enc;
  joints_[3].state.position = r_ear_pos_enc;

  PubFSRValues(fsr_values);
  PubBatteryVoltage(battery_voltage);
  PubBatteryChargingStatus(is_plug_charging, is_station_charging);
  PubMotorEnableState(motor_enable_state_);
  PubLeftWheelVelocity(l_wheel_rpm);
  PubRightWheelVelocity(r_wheel_rpm);
  PubLeftWheelEncoder(l_wheel_pos_enc);
  PubRightWheelEncoder(r_wheel_pos_enc);
  PubLeftLegSwitch(l_leg_limit_sw);
  PubRightLegSwitch(r_leg_limit_sw);

  return return_type::OK;
}

return_type EdieHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!comms_.IsConnected())
  {
    RCLCPP_FATAL(rclcpp::get_logger("EdieHardware"), "Serial Connection Error!");
    return hardware_interface::return_type::ERROR;
  }

  // command -> rad/s
  // command * ENCODER_REV / (2.0 * PI) * GEAR_RATIO
  // cmd -> encoder/s


  uint8_t enable_motor = motor_enable_cmd_;

  int16_t l_whl_cmd = (int16_t)(joints_[0].command.velocity * cfg_.enc_counts_per_rev / (2.0 * M_PI) * cfg_.l_wheel_coeff) * 1.0;     // cps
  int16_t r_whl_cmd = (int16_t)(joints_[1].command.velocity * cfg_.enc_counts_per_rev / (2.0 * M_PI) * cfg_.r_wheel_coeff) * 1.0;     // cps

  if (debug_mode_)
  {
    RCLCPP_INFO(rclcpp::get_logger("EdieHardware"), "Left Wheel Command: %d, Right Wheel Command: %d", l_whl_cmd, r_whl_cmd);
  }

  if (joints_[0].command.position > cfg_.leg_limit)
      joints_[0].command.position = cfg_.leg_limit;
  else if (joints_[0].command.position < 0)
      joints_[0].command.position = 0;

  if (joints_[1].command.position > cfg_.leg_limit)
      joints_[1].command.position = cfg_.leg_limit;
  else if (joints_[1].command.position < 0)
      joints_[1].command.position = 0;

  if (joints_[2].command.position > cfg_.ear_limit)
      joints_[2].command.position = cfg_.ear_limit;
  else if (joints_[2].command.position < -cfg_.ear_limit)
      joints_[2].command.position = -cfg_.ear_limit;

  if (joints_[3].command.position > cfg_.ear_limit)
      joints_[3].command.position = cfg_.ear_limit;
  else if (joints_[3].command.position < -cfg_.ear_limit)
      joints_[3].command.position = -cfg_.ear_limit;

  int32_t l_leg_cmd = joints_[0].command.position;
  int32_t r_leg_cmd = joints_[1].command.position;  

  int32_t l_ear_cmd = joints_[2].command.position;
  int32_t r_ear_cmd = joints_[3].command.position;

  command_sender_.SendMotorCommand(comms_, enable_motor, l_whl_cmd, r_whl_cmd, l_leg_cmd, r_leg_cmd, l_ear_cmd, r_ear_cmd);

  return return_type::OK;
}

void EdieHardware::PubFSRValues(std::vector<int16_t> fsr_values)
{
    std_msgs::msg::Int16MultiArray fsr_msg;
    fsr_msg.data = fsr_values;
    fsr_values_pub->publish(fsr_msg);
}

void EdieHardware::PubBatteryVoltage(float battery_voltage)
{
    std_msgs::msg::Float32 battery_volt_msg;
    battery_volt_msg.data = battery_voltage;
    battery_volt_pub->publish(battery_volt_msg);
} 

void EdieHardware::PubBatteryChargingStatus(uint8_t is_plug_charging, uint8_t is_station_charging)
{
    std_msgs::msg::UInt8 battery_charging_status_msg;

    // is_plug_charging = LSB, is_station_charging = MSB
    uint8_t status = (is_station_charging << 1) | (is_plug_charging & 0x01);

    battery_charging_status_msg.data = status;
    battery_charging_status_pub->publish(battery_charging_status_msg);
}

void EdieHardware::PubMotorEnableState(uint8_t motor_enable_state)
{
    std_msgs::msg::Bool motor_enabled_state_msg;
    motor_enabled_state_msg.data = motor_enable_state;
    motor_enable_state_pub->publish(motor_enabled_state_msg);
}   

void EdieHardware::PubLeftWheelVelocity(int16_t left_wheel_rpm)
{
    std_msgs::msg::Int16 left_wheel_vel_msg;
    left_wheel_vel_msg.data = left_wheel_rpm;
    left_wheel_vel_pub->publish(left_wheel_vel_msg);
} 

void EdieHardware::PubRightWheelVelocity(int16_t right_wheel_rpm)
{
    std_msgs::msg::Int16 right_wheel_vel_msg;
    right_wheel_vel_msg.data = right_wheel_rpm;
    right_wheel_vel_pub->publish(right_wheel_vel_msg);
}

void EdieHardware::PubLeftWheelEncoder(int32_t left_wheel_encoder)
{
  std_msgs::msg::Int32 left_wheel_enc_msg;
  left_wheel_enc_msg.data = left_wheel_encoder;
  left_wheel_enc_pub->publish(left_wheel_enc_msg);
}

void EdieHardware::PubRightWheelEncoder(int32_t right_wheel_encoder)
{
  std_msgs::msg::Int32 right_wheel_enc_msg;
  right_wheel_enc_msg.data = right_wheel_encoder;
  right_wheel_enc_pub->publish(right_wheel_enc_msg);
}

void EdieHardware::PubLeftLegSwitch(uint8_t left_leg_limit_sw)
{
  std_msgs::msg::Bool left_leg_limit_sw_msg;
  left_leg_limit_sw_msg.data = left_leg_limit_sw;
  left_leg_limit_sw_pub->publish(left_leg_limit_sw_msg);
}

void EdieHardware::PubRightLegSwitch(uint8_t right_leg_limit_sw)
{
  std_msgs::msg::Bool right_leg_limit_sw_msg;
  right_leg_limit_sw_msg.data = right_leg_limit_sw;
  right_leg_limit_sw_pub->publish(right_leg_limit_sw_msg);
}

}  // namespace edie_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(edie_hardware::EdieHardware, hardware_interface::SystemInterface)
