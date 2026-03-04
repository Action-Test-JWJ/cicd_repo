#ifndef EDIE_ROS2_CONTROL_HARDWARE_HPP_
#define EDIE_ROS2_CONTROL_HARDWARE_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <vector>

#include "edie_hardware/visibility_control.h"

#include "edie_hardware/serial_comms.hpp"
#include "edie_hardware/protocol_commands.hpp"
#include "edie_hardware/protocol_status.hpp"
#include "edie_hardware/wheel.hpp"
#include "libserial/SerialPort.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "edie_msgs/srv/set_pid_gain.hpp"
#include "edie_msgs/srv/get_pid_gain.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace edie_hardware
{

struct RobotStatus {
  uint8_t enabled = 0;
  float battery_v = 0.0f;

  int32_t l_wheel_rpm = 0, r_wheel_rpm = 0;
  int32_t l_wheel_enc = 0, r_wheel_enc = 0;
  int32_t l_leg_enc = 0,  r_leg_enc = 0;
  int32_t l_ear_enc = 0,  r_ear_enc = 0;

  bool l_leg_limit_sw = false, r_leg_limit_sw = false;
  std::array<int16_t, 12> fsr{};

  bool is_plug_charging;
  bool is_station_charging;
};

struct Config
{
  int loop_rate = 0;
  std::string port = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  float enc_counts_per_rev = 0.0;
  float l_wheel_coeff = 1.0;
  float r_wheel_coeff = 1.0;
  int leg_limit = 90;
  int ear_limit = 70;
};

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  // double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
};

enum class ControlMode {
  Position,
  Velocity,
  // Torque,
  // Currrent,
  // ExtendedPosition,
  // MultiTurn,
  // CurrentBasedPosition,
  PWM,
  // TorqueOnOff,
};

class EdieHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EdieHardware)

  EDIE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  EDIE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  EDIE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  EDIE_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  EDIE_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  EDIE_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  EDIE_HARDWARE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  EDIE_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  EDIE_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);
  return_type reset_command();
  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_params();

  /* Pub Functions*/
  void PubFSRValues(std::vector<int16_t> fsr_values);
  void PubBatteryVoltage(float battery_voltage);
  void PubBatteryChargingStatus(uint8_t is_plug_charging, uint8_t is_station_charging);
  void PubMotorEnableState(uint8_t motor_enable_state);
  void PubLeftWheelVelocity(int16_t left_wheel_rpm);
  void PubRightWheelVelocity(int16_t right_wheel_rpm);
  void PubLeftWheelEncoder(int32_t left_wheel_encoder);
  void PubRightWheelEncoder(int32_t right_wheel_encoder);
  void PubLeftLegSwitch(uint8_t left_leg_limit_sw);
  void PubRightLegSwitch(uint8_t right_leg_limit_sw); 

  /* Topic Callbacks */
  void MotorEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void DebugModeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /* Service Callbacks */
  void SetLeftWheelPidGainCallback(const edie_msgs::srv::SetPidGain::Request::SharedPtr req, edie_msgs::srv::SetPidGain::Response::SharedPtr res);
  void SetRightWheelPidGainCallback(const edie_msgs::srv::SetPidGain::Request::SharedPtr req, edie_msgs::srv::SetPidGain::Response::SharedPtr res);
  void SetLeftLegPidGainCallback(const edie_msgs::srv::SetPidGain::Request::SharedPtr req, edie_msgs::srv::SetPidGain::Response::SharedPtr res);
  void SetRightLegPidGainCallback(const edie_msgs::srv::SetPidGain::Request::SharedPtr req, edie_msgs::srv::SetPidGain::Response::SharedPtr res);
  void SetLeftEarPidGainCallback(const edie_msgs::srv::SetPidGain::Request::SharedPtr req, edie_msgs::srv::SetPidGain::Response::SharedPtr res);
  void SetRightEarPidGainCallback(const edie_msgs::srv::SetPidGain::Request::SharedPtr req, edie_msgs::srv::SetPidGain::Response::SharedPtr res);
  void GetWheelsPidGainCallback(const edie_msgs::srv::GetPidGain::Request::SharedPtr req, edie_msgs::srv::GetPidGain::Response::SharedPtr res);
  void GetLegsPidGainCallback(const edie_msgs::srv::GetPidGain::Request::SharedPtr req, edie_msgs::srv::GetPidGain::Response::SharedPtr res);
  void GetEarsPidGainCallback(const edie_msgs::srv::GetPidGain::Request::SharedPtr req, edie_msgs::srv::GetPidGain::Response::SharedPtr res);
  void ResetOdometryCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  
private:
  std::vector<Joint> joints_;
  ControlMode control_mode_{ControlMode::Position};

  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  SerialComms comms_;
  CommandSender command_sender_;
  StatusRequester status_requester_;

  rclcpp::Node::SharedPtr node_;

  /* Publishers */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr fsr_values_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_volt_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_charging_status_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_enable_state_pub;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr left_wheel_vel_pub;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr right_wheel_vel_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_wheel_enc_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_wheel_enc_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_leg_limit_sw_pub;  
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_leg_limit_sw_pub;

  /* Subscriptions */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_enabled_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr debug_mode_sub;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr left_wheel_target_rpm_sub;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr right_wheel_target_rpm_sub; 

  /* Services */
  rclcpp::Service<edie_msgs::srv::SetPidGain>::SharedPtr set_left_wheel_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::SetPidGain>::SharedPtr set_right_wheel_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::SetPidGain>::SharedPtr set_left_leg_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::SetPidGain>::SharedPtr set_right_leg_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::SetPidGain>::SharedPtr set_left_ear_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::SetPidGain>::SharedPtr set_right_ear_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::GetPidGain>::SharedPtr get_wheels_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::GetPidGain>::SharedPtr get_legs_pid_gain_srv;
  rclcpp::Service<edie_msgs::srv::GetPidGain>::SharedPtr get_ears_pid_gain_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr req_reset_odometry_srv;

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;
  std::atomic<bool> stop_executor_;

  uint8_t motor_enable_cmd_;
  uint8_t motor_enable_state_;
  float battery_voltage;
  int16_t l_wheel_rpm;
  int16_t r_wheel_rpm;
  int16_t l_wheel_last_enc;
  int16_t r_wheel_last_enc;
  int32_t l_leg_pos_enc;
  int32_t r_leg_pos_enc;
  int16_t l_ear_pos_enc;
  int16_t r_ear_pos_enc;
  bool l_leg_limit_sw;
  bool r_leg_limit_sw;
  std::vector<int16_t> fsr_values;

  bool is_plug_charging;
  bool is_station_charging;

  /* Debug Mode */
  bool debug_mode_;
};
}  // namespace edie_hardware

#endif  // EDIE_ROS2_CONTROL_HARDWARE_HPP_