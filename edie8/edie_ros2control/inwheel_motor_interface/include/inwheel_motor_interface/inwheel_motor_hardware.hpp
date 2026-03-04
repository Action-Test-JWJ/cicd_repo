#ifndef INWHEEL_MOTOR_HARDWARE__INWHEEL_MOTOR_HARDWARE_HPP_
#define INWHEEL_MOTOR_HARDWARE__INWHEEL_MOTOR_HARDWARE_HPP_

#include <yaml-cpp/yaml.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include "inwheel_motor_interface/custom_hardware_interface_type_values.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <map>
#include <vector>
#include "inwheel_motor_interface/visiblity_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_math/math_tool.hpp"
#include "inwheel_motor_interface/inwheel_motor_can.hpp"

#include <sensor_msgs/msg/joint_state.hpp>


using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace inwheel_motor_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double velocity_p_gain{0.0}; 
  double velocity_i_gain{0.0}; 
  double velocity_d_gain{0.0}; 
  double torque_p_gain{0.0}; 
  double torque_i_gain{0.0}; 
  double torque_d_gain{0.0}; 
  double estop{0.0}; 
  double torque_on_off{0.0};
  double bus_voltage{0.0}; 
  double bus_current{0.0}; 
  double iq_setpoint{0.0}; 
  double iq_measured{0.0}; 
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
  bool position_control_complete = false;

};

struct Signal {
    std::string name;
    int start_byte;
    std::string type;
    int bits;
    float factor;
    float offset;
    std::string enum_name;
};

struct CANMessage {
    int cmd_id;
    std::string name;
    std::vector<Signal> signals;
};

struct ODriveEnumValue {
    std::string enum_name;
    uint64_t value;
    std::string string_value;
};

class InwheelMotorHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(InwheelMotorHardware)


  INWHEEL_MOTOR_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  INWHEEL_MOTOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  INWHEEL_MOTOR_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  INWHEEL_MOTOR_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  INWHEEL_MOTOR_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  INWHEEL_MOTOR_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  INWHEEL_MOTOR_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type reset_command();

  CallbackReturn set_joint_efforts();
  CallbackReturn set_joint_params();

  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::vector<uint8_t> can_ids_;
  std::vector<double> motor_constant_;
  bool torque_enabled_{false};
  bool mode_changed_{false};
  bool use_dummy_{false};
  bool debug_mode_{false};
  float damping_value;

  std::shared_ptr<inwheel_motor::InwheelMotorCAN> can_communication_;
    // CAN 메시지 맵
  std::unordered_map<int, CANMessage> can_message_map_;
  std::vector<ODriveEnumValue> odrive_enums_;
  std::unordered_map<std::string, std::unordered_map<uint64_t, std::string>> enum_map_;
  std::vector<std::string> matchEnums(const std::string& enum_name, uint64_t value);


  std::string GetErrorString(uint32_t error_code, const std::string& error_type);

  double convertCanDataToPosition(const uint8_t *data);
  double convertCanDataToVelocity(const uint8_t *data);
  double convertCanDataToEffort(const uint8_t *data);

  void HandleReceivedFrame(const struct can_frame& frame);
  bool InitializeCanInterface(const char* ifname, int &socket_fd);
  void GetEncoderEstimates(const struct can_frame& frame);
  void GetIqData(const struct can_frame& frame);
  void GetBusVoltage(const struct can_frame& frame);

  void SendMotorVelCommands(int socket_fd, int node_id, float velocity = 0.0f, float torque_ff = 0.0f);
  void SetControllerMode(int socket_fd, int node_id, uint32_t control_mode, uint32_t input_mode);
  void SendReqStateChange(int socket_fd, int node_id, uint32_t state);
  void SetVelocityGains(int socket_fd, int node_id, float velocity_gain, float velocity_integrator_gain);
  void SendEstopCommand(int socket_fd, int node_id, float estop);

  void SendClearErrorsCommand(int socket_fd, int node_id);

  std::tuple<double, double, double, double, double, double>
  CalculateHipLinearMap(
    double linear_1,
    double linear_2,
    double linear_1_dot,
    double linear_2_dot,
    double linear_1_effort,
    double linear_2_effort);


  std::tuple<double, double, double, double, double, double>
  CalculateAnkleLinearMap(
    double linear_1,
    double linear_2,
    double linear_1_dot,
    double linear_2_dot,
    double linear_1_effort,
    double linear_2_effort);

  // YAML 파일을 로드하여 CAN 메시지 구성 정보를 읽어오는 함수
  bool LoadConfig(const std::string& config_path);
  void LoadODriveEnums(const std::string& config_path);
  void PrintParsedError(uint32_t error_code, const std::string& error_type);
  uint64_t parseUnsignedInt(const uint8_t* data, int start_byte, int bit_length);
  
  int can_socket;
  bool motordriver_error_flag = false;
  // bool heartbeat_error_flag = false;
  std::unordered_map<int, bool> heartbeat_error_flags_;

  std::unordered_map<int, float> current_positions_;
  std::unordered_map<int, bool> received_flags_;
  std::vector<int> node_ids_, node_ids_can0_, node_ids_can1_;
  std::vector<std::string> motordriver_error_flag_names = {
      "Motor Error Flag", "Encoder Error Flag","Controller Error Flag"
  };
  std::thread receive_thread_;
  static const std::unordered_map<int, std::string> joint_name_to_id;
  std::unordered_map<int, uint8_t> axis_states_;



};
}  // namespace inwheel_motor_hardware

#endif  // INWHEEL_MOTOR_HARDWARE__INWHEEL_MOTOR_HARDWARE_HPP_