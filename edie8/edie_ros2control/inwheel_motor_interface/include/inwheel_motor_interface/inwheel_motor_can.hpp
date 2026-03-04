#ifndef INWHEEL_MOTOR_HARDWARE__INWHEEL_MOTOR_CAN_HPP_
#define INWHEEL_MOTOR_HARDWARE__INWHEEL_MOTOR_CAN_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <array>
#include <unordered_map>

namespace inwheel_motor
{
class InwheelMotorCAN
{
public:
  InwheelMotorCAN() = default;
  ~InwheelMotorCAN();

  // 다수의 CAN 인터페이스 초기화
  bool InitializeCanInterfaces(const std::vector<std::string>& ifnames);

  // CAN 패킷 전송 함수
  void send_can_packet(int socket_fd, int can_id, const uint8_t *data, uint8_t data_length);

  // 모터 위치 설정 함수
  void set_position(int socket_fd, int node_id, float position, float velocity_ff = 0.0, float torque_ff = 0.0);

  // CAN 데이터 수신 및 처리 함수
  std::array<float, 8> receive_can_data();

  // CAN 소켓을 얻어오는 함수 (수정된 부분)
  int get_can_socket(int index) const;

  // CAN 데이터 변환 함수
  double convertCanDataToPosition(const uint8_t *data);
  double convertCanDataToVelocity(const uint8_t *data);
  double convertCanDataToEffort(const uint8_t *data);

private:
  // 개별 CAN 인터페이스 초기화 함수
  bool InitializeCanInterface(const char* ifname, int &socket_fd);

  // 수신된 CAN 프레임 처리 함수
  void HandleReceivedFrame(const struct can_frame& frame);

  // 수신된 Iq 데이터 처리 함수
  void process_iq_data(const struct can_frame& frame);

  // 수신된 위치 데이터 처리 함수
  void process_position_data(const struct can_frame& frame, int node_id);

  // 수신된 센서 데이터 처리 함수
  void process_sensor_data(const struct can_frame& frame);

  // 수신된 하트비트 데이터 처리 함수
  void process_heartbeat_data(const struct can_frame& frame);

  // CAN 소켓을 저장하는 벡터
  std::vector<int> can_sockets_;

  // Iq 값을 저장하는 맵
  std::unordered_map<int, float> iq_values_;

  // 현재 위치를 저장하는 맵
  std::unordered_map<int, float> current_positions_;

  // 목표 위치를 수신했는지 여부를 저장하는 플래그 맵
  std::unordered_map<int, bool> received_flags_;

  // 센서 데이터를 저장하는 배열
  std::array<float, 8> force_data_ = {0.0f};

  // 하트비트 상태를 저장하는 변수
  bool heartbeat_ = false;

  // CAN 명령 상수
  static constexpr int SET_INPUT_POS_CMD = 0x00C; // 모터 위치 설정 명령
  static constexpr int SET_AXIS_REQUESTED_STATE_CMD = 0x007; // 모터 상태 설정 명령
};
}  // namespace linear_actuator

#endif  // INWHEEL_MOTOR_HARDWARE__INWHEEL_MOTOR_CAN_HPP_
