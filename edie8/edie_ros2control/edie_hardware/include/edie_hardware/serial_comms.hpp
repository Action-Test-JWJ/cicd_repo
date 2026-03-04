// serial_comms.hpp
#pragma once
#include <libserial/SerialPort.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

struct StatusFrame {
  uint8_t enabled{};
  float   battery_voltage{};
  int16_t l_wheel_rpm{}, r_wheel_rpm{};
  int16_t l_wheel_pos_enc{}, r_wheel_pos_enc{};
  int32_t l_leg_pos_enc{},  r_leg_pos_enc{};
  int16_t l_ear_pos_enc{},  r_ear_pos_enc{};
  bool    l_leg_limit_sw{}, r_leg_limit_sw{};
  std::array<int16_t, 12> fsr{};
  bool    is_plug_charging{}, is_station_charging{};
};

inline LibSerial::BaudRate convert_baud_rate(int32_t baud_rate)
{
  switch (baud_rate) {
    case 1200:   return LibSerial::BaudRate::BAUD_1200;
    case 1800:   return LibSerial::BaudRate::BAUD_1800;
    case 2400:   return LibSerial::BaudRate::BAUD_2400;
    case 4800:   return LibSerial::BaudRate::BAUD_4800;
    case 9600:   return LibSerial::BaudRate::BAUD_9600;
    case 19200:  return LibSerial::BaudRate::BAUD_19200;
    case 38400:  return LibSerial::BaudRate::BAUD_38400;
    case 57600:  return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class SerialComms {
public:
  void Connect(const std::string &serial_device, int32_t baud_rate);
  void Disconnect();
  bool IsConnected() const;

  // 기존 동기 TX
  void WriteByte(const uint8_t& byte);
  void Write(const std::vector<uint8_t>& data);
  void DrainWriteBuffer();
  void FlushIOBuffers();
  void Read(std::vector<uint8_t>& data, size_t size); // ← 기존 블로킹 (RX 스레드 쓰면 사용 지양)

  static void FloatToBytes(float gain, uint8_t* buffer);

  // === 여기부터 비동기 RX 인터페이스 ===
  void StartAsyncRx();
  void StopAsyncRx();
  bool TryGetLatest(StatusFrame& out);  // 스냅샷 복사 (논블로킹)

private:
  // RX 루프 & 파서
  void RxThreadLoop_();
  void ParseFrames_();

  static inline uint32_t be_u32_(const uint8_t* p) {
    return (uint32_t(p[0])<<24) | (uint32_t(p[1])<<16) | (uint32_t(p[2])<<8) | uint32_t(p[3]);
  }
  static inline int16_t be_i16_(const uint8_t* p) {
    return int16_t((int16_t(p[0])<<8) | int16_t(p[1]));
  }

private:
  LibSerial::SerialPort serial_;

  // === 비동기 RX 상태 ===
  std::thread rx_thread_;
  std::atomic<bool> running_{false};
  std::vector<uint8_t> fifo_; 
  static constexpr size_t FRAME_SIZE = 73;

  // 최신 스냅샷
  std::mutex latest_mtx_;
  StatusFrame latest_{};
  bool has_latest_{false};
};
