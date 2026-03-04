#include "edie_hardware/serial_comms.hpp"

void SerialComms::Connect(const std::string &serial_device, int32_t baud_rate)
{  
  try {
    serial_.Open(serial_device);
    serial_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    // 장치가 리셋/배너를 뿌릴 수 있으므로 잠깐 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 입력 버퍼 깔끔히 비우기
    std::vector<uint8_t> buf(1024);
    while (true) {
        const size_t avail = serial_.GetNumberOfBytesAvailable();
        if (avail == 0) break;
        const size_t n_to_read = std::min<size_t>(avail, buf.size());
        buf.resize(n_to_read);
        try {
            serial_.Read(buf, n_to_read, 5);
        } catch (const LibSerial::ReadTimeout&) {
            break;
        }
    }

    // 출력 버퍼 비우기(필요 시)
    serial_.DrainWriteBuffer();
    // 입출력 버퍼 최종 플러시(선택)
    serial_.FlushIOBuffers();
  }
  catch (const std::exception& e) {
    // RCLCPP_ERROR(rclcpp::get_logger("SerialComms"),
    //              "Failed to open/configure serial %s: %s",
    //              serial_device.c_str(), e.what());
    throw; // 상위에서 처리하게 그대로 던짐(원하시면 false 리턴 등으로 바꾸세요)
  }
}

void SerialComms::Disconnect()
{
    serial_.Close();
}

bool SerialComms::IsConnected() const
{
    return serial_.IsOpen();
}

void SerialComms::WriteByte(const uint8_t& byte)
{
    serial_.WriteByte(byte);
}

void SerialComms::Write(const std::vector<uint8_t>& data)
{
    serial_.Write(data);
}

void SerialComms::Read(std::vector<uint8_t>& data, size_t size)
{
    serial_.Read(data, size);
}

void SerialComms::DrainWriteBuffer()
{
    serial_.DrainWriteBuffer();
}

void SerialComms::FlushIOBuffers()
{
    serial_.FlushIOBuffers();
}

void SerialComms::FloatToBytes(float gain, uint8_t* buffer)
{
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&gain);
    for (int i = 0; i < 4; i++)
    {
        buffer[i] = bytes[i];
    }
}

void SerialComms::StartAsyncRx()
{
  if (running_.exchange(true)) return;

  // 버퍼 준비
  fifo_.clear();
  fifo_.reserve(1024);
  {
    std::scoped_lock lk(latest_mtx_);
    has_latest_ = false;
  }

  // RX 스레드 시작
  rx_thread_ = std::thread([this]{ RxThreadLoop_(); });
}

void SerialComms::StopAsyncRx()
{
  if (!running_.exchange(false)) return;
  try { serial_.Close(); } catch (...) {}
  if (rx_thread_.joinable()) rx_thread_.join();
}

bool SerialComms::TryGetLatest(StatusFrame& out)
{
  std::scoped_lock lk(latest_mtx_);
  if (!has_latest_) return false;
  out = latest_;
  return true;
}

void SerialComms::RxThreadLoop_()
{
  using namespace std::chrono_literals;
  std::vector<uint8_t> tmp(256);

  while (running_) {
    try {
      if (!serial_.IsOpen()) {
        std::this_thread::sleep_for(20ms);
        continue;
      }

      // 가용 바이트 확인
      const auto avail = serial_.GetNumberOfBytesAvailable();
      if (avail == 0) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      // 들어온 만큼만 읽기 (짧은 timeout)
      tmp.resize(std::min<size_t>(tmp.size(), avail));
      try {
        const auto avail     = serial_.GetNumberOfBytesAvailable();
        const size_t n_read  = std::min<size_t>(avail, tmp.size());
        tmp.resize(n_read);
        serial_.Read(tmp, n_read, 5);   // 5 ms timeout (정수)
      } catch (const LibSerial::ReadTimeout&) {
        continue; // 정상 케이스
      }

      // 누적 후 파싱
      fifo_.insert(fifo_.end(), tmp.begin(), tmp.end());
      ParseFrames_();
    }
    catch (...) {
      // 포트 에러 등: 짧게 쉬고 재시도
      std::this_thread::sleep_for(10ms);
    }
  }
}

void SerialComms::ParseFrames_()
{
  while (fifo_.size() >= FRAME_SIZE) {
    // 헤더 동기화 (0xFA 0xFE … 0xFA 0xFD)
    if (fifo_[0] != 0xFA || fifo_[1] != 0xFE) {
      fifo_.erase(fifo_.begin());
      continue;
    }
    if (fifo_[FRAME_SIZE-2] != 0xFA || fifo_[FRAME_SIZE-1] != 0xFD) {
      fifo_.erase(fifo_.begin());
      continue;
    }
    if (fifo_[2] != 0x92) {
      fifo_.erase(fifo_.begin());
      continue;
    }

    const uint8_t* fr = fifo_.data();
    StatusFrame s{};

    s.enabled = fr[3];

    union { float f; uint32_t u; } vu;
    vu.u = be_u32_(fr + 4);
    s.battery_voltage = vu.f;

    // ⚠️ 실제 프로토콜이 16비트라면 아래를 be_i16_()으로 바꾸세요.
    s.l_wheel_rpm     = static_cast<int16_t>(be_u32_(fr +  8));
    s.r_wheel_rpm     = static_cast<int16_t>(be_u32_(fr + 12));
    s.l_wheel_pos_enc = static_cast<int16_t>(be_u32_(fr + 16));
    s.r_wheel_pos_enc = static_cast<int16_t>(be_u32_(fr + 20));
    s.l_leg_pos_enc   = static_cast<int32_t>(be_u32_(fr + 24));
    s.r_leg_pos_enc   = static_cast<int32_t>(be_u32_(fr + 28));
    s.l_ear_pos_enc   = static_cast<int16_t>(be_u32_(fr + 32));
    s.r_ear_pos_enc   = static_cast<int16_t>(be_u32_(fr + 36));

    s.l_leg_limit_sw = fr[40] != 0;
    s.r_leg_limit_sw = fr[41] != 0;

    for (int i=0;i<12;i++) {
      s.fsr[i] = be_i16_(fr + 42 + 2*i);
    }

    s.is_plug_charging = fr[66] != 0;
    s.is_station_charging = fr[67] != 0;

    {
      std::scoped_lock lk(latest_mtx_);
      latest_ = s;
      has_latest_ = true;
    }

    // 소비
    fifo_.erase(fifo_.begin(), fifo_.begin() + FRAME_SIZE);
  }
}