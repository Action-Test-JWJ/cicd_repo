#include "inwheel_motor_interface/inwheel_motor_can.hpp"
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace inwheel_motor
{
  InwheelMotorCAN::~InwheelMotorCAN()
  {
    for (int socket_fd : can_sockets_)
    {
      if (socket_fd != -1)
      {
        close(socket_fd);
      }
    }
  }

  bool InwheelMotorCAN::InitializeCanInterfaces(const std::vector<std::string>& ifnames)
  {
    for (const auto& ifname : ifnames)
    {
      int socket_fd;
      if (!InitializeCanInterface(ifname.c_str(), socket_fd))
      {
        throw std::runtime_error("Error initializing CAN interface: " + ifname);
      }
      can_sockets_.push_back(socket_fd);
    }
    return true;
  }

  bool InwheelMotorCAN::InitializeCanInterface(const char* ifname, int &socket_fd)
  {
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0)
    {
      return false;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      std::cout << "bind error" << std::endl;
      return false;
    }

    struct timeval timeout = {0, 4000};
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    return true;
  }

  void InwheelMotorCAN::send_can_packet(int socket_fd, int can_id, const uint8_t *data, uint8_t data_length)
  {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data_length;
    std::memcpy(frame.data, data, data_length);

    if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
      throw std::runtime_error("Error in sending CAN packet");
    }
  }

  void InwheelMotorCAN::set_position(int socket_fd, int node_id, float position, float velocity_ff, float torque_ff)
  {
    struct can_frame frame;
    frame.can_id = SET_INPUT_POS_CMD | (node_id << 5);
    frame.can_dlc = 8;
    struct {
      float pos;
      int16_t vel_ff;
      int16_t torque_ff;
    } data = {position, static_cast<int16_t>(velocity_ff * 1000), static_cast<int16_t>(torque_ff * 1000)};
    std::memcpy(frame.data, &data, sizeof(data));
    if (write(socket_fd, &frame, sizeof(frame)) != sizeof(frame))
    {
      throw std::runtime_error("Error in sending position command");
    }
  }

  std::array<float, 8> InwheelMotorCAN::receive_can_data()
  {
    struct can_frame frame;
    fd_set read_fds;
    int max_fd = -1;

    FD_ZERO(&read_fds);
    for (int socket_fd : can_sockets_)
    {
      FD_SET(socket_fd, &read_fds);
      if (socket_fd > max_fd)
      {
        max_fd = socket_fd;
      }
    }

    int activity = select(max_fd + 1, &read_fds, NULL, NULL, NULL);
    if (activity < 0 && errno != EINTR)
    {
      throw std::runtime_error("select() error: " + std::string(strerror(errno)));
    }

    for (int socket_fd : can_sockets_)
    {
      if (FD_ISSET(socket_fd, &read_fds))
      {
        int nbytes = read(socket_fd, &frame, sizeof(frame));
        if (nbytes > 0)
        {
          HandleReceivedFrame(frame);
        }
        else if (nbytes < 0)
        {
          throw std::runtime_error("Error reading CAN frame: " + std::string(strerror(errno)));
        }
      }
    }

    return force_data_;
  }

  int InwheelMotorCAN::get_can_socket(int index) const
  {
    if (index < 0 || index >= static_cast<int>(can_sockets_.size()))
    {
      throw std::out_of_range("Invalid CAN socket index");
    }
    return can_sockets_[index];
  }

  void InwheelMotorCAN::HandleReceivedFrame(const struct can_frame& frame)
  {
    int node_id = (frame.can_id >> 5) & 0xFF;

    if ((frame.can_id & 0x1F) == 0x14)
    {
      process_iq_data(frame);
    }
    else if ((frame.can_id & 0x1F) == 0x09)
    {
      process_position_data(frame, node_id);
    }
    else if (frame.can_id == 0x81 || frame.can_id == 0x83)
    {
      process_sensor_data(frame);
    }
    else if (frame.can_id == 0x80 || frame.can_id == 0x82)
    {
      process_heartbeat_data(frame);
    }
  }

  void InwheelMotorCAN::process_iq_data(const struct can_frame& frame)
  {
    if (frame.can_dlc == 8)
    {
      float iq_measured;
      std::memcpy(&iq_measured, frame.data + 4, sizeof(iq_measured));
      iq_values_[frame.can_id >> 5] = iq_measured;
    }
    else
    {
      throw std::runtime_error("Received Iq message with incorrect data length");
    }
  }

  void InwheelMotorCAN::process_position_data(const struct can_frame& frame, int node_id)
  {
    float position;
    std::memcpy(&position, frame.data, sizeof(position));
    current_positions_[node_id] = position;
    received_flags_[node_id] = true;
  }

  void InwheelMotorCAN::process_sensor_data(const struct can_frame& frame)
  {
    if (frame.can_dlc == 8)
    {
      if (frame.can_id == 0x81)
      {
        force_data_[0] = static_cast<float>((frame.data[0] << 8) | frame.data[1]);
        force_data_[1] = static_cast<float>((frame.data[2] << 8) | frame.data[3]);
        force_data_[2] = static_cast<float>((frame.data[4] << 8) | frame.data[5]);
        force_data_[3] = static_cast<float>((frame.data[6] << 8) | frame.data[7]);
      }
      else if (frame.can_id == 0x83)
      {
        force_data_[4] = static_cast<float>((frame.data[0] << 8) | frame.data[1]);
        force_data_[5] = static_cast<float>((frame.data[2] << 8) | frame.data[3]);
        force_data_[6] = static_cast<float>((frame.data[4] << 8) | frame.data[5]);
        force_data_[7] = static_cast<float>((frame.data[6] << 8) | frame.data[7]);
      }
    }
    else
    {
      throw std::runtime_error("Received sensor message with incorrect data length");
    }
  }

  void InwheelMotorCAN::process_heartbeat_data(const struct can_frame& frame)
  {
    if (frame.can_dlc == 1)
    {
      heartbeat_ = frame.data[0] != 0;
    }
    else
    {
      throw std::runtime_error("Received heartbeat message with incorrect data length");
    }
  }

  double InwheelMotorCAN::convertCanDataToPosition(const uint8_t *data)
  {
    int32_t raw_position;
    std::memcpy(&raw_position, data, sizeof(raw_position));
    return static_cast<double>(raw_position);
  }

  double InwheelMotorCAN::convertCanDataToVelocity(const uint8_t *data)
  {
    int32_t raw_velocity;
    std::memcpy(&raw_velocity, data, sizeof(raw_velocity));
    return static_cast<double>(raw_velocity);
  }

  double InwheelMotorCAN::convertCanDataToEffort(const uint8_t *data)
  {
    int32_t raw_effort;
    std::memcpy(&raw_effort, data, sizeof(raw_effort));
    return static_cast<double>(raw_effort);
  }
}
