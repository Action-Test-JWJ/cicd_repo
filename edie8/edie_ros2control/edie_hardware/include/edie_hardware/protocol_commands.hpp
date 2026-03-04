#ifndef PROTOCOL_COMMANDS_HPP
#define PROTOCOL_COMMANDS_HPP

#include <cstdint>
#include <iostream>
#include <vector>
#include "edie_hardware/serial_comms.hpp"
#include "edie_hardware/protocol.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"
#include "edie_msgs/srv/set_pid_gain.hpp"
class CommandSender
{
public:
    CommandSender();
    ~CommandSender();
    void SendMotorCommand(SerialComms &comms, uint8_t motor_enable, int32_t l_whl_vel, int32_t r_whl_vel, int32_t l_leg_pos, int32_t r_leg_pos, int32_t l_ear_pos, int32_t r_ear_pos);  // 0x02
    bool SetLeftWheelPidGains(SerialComms &comms, float p, float i, float d);                                                                                            // 0x04
    bool SetRightWheelPidGains(SerialComms &comms, float p, float i, float d);                                                                                           // 0x05
    bool SetLeftLegsPidGains(SerialComms &comms, float p, float i, float d);                                                                                             // 0x06
    bool SetRightLegsPidGains(SerialComms &comms, float p, float i, float d);                                                                                            // 0x07
    bool SetLeftEarsPidGains(SerialComms &comms, float p, float i, float d);                                                                                             // 0x08
    bool SetRightEarsPidGains(SerialComms &comms, float p, float i, float d);                                                                                            // 0x09
};

#endif // PROTOCOL_COMMANDS_HPP