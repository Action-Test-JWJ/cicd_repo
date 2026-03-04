#ifndef PROTOCOL_STATUS_HPP
#define PROTOCOL_STATUS_HPP

#include <cstdint>
#include <iostream>
#include <vector>
#include "edie_hardware/serial_comms.hpp"
#include "edie_hardware/protocol.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"
#include "edie_msgs/srv/get_pid_gain.hpp"

class StatusRequester
{
public:
    StatusRequester();
    ~StatusRequester();
    void ReadStatusInfo(SerialComms &comms, uint8_t &enabled, float &battery_voltage, 
                                        int16_t &l_wheel_rpm, int16_t &r_wheel_rpm,
                                        int16_t &l_wheel_pos_enc, int16_t &r_wheel_pos_enc,
                                        int32_t &l_leg_pos_enc, int32_t &r_leg_pos_enc,
                                        int16_t &l_ear_pos_enc, int16_t &r_ear_pos_enc,
                                        bool &l_leg_limit_sw, bool &r_leg_limit_sw,
                                        std::vector<int16_t> &fsr_values);
    void RequestWheelsPidGains(SerialComms &comms, float &left_p, float &left_i, float &left_d, float &right_p, float &right_i, float &right_d);
    void RequestLegsPidGains(SerialComms &comms, float &left_p, float &left_i, float &left_d, float &right_p, float &right_i, float &right_d);
    void RequestEarsPidGains(SerialComms &comms, float &left_p, float &left_i, float &left_d, float &right_p, float &right_i, float &right_d);
};

#endif // PROTOCOL_STATUS_HPP