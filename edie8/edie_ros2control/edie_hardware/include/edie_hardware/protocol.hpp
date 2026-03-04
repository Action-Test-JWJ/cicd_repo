#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

enum Protocol
{
    k_motor_cmd = 0x02,
    k_set_l_wheel_pid_gain = 0x03,
    k_set_r_wheel_pid_gain = 0x04,
    k_set_l_leg_pid_gain = 0x05,
    k_set_r_leg_pid_gain = 0x06,
    k_set_l_ear_pid_gain = 0x07,
    k_set_r_ear_pid_gain = 0x08,
    k_get_wheels_pid_gain = 0x09,
    k_get_legs_pid_gain = 0x0a,
    k_get_ears_pid_gain = 0x0b,
    k_check_motor_spi = 0x0c,
};

#endif // PROTOCOL_HPP