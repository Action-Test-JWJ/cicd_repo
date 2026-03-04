#include "edie_hardware/protocol_status.hpp"
#include "edie_hardware/serial_comms.hpp"

StatusRequester::StatusRequester()
{}

StatusRequester::~StatusRequester()
{}

// void StatusRequester::ReadStatusNonBlock(SerialComms &comms, uint8_t &enabled, float &battery_voltage,

void StatusRequester::ReadStatusInfo(SerialComms &comms, uint8_t &enabled, float &battery_voltage,
                                            int16_t &l_wheel_rpm, int16_t &r_wheel_rpm,
                                            int16_t &l_wheel_pos_enc, int16_t &r_wheel_pos_enc,
                                            int32_t &l_leg_pos_enc, int32_t &r_leg_pos_enc,
                                            int16_t &l_ear_pos_enc, int16_t &r_ear_pos_enc,
                                            bool &l_leg_limit_sw, bool &r_leg_limit_sw,
                                            std::vector<int16_t> &fsr_values)
{
    const uint8_t recv_buf_size = 68;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolStatus"), "Exceptions: \033[91m%s\033[0m", e.what());
        return;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) 
    {
        if (recv_buf[2] == 0x92)
        {
            // 데이터 처리
            union {
                float f;
                uint32_t u32;
            } voltage_union;

            enabled = recv_buf[3];
            voltage_union.u32 = (uint32_t)((recv_buf[4] << 24) | (recv_buf[5] << 16) | (recv_buf[6] << 8) | recv_buf[7]);
            battery_voltage = voltage_union.f;
            l_wheel_rpm = (int32_t)((recv_buf[8] << 24) | (recv_buf[9] << 16) | (recv_buf[10] << 8) | recv_buf[11]);
            r_wheel_rpm = (int32_t)((recv_buf[12] << 24) | (recv_buf[13] << 16) | (recv_buf[14] << 8) | recv_buf[15]);
            l_wheel_pos_enc = (int32_t)((recv_buf[16] << 24) | (recv_buf[17] << 16) | (recv_buf[18] << 8) | recv_buf[19]);
            r_wheel_pos_enc = (int32_t)((recv_buf[20] << 24) | (recv_buf[21] << 16) | (recv_buf[22] << 8) | recv_buf[23]);
            l_leg_pos_enc = (int32_t)((recv_buf[24] << 24) | (recv_buf[25] << 16) | (recv_buf[26] << 8) | recv_buf[27]);
            r_leg_pos_enc = (int32_t)((recv_buf[28] << 24) | (recv_buf[29] << 16) | (recv_buf[30] << 8) | recv_buf[31]);
            l_ear_pos_enc = (int32_t)((recv_buf[32] << 24) | (recv_buf[33] << 16) | (recv_buf[34] << 8) | recv_buf[35]);
            r_ear_pos_enc = (int32_t)((recv_buf[36] << 24) | (recv_buf[37] << 16) | (recv_buf[38] << 8) | recv_buf[39]);
            l_leg_limit_sw = (bool)(recv_buf[40]);
            r_leg_limit_sw = (bool)(recv_buf[41]);
            fsr_values.clear();
            for (int i = 0; i < 12; i++)
            {
                int16_t fsr_value = (int16_t)((recv_buf[2*i + 42] << 8) | recv_buf[2*i + 43]);
                fsr_values.push_back(fsr_value);
            }
        }
        else
        {
            return ;
        }
    }
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolStatus"), "Received invalid frame");
        comms.FlushIOBuffers();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void StatusRequester::RequestWheelsPidGains(SerialComms &comms, float &left_p, float &left_i, float &left_d, float &right_p, float &right_i, float &right_d)
{
    // request
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    send_buf[2] = Protocol::k_get_wheels_pid_gain;

    const bool need_res = true;
    const uint8_t send_buf_size = send_buf.size();
    const uint8_t length = send_buf_size - 6;
    send_buf[send_buf_size - 5] = need_res;
    send_buf[send_buf_size - 4] = length;

    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[send_buf_size - 3] = (uint8_t)checksum;
    send_buf[send_buf_size - 2] = 0xfa;
    send_buf[send_buf_size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolStatus"), "Exceptions: \033[91m%s\033[0m", e.what());
        return;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    // if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
    //     // 데이터 처리
    //     left_p = (float)((recv_buf[3] << 24) | (recv_buf[4] << 16) | (recv_buf[5] << 8) | recv_buf[6]);
    //     left_i = (float)((recv_buf[7] << 24) | (recv_buf[8] << 16) | (recv_buf[9] << 8) | recv_buf[10]);
    //     left_d = (float)((recv_buf[11] << 24) | (recv_buf[12] << 16) | (recv_buf[13] << 8) | recv_buf[14]);
    //     right_p = (float)((recv_buf[15] << 24) | (recv_buf[16] << 16) | (recv_buf[17] << 8) | recv_buf[18]);
    //     right_i = (float)((recv_buf[19] << 24) | (recv_buf[20] << 16) | (recv_buf[21] << 8) | recv_buf[22]);
    //     right_d = (float)((recv_buf[23] << 24) | (recv_buf[24] << 16) | (recv_buf[25] << 8) | recv_buf[26]);
    // } else {
    //     // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    // }
    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) 
    {
        if (recv_buf[2] == 0x93)
        {
            union {
                float f;
                uint32_t u32;
            } left_k_p;

            union {
                float f;
                uint32_t u32;
            } left_k_i;

            union {
                float f;
                uint32_t u32;
            } left_k_d;

            union {
                float f;
                uint32_t u32;
            } right_k_p;

            union {
                float f;
                uint32_t u32;
            } right_k_i;

            union {
                float f;
                uint32_t u32;
            } right_k_d;

            left_k_p.u32 = (uint32_t)((recv_buf[3] << 24) | (recv_buf[4] << 16) | (recv_buf[5] << 8) | recv_buf[6]);
            left_k_i.u32 = (uint32_t)((recv_buf[7] << 24) | (recv_buf[8] << 16) | (recv_buf[9] << 8) | recv_buf[10]);
            left_k_d.u32 = (uint32_t)((recv_buf[11] << 24) | (recv_buf[12] << 16) | (recv_buf[13] << 8) | recv_buf[14]);

            right_k_p.u32 = (uint32_t)((recv_buf[15] << 24) | (recv_buf[16] << 16) | (recv_buf[17] << 8) | recv_buf[18]);
            right_k_i.u32 = (uint32_t)((recv_buf[19] << 24) | (recv_buf[20] << 16) | (recv_buf[21] << 8) | recv_buf[22]);
            right_k_d.u32 = (uint32_t)((recv_buf[23] << 24) | (recv_buf[24] << 16) | (recv_buf[25] << 8) | recv_buf[26]);

            ROS_RED_STREAM("\nWheel PID Gains: \n" << "- left_p: " << left_k_p.f << "\n" << "- left_i: " << left_k_i.f << "\n" << "- left_d: " << left_k_d.f << "\n" 
                                                    << "- right_p: " << right_k_p.f << "\n" << "- right_i: " << right_k_i.f << "\n" << "- right_d: " << right_k_d.f);

            left_p = left_k_p.f;
            left_i = left_k_i.f;
            left_d = left_k_d.f;
            right_p = right_k_p.f;
            right_i = right_k_i.f;
            right_d = right_k_d.f;
        }
        else
        {
            return;
        }
    }
    else
    {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }
}

void StatusRequester::RequestLegsPidGains(SerialComms &comms, float &left_p, float &left_i, float &left_d, float &right_p, float &right_i, float &right_d)
{
    // request
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    send_buf[2] = Protocol::k_get_legs_pid_gain;

    const bool need_res = true;
    const uint8_t send_buf_size = send_buf.size();
    const uint8_t length = send_buf_size - 6;
    send_buf[send_buf_size - 5] = need_res;
    send_buf[send_buf_size - 4] = length;

    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[send_buf_size - 3] = (uint8_t)checksum;
    send_buf[send_buf_size - 2] = 0xfa;
    send_buf[send_buf_size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolStatus"), "Exceptions: \033[91m%s\033[0m", e.what());
        return;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
        // 데이터 처리
        left_p = (float)((recv_buf[3] << 24) | (recv_buf[4] << 16) | (recv_buf[5] << 8) | recv_buf[6]);
        left_i = (float)((recv_buf[7] << 24) | (recv_buf[8] << 16) | (recv_buf[9] << 8) | recv_buf[10]);
        left_d = (float)((recv_buf[11] << 24) | (recv_buf[12] << 16) | (recv_buf[13] << 8) | recv_buf[14]);
        right_p = (float)((recv_buf[15] << 24) | (recv_buf[16] << 16) | (recv_buf[17] << 8) | recv_buf[18]);
        right_i = (float)((recv_buf[19] << 24) | (recv_buf[20] << 16) | (recv_buf[21] << 8) | recv_buf[22]);
        right_d = (float)((recv_buf[23] << 24) | (recv_buf[24] << 16) | (recv_buf[25] << 8) | recv_buf[26]);
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }

    ROS_RED_STREAM("\nLegs PID Gains: \n" << "- left_p: " << left_p << "\n" << "- left_i: " << left_i << "\n" << "- left_d: " << left_d << "\n" 
                                            << "- right_p: " << right_p << "\n" << "- right_i: " << right_i << "\n" << "- right_d: " << right_d);
}

void StatusRequester::RequestEarsPidGains(SerialComms &comms, float &left_p, float &left_i, float &left_d, float &right_p, float &right_i, float &right_d)
{
    // request
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    send_buf[2] = Protocol::k_get_ears_pid_gain;

    const bool need_res = true;
    const uint8_t send_buf_size = send_buf.size();
    const uint8_t length = send_buf_size - 6;
    send_buf[send_buf_size - 5] = need_res;
    send_buf[send_buf_size - 4] = length;

    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[send_buf_size - 3] = (uint8_t)checksum;
    send_buf[send_buf_size - 2] = 0xfa;
    send_buf[send_buf_size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolStatus"), "Exceptions: \033[91m%s\033[0m", e.what());
        return;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
        // 데이터 처리
        left_p = (float)((recv_buf[3] << 24) | (recv_buf[4] << 16) | (recv_buf[5] << 8) | recv_buf[6]);
        left_i = (float)((recv_buf[7] << 24) | (recv_buf[8] << 16) | (recv_buf[9] << 8) | recv_buf[10]);
        left_d = (float)((recv_buf[11] << 24) | (recv_buf[12] << 16) | (recv_buf[13] << 8) | recv_buf[14]);
        right_p = (float)((recv_buf[15] << 24) | (recv_buf[16] << 16) | (recv_buf[17] << 8) | recv_buf[18]);
        right_i = (float)((recv_buf[19] << 24) | (recv_buf[20] << 16) | (recv_buf[21] << 8) | recv_buf[22]);
        right_d = (float)((recv_buf[23] << 24) | (recv_buf[24] << 16) | (recv_buf[25] << 8) | recv_buf[26]);
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }

    ROS_RED_STREAM("\nEars PID Gains: \n" << "- left_p: " << left_p << "\n" << "- left_i: " << left_i << "\n" << "- left_d: " << left_d << "\n" 
                                            << "- right_p: " << right_p << "\n" << "- right_i: " << right_i << "\n" << "- right_d: " << right_d);
}