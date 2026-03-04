#include "edie_hardware/protocol_commands.hpp"

CommandSender::CommandSender()
{}

CommandSender::~CommandSender()
{}

void CommandSender::SendMotorCommand(SerialComms &comms, uint8_t motor_enable,
                                    int32_t l_whl_vel, int32_t r_whl_vel, 
                                    int32_t l_leg_pos, int32_t r_leg_pos, 
                                    int32_t l_ear_pos, int32_t r_ear_pos)
{
    const uint8_t send_buf_size = 33;
    std::vector<uint8_t> send_buf(send_buf_size, 0);

    send_buf[0] = 0xfa; // Header
    send_buf[1] = 0xfe; // Header
    
    send_buf[2] = Protocol::k_motor_cmd;
    send_buf[3] = motor_enable;

    send_buf[4] = (uint8_t)((int32_t)l_whl_vel >> 24);
    send_buf[5] = (uint8_t)((int32_t)l_whl_vel >> 16);
    send_buf[6] = (uint8_t)((int32_t)l_whl_vel >> 8);
    send_buf[7] = (uint8_t)((int32_t)l_whl_vel);

    send_buf[8] = (uint8_t)((int32_t)r_whl_vel >> 24);
    send_buf[9] = (uint8_t)((int32_t)r_whl_vel >> 16);
    send_buf[10] = (uint8_t)((int32_t)r_whl_vel >> 8);
    send_buf[11] = (uint8_t)((int32_t)r_whl_vel);

    send_buf[12] = (uint8_t)((int32_t)l_leg_pos >> 24);
    send_buf[13] = (uint8_t)((int32_t)l_leg_pos >> 16);
    send_buf[14] = (uint8_t)((int32_t)l_leg_pos >> 8);
    send_buf[15] = (uint8_t)((int32_t)l_leg_pos);

    send_buf[16] = (uint8_t)((int32_t)r_leg_pos >> 24);
    send_buf[17] = (uint8_t)((int32_t)r_leg_pos >> 16);
    send_buf[18] = (uint8_t)((int32_t)r_leg_pos >> 8);
    send_buf[19] = (uint8_t)((int32_t)r_leg_pos);

    send_buf[20] = (uint8_t)((int32_t)l_ear_pos >> 24);
    send_buf[21] = (uint8_t)((int32_t)l_ear_pos >> 16);
    send_buf[22] = (uint8_t)((int32_t)l_ear_pos >> 8);
    send_buf[23] = (uint8_t)((int32_t)l_ear_pos);

    send_buf[24] = (uint8_t)((int32_t)r_ear_pos >> 24);
    send_buf[25] = (uint8_t)((int32_t)r_ear_pos >> 16);
    send_buf[26] = (uint8_t)((int32_t)r_ear_pos >> 8);
    send_buf[27] = (uint8_t)((int32_t)r_ear_pos);

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;

    send_buf[size - 2] = 0xfa; // Footer
    send_buf[size - 1] = 0xfd; // Footer

    // Print the content of send_buf
    // std::cout << "Sent data: ";
    // for (const auto& byte : send_buf)
    // {
    //      std::cout << std::hex << static_cast<int>(byte) << " ";
    // }
    //  std::cout << std::dec << std::endl; // Reset cout to decimal output

    // std::cout << "Sent data: ";
    // std::cout << l_whl_vel << " " << r_whl_vel << " " << l_leg_pos << " " << r_leg_pos << " " << l_ear_pos << " " << r_ear_pos << std::endl;

    // for (size_t i = 0; i < send_buf.size(); ++i)
    // {
    //     comms.WriteByte(send_buf[i]);
    //     // serial_.DrainWriteBuffer();
    // }

    // comms.DrainWriteBuffer();

    comms.Write(send_buf);
    comms.DrainWriteBuffer();
}

bool CommandSender::SetLeftWheelPidGains(SerialComms &comms, float p, float i, float d) // length 13
{
    ROS_BLUE_STREAM("\nLeft Wheel PID Gains: \n" << "- p: " << p << "\n" << "- i: " << i << "\n" << "- d: " << d);
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x1, 0x2, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    uint8_t p_bytes[4], i_bytes[4], d_bytes[4];
    comms.FloatToBytes(p, p_bytes);
    comms.FloatToBytes(i, i_bytes);
    comms.FloatToBytes(d, d_bytes);

    send_buf[0] = 0xfa;
    send_buf[1] = 0xfe;
    send_buf[2] = Protocol::k_set_l_wheel_pid_gain;
    send_buf[3] = p_bytes[0];
    send_buf[4] = p_bytes[1];
    send_buf[5] = p_bytes[2];
    send_buf[6] = p_bytes[3];
    send_buf[7] = i_bytes[0];
    send_buf[8] = i_bytes[1];
    send_buf[9] = i_bytes[2];
    send_buf[10] = i_bytes[3];
    send_buf[11] = d_bytes[0];
    send_buf[12] = d_bytes[1];
    send_buf[13] = d_bytes[2];
    send_buf[14] = d_bytes[3];

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;
    send_buf[size - 2] = 0xfa;
    send_buf[size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    // const uint8_t recv_buf_size = 54;
    // std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    bool set_success = false;
    
    // try
    // {
    //     comms.Read(recv_buf, recv_buf_size);
    // }
    // catch(LibSerial::ReadTimeout &e)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("ProtocolCommander"), "Exceptions: \033[91m%s\033[0m", e.what());
    //     return set_success;
    // }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    // if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
    //     // 데이터 처리
    //     set_success = recv_buf[3];
    // } else {
    //     // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    // }

    return set_success;
}

bool CommandSender::SetRightWheelPidGains(SerialComms &comms, float p, float i, float d)
{
    ROS_BLUE_STREAM("\nRight Wheel PID Gains: \n" << "- p: " << p << "\n" << "- i: " << i << "\n" << "- d: " << d);
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x1, 0x2, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    uint8_t p_bytes[4], i_bytes[4], d_bytes[4];
    comms.FloatToBytes(p, p_bytes);
    comms.FloatToBytes(i, i_bytes);
    comms.FloatToBytes(d, d_bytes);

    send_buf[0] = 0xfa;
    send_buf[1] = 0xfe;
    send_buf[2] = Protocol::k_set_r_wheel_pid_gain;
    send_buf[3] = p_bytes[0];
    send_buf[4] = p_bytes[1];
    send_buf[5] = p_bytes[2];
    send_buf[6] = p_bytes[3];
    send_buf[7] = i_bytes[0];
    send_buf[8] = i_bytes[1];
    send_buf[9] = i_bytes[2];
    send_buf[10] = i_bytes[3];
    send_buf[11] = d_bytes[0];
    send_buf[12] = d_bytes[1];
    send_buf[13] = d_bytes[2];
    send_buf[14] = d_bytes[3];

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;
    send_buf[size - 2] = 0xfa;
    send_buf[size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    // const uint8_t recv_buf_size = 54;
    // std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    bool set_success = false;

    // try
    // {
    //     comms.Read(recv_buf, recv_buf_size);
    // }
    // catch(LibSerial::ReadTimeout &e)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("ProtocolCommander"), "Exceptions: \033[91m%s\033[0m", e.what());
    //     return set_success;
    // }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    // if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
    //     // 데이터 처리
    //     set_success = recv_buf[3];
    // } else {
    //     // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    // }

    return set_success;
}

bool CommandSender::SetLeftLegsPidGains(SerialComms &comms, float p, float i, float d)
{
    ROS_BLUE_STREAM("\nLeft Leg PID Gains: \n" << "- p: " << p << "\n" << "- i: " << i << "\n" << "- d: " << d);
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x1, 0x2, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    uint8_t p_bytes[4], i_bytes[4], d_bytes[4];
    comms.FloatToBytes(p, p_bytes);
    comms.FloatToBytes(i, i_bytes);
    comms.FloatToBytes(d, d_bytes);

    send_buf[2] = Protocol::k_set_l_leg_pid_gain;
    send_buf[3] = p_bytes[0];
    send_buf[4] = p_bytes[1];
    send_buf[5] = p_bytes[2];
    send_buf[6] = p_bytes[3];
    send_buf[7] = i_bytes[0];
    send_buf[8] = i_bytes[1];
    send_buf[9] = i_bytes[2];
    send_buf[10] = i_bytes[3];
    send_buf[11] = d_bytes[0];
    send_buf[12] = d_bytes[1];
    send_buf[13] = d_bytes[2];
    send_buf[14] = d_bytes[3];

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;
    send_buf[size - 2] = 0xfa;
    send_buf[size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    bool set_success = false;
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolCommander"), "Exceptions: \033[91m%s\033[0m", e.what());
        return set_success;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
        // 데이터 처리
        set_success = recv_buf[3];
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }

    return set_success;
}

bool CommandSender::SetRightLegsPidGains(SerialComms &comms, float p, float i, float d)
{
    ROS_BLUE_STREAM("\nRight Leg PID Gains: \n" << "- p: " << p << "\n" << "- i: " << i << "\n" << "- d: " << d);
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x1, 0x2, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    uint8_t p_bytes[4], i_bytes[4], d_bytes[4];
    comms.FloatToBytes(p, p_bytes);
    comms.FloatToBytes(i, i_bytes);
    comms.FloatToBytes(d, d_bytes);

    send_buf[2] = Protocol::k_set_r_leg_pid_gain;
    send_buf[3] = p_bytes[0];
    send_buf[4] = p_bytes[1];
    send_buf[5] = p_bytes[2];
    send_buf[6] = p_bytes[3];
    send_buf[7] = i_bytes[0];
    send_buf[8] = i_bytes[1];
    send_buf[9] = i_bytes[2];
    send_buf[10] = i_bytes[3];
    send_buf[11] = d_bytes[0];
    send_buf[12] = d_bytes[1];
    send_buf[13] = d_bytes[2];
    send_buf[14] = d_bytes[3];

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;
    send_buf[size - 2] = 0xfa;
    send_buf[size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    bool set_success = false;

    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolCommander"), "Exceptions: \033[91m%s\033[0m", e.what());
        return set_success;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
        // 데이터 처리
        set_success = recv_buf[3];
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }

    return set_success;
}

bool CommandSender::SetLeftEarsPidGains(SerialComms &comms, float p, float i, float d)
{
    ROS_BLUE_STREAM("\nLeft Ear PID Gains: \n" << "- p: " << p << "\n" << "- i: " << i << "\n" << "- d: " << d);
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x1, 0x2, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    uint8_t p_bytes[4], i_bytes[4], d_bytes[4];
    comms.FloatToBytes(p, p_bytes);
    comms.FloatToBytes(i, i_bytes);
    comms.FloatToBytes(d, d_bytes);

    send_buf[2] = Protocol::k_set_l_ear_pid_gain;
    send_buf[3] = p_bytes[0];
    send_buf[4] = p_bytes[1];
    send_buf[5] = p_bytes[2];
    send_buf[6] = p_bytes[3];
    send_buf[7] = i_bytes[0];
    send_buf[8] = i_bytes[1];
    send_buf[9] = i_bytes[2];
    send_buf[10] = i_bytes[3];
    send_buf[11] = d_bytes[0];
    send_buf[12] = d_bytes[1];
    send_buf[13] = d_bytes[2];
    send_buf[14] = d_bytes[3];

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;
    send_buf[size - 2] = 0xfa;
    send_buf[size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    bool set_success = false;
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolCommander"), "Exceptions: \033[91m%s\033[0m", e.what());
        return set_success;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
        // 데이터 처리
        set_success = recv_buf[3];
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }

    return set_success;
}

bool CommandSender::SetRightEarsPidGains(SerialComms &comms, float p, float i, float d)
{
    ROS_BLUE_STREAM("\nRight Ear PID Gains: \n" << "- p: " << p << "\n" << "- i: " << i << "\n" << "- d: " << d);
    std::vector<uint8_t> send_buf {0xfa, 0xfe, 0x0, 0x1, 0x2, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfa, 0xfd};

    uint8_t p_bytes[4], i_bytes[4], d_bytes[4];
    comms.FloatToBytes(p, p_bytes);
    comms.FloatToBytes(i, i_bytes);
    comms.FloatToBytes(d, d_bytes);

    send_buf[2] = Protocol::k_set_r_ear_pid_gain;
    send_buf[3] = p_bytes[0];
    send_buf[4] = p_bytes[1];
    send_buf[5] = p_bytes[2];
    send_buf[6] = p_bytes[3];
    send_buf[7] = i_bytes[0];
    send_buf[8] = i_bytes[1];
    send_buf[9] = i_bytes[2];
    send_buf[10] = i_bytes[3];
    send_buf[11] = d_bytes[0];
    send_buf[12] = d_bytes[1];
    send_buf[13] = d_bytes[2];
    send_buf[14] = d_bytes[3];

    const bool need_res = true;
    const uint8_t size = send_buf.size();
    const uint8_t length = size - 6;
    send_buf[size - 5] = need_res;
    send_buf[size - 4] = length;
    
    uint16_t checksum = 0;
    for(int i = 0; i < length; i++)
    {
        checksum += send_buf[2 + i];
    }

    send_buf[size - 3] = (uint8_t)checksum;
    send_buf[size - 2] = 0xfa;
    send_buf[size - 1] = 0xfd;

    comms.Write(send_buf);
    comms.DrainWriteBuffer();

    // response
    const uint8_t recv_buf_size = 54;
    std::vector<uint8_t> recv_buf(recv_buf_size, 0);
    bool set_success = false;
    
    try
    {
        comms.Read(recv_buf, recv_buf_size);
    }
    catch(LibSerial::ReadTimeout &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ProtocolCommander"), "Exceptions: \033[91m%s\033[0m", e.what());
        return set_success;
    }

    // std::cout << "Received data in hex: ";
    // for (const auto& byte : recv_buf) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)byte << " ";
    // }
    // std::cout << std::endl;

    if (recv_buf[0] == 0xFA && recv_buf[1] == 0xFE && recv_buf[recv_buf_size-2] == 0xFA && recv_buf[recv_buf_size-1] == 0xFD) {
        // 데이터 처리
        set_success = recv_buf[3];
    } else {
        // RCLCPP_ERROR(this->get_logger(), "Received invalid frame");
    }

    return set_success;
}