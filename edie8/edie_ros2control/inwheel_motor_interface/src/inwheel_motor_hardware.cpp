#include "inwheel_motor_interface/inwheel_motor_hardware.hpp"
#include "inwheel_motor_interface/inwheel_motor_can.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <errno.h>
#include <future> 
#include <thread>
#include <chrono>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/join.hpp"
#include <transmission_interface/simple_transmission.hpp>


constexpr int k_axis_state_idle = 1; 
constexpr int k_axis_state_closed_loop_control = 8; 
constexpr int k_axis_state_homing = 11; 

constexpr int k_set_controller_mode_cmd = 0x00B;
constexpr int k_set_input_pos_cmd = 0x00C;
constexpr int k_set_input_vel_cmd = 0x00D;
constexpr int k_set_axis_requested_state_cmd = 0x007; 
constexpr int k_set_estop_cmd = 0x002;
constexpr int k_set_position_gain_cmd = 0x01A;
constexpr int k_set_vel_gains_cmd = 0x01B; 
constexpr int k_set_heartbeat_message = 0x001;

namespace inwheel_motor_hardware
{
    constexpr const char *kInwheelMotorHardware = "InwheelMotorHardware";

    CallbackReturn InwheelMotorHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(kInwheelMotorHardware), "configure");
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        joints_.resize(info_.joints.size(), Joint());
        joint_ids_.resize(info_.joints.size(), 0);
        can_ids_.resize(info_.joints.size(), 0);
        motor_constant_.resize(info_.joints.size(), 0);

        for (auto id : can_ids_) {
            heartbeat_error_flags_[id] = false;
        }

        auto usb_port = info_.hardware_parameters.at("usb_port");

        //Load CAN configuration
        std::string config_path_can = ament_index_cpp::get_package_share_directory("edie8_parameters") + "/config/can_messages_config.yaml";
        if (!LoadConfig(config_path_can)) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("InwheelMotorHardware"), "Failed to load CAN configuration");
            return CallbackReturn::ERROR;
        }
        
        auto debug_it = info_.hardware_parameters.find("debug_mode");
        debug_mode_ = (debug_it != info_.hardware_parameters.end()) && (debug_it->second == "true");
        RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "Debug mode: %s", debug_mode_ ? "ON" : "OFF");

        for (uint i = 0; i < info_.joints.size(); i++)
        {
            auto id_it = info_.joints[i].parameters.find("id");
            auto can_id_it = info_.joints[i].parameters.find("can_id");

            // model이 만약 버츄얼이면 에러 반환 안 하게 하기
            if (id_it != info_.joints[i].parameters.end())
            {
                joint_ids_[i] = std::stof(id_it->second); 
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "ID not found for joint: %s", info_.joints[i].name.c_str());
                return CallbackReturn::ERROR;
            }

            if (can_id_it != info_.joints[i].parameters.end())
            {
                can_ids_[i] = std::stoi(can_id_it->second);
                RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "CAN ID: %d", can_ids_[i]);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "CAN ID not found for joint: %s", info_.joints[i].name.c_str());
                return CallbackReturn::ERROR;
            }

            // if (motor_constant_it != info_.joints[i].parameters.end()) 
            // {
            //     motor_constant_[i] = std::stof(motor_constant_it->second); 
            // }
            // else
            // {
            //     RCLCPP_ERROR(rclcpp::get_logger(kLinearActuatorHardware), "Pitch Lenth not found for joint: %s", info_.joints[i].name.c_str());
            //     return CallbackReturn::ERROR;
            // }

            joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
            // joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
            joints_[i].state.torque_on_off = std::numeric_limits<double>::quiet_NaN();

            joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
            joints_[i].prev_command.velocity = joints_[i].command.velocity;

            joints_[i].command.velocity_p_gain = 1.5;
            joints_[i].command.velocity_i_gain = 0.001;
            joints_[i].command.velocity_d_gain = 0.0;

            joints_[i].command.torque_p_gain = 0.0;
            joints_[i].command.torque_i_gain = 0.0;
            joints_[i].command.torque_d_gain = 0.0;

            joints_[i].command.estop = 0.0;

            joints_[i].command.torque_on_off = std::numeric_limits<double>::quiet_NaN();

            joints_[i].state.bus_voltage = 0.0;
            joints_[i].state.bus_current = 0.0;

            joints_[i].state.iq_setpoint = 0.0;
            joints_[i].state.iq_measured = 0.0;

            joints_[i].prev_command.velocity_p_gain = joints_[i].command.velocity_p_gain;
            joints_[i].prev_command.velocity_i_gain = joints_[i].command.velocity_i_gain;
            joints_[i].prev_command.velocity_d_gain = joints_[i].command.velocity_d_gain;

            joints_[i].prev_command.torque_p_gain = joints_[i].command.torque_p_gain;
            joints_[i].prev_command.torque_i_gain = joints_[i].command.torque_i_gain;
            joints_[i].prev_command.torque_d_gain = joints_[i].command.torque_d_gain;

            joints_[i].prev_command.estop = joints_[i].command.estop;

            joints_[i].prev_command.torque_on_off = joints_[i].command.torque_on_off;
        }

        if (
            info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
            info_.hardware_parameters.at("use_dummy") == "true")
        {
            use_dummy_ = true;
            RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "dummy mode");
            return CallbackReturn::SUCCESS;
        }

        if (!InitializeCanInterface(usb_port.c_str(), can_socket))
        {
            RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "Error initializing CAN bus");
            rclcpp::shutdown();
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware),
                    "CAN interface initialized successfully. Socket: %d", can_socket);
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> InwheelMotorHardware::export_state_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(kInwheelMotorHardware), "export_state_interfaces");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY_P_GAIN , &joints_[i].state.velocity_p_gain)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY_I_GAIN , &joints_[i].state.velocity_i_gain)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY_D_GAIN , &joints_[i].state.velocity_d_gain)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_P_GAIN , &joints_[i].state.torque_p_gain)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_I_GAIN , &joints_[i].state.torque_i_gain)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_D_GAIN , &joints_[i].state.torque_d_gain)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_BUS_VOLTAGE , &joints_[i].state.bus_voltage)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_BUS_CURRENT , &joints_[i].state.bus_current)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_IQ_SETPOINT , &joints_[i].state.iq_setpoint)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_IQ_MEASURED , &joints_[i].state.iq_measured)); 
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_ON_OFF, &joints_[i].state.torque_on_off)); 
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> InwheelMotorHardware::export_command_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(kInwheelMotorHardware), "export_command_interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY_P_GAIN , &joints_[i].command.velocity_p_gain)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY_I_GAIN , &joints_[i].command.velocity_i_gain)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY_D_GAIN , &joints_[i].command.velocity_d_gain)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_P_GAIN , &joints_[i].command.torque_p_gain)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_I_GAIN , &joints_[i].command.torque_i_gain)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_D_GAIN , &joints_[i].command.torque_d_gain)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ESTOP , &joints_[i].command.estop)); 
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_TORQUE_ON_OFF, &joints_[i].command.torque_on_off)); 
        }

        return command_interfaces;
    }

    CallbackReturn InwheelMotorHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(kInwheelMotorHardware), "start");
        for (uint i = 0; i < joints_.size(); i++)
        {
            if (use_dummy_ && std::isnan(joints_[i].state.position))
            {
                joints_[i].state.position = 0.0;
                joints_[i].state.velocity = 0.0;
                // joints_[i].state.effort = 0.0;
                joints_[i].state.torque_on_off = 0.0;
            }
        }
        read(rclcpp::Time{}, rclcpp::Duration(0, 0));
        write(rclcpp::Time{}, rclcpp::Duration(0, 0));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn InwheelMotorHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(kInwheelMotorHardware), "Deactivating and turning off motor torques");

        return CallbackReturn::SUCCESS;
    }

    return_type InwheelMotorHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        if (use_dummy_)
        {
        return return_type::OK;
        }

        // 논블로킹 모드: 데이터가 없으면 즉시 EAGAIN/EWOULDBLOCK 리턴
        struct can_frame frame;
        while (true)
        {
            int nbytes = ::read(can_socket, &frame, sizeof(frame));

            if (nbytes > 0)
            {
                // 성공적으로 프레임을 읽었으면 파싱
                HandleReceivedFrame(frame);
            }
            else if (nbytes < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    // 더 이상 읽을 데이터가 없으므로 루프 종료
                    break;
                }
                else if (errno == EINTR)
                {
                    // 인터럽트로 중단된 경우 다시 읽기 시도
                    continue;
                }
                else
                {
                    // 기타 오류 발생 시 로깅 후 종료
                    RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware),
                                "Error reading CAN frame in read(): %s (errno=%d)",
                                strerror(errno), errno);
                break;
                }
            }
            else
            {
                // nbytes == 0: 소켓이 닫혔거나 EOF → 종료
                break;
            }
        }

        return return_type::OK;
    }

    return_type InwheelMotorHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {

        if (use_dummy_)
        {
            for (uint i = 0; i < joints_.size(); ++i)
            {
            auto &joint = joints_[i];
            joint.prev_command.velocity = joint.command.velocity;
            joint.state.velocity = joint.command.velocity;
            // joint.state.effort = 0.0;
            }

            return return_type::OK;
        }


        for (size_t i = 0; i < joints_.size(); ++i)
        {              
            if (joints_[i].command.torque_on_off != joints_[i].prev_command.torque_on_off)
            {

                // if (joints_[i].command.torque_on_off == -1.0)
                // {
                //     SendReqStateChange(can_socket, can_ids_[i], k_axis_state_idle);
                //     RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "off");
                //     return return_type::OK;

                // }
                if (joints_[i].command.torque_on_off == -1.0)
                {
                    joints_[i].command.velocity = 0.0;
                    joints_[i].command.velocity_p_gain = damping_value;
                    joints_[i].prev_command.velocity = joints_[i].command.velocity;
                    joints_[i].prev_command.velocity_p_gain = joints_[i].command.velocity_p_gain;

                    static std::vector<bool> idle_sent;
                    if (idle_sent.empty()) {
                        idle_sent.assign(joints_.size(), false);
                    }

                    if (!idle_sent[i])
                    {
                        SendReqStateChange(can_socket, can_ids_[i], k_axis_state_idle);
                        SendReqStateChange(can_socket, can_ids_[i], k_axis_state_closed_loop_control);
                        idle_sent[i] = true;
                    }

                    SetControllerMode(can_socket, can_ids_[i], 2.0, 1.0);
                    SetVelocityGains(can_socket, can_ids_[i], joints_[i].command.velocity_p_gain, 0.0);
                    SendMotorVelCommands(can_socket, can_ids_[i], joints_[i].command.velocity, 0.0);

                    RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "Damped off");
                    return return_type::OK;
                }
                else if (joints_[i].command.torque_on_off == 1.0)
                {

                    // joints_[i].command.position = joints_[i].state.position;
                    // joints_[i].command.velocity = 0.0;
                    // joints_[i].command.effort = 0.0;
                    // joints_[i].prev_command.position = joints_[i].command.position;
                    // joints_[i].prev_command.velocity = joints_[i].command.velocity;
                    // joints_[i].prev_command.effort = joints_[i].command.effort;
                    // SendReqStateChange(can_socket, can_ids_[i], k_axis_state_closed_loop_control);
                    // RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "on");
                }
                joints_[i].prev_command.torque_on_off = joints_[i].command.torque_on_off;
            }
        
            //estop
            if (joints_[i].command.estop != joints_[i].prev_command.estop)
            {
                SendEstopCommand(can_socket, can_ids_[i], joints_[i].command.estop);
                joints_[i].prev_command.estop = joints_[i].command.estop;
            }


            //setting gain
            if (joints_[i].command.velocity_p_gain != joints_[i].prev_command.velocity_p_gain ||
            joints_[i].command.velocity_i_gain != joints_[i].prev_command.velocity_i_gain)
            {
                // 속도 게인 설정
                SetVelocityGains(can_socket, can_ids_[i], joints_[i].command.velocity_p_gain, joints_[i].command.velocity_i_gain);

                // 이전 명령 값을 현재 명령 값으로 업데이트
                joints_[i].prev_command.velocity_p_gain = joints_[i].command.velocity_p_gain;
                joints_[i].prev_command.velocity_i_gain = joints_[i].command.velocity_i_gain;
            }
        }

        // Velocity control
        for (size_t i = 0; i < joints_.size(); ++i) 
        {
            // RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "heartbeat_error_flag: %d", heartbeat_error_flag);
            if (heartbeat_error_flags_[can_ids_[i]])
            {
                int node_id = can_ids_[i];
        
                RCLCPP_WARN(rclcpp::get_logger(kInwheelMotorHardware), "Heartbeat error on node %d. Attempting recovery...", node_id);
                
                // 1. clear
                SendClearErrorsCommand(can_socket, node_id);

                // 2. 충분히 대기
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // 3. idle로 안정화
                SendReqStateChange(can_socket, node_id, k_axis_state_idle);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // 4. closed loop 재진입
                SendReqStateChange(can_socket, node_id, k_axis_state_closed_loop_control);

                heartbeat_error_flags_[can_ids_[i]] = false;

                return return_type::OK;
            }

            auto &joint = joints_[i];
            if (joint.command.velocity != joint.prev_command.velocity) 
            {
                if (std::isnan(joint.command.velocity)) {
                    // RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "Velocity is NaN");
                    continue;
                }

                double left_vel_gain = 1.0;
                double right_vel_gain = 1.0;
                double wheel_cmd_rev_per_sec = joint.command.velocity / (2 * M_PI); // rad/s -> rev/s

                if (i % 2 == 1)
                { wheel_cmd_rev_per_sec *= -1.0 * left_vel_gain; }
                else { wheel_cmd_rev_per_sec *= 1.0 * right_vel_gain; }

                // SetControllerMode(can_socket, can_ids_[i], 2.0, 1.0);
                SetVelocityGains(can_socket, can_ids_[i], joint.command.velocity_p_gain, joint.command.velocity_i_gain);
                SendMotorVelCommands(can_socket, can_ids_[i], wheel_cmd_rev_per_sec, 0.0);

                RCLCPP_INFO(rclcpp::get_logger("InwheelMotorHardware"), "can_id: %d", int(can_ids_[i]));

                joint.prev_command.velocity = joint.command.velocity;

                RCLCPP_INFO(
                    rclcpp::get_logger(kInwheelMotorHardware),
                    "Wheel %zu | velocity: %.4f rad/s → %.4f rev/s",
                    i, joint.command.velocity, wheel_cmd_rev_per_sec
                );
            }
        }

        return return_type::OK;
    }


    return_type InwheelMotorHardware::reset_command()
    {
        for (uint i = 0; i < joints_.size(); i++)
        {
            joints_[i].command.velocity = 0.0;
            joints_[i].prev_command.velocity = joints_[i].command.velocity;
        }
        return return_type::OK;
    }


    //CUSTOM METHOD

    //CAN SETTING
    bool InwheelMotorHardware::InitializeCanInterface(const char *ifname, int &socket_fd)
    {
        socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd < 0)
        {
        return false;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, ifname);
        ioctl(socket_fd, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
        close(socket_fd);
        return false;
        }

        // 논블로킹 모드로 변경
        int flags = fcntl(socket_fd, F_GETFL, 0);
        if (flags == -1) {
        close(socket_fd);
        return false;
        }
        if (fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        close(socket_fd);
        return false;
        }

        return true;
    }

    void InwheelMotorHardware::SendReqStateChange(int socket_fd, int node_id, uint32_t state)
    {
        struct can_frame frame;
        frame.can_id = k_set_axis_requested_state_cmd | (node_id << 5);
        frame.can_dlc = 4;
        std::memcpy(frame.data, &state, sizeof(state));

        ssize_t nbytes = ::write(socket_fd, &frame, sizeof(frame));

        if (nbytes != sizeof(frame)) 
        {
            RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "Failed to send state command to node_id: %d", node_id);
        } 
        else 
        {
            RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "Sent state command: node_id=%d, state=%u", node_id, state);
        }
    }

    // RECEIVE DATA
    // void InwheelMotorHardware::GetEncoderEstimates(const struct can_frame& frame)
    // {
    //     int node_id = (frame.can_id >> 5) & 0xFF;
    //     float encoder_pos;
    //     float encoder_vel;

    //     std::memcpy(&encoder_pos, frame.data, sizeof(encoder_pos));
    //     std::memcpy(&encoder_vel, frame.data + 4, sizeof(encoder_vel)); 

    //     auto it = std::find(can_ids_.begin(), can_ids_.end(), node_id);
    //     if (it != can_ids_.end()) 
    //     {
    //         size_t index = std::distance(can_ids_.begin(), it);
            
    //         joints_[index].state.position = static_cast<double>(encoder_pos) * 2.0 * M_PI;
    //         joints_[index].state.velocity = static_cast<double>(encoder_vel) * 2.0 * M_PI;
    //     }
    // }
    void InwheelMotorHardware::GetEncoderEstimates(const struct can_frame& frame)
    {
        int node_id = (frame.can_id >> 5) & 0xFF;
        float encoder_pos;
        float encoder_vel;

        std::memcpy(&encoder_pos, frame.data, sizeof(encoder_pos));
        std::memcpy(&encoder_vel, frame.data + 4, sizeof(encoder_vel)); 

        auto it = std::find(can_ids_.begin(), can_ids_.end(), node_id);
        if (it != can_ids_.end()) 
        {
            size_t index = std::distance(can_ids_.begin(), it);

            double sign = 1.0;
            if (node_id % 2 == 1)  // 왼쪽 바퀴는 부호 반전
                sign = -1.0;

            joints_[index].state.position = static_cast<double>(encoder_pos) * 2.0 * M_PI * sign;
            joints_[index].state.velocity = static_cast<double>(encoder_vel) * 2.0 * M_PI * sign;
        }
    }

    void InwheelMotorHardware::GetIqData(const struct can_frame& frame)
    {
        int node_id = (frame.can_id >> 5) & 0xFF;
        float iq_setpoint;
        float iq_measured;

        std::memcpy(&iq_setpoint, frame.data , sizeof(iq_setpoint));
        std::memcpy(&iq_measured, frame.data + 4, sizeof(iq_measured));

        auto it = std::find(can_ids_.begin(), can_ids_.end(), node_id);
        if (it != can_ids_.end()) 
        {
            size_t index = std::distance(can_ids_.begin(), it);

            double torque_constant = motor_constant_[index];  // 모터 상수 선택

            // Iq 값 가져오기
            double iq_value = static_cast<double>(iq_measured);

            // 토크 계산
            double torque_measured = torque_constant * iq_value;

            // 상태 업데이트
            // joints_[index].state.effort = torque_measured;       // 실제 측정된 힘
            joints_[index].state.iq_setpoint = static_cast<double>(iq_setpoint);
            joints_[index].state.iq_measured = iq_value;
        }
    }


    void InwheelMotorHardware::GetBusVoltage(const struct can_frame& frame)
    {
        int node_id = (frame.can_id >> 5) & 0xFF;
        float bus_voltage;
        float bus_current;

        // CAN 데이터에서 버스 전압과 전류 추출
        std::memcpy(&bus_voltage, frame.data, sizeof(bus_voltage));
        std::memcpy(&bus_current, frame.data + 4, sizeof(bus_current));

        auto it = std::find(can_ids_.begin(), can_ids_.end(), node_id);
        if (it != can_ids_.end()) 
        {
            size_t index = std::distance(can_ids_.begin(), it);
            joints_[index].state.bus_voltage = static_cast<double>(bus_voltage);
            joints_[index].state.bus_current = static_cast<double>(bus_current);
        } 
    }

    // VELOCITY_CONTROL
    void InwheelMotorHardware::SendMotorVelCommands(int socket_fd, int node_id, float velocity, float torque_ff)
    {
        struct can_frame frame;
        frame.can_id = k_set_input_vel_cmd | (node_id << 5);
        frame.can_dlc = 8;
        struct {
            float vel;
            int16_t torque_ff;
        }

        data = {velocity, static_cast<int16_t>(torque_ff * 1000)};

        std::memcpy(frame.data, &data, sizeof(data));

        ssize_t nbytes = ::write(socket_fd, &frame, sizeof(frame));

        RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "Send Velocity Command!!! Function");

        if (nbytes != sizeof(frame)) 
        {
            RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "CAN frame write failed");
        }
    }


    void InwheelMotorHardware::SetControllerMode(int socket_fd, int node_id, uint32_t control_mode, uint32_t input_mode)
    {
        struct can_frame frame;
        frame.can_id  = k_set_controller_mode_cmd | (node_id << 5);
        frame.can_dlc = 8;

        struct {
            uint32_t control_mode;
            uint32_t input_mode;
        } 
        
        data = { control_mode, input_mode };

        std::memcpy(frame.data, &data, sizeof(data));

        ssize_t nbytes = ::write(socket_fd, &frame, sizeof(frame));
        if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(rclcpp::get_logger("InwheelMotorHardware"),
                        "Failed to send controller mode to node %d (errno=%d)", node_id, errno);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("InwheelMotorHardware"),
                        "Sent controller mode: node=%d, control_mode=%u, input_mode=%u",
                        node_id, control_mode, input_mode);
        }
    }

    void InwheelMotorHardware::SendEstopCommand(int socket_fd, int node_id, float estop)
    {
        struct can_frame frame;
        frame.can_id = k_set_estop_cmd | (node_id << 5);  // 위치 게인 설정 명령의 CAN ID 설정
        frame.can_dlc = 4;

        std::memcpy(frame.data, &estop, sizeof(estop));

        ssize_t nbytes = ::write(socket_fd, &frame, sizeof(frame));

        if (nbytes != sizeof(frame)) {
            RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "Failed to send estop to node_id: %d", node_id);
        } else {
            ROS_RED_STREAM("ESTOP");
        }
    }

    void InwheelMotorHardware::SetVelocityGains(int socket_fd, int node_id, float velocity_gain, float velocity_integrator_gain)
    {
        struct can_frame frame;
        frame.can_id = k_set_vel_gains_cmd | (node_id << 5);  // 속도 게인 설정 명령의 CAN ID 설정
        frame.can_dlc = 8;
        struct {
            float vel_gain;
            float vel_integrator_gain;
        } data = {velocity_gain, velocity_integrator_gain};

        std::memcpy(frame.data, &data, sizeof(data));

        ssize_t nbytes = ::write(socket_fd, &frame, sizeof(frame));

        if (nbytes != sizeof(frame)) 
        {
        RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), "Failed to send velocity gains to node_id: %d", node_id);
        } 
        else 
        {
        RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), "Sent velocity gains: node_id=%d, vel_gain=%f, vel_integrator_gain=%f", node_id, velocity_gain, velocity_integrator_gain);
        }
    }

    bool InwheelMotorHardware::LoadConfig(const std::string& config_path)
    {
    try 
    {
        YAML::Node config = YAML::LoadFile(config_path);
            
        if (!config["messages"]) 
        {
            return false;
        }

            for (const auto& msg : config["messages"]) 
            {
                CANMessage message;
                std::string cmd_id_str = msg["cmd_id"].as<std::string>();
                if (cmd_id_str.substr(0, 2) == "0x") 
                {
                    cmd_id_str = cmd_id_str.substr(2);
                }
                int cmd_id = std::stoi(cmd_id_str, nullptr, 16);
                message.cmd_id = cmd_id;
                message.name = msg["name"].as<std::string>();

                if (msg["signals"]) 
                {
                    for (const auto& sig : msg["signals"]) 
                    {
                        Signal signal;
                        signal.name = sig["name"].as<std::string>();
                        signal.start_byte = sig["start_byte"].as<int>();
                        signal.type = sig["type"].as<std::string>();
                        signal.bits = sig["bits"].as<int>();
                        signal.factor = sig["factor"].as<float>();
                        signal.offset = sig["offset"].as<float>();
                        message.signals.push_back(signal);
                    }
                }

                can_message_map_[cmd_id] = std::move(message);
            }
            std::string config_path_onum = ament_index_cpp::get_package_share_directory("edie8_parameters") + "/config/odrive_enum.yaml";
            LoadODriveEnums(config_path_onum);

            return true;
        } 
        catch (const YAML::Exception& e) 
        {
            return false;
        } 
        catch (const std::exception& e) 
        {
            return false;
        }
    }

    void InwheelMotorHardware::LoadODriveEnums(const std::string& config_path) 
    {
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config.IsSequence()) 
        {
            throw YAML::Exception(YAML::Mark::null_mark(), "Root node is not a sequence");
        }
        for (const auto& enum_node : config) 
        {
            if (!enum_node.IsMap()) 
            {
                continue;
            }
            std::string enum_name = enum_node["enum_name"].as<std::string>();
            const auto& values = enum_node["values"];

            if (!values.IsSequence()) 
            {
                continue;
            }
            for (const auto& value_node : values) 
            {
                ODriveEnumValue value;
                value.enum_name = enum_name;
                value.string_value = value_node["name"].as<std::string>();
                value.value = value_node["value"].as<uint64_t>();
                odrive_enums_.push_back(value);
                enum_map_[enum_name][value.value] = value.string_value;
            }
        }
    } 
    catch (const YAML::Exception& e) 
    {
        RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), 
                    "Failed to load ODrive enums from %s. Error: %s", 
                    config_path.c_str(), e.what());
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), 
                    "Unexpected error while loading ODrive enums from %s. Error: %s", 
                    config_path.c_str(), e.what());
    }
    }

    std::vector<std::string> InwheelMotorHardware::matchEnums(const std::string& enum_name, uint64_t value) {
        std::vector<std::string> matched_enums;
        if (enum_map_.find(enum_name) != enum_map_.end()) {
            if (enum_name == "AxisState") {
                // AxisState는 단일 값으로 처리
                auto it = enum_map_[enum_name].find(value);
                if (it != enum_map_[enum_name].end()) {
                    matched_enums.push_back(it->second);
                }
            } else {
                // 다른 enum들은 비트 마스크로 처리
                for (const auto& [enum_value, enum_string] : enum_map_[enum_name]) {
                    // 매칭 조건 수정: enum_value가 0이 아니고, value & enum_value가 정확히 enum_value와 같을 때만 매칭
                    if (enum_value != 0 && (value & enum_value) == enum_value) {
                        matched_enums.push_back(enum_string);
                    }
                }

                // ENCODER_ERROR_NONE은 별도로 처리 (value가 0일 때만 추가)
                if (value == 0) {
                    matched_enums.push_back("ENCODER_ERROR_NONE");
                }
            }
        }
        return matched_enums;
    }

    uint64_t InwheelMotorHardware::parseUnsignedInt(const uint8_t* data, int start_byte, int bit_length) {
        uint64_t value = 0;
        int num_bytes = (bit_length + 7) / 8;  // 필요한 바이트 수 계산

        // 빅 엔디안 방식으로 데이터 읽기 (가장 왼쪽 바이트부터 시작)
        for (int i = 0; i < num_bytes; i++) {
            uint64_t byte_value = static_cast<uint64_t>(data[start_byte + i]);
            value = (value << 8) | byte_value;  // 왼쪽으로 시프트 후 현재 바이트 추가
        }

        // 리틀 엔디안 형식으로 변환
        uint64_t little_endian_value = 0;
        for (int i = 0; i < num_bytes; i++) {
            uint64_t byte_value = (value >> (8 * i)) & 0xFF;  // 각 바이트를 추출
            little_endian_value = (little_endian_value << 8) | byte_value;  // 왼쪽으로 시프트 후 바이트 추가
        }

        return little_endian_value;
    }

    void InwheelMotorHardware::HandleReceivedFrame(const struct can_frame& frame) 
    {
        int cmd_id = frame.can_id & 0x1F; 
        int node_id = (frame.can_id >> 5) & 0xFF;

        if (cmd_id == 0x09) 
        {
            GetEncoderEstimates(frame);
        }
        else if (cmd_id == 0x14) 
        {
            GetIqData(frame);
        } 
        else if (cmd_id == 0x17) 
        {
            GetBusVoltage(frame);
        } 
        else if (frame.can_id == 0x81 || frame.can_id == 0x83) 
        {

            if (frame.can_dlc == 8) 
            {
            } 
        }
        
        if (node_id < 16 || node_id > 25 )
        {
            return;
        }


        if (can_message_map_.find(cmd_id) != can_message_map_.end()) 
        {
            const CANMessage& message = can_message_map_[cmd_id];

            for (const Signal& signal : message.signals) {
                std::vector<std::string> matched_enums;
                std::string value_str;
                uint64_t value = 0;

                try 
                {
                    if (signal.type == "IEEE 754 Float") 
                    {
                        float float_value;
                        std::memcpy(&float_value, &frame.data[signal.start_byte], sizeof(float_value));
                        value_str = std::to_string(float_value);
                    } 
                    else if (signal.type == "Signed Int") 
                    {
                        if (signal.bits == 32)
                        {
                            int32_t int_value;
                            std::memcpy(&int_value, &frame.data[signal.start_byte], sizeof(int_value));
                            value_str = std::to_string(int_value);
                            value = static_cast<uint64_t>(int_value);
                        } 
                        else if (signal.bits == 16) 
                        {
                            int16_t short_value;
                            std::memcpy(&short_value, &frame.data[signal.start_byte], sizeof(short_value));
                            value_str = std::to_string(short_value);
                            value = static_cast<uint64_t>(short_value);
                        }
                    } 
                    else if (signal.type == "Unsigned Int") 
                    {
                        if (cmd_id == k_set_heartbeat_message) 
                        {
                            if (signal.name == "Trajectory Done Flag") 
                            {
                                value = (frame.data[signal.start_byte] >> 7) & 0x01;  // 최상위 비트 추출
                                value_str = std::to_string(value);
                                matched_enums = {value == 0 ? "False" : "True"};
                            } 
                            else if (signal.name == "AxisError") 
                            {
                                value = parseUnsignedInt(frame.data, signal.start_byte, signal.bits);
                                value_str = std::to_string(value);
                                matched_enums = matchEnums(signal.name, value);
                                if (value != 0) {
                                    heartbeat_error_flags_[node_id] = true;
                                }
                            } 
                            else if (signal.name == "Axis Current State") 
                            {
                                value = parseUnsignedInt(frame.data, signal.start_byte, signal.bits);
                                value_str = std::to_string(value);
                                matched_enums = matchEnums("AxisState", value);
                                uint8_t axis_state = frame.data[4];
                                axis_states_[node_id] = axis_state;
                                // RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware),
                                //     "축 상태 업데이트 - 노드 ID: %d, 새 상태: 0x%02X", node_id, axis_state);
                            }
                            else if (std::find(motordriver_error_flag_names.begin(), motordriver_error_flag_names.end(), signal.name) != motordriver_error_flag_names.end()) 
                            {
                                value = (frame.data[signal.start_byte] >> 0) & 0x01;
                                value_str = std::to_string(value);
                                matched_enums = {value == 0 ? "False" : "True"};
                                if (value != 0) {
                                    heartbeat_error_flags_[node_id] = true;
                                }
                            }
                        } 
                        else 
                        {
                            value = parseUnsignedInt(frame.data, signal.start_byte, signal.bits);
                            value_str = std::to_string(value);
                            matched_enums = matchEnums(signal.name, value);
                            if (value != 0) 
                            {
                                motordriver_error_flag = true;
                            }
                        }
                    } 
                    else 
                    {
                        throw std::runtime_error("Unsupported signal type: " + signal.type);
                    }
                    if (debug_mode_) 
                    {
                        std::string enum_str = matched_enums.empty() ? "N/A" : rcpputils::join(matched_enums, " | ");
                                                
                        auto it = std::find(can_ids_.begin(), can_ids_.end(), node_id);
                        std::string joint_name = (it != can_ids_.end()) ? info_.joints[std::distance(can_ids_.begin(), it)].name : "Unknown Joint";
                    
                        if (motordriver_error_flag) 
                        {
                            ROS_RED_STREAM("Error Signal - Joint Name: " 
                                << joint_name.c_str() << ", Node ID: " << node_id << ", CMD ID: 0x" << cmd_id <<
                                ", Message: " << message.name.c_str() << ", Signal: " << signal.name.c_str() << ", codes: " << enum_str.c_str());
                            motordriver_error_flag = false;

                        } 
                        else if (heartbeat_error_flags_[node_id]) 
                        {
                            ROS_RED_STREAM("Heartbeat Error - Joint Name: " 
                                << joint_name.c_str() << ", Node ID: " << node_id << ", CMD ID: 0x" << cmd_id <<
                                ", Message: " << message.name.c_str() << ", Signal: " << signal.name.c_str() << ", codes: " << enum_str.c_str());
                            // heartbeat_error_flag = false;
                        }
                        // else {
                        //     RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware), 
                        //         "Parsed Signal - Node ID: %d, CMD ID: 0x%X, Message: %s, Signal: %s, Value: %s, codes: %s", 
                        //         node_id, cmd_id, message.name.c_str(), signal.name.c_str(), value_str.c_str(), enum_str.c_str());
                        // }
                    }
                } 
                catch (const std::exception& e) 
                {
                    if (debug_mode_) 
                    {
                    RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware), 
                        "Error parsing signal %s: %s", signal.name.c_str(), e.what());
                    }
                }
            }
        }
    }

    void InwheelMotorHardware::SendClearErrorsCommand(int socket_fd, int node_id)
    {
        struct can_frame frame;
        frame.can_id  = 0x018 | (node_id << 5);  // clear_errors() 명령
        frame.can_dlc = 0;                       // 데이터 없음

        ssize_t nbytes = ::write(socket_fd, &frame, sizeof(frame));
        if (nbytes != sizeof(frame))
        {
            RCLCPP_ERROR(rclcpp::get_logger(kInwheelMotorHardware),
                        "Failed to send clear_errors() to node_id: %d (errno=%d)", node_id, errno);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger(kInwheelMotorHardware),
                        "Sent clear_errors() to node_id: %d", node_id);
        }
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(inwheel_motor_hardware::InwheelMotorHardware, hardware_interface::SystemInterface)
