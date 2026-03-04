#ifndef CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_
#define CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hardware_interface
{
constexpr char HW_IF_TORQUE_ON_OFF[] = "torque_on_off";

constexpr char HW_IF_VELOCITY_P_GAIN[] = "velocity_p_gain";
constexpr char HW_IF_VELOCITY_I_GAIN[] = "velocity_i_gain";
constexpr char HW_IF_VELOCITY_D_GAIN[] = "velocity_d_gain";

constexpr char HW_IF_TORQUE_P_GAIN[] = "torque_p_gain";
constexpr char HW_IF_TORQUE_I_GAIN[] = "torque_i_gain";
constexpr char HW_IF_TORQUE_D_GAIN[] = "torque_d_gain";

constexpr char HW_IF_ESTOP[] = "estop";

constexpr char HW_IF_BUS_VOLTAGE[] = "bus_voltage";
constexpr char HW_IF_BUS_CURRENT[] = "bus_current";

constexpr char HW_IF_IQ_SETPOINT[] = "iq_setpoint";
constexpr char HW_IF_IQ_MEASURED[] = "iq_measured";


}  // namespace hardware_interface

#endif  // CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_
