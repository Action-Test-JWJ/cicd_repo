/*
 * aeirobot_pid_control.cpp
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#include "aeirobot_toolbox/aeirobot_pid_control.h"

using namespace aeirobot;

PDController::PDController()
{
  control_time_sec_ = aeirobot::GetParameter<double>("time_parameter", "control_time");
  desired_ = 0;
  p_gain_ = 0;
  d_gain_ = 0;
  curr_err_ = 0;
  prev_err_ = 0;
}

PDController::~PDController()
{
}

double PDController::GetFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;

  return (p_gain_ * curr_err_ + d_gain_ * (curr_err_ - prev_err_) / control_time_sec_);
}

PIDController::PIDController()
{
  control_time_sec_ = aeirobot::GetParameter<double>("time_parameter", "control_time");
  desired_ = 0;
  p_gain_ = 0;
  i_gain_ = 0;
  d_gain_ = 0;
  curr_err_ = 0;
  prev_err_ = 0;
  sum_err_ = 0;
}

PIDController::~PIDController()
{
}

void PIDController::SetPidGains(double Kp, double Ki, double Kd)
{
  p_gain_ = Kp;
  i_gain_ = Ki;
  d_gain_ = Kd;
}

void PIDController::ResetPidIntegral()
{
  sum_err_ = 0;
}

double PIDController::PidProcess(double desired, double present)
{
  double output = 0;
  curr_err_ = desired - present;
  sum_err_ += curr_err_;

  output = p_gain_ * curr_err_;
  output += i_gain_ * sum_err_ * control_time_sec_;
  output += d_gain_ * (curr_err_ - prev_err_) / control_time_sec_;

  prev_err_ = curr_err_;

  return output;
}

double PIDController::GetFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;
  sum_err_ += curr_err_;

  return (p_gain_ * curr_err_ + i_gain_ * sum_err_ * control_time_sec_ + d_gain_ * (curr_err_ - prev_err_) / control_time_sec_);
}
