#ifndef THIRD_ORDER_TRAJECTORY_GENERATOR_HPP_
#define THIRD_ORDER_TRAJECTORY_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "aeirobot_toolbox/basic_tools.hpp"

class ThirdOrderTrajectoryGenerator
{
public:
  ThirdOrderTrajectoryGenerator();
  ~ThirdOrderTrajectoryGenerator();

  double GenerateThirdOrderTrajectory(double initial_value_, double final_value_,
                                      double initial_velocity_, double final_velocity_,
                                      double initial_time_, double final_time_);

  bool DetectChangeFinalValue(double pose_, double velocity_, double time_);

  double a[4];
  double d_t;

  double current_time;
  double control_time;

  double initial_time, final_time;
  double initial_pose, final_pose;
  double initial_velocity, final_velocity;

  double current_pose;
  double current_velocity;

  double trajectory_final_value;
  bool is_moving_traj;
};

#endif  // THIRD_ORDER_TRAJECTORY_GENERATOR_HPP_
