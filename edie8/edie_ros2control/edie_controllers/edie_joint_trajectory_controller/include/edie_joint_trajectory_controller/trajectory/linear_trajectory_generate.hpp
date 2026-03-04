#ifndef LINEAR_TRAJECTORY_GENERATOR_HPP_
#define LINEAR_TRAJECTORY_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Eigen>

class LinearTrajectoryGenerator
{
public:
  LinearTrajectoryGenerator();
  ~LinearTrajectoryGenerator();

  double GenerateLinearTrajectory(double initial_value, double final_value,
                                  double initial_time, double final_time);
  bool DetectChangeFinalValue(double value, double time);

private:
  double initial_time;
  double initial_pos;

  double current_time;
  double current_pos;

  double final_time;
  double final_pos;

  bool is_moving_;
  double control_time;
};

#endif  // LINEAR_TRAJECTORY_GENERATOR_HPP_
