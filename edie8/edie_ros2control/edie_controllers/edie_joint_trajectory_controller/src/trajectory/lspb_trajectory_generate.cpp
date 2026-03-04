#include "edie_joint_trajectory_controller/trajectory/lspb_trajectory_generate.hpp"
#include <cmath>

LspbTrajectoryGenerator::LspbTrajectoryGenerator()
{
  current_time = 0;
  is_moving_traj = false;

  a[0] = a[1] = a[2] = a[3] = 0;

  initial_time = 0;
  final_time = 0;
  d_t = 0;

  initial_pose = 0;
  final_pose = 0;
  initial_velocity = 0;
  final_velocity = 0;

  current_pose = 0;
  current_velocity = 0;

  control_time = aeirobot::GetParameter<double>("time_parameter", "control_time");
}

LspbTrajectoryGenerator::~LspbTrajectoryGenerator()
{
}

bool LspbTrajectoryGenerator::DetectChangeFinalValue(double pose_, double velocity_, double time_)
{
  if (pose_ != final_pose || velocity_ != final_velocity || time_ != final_time)
  {
    final_pose = pose_;
    final_velocity = velocity_;
    final_time = time_;
    current_time = 0;
    return true;
  }
  return false;
}

double LspbTrajectoryGenerator::GenerateLspbTrajectory(double initial_value_, double final_value_,
                                                                    double initial_velocity_, double final_velocity_,
                                                                    double initial_time_, double final_time_)
{
  if (current_time == 0)
  {
    // 초기 설정
    initial_time = initial_time_;
    final_time = final_time_;
    initial_pose = initial_value_;
    final_pose = final_value_;
    initial_velocity = initial_velocity_;
    final_velocity = final_velocity_;
    d_t = final_time - initial_time;

    // 계수 계산
    a[0] = initial_pose;
    a[1] = initial_velocity;
    a[2] = (3 * (final_pose - initial_pose) / (d_t * d_t)) - (2 * initial_velocity + final_velocity) / d_t;
    a[3] = (-2 * (final_pose - initial_pose) / (d_t * d_t * d_t)) + (initial_velocity + final_velocity) / (d_t * d_t);
  }

  current_time += control_time;

  if (current_time > final_time_)
  {
    is_moving_traj = false;
    return current_pose;
  }
  else
  {
    double t = current_time - initial_time;
    trajectory_final_value = a[0] + a[1] * t + a[2] * t * t + a[3] * t * t * t;

    current_pose = trajectory_final_value;
    current_velocity = a[1] + 2 * a[2] * t + 3 * a[3] * t * t;

    is_moving_traj = true;
    return trajectory_final_value;
  }
}
