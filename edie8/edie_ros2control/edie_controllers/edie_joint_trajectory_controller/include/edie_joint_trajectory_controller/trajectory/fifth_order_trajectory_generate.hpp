#ifndef FIFTH_ORDER_TRAJECTORY_GENERATE_H_
#define FIFTH_ORDER_TRAJECTORY_GENERATE_H_

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Eigen>
#include "aeirobot_toolbox/basic_tools.hpp"

class FifthOrderTrajectoryGenerator
{
public:
    FifthOrderTrajectoryGenerator();
    ~FifthOrderTrajectoryGenerator();

    double GenerateFifthOderTrajectory(double initial_value_, double final_value_,
                                                                            double initial_velocity_, double final_velocity_,
                                                                            double initial_acc, double final_acc,
                                                                            double initial_time_, double final_time_);

    bool DetectChangeFinalValue(double pose, double velocity_, double time_);
    double GenerateOneFifthOderTrajectory(Eigen::MatrixXd joint_);

    bool is_moving_traj;

    double a[6];
    double d_t;
    double trajectory_final_value;

    double initial_time;
    double initial_pose;
    double initial_velocity;
    double initial_acc;

    double current_time;
    double current_pose;
    double current_velocity;
    double current_acc;

    double final_time;
    double final_pose;
    double final_velocity;
    double final_acc;

    double temp_value;
    double control_time;
};

#endif /* FIFTH_ORDER_TRAJECTORY_GENERATE_H_ */