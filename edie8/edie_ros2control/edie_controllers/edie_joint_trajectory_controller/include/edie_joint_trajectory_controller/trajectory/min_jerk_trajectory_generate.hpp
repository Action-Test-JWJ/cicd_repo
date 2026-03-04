#ifndef MIN_JERK_TRAJECTORY_GENERATOR_HPP
#define MIN_JERK_TRAJECTORY_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include <stdint.h>
#include <vector>
#include "Eigen/Dense"
#include <cmath>

inline double powDI(double a, int b)
{
	return (b == 0 ? 1 : (b > 0 ? a * powDI(a, b - 1) : 1 / powDI(a, -b)));
}

class MinimumJerkTrajectoryGenerator
{
public:
    MinimumJerkTrajectoryGenerator(double ini_time, double fin_time,
                         std::vector<double_t> ini_pos, std::vector<double_t> ini_vel, std::vector<double_t> ini_acc,
                         std::vector<double_t> fin_pos, std::vector<double_t> fin_vel, std::vector<double_t> fin_acc);
    ~MinimumJerkTrajectoryGenerator();

    std::vector<double_t> getPosition(double time);
    std::vector<double_t> getVelocity(double time);
    std::vector<double_t> getAcceleration(double time);

    double cur_time_;
    std::vector<double_t> cur_pos_;
    std::vector<double_t> cur_vel_;
    std::vector<double_t> cur_acc_;

    Eigen::MatrixXd position_coeff_;
    Eigen::MatrixXd velocity_coeff_;
    Eigen::MatrixXd acceleration_coeff_;
    Eigen::MatrixXd time_variables_;

  private:
    int number_of_joint_;
    double ini_time_, fin_time_;
    std::vector<double_t> ini_pos_, ini_vel_, ini_acc_;
    std::vector<double_t> fin_pos_, fin_vel_, fin_acc_;

private:
};

#endif // MIN_JERK_TRAJECTORY_GENERATOR_HPP