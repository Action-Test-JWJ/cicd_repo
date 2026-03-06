#include "aeirobot_math/trajectory/ninth_order_polynomial_trajectory.hpp"

namespace aeirobot
{
  NinthOrderPolynomialTrajectory::NinthOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                                 double final_time, double final_pos, double final_vel, double final_acc)
  {
    position_coeff_.resize(10, 1);
    velocity_coeff_.resize(10, 1);
    acceleration_coeff_.resize(10, 1);
    jerk_coeff_.resize(10, 1);
    snap_coeff_.resize(10, 1);
    time_variables_.resize(1, 10);

    position_coeff_.fill(0);
    velocity_coeff_.fill(0);
    acceleration_coeff_.fill(0);
    jerk_coeff_.fill(0);
    snap_coeff_.fill(0);
    time_variables_.fill(0);

    if (final_time > initial_time)
    {
      initial_time_ = initial_time;
      initial_pos_ = initial_pos;
      initial_vel_ = initial_vel;
      initial_acc_ = initial_acc;
      initial_jerk_ = 0;
      initial_snap_ = 0;

      current_time_ = initial_time;
      current_pos_ = initial_pos;
      current_vel_ = initial_vel;
      current_acc_ = initial_acc;
      current_jerk_ = 0;
      current_snap_ = 0;

      final_time_ = final_time;
      final_pos_ = final_pos;
      final_vel_ = final_vel;
      final_acc_ = final_acc;
      final_jerk_ = 0;
      final_snap_ = 0;

      /* Before */
      // Eigen::MatrixXd time_mat;
      // Eigen::MatrixXd conditions_mat;

      // time_mat.resize(10, 10);
      Eigen::Matrix<double, 10, 10> time_mat;

      time_mat << powDI(initial_time_, 9), powDI(initial_time_, 8), powDI(initial_time_, 7), powDI(initial_time_, 6), powDI(initial_time_, 5), powDI(initial_time_, 4), powDI(initial_time_, 3), powDI(initial_time_, 2), initial_time_, 1.0,
          9.0 * powDI(initial_time_, 8), 8.0 * powDI(initial_time_, 7), 7.0 * powDI(initial_time_, 6), 6.0 * powDI(initial_time_, 5), 5.0 * powDI(initial_time_, 4), 4.0 * powDI(initial_time_, 3), 3.0 * powDI(initial_time_, 2), 2.0 * initial_time_, 1.0, 0.0,
          72.0 * powDI(initial_time_, 7), 56.0 * powDI(initial_time_, 6), 42.0 * powDI(initial_time_, 5), 30.0 * powDI(initial_time_, 4), 20.0 * powDI(initial_time_, 3), 12.0 * powDI(initial_time_, 2), 6.0 * initial_time_, 2.0, 0.0, 0.0,
          504.0 * powDI(initial_time_, 6), 336.0 * powDI(initial_time_, 5), 210.0 * powDI(initial_time_, 4), 120.0 * powDI(initial_time_, 3), 60.0 * powDI(initial_time_, 2), 24.0 * initial_time_, 6.0, 0.0, 0.0, 0.0,
          3024.0 * powDI(initial_time_, 5), 1680.0 * powDI(initial_time_, 4), 840.0 * powDI(initial_time_, 3), 360.0 * powDI(initial_time_, 2), 120.0 * initial_time_, 24.0, 0.0, 0.0, 0.0, 0.0,
          powDI(final_time_, 9), powDI(final_time_, 8), powDI(final_time_, 7), powDI(final_time_, 6), powDI(final_time_, 5), powDI(final_time_, 4), powDI(final_time_, 3), powDI(final_time_, 2), final_time_, 1.0,
          9.0 * powDI(final_time_, 8), 8.0 * powDI(final_time_, 7), 7.0 * powDI(final_time_, 6), 6.0 * powDI(final_time_, 5), 5.0 * powDI(final_time_, 4), 4.0 * powDI(final_time_, 3), 3.0 * powDI(final_time_, 2), 2.0 * final_time_, 1.0, 0.0,
          72.0 * powDI(final_time_, 7), 56.0 * powDI(final_time_, 6), 42.0 * powDI(final_time_, 5), 30.0 * powDI(final_time_, 4), 20.0 * powDI(final_time_, 3), 12.0 * powDI(final_time_, 2), 6.0 * final_time_, 2.0, 0.0, 0.0,
          504.0 * powDI(final_time_, 6), 336.0 * powDI(final_time_, 5), 210.0 * powDI(final_time_, 4), 120.0 * powDI(final_time_, 3), 60.0 * powDI(final_time_, 2), 24.0 * final_time_, 6.0, 0.0, 0.0, 0.0,
          3024.0 * powDI(final_time_, 5), 1680.0 * powDI(final_time_, 4), 840.0 * powDI(final_time_, 3), 360.0 * powDI(final_time_, 2), 120.0 * final_time_, 24.0, 0.0, 0.0, 0.0, 0.0;

      // conditions_mat.resize(10, 1);
      Eigen::Matrix<double, 10, 1> conditions_mat;
      conditions_mat << initial_pos_, initial_vel_, initial_acc_, initial_jerk_, initial_snap_, final_pos_, final_vel_, final_acc_, final_jerk_, final_snap_;

      position_coeff_ = time_mat.colPivHouseholderQr().solve(conditions_mat);
      velocity_coeff_ << 0.0,
          9.0 * position_coeff_.coeff(0, 0),
          8.0 * position_coeff_.coeff(1, 0),
          7.0 * position_coeff_.coeff(2, 0),
          6.0 * position_coeff_.coeff(3, 0),
          5.0 * position_coeff_.coeff(4, 0),
          4.0 * position_coeff_.coeff(5, 0),
          3.0 * position_coeff_.coeff(6, 0),
          2.0 * position_coeff_.coeff(7, 0),
          1.0 * position_coeff_.coeff(8, 0);
      acceleration_coeff_ << 0.0,
          0.0,
          72.0 * position_coeff_.coeff(0, 0),
          56.0 * position_coeff_.coeff(1, 0),
          42.0 * position_coeff_.coeff(2, 0),
          24.0 * position_coeff_.coeff(3, 0),
          20.0 * position_coeff_.coeff(4, 0),
          12.0 * position_coeff_.coeff(5, 0),
          6.0 * position_coeff_.coeff(6, 0),
          2.0 * position_coeff_.coeff(7, 0);
      jerk_coeff_ << 0.0,
          0.0,
          0.0,
          504.0 * position_coeff_.coeff(0, 0),
          336.0 * position_coeff_.coeff(1, 0),
          210.0 * position_coeff_.coeff(2, 0),
          120.0 * position_coeff_.coeff(3, 0),
          60.0 * position_coeff_.coeff(4, 0),
          24.0 * position_coeff_.coeff(5, 0),
          6.0 * position_coeff_.coeff(6, 0);
      snap_coeff_ << 0.0,
          0.0,
          0.0,
          0.0,
          3024.0 * position_coeff_.coeff(0, 0),
          1680.0 * position_coeff_.coeff(1, 0),
          840.0 * position_coeff_.coeff(2, 0),
          360.0 * position_coeff_.coeff(3, 0),
          120.0 * position_coeff_.coeff(4, 0),
          24.0 * position_coeff_.coeff(5, 0);
    }
  }

  NinthOrderPolynomialTrajectory::NinthOrderPolynomialTrajectory()
  {
    initial_time_ = 0;
    initial_pos_ = 0;
    initial_vel_ = 0;
    initial_acc_ = 0;
    initial_jerk_ = 0;
    initial_snap_ = 0;

    current_time_ = 0;
    current_pos_ = 0;
    current_vel_ = 0;
    current_acc_ = 0;
    current_jerk_ = 0;
    current_snap_ = 0;

    final_time_ = 0;
    final_pos_ = 0;
    final_vel_ = 0;
    final_acc_ = 0;
    final_jerk_ = 0;
    final_snap_ = 0;

    position_coeff_.resize(10, 1);
    velocity_coeff_.resize(10, 1);
    acceleration_coeff_.resize(10, 1);
    jerk_coeff_.resize(10, 1);
    snap_coeff_.resize(10, 1);
    time_variables_.resize(1, 10);

    position_coeff_.fill(0);
    velocity_coeff_.fill(0);
    acceleration_coeff_.fill(0);
    jerk_coeff_.fill(0);
    snap_coeff_.fill(0);
    time_variables_.fill(0);
  }

  NinthOrderPolynomialTrajectory::~NinthOrderPolynomialTrajectory()
  {
  }

  bool NinthOrderPolynomialTrajectory::changeTrajectory(double final_pos, double final_vel, double final_acc)
  {
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_jerk_ = 0;
    final_snap_ = 0;

    // /* Before */
    // Eigen::MatrixXd time_mat;
    // Eigen::MatrixXd conditions_mat;

    // time_mat.resize(10, 10);
    Eigen::Matrix<double, 10, 10> time_mat;

    time_mat << powDI(initial_time_, 9), powDI(initial_time_, 8), powDI(initial_time_, 7), powDI(initial_time_, 6), powDI(initial_time_, 5), powDI(initial_time_, 4), powDI(initial_time_, 3), powDI(initial_time_, 2), initial_time_, 1.0,
        9.0 * powDI(initial_time_, 8), 8.0 * powDI(initial_time_, 7), 7.0 * powDI(initial_time_, 6), 6.0 * powDI(initial_time_, 5), 5.0 * powDI(initial_time_, 4), 4.0 * powDI(initial_time_, 3), 3.0 * powDI(initial_time_, 2), 2.0 * initial_time_, 1.0, 0.0,
        72.0 * powDI(initial_time_, 7), 56.0 * powDI(initial_time_, 6), 42.0 * powDI(initial_time_, 5), 30.0 * powDI(initial_time_, 4), 20.0 * powDI(initial_time_, 3), 12.0 * powDI(initial_time_, 2), 6.0 * initial_time_, 2.0, 0.0, 0.0,
        504.0 * powDI(initial_time_, 6), 336.0 * powDI(initial_time_, 5), 210.0 * powDI(initial_time_, 4), 120.0 * powDI(initial_time_, 3), 60.0 * powDI(initial_time_, 2), 24.0 * initial_time_, 6.0, 0.0, 0.0, 0.0,
        3024.0 * powDI(initial_time_, 5), 1680.0 * powDI(initial_time_, 4), 840.0 * powDI(initial_time_, 3), 360.0 * powDI(initial_time_, 2), 120.0 * initial_time_, 24.0, 0.0, 0.0, 0.0, 0.0,
        powDI(final_time_, 9), powDI(final_time_, 8), powDI(final_time_, 7), powDI(final_time_, 6), powDI(final_time_, 5), powDI(final_time_, 4), powDI(final_time_, 3), powDI(final_time_, 2), final_time_, 1.0,
        9.0 * powDI(final_time_, 8), 8.0 * powDI(final_time_, 7), 7.0 * powDI(final_time_, 6), 6.0 * powDI(final_time_, 5), 5.0 * powDI(final_time_, 4), 4.0 * powDI(final_time_, 3), 3.0 * powDI(final_time_, 2), 2.0 * final_time_, 1.0, 0.0,
        72.0 * powDI(final_time_, 7), 56.0 * powDI(final_time_, 6), 42.0 * powDI(final_time_, 5), 30.0 * powDI(final_time_, 4), 20.0 * powDI(final_time_, 3), 12.0 * powDI(final_time_, 2), 6.0 * final_time_, 2.0, 0.0, 0.0,
        504.0 * powDI(final_time_, 6), 336.0 * powDI(final_time_, 5), 210.0 * powDI(final_time_, 4), 120.0 * powDI(final_time_, 3), 60.0 * powDI(final_time_, 2), 24.0 * final_time_, 6.0, 0.0, 0.0, 0.0,
        3024.0 * powDI(final_time_, 5), 1680.0 * powDI(final_time_, 4), 840.0 * powDI(final_time_, 3), 360.0 * powDI(final_time_, 2), 120.0 * final_time_, 24.0, 0.0, 0.0, 0.0, 0.0;

    // conditions_mat.resize(10, 1);
    Eigen::Matrix<double, 10, 1> conditions_mat;
    conditions_mat << initial_pos_, initial_vel_, initial_acc_, initial_jerk_, initial_snap_, final_pos_, final_vel_, final_acc_, final_jerk_, final_snap_;

    position_coeff_ = time_mat.colPivHouseholderQr().solve(conditions_mat);
    velocity_coeff_ << 0.0,
        9.0 * position_coeff_.coeff(0, 0),
        8.0 * position_coeff_.coeff(1, 0),
        7.0 * position_coeff_.coeff(2, 0),
        6.0 * position_coeff_.coeff(3, 0),
        5.0 * position_coeff_.coeff(4, 0),
        4.0 * position_coeff_.coeff(5, 0),
        3.0 * position_coeff_.coeff(6, 0),
        2.0 * position_coeff_.coeff(7, 0),
        1.0 * position_coeff_.coeff(8, 0);
    acceleration_coeff_ << 0.0,
        0.0,
        72.0 * position_coeff_.coeff(0, 0),
        56.0 * position_coeff_.coeff(1, 0),
        42.0 * position_coeff_.coeff(2, 0),
        24.0 * position_coeff_.coeff(3, 0),
        20.0 * position_coeff_.coeff(4, 0),
        12.0 * position_coeff_.coeff(5, 0),
        6.0 * position_coeff_.coeff(6, 0),
        2.0 * position_coeff_.coeff(7, 0);
    jerk_coeff_ << 0.0,
        0.0,
        0.0,
        504.0 * position_coeff_.coeff(0, 0),
        336.0 * position_coeff_.coeff(1, 0),
        210.0 * position_coeff_.coeff(2, 0),
        120.0 * position_coeff_.coeff(3, 0),
        60.0 * position_coeff_.coeff(4, 0),
        24.0 * position_coeff_.coeff(5, 0),
        6.0 * position_coeff_.coeff(6, 0);
    snap_coeff_ << 0.0,
        0.0,
        0.0,
        0.0,
        3024.0 * position_coeff_.coeff(0, 0),
        1680.0 * position_coeff_.coeff(1, 0),
        840.0 * position_coeff_.coeff(2, 0),
        360.0 * position_coeff_.coeff(3, 0),
        120.0 * position_coeff_.coeff(4, 0),
        24.0 * position_coeff_.coeff(5, 0);

    return true;
  }

  bool NinthOrderPolynomialTrajectory::changeTrajectory(double final_time, double final_pos, double final_vel, double final_acc)
  {
    if (final_time < initial_time_)
      return false;

    final_time_ = final_time;
    return changeTrajectory(final_pos, final_vel, final_acc);
  }

  bool NinthOrderPolynomialTrajectory::changeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                        double final_time, double final_pos, double final_vel, double final_acc)
  {
    if (final_time < initial_time)
      return false;

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    initial_vel_ = initial_vel;
    initial_acc_ = initial_acc;
    initial_jerk_ = 0;
    initial_snap_ = 0;

    final_time_ = final_time;

    return changeTrajectory(final_pos, final_vel, final_acc);
  }

  // Get the position at a given time
  double NinthOrderPolynomialTrajectory::getPosition(double time)
  {
    if (time >= final_time_)
    {
      current_time_ = final_time_;
      current_pos_ = final_pos_;
      return final_pos_;
    }
    else if (time <= initial_time_)
    {
      current_time_ = initial_time_;
      current_pos_ = initial_pos_;
      return initial_pos_;
    }
    else
    {
      current_time_ = time;
      time_variables_ << powDI(time, 9), powDI(time, 8), powDI(time, 7), powDI(time, 6), powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
      current_pos_ = (time_variables_ * position_coeff_).coeff(0, 0);
      return current_pos_;
    }
  }

  // Get the velocity at a given time
  double NinthOrderPolynomialTrajectory::getVelocity(double time)
  {
    if (time >= final_time_)
    {
      current_time_ = final_time_;
      current_vel_ = final_vel_;
      return final_vel_;
    }
    else if (time <= initial_time_)
    {
      current_time_ = initial_time_;
      current_vel_ = initial_vel_;
      return initial_vel_;
    }
    else
    {
      current_time_ = time;
      time_variables_ << powDI(time, 9), powDI(time, 8), powDI(time, 7), powDI(time, 6), powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
      current_vel_ = (time_variables_ * velocity_coeff_).coeff(0, 0);
      return current_vel_;
    }
  }

  // Get the acceleration at a given time
  double NinthOrderPolynomialTrajectory::getAcceleration(double time)
  {
    if (time >= final_time_)
    {
      current_time_ = final_time_;
      current_acc_ = final_acc_;
      return final_acc_;
    }
    else if (time <= initial_time_)
    {
      current_time_ = initial_time_;
      current_acc_ = initial_acc_;
      return initial_acc_;
    }
    else
    {
      current_time_ = time;
      time_variables_ << powDI(time, 9), powDI(time, 8), powDI(time, 7), powDI(time, 6), powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
      current_acc_ = (time_variables_ * acceleration_coeff_).coeff(0, 0);
      return current_acc_;
    }
  }

  // Get the jerk at a given time
  double NinthOrderPolynomialTrajectory::getJerk(double time)
  {
    if (time >= final_time_)
    {
      current_time_ = final_time_;
      current_jerk_ = final_jerk_;
      return final_jerk_;
    }
    else if (time <= initial_time_)
    {
      current_time_ = initial_time_;
      current_jerk_ = initial_jerk_;
      return initial_jerk_;
    }
    else
    {
      current_time_ = time;
      time_variables_ << powDI(time, 9), powDI(time, 8), powDI(time, 7), powDI(time, 6), powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
      current_jerk_ = (time_variables_ * jerk_coeff_).coeff(0, 0);
      return current_jerk_;
    }
  }

  // Get the snap at a given time
  double NinthOrderPolynomialTrajectory::getSnap(double time)
  {
    if (time >= final_time_)
    {
      current_time_ = final_time_;
      current_snap_ = final_snap_;
      return final_snap_;
    }
    else if (time <= initial_time_)
    {
      current_time_ = initial_time_;
      current_snap_ = initial_snap_;
      return initial_snap_;
    }
    else
    {
      current_time_ = time;
      time_variables_ << powDI(time, 9), powDI(time, 8), powDI(time, 7), powDI(time, 6), powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
      current_snap_ = (time_variables_ * snap_coeff_).coeff(0, 0);
      return current_snap_;
    }
  }

  // Set the current time and update all values accordingly
  void NinthOrderPolynomialTrajectory::setTime(double time)
  {
    if (time >= final_time_)
    {
      current_time_ = final_time_;
      current_pos_ = final_pos_;
      current_vel_ = final_vel_;
      current_acc_ = final_acc_;
      current_jerk_ = final_jerk_;
      current_snap_ = final_snap_;
    }
    else if (time <= initial_time_)
    {
      current_time_ = initial_time_;
      current_pos_ = initial_pos_;
      current_vel_ = initial_vel_;
      current_acc_ = initial_acc_;
      current_jerk_ = initial_jerk_;
      current_snap_ = initial_snap_;
    }
    else
    {
      current_time_ = time;
      time_variables_ << powDI(time, 9), powDI(time, 8), powDI(time, 7), powDI(time, 6), powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;
      current_pos_ = (time_variables_ * position_coeff_).coeff(0, 0);
      current_vel_ = (time_variables_ * velocity_coeff_).coeff(0, 0);
      current_acc_ = (time_variables_ * acceleration_coeff_).coeff(0, 0);
      current_jerk_ = (time_variables_ * jerk_coeff_).coeff(0, 0);
      current_snap_ = (time_variables_ * snap_coeff_).coeff(0, 0);
    }
  }

  // Get current position
  double NinthOrderPolynomialTrajectory::getPosition()
  {
    return current_pos_;
  }

  // Get current velocity
  double NinthOrderPolynomialTrajectory::getVelocity()
  {
    return current_vel_;
  }

  // Get current acceleration
  double NinthOrderPolynomialTrajectory::getAcceleration()
  {
    return current_acc_;
  }

  // Get current jerk
  double NinthOrderPolynomialTrajectory::getJerk()
  {
    return current_jerk_;
  }

  // Get current snap
  double NinthOrderPolynomialTrajectory::getSnap()
  {
    return current_snap_;
  }
}