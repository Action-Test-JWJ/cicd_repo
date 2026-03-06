#include "aeirobot_math/trajectory/fifth_order_polynomial_trajectory.hpp"

namespace aeirobot
{
  FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory() {}
  FifthOrderPolynomialTrajectory::FifthOrderPolynomialTrajectory(
      double initial_time, double initial_pos, double initial_vel, double initial_acc,
      double final_time, double final_pos, double final_vel, double final_acc) : initial_time_(initial_time), initial_pos_(initial_pos), initial_vel_(initial_vel), initial_acc_(initial_acc),
                                                                                 current_time_(initial_time), current_pos_(initial_pos), current_vel_(initial_vel), current_acc_(initial_acc),
                                                                                 final_time_(final_time), final_pos_(final_pos), final_vel_(final_vel), final_acc_(final_acc)
  {
    if (final_time <= initial_time)
    {
      // 응급조치(도착정보에 초기정보를 넣어줌)
      final_time = initial_time;
      final_pos = initial_pos;
      final_vel = initial_vel;
      final_acc = initial_acc;
      throw std::invalid_argument("Final time must be greater than or equal to initial time. This code will not work properly. ");
    }
    SetCoefficients();
  }

  FifthOrderPolynomialTrajectory::~FifthOrderPolynomialTrajectory() = default;

  bool FifthOrderPolynomialTrajectory::changeTrajectory(double final_pos, double final_vel, double final_acc)
  {
    if (final_pos_ == final_pos &&
        final_vel_ == final_vel &&
        final_acc_ == final_acc)
    {
      return true;
    }

    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;

    SetCoefficients();

    return true;
  }

  bool FifthOrderPolynomialTrajectory::changeTrajectory(double final_time, double final_pos, double final_vel, double final_acc)
  {
    if (final_pos_ == final_pos &&
        final_vel_ == final_vel &&
        final_acc_ == final_acc &&
        final_time_ == final_time)
      return true;
    else if (final_time < initial_time_)
      return false;

    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_time_ = final_time;

    SetCoefficients();

    return true;
  }

  bool FifthOrderPolynomialTrajectory::changeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                                        double final_time, double final_pos, double final_vel, double final_acc)
  {
    if (initial_pos_ == initial_pos &&
        initial_vel_ == initial_vel &&
        initial_acc_ == initial_acc &&
        initial_time_ == initial_time &&
        final_pos_ == final_pos &&
        final_vel_ == final_vel &&
        final_acc_ == final_acc &&
        final_time_ == final_time)
      return true;
    if (final_time < initial_time)
      return false;

    initial_time_ = initial_time;
    initial_pos_ = initial_pos;
    initial_vel_ = initial_vel;
    initial_acc_ = initial_acc;
    final_pos_ = final_pos;
    final_vel_ = final_vel;
    final_acc_ = final_acc;
    final_time_ = final_time;

    SetCoefficients();

    return true;
  }

  double FifthOrderPolynomialTrajectory::getPosition(double time)
  {
    Calculate(time);
    return current_pos_;
  }

  double FifthOrderPolynomialTrajectory::getVelocity(double time)
  {
    Calculate(time);
    return current_vel_;
  }

  double FifthOrderPolynomialTrajectory::getAcceleration(double time)
  {
    Calculate(time);
    return current_acc_;
  }

  // (Deprecated.) 없애고 싶은데, 다른 코드들까지 싹 고쳐야해서, 일단 놔둠.
  void FifthOrderPolynomialTrajectory::setTime(double time) { Calculate(time); }
  double FifthOrderPolynomialTrajectory::getPosition() { return current_pos_; }
  double FifthOrderPolynomialTrajectory::getVelocity() { return current_vel_; }
  double FifthOrderPolynomialTrajectory::getAcceleration() { return current_acc_; }

  void FifthOrderPolynomialTrajectory::SetCoefficients()
  {
    // 1) 지속시간 T 계산 및 유효성 검사
    const double T = final_time_ - initial_time_;
    if (T <= 0.0)
    {
      // 안전망: 초기값 유지
      position_coeff_.setZero();
      velocity_coeff_.setZero();
      acceleration_coeff_.setZero();
      current_time_ = initial_time_;
      current_pos_ = initial_pos_;
      current_vel_ = initial_vel_;
      current_acc_ = initial_acc_;
      return;
    }

    // 2) 경계조건 행렬 (tau=0, tau=T)
    const double T1 = T;
    const double T2 = T1 * T1;
    const double T3 = T2 * T1;
    const double T4 = T3 * T1;
    const double T5 = T4 * T1;

    Eigen::Matrix<double, 6, 6> A;
    // [a5 a4 a3 a2 a1 a0]ᵀ 기준, 행은 p, p', p'' at tau=0 and tau=T
    A << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,            // p(0)   = a0
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,             // p'(0)  = a1
        0.0, 0.0, 0.0, 2.0, 0.0, 0.0,             // p''(0) = 2 a2
        T5, T4, T3, T2, T1, 1.0,                  // p(T)
        5 * T4, 4 * T3, 3 * T2, 2 * T1, 1.0, 0.0, // p'(T)
        20 * T3, 12 * T2, 6 * T1, 2.0, 0.0, 0.0;  // p''(T)

    Eigen::Matrix<double, 6, 1> b;
    b << initial_pos_, initial_vel_, initial_acc_,
        final_pos_, final_vel_, final_acc_;

    // 역행렬 대신 선형해법(안정적)
    position_coeff_ = A.fullPivLu().solve(b);

    // 파생 계수(동일한 tau-기저에 맞춤)
    velocity_coeff_ << 0.0,
        5.0 * position_coeff_(0), // a5
        4.0 * position_coeff_(1), // a4
        3.0 * position_coeff_(2), // a3
        2.0 * position_coeff_(3), // a2
        1.0 * position_coeff_(4); // a1

    acceleration_coeff_ << 0.0,
        0.0,
        20.0 * position_coeff_(0), // a5
        12.0 * position_coeff_(1), // a4
        6.0 * position_coeff_(2),  // a3
        2.0 * position_coeff_(3);  // a2
  }

  void FifthOrderPolynomialTrajectory::Calculate(double time)
  {
    // tau로 변환하여 평가
    const double t0 = initial_time_;
    const double tf = final_time_;
    if (time >= tf)
    {
      current_time_ = tf;
      current_pos_ = final_pos_;
      current_vel_ = final_vel_;
      current_acc_ = final_acc_;
      return;
    }
    if (time <= t0)
    {
      current_time_ = t0;
      current_pos_ = initial_pos_;
      current_vel_ = initial_vel_;
      current_acc_ = initial_acc_;
      return;
    }

    const double tau = time - t0; // 정규화 시간
    const double t1 = tau;
    const double t2 = t1 * t1;
    const double t3 = t2 * t1;
    const double t4 = t3 * t1;
    const double t5 = t4 * t1;

    current_time_ = time;

    // 주의: 여기서의 time_variables_는 [tau^5, tau^4, ..., 1]
    time_variables_ << t5, t4, t3, t2, t1, 1.0;
    current_pos_ = time_variables_.dot(position_coeff_);
    current_vel_ = time_variables_.dot(velocity_coeff_);
    current_acc_ = time_variables_.dot(acceleration_coeff_);
  }
}
