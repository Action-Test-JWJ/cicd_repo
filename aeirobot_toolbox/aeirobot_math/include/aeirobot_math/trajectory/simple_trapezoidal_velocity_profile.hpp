/**
 * @file simple_trapezoidal_velocity_profile.hpp
 * @brief 단순 트랩에조이달 속도 프로파일을 생성하고 관련 시뮬레이션을 수행하는 클래스 선언
 */
#ifndef TRAPEZOIDAL_VELOCITY_PROFILE_H_
#define TRAPEZOIDAL_VELOCITY_PROFILE_H_

#include "aeirobot_math/algebra/linear_algebra.hpp"

namespace aeirobot
{
  /**
   * @class SimpleTrapezoidalVelocityProfile
   * @brief 가속-등속-감속 3구간으로 구성된 트래펫조이달 속도 궤적을 계산
   */
  class SimpleTrapezoidalVelocityProfile
  {
  public:
    /**
     * @brief 생성자
     */
    SimpleTrapezoidalVelocityProfile();

    /**
     * @brief 소멸자
     */
    ~SimpleTrapezoidalVelocityProfile();

    /**
     * @brief 가속도와 최대 속도 기반으로 궤적 조건 설정
     * @param init_pos 시작 위치
     * @param final_pos 종료 위치
     * @param acceleration 가속도
     * @param max_velocity 최대 속도
     */
    void setVelocityBaseTrajectory(
        double init_pos, double final_pos,
        double acceleration, double max_velocity);

    /**
     * @brief 가속도, 감속도 및 최대 속도 기반으로 궤적 조건 설정
     * @param init_pos 시작 위치
     * @param final_pos 종료 위치
     * @param acceleration 가속도
     * @param deceleration 감속도
     * @param max_velocity 최대 속도
     */
    void setVelocityBaseTrajectory(
        double init_pos, double final_pos,
        double acceleration, double deceleration, double max_velocity);

    /**
     * @brief 가속 및 전체 시간 기반으로 궤적 조건 설정
     * @param init_pos 시작 위치
     * @param final_pos 종료 위치
     * @param accel_time 가속 구간 시간
     * @param total_time 전체 구간(가속+등속+감속) 시간
     */
    void setTimeBaseTrajectory(
        double init_pos, double final_pos,
        double accel_time, double total_time);

    /**
     * @brief 가속, 감속 및 전체 시간 기반으로 궤적 조건 설정
     * @param init_pos 시작 위치
     * @param final_pos 종료 위치
     * @param accel_time 가속 구간 시간
     * @param decel_time 감속 구간 시간
     * @param total_time 전체 구간 시간
     */
    void setTimeBaseTrajectory(
        double init_pos, double final_pos,
        double accel_time, double decel_time, double total_time);

    /**
     * @brief 특정 시각에서 위치 계산
     * @param time 계산할 시간
     * @return 위치
     */
    double getPosition(double time);

    /**
     * @brief 특정 시각에서 속도 계산
     * @param time 계산할 시간
     * @return 속도
     */
    double getVelocity(double time);

    /**
     * @brief 특정 시각에서 가속도 계산
     * @param time 계산할 시간
     * @return 가속도
     */
    double getAcceleration(double time);

    /**
     * @brief 내부 시간 변수 갱신
     * @param time 현재 시간
     */
    void setTime(double time);

    /**
     * @brief 마지막으로 계산된 위치 반환
     * @return 현재 위치
     */
    double getPosition();

    /**
     * @brief 마지막으로 계산된 속도 반환
     * @return 현재 속도
     */
    double getVelocity();

    /**
     * @brief 마지막으로 계산된 가속도 반환
     * @return 현재 가속도
     */
    double getAcceleration();

    /**
     * @brief 총 궤적 수행 시간 반환
     * @return 총 시간
     */
    double getTotalTime();

    /**
     * @brief 등속 구간 시작 시간 반환
     * @return 등속 구간 시작 시각
     */
    double getConstantVelocitySectionStartTime();

    /**
     * @brief 감속 구간 시작 시간 반환
     * @return 감속 구간 시작 시각
     */
    double getDecelerationSectionStartTime();

  private:
    /** @brief 가속 구간 위치 계수 (2차 다항식) */
    // Eigen::MatrixXd pos_coeff_accel_section_;
    Eigen::Matrix<double, 3, 1> pos_coeff_accel_section_;
    /** @brief 가속 구간 속도 계수 (1차 다항식) */
    // Eigen::MatrixXd vel_coeff_accel_section_;
    Eigen::Matrix<double, 3, 1> vel_coeff_accel_section_;

    /** @brief 등속 구간 위치 계수 (1차) */
    // Eigen::MatrixXd pos_coeff_const_section_;
    Eigen::Matrix<double, 3, 1> pos_coeff_const_section_;
    /** @brief 등속 구간 속도 계수 (상수) */
    // Eigen::MatrixXd vel_coeff_const_section_;
    Eigen::Matrix<double, 3, 1> vel_coeff_const_section_;

    /** @brief 감속 구간 위치 계수 (2차) */
    // Eigen::MatrixXd pos_coeff_decel_section_;
    Eigen::Matrix<double, 3, 1> pos_coeff_decel_section_;
    /** @brief 감속 구간 속도 계수 (1차) */
    // Eigen::MatrixXd vel_coeff_decel_section_;
    Eigen::Matrix<double, 3, 1> vel_coeff_decel_section_;

    /** @brief 시간 변수 벡터 [t^2, t, 1] */
    // Eigen::MatrixXd time_variables_;
    Eigen::Matrix<double, 1, 3> time_variables_;

    /** @brief 가속도 */
    double acceleration_;
    /** @brief 감속도 */
    double deceleration_;
    /** @brief 최대 속도 */
    double max_velocity_;

    /** @brief 시작 위치 */
    double initial_pos_;
    /** @brief 종료 위치 */
    double final_pos_;

    /** @brief 현재 시간 */
    double current_time_;
    /** @brief 현재 위치 */
    double current_pos_;
    /** @brief 현재 속도 */
    double current_vel_;
    /** @brief 현재 가속도 */
    double current_acc_;

    /** @brief 가속 구간 시간 */
    double accel_time_;
    /** @brief 등속 구간 시간 */
    double const_time_;
    /** @brief 감속 구간 시간 */
    double decel_time_;

    /** @brief 등속 구간 시작 시간 */
    double const_start_time_;
    /** @brief 감속 구간 시작 시간 */
    double decel_start_time_;
    /** @brief 전체 궤적 시간 */
    double total_time_;
  };

} // namespace aeirobot

#endif // TRAPEZOIDAL_VELOCITY_PROFILE_H_
