/**
 * @file seventh_order_polynomial_trajectory.hpp
 * @brief 7차 다항식 궤적 생성 및 관련 연산 클래스 선언
 */
#ifndef SEVENTH_ORDER_POLYNOMIAL_TRAJECTORY_H_
#define SEVENTH_ORDER_POLYNOMIAL_TRAJECTORY_H_

#include "aeirobot_math/algebra/linear_algebra.hpp"

namespace aeirobot
{
  /**
   * @class SeventhOrderPolynomialTrajectory
   * @brief 초기 및 최종 조건을 기반으로 7차 다항식 궤적을 계산
   */
  class SeventhOrderPolynomialTrajectory
  {
  public:
    /**
     * @brief 파라미터화된 궤적 초기화 생성자
     * @param initial_time 시작 시간
     * @param initial_pos 시작 위치
     * @param initial_vel 시작 속도
     * @param initial_acc 시작 가속도
     * @param final_time 종료 시간
     * @param final_pos 종료 위치
     * @param final_vel 종료 속도
     * @param final_acc 종료 가속도
     */
    SeventhOrderPolynomialTrajectory(
        double initial_time, double initial_pos, double initial_vel, double initial_acc,
        double final_time, double final_pos, double final_vel, double final_acc);

    /**
     * @brief 기본 생성자 (모든 값 0으로 초기화)
     */
    SeventhOrderPolynomialTrajectory();

    /**
     * @brief 소멸자
     */
    ~SeventhOrderPolynomialTrajectory();

    /**
     * @brief 최종 위치, 속도, 가속도만 변경하여 궤적 갱신
     * @param final_pos 새 최종 위치
     * @param final_vel 새 최종 속도
     * @param final_acc 새 최종 가속도
     * @return 갱신 성공 시 true
     */
    bool changeTrajectory(double final_pos, double final_vel, double final_acc);

    /**
     * @brief 최종 시간 및 조건 변경하여 궤적 갱신
     * @param final_time 새 종료 시간
     * @param final_pos 새 최종 위치
     * @param final_vel 새 최종 속도
     * @param final_acc 새 최종 가속도
     * @return 갱신 성공 시 true
     */
    bool changeTrajectory(double final_time, double final_pos, double final_vel, double final_acc);

    /**
     * @brief 시작 및 종료 조건 모두 변경하여 궤적 갱신
     * @param initial_time 새 시작 시간
     * @param initial_pos 새 시작 위치
     * @param initial_vel 새 시작 속도
     * @param initial_acc 새 시작 가속도
     * @param final_time 새 종료 시간
     * @param final_pos 새 최종 위치
     * @param final_vel 새 최종 속도
     * @param final_acc 새 최종 가속도
     * @return 갱신 성공 시 true
     */
    bool changeTrajectory(
        double initial_time, double initial_pos, double initial_vel, double initial_acc,
        double final_time, double final_pos, double final_vel, double final_acc);

    /**
     * @brief 특정 시간에서 위치 계산
     * @param time 계산할 시간
     * @return 위치 값
     */
    double getPosition(double time);

    /**
     * @brief 특정 시간에서 속도 계산
     * @param time 계산할 시간
     * @return 속도 값
     */
    double getVelocity(double time);

    /**
     * @brief 특정 시간에서 가속도 계산
     * @param time 계산할 시간
     * @return 가속도 값
     */
    double getAcceleration(double time);

    /**
     * @brief 특정 시간에서 저크 계산
     * @param time 계산할 시간
     * @return 저크 값
     */
    double getJerk(double time);

    /**
     * @brief 내부 시간 변수 갱신
     * @param time 현재 시간 설정
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
     * @brief 마지막으로 계산된 저크 반환
     * @return 현재 저크
     */
    double getJerk();

    /** @brief 시작 시간 */
    double initial_time_;
    /** @brief 시작 위치 */
    double initial_pos_;
    /** @brief 시작 속도 */
    double initial_vel_;
    /** @brief 시작 가속도 */
    double initial_acc_;
    /** @brief 시작 저크 */
    double initial_jerk_;

    /** @brief 현재 시간 */
    double current_time_;
    /** @brief 현재 위치 */
    double current_pos_;
    /** @brief 현재 속도 */
    double current_vel_;
    /** @brief 현재 가속도 */
    double current_acc_;
    /** @brief 현재 저크 */
    double current_jerk_;

    /** @brief 종료 시간 */
    double final_time_;
    /** @brief 종료 위치 */
    double final_pos_;
    /** @brief 종료 속도 */
    double final_vel_;
    /** @brief 종료 가속도 */
    double final_acc_;
    /** @brief 종료 저크 */
    double final_jerk_;

    /** @brief 위치 다항식 계수 벡터 (8x1) */
    // Eigen::MatrixXd position_coeff_;
    Eigen::Matrix<double, 8, Eigen::Dynamic> position_coeff_;
    /** @brief 속도 다항식 계수 벡터 (8x1) */
    // Eigen::MatrixXd velocity_coeff_;
    Eigen::Matrix<double, 8, Eigen::Dynamic> velocity_coeff_;
    /** @brief 가속도 다항식 계수 벡터 (8x1) */
    // Eigen::MatrixXd acceleration_coeff_;
    Eigen::Matrix<double, 8, Eigen::Dynamic> acceleration_coeff_;
    /** @brief 저크 다항식 계수 벡터 (8x1) */
    // Eigen::MatrixXd jerk_coeff_;
    Eigen::Matrix<double, 8, Eigen::Dynamic> jerk_coeff_;
    /** @brief 시간 변수 벡터 [t^7...1] (1x8) */
    // Eigen::MatrixXd time_variables_;
    Eigen::Matrix<double, 1, 8> time_variables_;
  };

} // namespace aeirobot

#endif // SEVENTH_ORDER_POLYNOMIAL_TRAJECTORY_H_
