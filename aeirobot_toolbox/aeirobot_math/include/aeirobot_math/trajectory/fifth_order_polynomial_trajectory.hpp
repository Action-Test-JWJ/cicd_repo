/**
 * @file fifth_order_polynomial_trajectory.hpp
 * @brief 5차 다항식 궤적 생성 및 관련 연산 클래스 선언
 */
#ifndef FIFTH_ORDER_POLYNOMIAL_TRAJECTORY_H_
#define FIFTH_ORDER_POLYNOMIAL_TRAJECTORY_H_

#include "aeirobot_math/algebra/linear_algebra.hpp"

namespace aeirobot
{
  /**
   * @class FifthOrderPolynomialTrajectory
   * @brief 초기 및 최종 조건을 기반으로 5차 다항식 형태의 시간 궤적을 계산
   */
  class FifthOrderPolynomialTrajectory
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
    FifthOrderPolynomialTrajectory(
        double initial_time, double initial_pos, double initial_vel, double initial_acc,
        double final_time, double final_pos, double final_vel, double final_acc);

    /**
     * @brief 기본 생성자 (모든 값 0으로 초기화)
     */
    FifthOrderPolynomialTrajectory();

    /**
     * @brief 소멸자
     */
    ~FifthOrderPolynomialTrajectory();

    /**
     * @brief 최종 조건만 변경하여 궤적을 갱신
     * @param final_pos 새 최종 위치
     * @param final_vel 새 최종 속도
     * @param final_acc 새 최종 가속도
     * @return 갱신 성공 시 true
     */
    bool changeTrajectory(double final_pos, double final_vel, double final_acc);

    /**
     * @brief 최종 시간 및 조건 변경하여 궤적을 갱신
     * @param final_time 새 종료 시간
     * @param final_pos 새 최종 위치
     * @param final_vel 새 최종 속도
     * @param final_acc 새 최종 가속도
     * @return 갱신 성공 시 true
     */
    bool changeTrajectory(double final_time, double final_pos, double final_vel, double final_acc);

    /**
     * @brief 시작 및 종료 조건 모두 변경하여 궤적을 갱신
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
     * @brief 특정 시간에 대한 위치 계산
     * @param time 계산할 시간
     * @return 해당 시간의 위치
     */
    double getPosition(double time);

    /**
     * @brief 특정 시간에 대한 속도 계산
     * @param time 계산할 시간
     * @return 해당 시간의 속도
     */
    double getVelocity(double time);

    /**
     * @brief 특정 시간에 대한 가속도 계산
     * @param time 계산할 시간
     * @return 해당 시간의 가속도
     */
    double getAcceleration(double time);

    /**
     * @brief 내부 시간 변수 갱신
     * @param time 현재 시간으로 설정
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
     * @brief 현재 시간에 대한 위치, 속도, 가속도를 계산
     * @param time 현재 시간
     * @return 계산 성공 시 true
     * @note 시간 변수(time_variables_)를 업데이트하고,
     *       position_coeff_, velocity_coeff_, acceleration_coeff_를 사용하여
     *       현재 위치(current_pos_), 속도(current_vel_), 가속도(current_acc_)을 계산합니다.
     */
    void Calculate(double time);

    /**
     * @brief 계수를 설정하여 다항식 궤적을 초기화
     * @note 내부적으로 시간 행렬(time_mat)과 조건 행렬(conditions_mat)을 계산하여
     *       position_coeff_, velocity_coeff_, acceleration_coeff_를 설정합니다.
     */
    void SetCoefficients();

    /** @brief 시작 시간 */
    double initial_time_ = 0;
    /** @brief 시작 위치 */
    double initial_pos_ = 0;
    /** @brief 시작 속도 */
    double initial_vel_ = 0;
    /** @brief 시작 가속도 */
    double initial_acc_ = 0;

    /** @brief 현재 시간 */
    double current_time_ = 0;
    /** @brief 현재 위치 */
    double current_pos_ = 0;
    /** @brief 현재 속도 */
    double current_vel_ = 0;
    /** @brief 현재 가속도 */
    double current_acc_ = 0;

    /** @brief 종료 시간 */
    double final_time_ = 0;
    /** @brief 종료 위치 */
    double final_pos_ = 0;
    /** @brief 종료 속도 */
    double final_vel_ = 0;
    /** @brief 종료 가속도 */
    double final_acc_ = 0;

    /** @brief 위치 다항식 계수 벡터 (6x1) */
    Eigen::Matrix<double, 6, 1> position_coeff_ = Eigen::Matrix<double, 6, 1>::Zero();
    /** @brief 속도 다항식 계수 벡터 (6x1) */
    Eigen::Matrix<double, 6, 1> velocity_coeff_ = Eigen::Matrix<double, 6, 1>::Zero();
    /** @brief 가속도 다항식 계수 벡터 (6x1) */
    Eigen::Matrix<double, 6, 1> acceleration_coeff_ = Eigen::Matrix<double, 6, 1>::Zero();
    /** @brief 시간 변수 벡터 [t^5, t^4, ..., 1] (1x6) */
    Eigen::Matrix<double, 1, 6> time_variables_ = Eigen::Matrix<double, 1, 6>::Zero();
  };

} // namespace aeirobot

#endif /* FIFTH_ORDER_POLYNOMIAL_TRAJECTORY_H_ */
