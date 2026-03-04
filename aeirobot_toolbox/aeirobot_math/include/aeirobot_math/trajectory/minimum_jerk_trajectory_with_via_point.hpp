/**
 * @file minimum_jerk_trajectory_with_via_point.hpp
 * @brief 중간 지점을 포함하여 최소 저크 궤적을 계산하는 클래스 선언
 */
#ifndef MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_
#define MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_

#include "aeirobot_math/algebra/linear_algebra.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <vector>

namespace aeirobot
{
  /**
   * @class MinimumJerkViaPoint
   * @brief 시작, 중간, 종료 지점 조건에 기반한 최소 저크 궤적 생성
   */
  class MinimumJerkViaPoint
  {
  public:
    /**
     * @brief 생성자
     * @param ini_time 시작 시간
     * @param fin_time 종료 시간
     * @param via_time 중간 지점 시간
     * @param ratio 중간 구간 시간 비율
     * @param ini_pos 시작 위치 벡터
     * @param ini_vel 시작 속도 벡터
     * @param ini_acc 시작 가속도 벡터
     * @param fin_pos 종료 위치 벡터
     * @param fin_vel 종료 속도 벡터
     * @param fin_acc 종료 가속도 벡터
     * @param via_pos 중간 지점 위치 벡터
     * @param via_vel 중간 지점 속도 벡터 (미사용)
     * @param via_acc 중간 지점 가속도 벡터 (미사용)
     */
    MinimumJerkViaPoint(
        double ini_time, double fin_time, double via_time, double ratio,
        std::vector<double_t> ini_pos, std::vector<double_t> ini_vel, std::vector<double_t> ini_acc,
        std::vector<double_t> fin_pos, std::vector<double_t> fin_vel, std::vector<double_t> fin_acc,
        std::vector<double_t> via_pos, std::vector<double_t> via_vel, std::vector<double_t> via_acc);

    /**
     * @brief 소멸자
     */
    virtual ~MinimumJerkViaPoint();

    /**
     * @brief 특정 시간에서 위치 벡터 계산
     * @param time 계산할 시간
     * @return 위치 벡터
     */
    std::vector<double_t> getPosition(double time);

    /**
     * @brief 특정 시간에서 속도 벡터 계산
     * @param time 계산할 시간
     * @return 속도 벡터
     */
    std::vector<double_t> getVelocity(double time);

    /**
     * @brief 특정 시간에서 가속도 벡터 계산
     * @param time 계산할 시간
     * @return 가속도 벡터
     */
    std::vector<double_t> getAcceleration(double time);

    /** @brief 현재 시간 */
    double cur_time_;
    /** @brief 현재 위치 벡터 */
    std::vector<double_t> cur_pos_;
    /** @brief 현재 속도 벡터 */
    std::vector<double_t> cur_vel_;
    /** @brief 현재 가속도 벡터 */
    std::vector<double_t> cur_acc_;

    /** @brief 위치 계수 행렬 (7xN) */
    // Eigen::MatrixXd position_coeff_;
    Eigen::Matrix<double, 7, Eigen::Dynamic> position_coeff_;
    /** @brief 속도 계수 행렬 (7xN) */
    // Eigen::MatrixXd velocity_coeff_;
    Eigen::Matrix<double, 7, Eigen::Dynamic> velocity_coeff_;
    /** @brief 가속도 계수 행렬 (7xN) */
    // Eigen::MatrixXd acceleration_coeff_;
    Eigen::Matrix<double, 7, Eigen::Dynamic> acceleration_coeff_;
    /** @brief 시간 변수 행렬 [t^5...1,(t-via)^k] (1x7) */
    // Eigen::MatrixXd time_variables_;
    Eigen::Matrix<double, 1, 7> time_variables_;

  private:
    /** @brief 관절(차원) 수 */
    int number_of_joint_;
    /** @brief 중간 구간 비율 */
    double ratio_;
    /** @brief 입력 시작/종료 시간 */
    double input_ini_time_, input_fin_time_;
    /** @brief 내부 시작/종료/중간 시간 */
    double ini_time_, fin_time_, via_time_;
    /** @brief 시작/종료/중간 위치, 속도, 가속도 벡터 */
    std::vector<double_t> ini_pos_, ini_vel_, ini_acc_;
    std::vector<double_t> fin_pos_, fin_vel_, fin_acc_;
    std::vector<double_t> via_pos_, via_vel_, via_acc_;
  };

} // namespace aeirobot

#endif // MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_
