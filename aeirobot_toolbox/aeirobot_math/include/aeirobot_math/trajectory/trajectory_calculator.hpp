/**
 * @file trajectory_calculator.hpp
 * @brief 다양한 궤적 계산 함수를 선언하는 유틸리티 헤더
 */
#ifndef TRAJECTORY_CALCULATOR_H_
#define TRAJECTORY_CALCULATOR_H_

#include "aeirobot_math/trajectory/fifth_order_polynomial_trajectory.hpp"
#include "aeirobot_math/trajectory/seventh_order_polynomial_trajectory.hpp"
#include "aeirobot_math/trajectory/ninth_order_polynomial_trajectory.hpp"
#include "aeirobot_math/trajectory/simple_trapezoidal_velocity_profile.hpp"
#include <Eigen/Dense>

namespace aeirobot
{
    /**
     * @brief 단순 최소 저크 궤적을 계산하여 위치만 반환
     * @param pos_start 시작 위치
     * @param vel_start 시작 속도
     * @param accel_start 시작 가속도
     * @param pos_end 종료 위치
     * @param vel_end 종료 속도
     * @param accel_end 종료 가속도
     * @param smp_time 샘플링 주기
     * @param mov_time 이동 총 시간
     * @return 행렬(시간 단계 × 1) 형태의 위치 궤적
     */
    // Eigen::MatrixXd calcMinimumJerkTra(
    Eigen::Matrix<double, Eigen::Dynamic, 1> calcMinimumJerkTra(
        double pos_start, double vel_start, double accel_start,
        double pos_end, double vel_end, double accel_end,
        double smp_time, double mov_time);

    /**
     * @brief 최소 저크 궤적의 위치, 속도, 가속도를 함께 계산
     * @param pos_start 시작 위치
     * @param vel_start 시작 속도
     * @param accel_start 시작 가속도
     * @param pos_end 종료 위치
     * @param vel_end 종료 속도
     * @param accel_end 종료 가속도
     * @param smp_time 샘플링 주기
     * @param mov_time 이동 총 시간
     * @return 행렬(시간 단계 × 3) 형태의 [위치, 속도, 가속도]
     */
    // Eigen::MatrixXd calcMinimumJerkTraPlus(
    Eigen::Matrix<double, Eigen::Dynamic, 3> calcMinimumJerkTraPlus(
        double pos_start, double vel_start, double accel_start,
        double pos_end, double vel_end, double accel_end,
        double smp_time, double mov_time);

    /**
     * @brief 중간 지점을 포함한 최소 저크 궤적 계산 (위치, 속도, 가속도 제약)
     * @param via_num 중간 지점 수
     * @param pos_start 시작 위치
     * @param vel_start 시작 속도
     * @param accel_start 시작 가속도
     * @param pos_via 중간 지점 위치 벡터 (n×1)
     * @param vel_via 중간 지점 속도 벡터 (n×1)
     * @param accel_via 중간 지점 가속도 벡터 (n×1)
     * @param pos_end 종료 위치
     * @param vel_end 종료 속도
     * @param accel_end 종료 가속도
     * @param smp_time 샘플링 주기
     * @param via_time 중간 지점 통과 시간 벡터 (n×1)
     * @param mov_time 이동 총 시간
     * @return 행렬(시간 단계 × 1) 형태의 위치 궤적
     */
    // Eigen::MatrixXd calcMinimumJerkTraWithViaPoints(
    Eigen::Matrix<double, Eigen::Dynamic, 1> calcMinimumJerkTraWithViaPoints(
        int via_num,
        double pos_start, double vel_start, double accel_start,
        Eigen::MatrixXd pos_via, Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
        double pos_end, double vel_end, double accel_end,
        double smp_time, Eigen::MatrixXd via_time, double mov_time);

    /**
     * @brief 중간 지점 위치만 제약하는 최소 저크 궤적 계산
     * @param via_num 중간 지점 수
     * @param pos_start 시작 위치
     * @param vel_start 시작 속도
     * @param accel_start 시작 가속도
     * @param pos_via 중간 지점 위치 벡터 (n×1)
     * @param pos_end 종료 위치
     * @param vel_end 종료 속도
     * @param accel_end 종료 가속도
     * @param smp_time 샘플링 주기
     * @param via_time 중간 지점 통과 시간 벡터 (n×1)
     * @param mov_time 이동 총 시간
     * @return 행렬(시간 단계 × 1) 형태의 위치 궤적
     */
    // Eigen::MatrixXd calcMinimumJerkTraWithViaPointsPosition(
    Eigen::Matrix<double, Eigen::Dynamic, 1> calcMinimumJerkTraWithViaPointsPosition(
        int via_num,
        double pos_start, double vel_start, double accel_start,
        Eigen::MatrixXd pos_via,
        double pos_end, double vel_end, double accel_end,
        double smp_time, Eigen::MatrixXd via_time, double mov_time);

    /**
     * @brief 3D 호(arc) 궤적을 계산
     * @param smp_time 샘플링 주기
     * @param mov_time 이동 총 시간
     * @param center_point 호의 중심 좌표 (3×1)
     * @param normal_vector 호 면 법선 벡터 (3×1)
     * @param start_point 시작 좌표 (3×1)
     * @param rotation_angle 회전 각도
     * @param cross_ratio 호의 크로스 비율 (끝점 근처 변형)
     * @return 행렬(시간 단계 × 3) 형태의 3D 호 좌표 궤적
     */
    // Eigen::MatrixXd calcArc3dTra(
    Eigen::Matrix<double, Eigen::Dynamic, 3> calcArc3dTra(
        double smp_time, double mov_time,
        Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
        double rotation_angle, double cross_ratio);

} // namespace aeirobot

#endif // TRAJECTORY_CALCULATOR_H_
