/**
 * @file virtual_joint_map.h
 * @brief Alice4 로봇 가상 조인트 맵핑 클래스 정의
 *
 * @details
 *   - 허리, 골반, 무릎, 발목 가상 조인트와 리니어 액추에이터 간 변환 함수 선언
 *   - 순방향(FK) 및 역방향(IK) 매핑 함수 포함
 */
#ifndef VIRTUAL_JOINT_MAP_H_
#define VIRTUAL_JOINT_MAP_H_

#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_math/math_tool.hpp"
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

namespace aeirobot
{
  /**
   * @class VirtualJointMap
   * @brief 리니어 액추에이터와 가상 조인트(hip, knee, ankle) 간 변환 수행
   *
   * 버전에 따라 서로 다른 모델 파라미터 사용 가능
   */
  class VirtualJointMap
  {
  public:
    /**
     * @brief 생성자: Alice4 버전 파라미터 로드
     */
    VirtualJointMap();

    /** @brief Alice4 버전 */
    double alice4_version = 1;

    // --- 순방향 매핑 (Forward Kinematics) ---
    /**
     * @brief 무릎 가상 조인트 매핑 (pitch)
     *
     * @param linear_1 리니어 액추에이터 1 위치 [m]
     * @param linear_1_dot 리니어 액추에이터 1 속도 [m/s]
     * @param linear_1_effort 리니어 액추에이터 1 힘/토크 [N or Nm]
     * @return std::tuple<double, double, double> <pitch [rad], pitch_dot [rad/s], knee_torque [Nm]>
     */
    std::tuple<double, double, double>
    CalculateKneeLinearMap(double linear_1, double linear_1_dot, double linear_1_effort);

    /**
     * @brief 골반(hip) 가상 조인트 매핑 (pitch, roll)
     *
     * @param linear_1 리니어 액추에이터 1 위치 [m]
     * @param linear_2 리니어 액추에이터 2 위치 [m]
     * @param linear_1_dot 리니어 액추에이터 1 속도 [m/s]
     * @param linear_2_dot 리니어 액추에이터 2 속도 [m/s]
     * @param linear_1_effort 리니어 액추에이터 1 토크 [Nm]
     * @param linear_2_effort 리니어 액추에이터 2 토크 [Nm]
     * @return std::tuple<double, double, double, double, double, double>
     *         <pitch [rad], roll [rad], pitch_dot [rad/s], roll_dot [rad/s], hip_pitch_torque [Nm], hip_roll_torque [Nm]>
     */
    std::tuple<double, double, double, double, double, double>
    CalculateHipLinearMap(
        double linear_1,
        double linear_2,
        double linear_1_dot,
        double linear_2_dot,
        double linear_1_effort,
        double linear_2_effort);

    /**
     * @brief 발목(ankle) 가상 조인트 매핑 (pitch, roll)
     *
     * @param linear_1 리니어 액추에이터 1 위치 [m]
     * @param linear_2 리니어 액추에이터 2 위치 [m]
     * @param linear_1_dot 리니어 액추에이터 1 속도 [m/s]
     * @param linear_2_dot 리니어 액추에이터 2 속도 [m/s]
     * @param linear_1_effort 리니어 액추에이터 1 토크 [Nm]
     * @param linear_2_effort 리니어 액추에이터 2 토크 [Nm]
     * @return std::tuple<double, double, double, double, double, double>
     *         <pitch [rad], roll [rad], pitch_dot [rad/s], roll_dot [rad/s], ankle_pitch_torque [Nm], ankle_roll_torque [Nm]>
     */
    std::tuple<double, double, double, double, double, double>
    CalculateAnkleLinearMap(
        double linear_1,
        double linear_2,
        double linear_1_dot,
        double linear_2_dot,
        double linear_1_effort,
        double linear_2_effort);

    // --- 역방향 매핑 (Inverse Kinematics) ---
    /**
     * @brief 무릎 pitch 각도에서 리니어 위치 계산
     *
     * @param pitch 무릎 pitch 각도 [rad]
     * @return double linear actuator 길이
     */
    double MapLinearPosKnee(double pitch) const;

    /**
     * @brief 골반 pitch, roll에서 리니어 위치 계산
     *
     * @param pitch 골반 pitch 각도 [rad]
     * @param roll 골반 roll 각도 [rad]
     * @return std::vector<double> {left_actuator, right_actuator} 길이 벡터
     */
    std::vector<double> MapLinearPosPelvis(double pitch, double roll) const;

    /**
     * @brief 발목 pitch, roll에서 리니어 위치 계산
     *
     * @param pitch 발목 pitch 각도 [rad]
     * @param roll 발목 roll 각도 [rad]
     * @return std::vector<double> {left_actuator, right_actuator} 길이 벡터
     */
    std::vector<double> MapLinearPosAnkle(double pitch, double roll) const;

    /**
     * @brief 골반 pitch, roll 토크에서 리니어 effort 계산
     *
     * @param pitch 골반 pitch 각도 [rad]
     * @param hip_pitch_torque 골반 pitch 토크 [Nm]
     * @param hip_roll_torque 골반 roll 토크 [Nm]
     * @return std::vector<double> {effort_1, effort_2} 벡터
     */
    std::vector<double> MapLinearEffortPelvis(double pitch, double hip_pitch_torque, double hip_roll_torque);

    /**
     * @brief 발목 pitch, roll 토크에서 리니어 effort 계산
     *
     * @param pitch 발목 pitch 각도 [rad]
     * @param ankle_pitch_torque 발목 pitch 토크 [Nm]
     * @param ankle_roll_torque 발목 roll 토크 [Nm]
     * @return std::vector<double> {effort_1, effort_2} 벡터
     */
    std::vector<double> MapLinearEffortAnkle(double pitch, double ankle_pitch_torque, double ankle_roll_torque);

    /**
     * @brief 무릎 토크에서 리니어 effort를 계산하는 함수
     *
     * @param pitch 무릎의 pitch(각도, [rad])
     * @param knee_torque 무릎의 토크 값 [Nm]
     * @return double 계산된 리니어 effort 값
     */
    double MapLinearEffortKnee(double pitch, double knee_torque);
  };
}

#endif /* VIRTUAL_JOINT_MAP_H_ */
