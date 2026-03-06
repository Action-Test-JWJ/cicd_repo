/**
 * @file link_data.h
 * @brief 로봇 링크(관절) 하나의 정보를 저장하는 데이터 클래스 정의 헤더
 */

#ifndef ALICE_KINEMATICS_DYNAMICS_LINK_DATA_H_
#define ALICE_KINEMATICS_DYNAMICS_LINK_DATA_H_

#include "aeirobot_math/algebra/linear_algebra.hpp"

namespace aeirobot
{
  /**
   * @class LinkData
   * @brief 로봇의 한 링크(관절)의 파라미터, 동적 상태, 위치/자세 등을 저장하는 클래스
   */
  class LinkData
  {
  public:
    /**
     * @brief 생성자 - 모든 멤버 변수 기본값으로 초기화
     */
    LinkData();

    /**
     * @brief 소멸자
     */
    ~LinkData();

    /**
     * @brief 링크(관절) 이름 (예: "l_hip_y")
     */
    std::string name_;

    /**
     * @brief 부모 링크 ID (없으면 -1)
     */
    int parent_;

    /**
     * @brief 형제 링크 ID (없으면 -1)
     */
    int sibling_;

    /**
     * @brief 자식 링크 ID (없으면 -1)
     */
    int child_;

    /**
     * @brief 질량 (kg)
     */
    double mass_;

    /**
     * @brief 부모 기준 상대 위치 (3x1 벡터)
     */
    // Eigen::MatrixXd relative_position_;
    Eigen::Vector3d relative_position_;

    /**
     * @brief 조인트 회전축 (3x1 벡터)
     */
    // Eigen::MatrixXd joint_axis_;
    Eigen::Vector3d joint_axis_;

    /**
     * @brief 무게중심 (CoM) 위치 (3x1 벡터)
     */
    // Eigen::MatrixXd center_of_mass_;
    Eigen::Vector3d center_of_mass_;

    /**
     * @brief 링크의 관성 행렬 (3x3 또는 6x1 벡터)
     */
    // Eigen::MatrixXd inertia_;
    Eigen::Matrix<double, 3, 3> inertia_; // 현재는 3x3 행렬만 반환 값 받음(출처 -> aeirobot::getInertiaXYZ)

    /**
     * @brief 조인트 기준 무게중심 (3x1 벡터)
     */
    // Eigen::MatrixXd joint_center_of_mass_;
    Eigen::Matrix<double, 3, 1> joint_center_of_mass_;

    /**
     * @brief 조인트의 최대 가동 한계 (라디안)
     */
    double joint_limit_max_;

    /**
     * @brief 조인트의 최소 가동 한계 (라디안)
     */
    double joint_limit_min_;

    /**
     * @brief 현재 조인트 각도 (라디안)
     */
    double joint_angle_;

    /**
     * @brief 현재 조인트 속도 (라디안/초)
     */
    double joint_velocity_;

    /**
     * @brief 현재 조인트 가속도 (라디안/제곱초)
     */
    double joint_acceleration_;

    /**
     * @brief 링크의 월드 기준 위치 (3x1 벡터)
     */
    // Eigen::MatrixXd position_;
    Eigen::Vector3d position_;

    /**
     * @brief 링크의 월드 기준 회전 행렬 (3x3)
     */
    // Eigen::MatrixXd orientation_;
    Eigen::Matrix<double, 3, 3> orientation_;

    /**
     * @brief 링크의 전체 변환 행렬(4x4, 위치+자세)
     */
    // Eigen::MatrixXd transformation_;
    Eigen::Matrix<double, 4, 4> transformation_;
  };
}

#endif /* ALICE_KINEMATICS_DYNAMICS_LINK_DATA_H_ */
