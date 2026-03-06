/**
 * @file alice_7dof_arm_kinematics.h
 * @brief Alice 7자유도(7-DOF) 로봇 팔의 기구학(Forward/Inverse) 계산을 위한 클래스 헤더
 */

#ifndef ALICE_7DOF_ARM_KINEMATICS__H_
#define ALICE_7DOF_ARM_KINEMATICS__H_

#include <rclcpp/rclcpp.hpp>
#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_math/math_tool.hpp"
#include "aeirobot_math/algebra/linear_algebra.hpp"
#include <string>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace aeirobot
{
  /**
   * @class ArmKinematics
   * @brief Alice 7자유도 로봇 팔의 순기구학/역기구학 계산 기능 제공 클래스
   */
  class ArmKinematics
  {
  public:
    /**
     * @brief 생성자. 7-DOF 로봇 팔의 링크 길이 및 파라미터 초기화
     * @param link_0 - 베이스 높이(m)
     * @param link_1 - 어깨 오프셋(m)
     * @param link_3 - 상완 길이(m)
     * @param link_5 - 하완 길이(m)
     * @param link_7 - 손목 오프셋(m)
     * @param epsilon - 계산 안정성용 임계값(0에 가까운 값 처리)
     * @param left_arm - true: 왼팔, false: 오른팔
     */
    ArmKinematics(double link_0, double link_1, double link_3, double link_5, double link_7, double epsilon, bool left_arm);

    /**
     * @brief 소멸자
     */
    ~ArmKinematics();

    /**
     * @brief 7-DOF 팔 역기구학(위치+자세 → 조인트각) 계산
     * @param auto_on - 자동 엘보 구동 모드 여부
     * @param x, y, z - 목표 엔드이펙터 위치(m)
     * @param roll, pitch, yaw - 목표 엔드이펙터 자세(라디안)
     * @param elbow_degree - 엘보 각도(도)
     * @return (ik 성공 여부, 7개 조인트각(rad)) 쌍 반환
     */
    void SetLinkLength(double link_0, double link_1, double link_3, double link_5, double link_7);

    double CalcElbowDegree(geometry_msgs::msg::Pose endpoint, geometry_msgs::msg::Pose elbow);

    std::pair<bool, std::vector<double>> CalcInverseKinematics(
        bool auto_on, double x, double y, double z,
        double roll, double pitch, double yaw, double elbow_degree);

    /**
     * @brief 7-DOF 팔 순기구학(조인트각 → Pose) 계산
     * @param joint_angles - 7개 조인트각 벡터 (라디안)
     * @return PoseXYZRPY(x, y, z, roll, pitch, yaw)
     */
    aeirobot_msgs::msg::PoseXYZRPY CalcForwardKinematics(const std::vector<double> &joint_angles);

    /**
     * @brief X좌표 인덱스 (상수)
     */
    const int k_x = 0;

    /**
     * @brief Y좌표 인덱스 (상수)
     */
    const int k_y = 1;

    /**
     * @brief Z좌표 인덱스 (상수)
     */
    const int k_z = 2;

    /**
     * @brief 조인트1 인덱스(어깨 Yaw) (상수)
     */
    const int k_joint_1 = 0;

    /**
     * @brief 조인트2 인덱스(어깨 Pitch) (상수)
     */
    const int k_joint_2 = 1;

    /**
     * @brief 조인트3 인덱스(어깨 Roll) (상수)
     */
    const int k_joint_3 = 2;

    /**
     * @brief 조인트4 인덱스(엘보 Pitch) (상수)
     */
    const int k_joint_4 = 3;

    /**
     * @brief 조인트5 인덱스(손목 Yaw) (상수)
     */
    const int k_joint_5 = 4;

    /**
     * @brief 조인트6 인덱스(손목 Pitch) (상수)
     */
    const int k_joint_6 = 5;

    /**
     * @brief 조인트7 인덱스(손목 Roll) (상수)
     */
    const int k_joint_7 = 6;

  private:
    /**
     * @brief DH파라미터: 베이스(링크0) 높이 (m)
     */
    double l0;

    /**
     * @brief DH파라미터: 어깨 오프셋(링크1) (m)
     */
    double l1;

    /**
     * @brief DH파라미터: 상완 길이(링크3) (m)
     */
    double l3;

    /**
     * @brief DH파라미터: 하완 길이(링크5) (m)
     */
    double l5;

    /**
     * @brief DH파라미터: 손목 오프셋(링크7) (m)
     */
    double l7;

    /**
     * @brief 0근처에서 계산 불안정 방지용 작은 수(임계값)
     */
    double k_epsilon;

    /**
     * @brief true: 왼팔, false: 오른팔 사용
     */
    bool is_left_arm;

    float prev_theta[7] = {0, 0, 0, 0, 0, 0, 0};
  };
}

#endif /* ALICE_7DOF_ARM_KINEMATICS__H_ */
