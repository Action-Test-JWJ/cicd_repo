/**
 * @file alice4_kinematics_dynamics.h
 * @brief Alice4 로봇의 기구학/동역학 계산 및 IK/FK 유틸리티를 제공하는 클래스 정의
 */

#ifndef ALICE4_KINEMATICS_DYNAMICS_H_
#define ALICE4_KINEMATICS_DYNAMICS_H_

#include <rclcpp/rclcpp.hpp>
#include "aeirobot_robotics/kinematics/link_data.h"
#include "aeirobot_robotics/kinematics/kinematics_define.h"
#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_math/math_tool.hpp"
#include "aeirobot_math/algebra/linear_algebra.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace aeirobot
{
  /**
   * @class KinematicsDynamics
   * @brief Alice4 로봇의 기구학/동역학 계산을 담당하는 클래스
   */
  class KinematicsDynamics
  {
  public:
    /**
     * @brief 생성자: 로봇 파라미터 초기화 및 모델 로딩
     */
    KinematicsDynamics();

    /**
     * @brief 소멸자
     */
    ~KinematicsDynamics();

    /**
     * @brief kinematics_dynamics.yaml 파일로부터 기구학 정보 읽기
     * @return 없음
     */
    void ReadKinematicsYaml();

    /**
     * @brief 지정된 조인트 ID에 대해 재귀적으로 Forward Kinematics 계산
     * @param joint_ID - 계산을 시작할 조인트 ID
     * @return 없음
     */
    void CalcForwardKinematics(int joint_ID);

    /**
     * @brief YAML에서 로드된 kinematics 정보 저장용 객체
     */
    YAML::Node kinematics_doc_;

    /**
     * @brief Alice4 버전
     */
    int alice4_version = 1;

    /**
     * @brief 각 조인트 ID에 해당하는 링크 이름 (크기: ALL_JOINT_ID)
     */
    std::string kine_link_name_[ALL_JOINT_ID];

    /**
     * @brief 일반 다리(왼/오른 다리) 역기구학 계산 (내부 용도, 직접 호출 X)
     * @param out - 결과값(각 관절 각) 배열 포인터 (최소 6개)
     * @param x, y, z, roll, pitch, yaw - 목표 위치/자세(라디안)
     * @param is_left - true: 왼쪽 다리, false: 오른쪽 다리
     * @return 성공시 true, 실패시 false
     */
    bool ComputeInverseKinematics(double *out, double x, double y, double z, double roll, double pitch, double yaw, bool is_left);

    /**
     * @brief 오른쪽 다리 역기구학 계산 (일반 IK + Iterative IK 사용)
     * @param out - 결과값(각 관절 각) 배열 포인터 (최소 6개)
     * @param x, y, z, roll, pitch, yaw - 목표 위치/자세(라디안)
     * @return 성공시 true, 실패시 false
     */
    bool ComputeInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);

    /**
     * @brief 왼쪽 다리 역기구학 계산 (일반 IK + Iterative IK 사용)
     * @param out - 결과값(각 관절 각) 배열 포인터 (최소 6개)
     * @param x, y, z, roll, pitch, yaw - 목표 위치/자세(라디안)
     * @return 성공시 true, 실패시 false
     */
    bool ComputeInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);

    /**
     * @brief 각 링크 데이터 포인터 배열 (크기: ALL_JOINT_ID+1)
     */
    LinkData *alice4_link_data_[ALL_JOINT_ID + 1];

    /**
     * @brief 허벅지 길이 (m)
     */
    double thigh_length_m_;

    /**
     * @brief 종아리 길이 (m)
     */
    double calf_length_m_;

    /**
     * @brief 발목 길이 (m)
     */
    double ankle_length_m_;

    // === DH, FK/IK 등 기구학 변수들 ===

    /**
     * @brief 변환 행렬 H (크기: 8개, 각 4x4)
     */
    // Eigen::MatrixXd H[8];
    Eigen::Matrix<double, 4, 4> H[8];

    /**
     * @brief 바닥-중심 프레임 변환 행렬 (4x4)
     */
    // Eigen::MatrixXd H_ground_to_center;
    Eigen::Matrix<double, 4, 4> H_ground_to_center;

    /**
     * @brief IK 계산용 임시 변수: 각 관절의 각도 (7개)
     */
    double real_theta[7];

    /**
     * @brief DH 파라미터: 알파 (7개)
     */
    double dh_alpha[7];

    /**
     * @brief DH 파라미터: 링크 길이 (7개)
     */
    double dh_link[7];

    /**
     * @brief DH 파라미터: 링크 오프셋 (7개)
     */
    double dh_link_d[7];

    /**
     * @brief 전체 길이 (임의 단위)
     */
    double total_length_;

    /**
     * @brief FK 계산 임시 행렬 (4x4)
     */
    Eigen::Matrix4d P_;

    /**
     * @brief FK 역행렬 (4x4)
     */
    Eigen::Matrix4d P_inverse_;

    /**
     * @brief 조인트 각도 행렬 (7x1)
     */
    // Eigen::MatrixXd joint_radian;
    Eigen::Matrix<double, 7, 1> joint_radian;

    /**
     * @brief 센터-센서 변환 행렬 (오른쪽)
     */
    // Eigen::MatrixXd center_to_sensor_transform_right;
    Eigen::Matrix<double, 4, 4> center_to_sensor_transform_right;

    /**
     * @brief 센터-센서 변환 행렬 (왼쪽)
     */
    // Eigen::MatrixXd center_to_sensor_transform_left;
    Eigen::Matrix<double, 4, 4> center_to_sensor_transform_left;
    /**
     * @brief 센터-왼발 변환 행렬
     */
    // Eigen::MatrixXd center_to_foot_transform_left_leg;
    Eigen::Matrix<double, 4, 4> center_to_foot_transform_left_leg;

    /**
     * @brief 센터-오른발 변환 행렬
     */
    // Eigen::MatrixXd center_to_foot_transform_right_leg;
    Eigen::Matrix<double, 4, 4> center_to_foot_transform_right_leg;

    /**
     * @brief 기본 기구학 파라미터 및 DH행렬/좌표계 등 초기화
     * @return 항상 true(0) 반환
     */
    bool KinematicsGraig();

    /**
     * @brief Pinocchio 기반의 로봇 모델/데이터 로드 (URDF 불러오기)
     * @return 없음
     */
    void LoadRobotModel();

    /**
     * @brief Pinocchio 모델 (로봇 정의)
     */
    pinocchio::Model robot_model;

    /**
     * @brief Pinocchio 데이터(내부 계산 용)
     */
    pinocchio::Data robot_data;

    /**
     * @brief 조인트 상태 데이터 (JointData vector)
     */
    std::vector<JointData> joint_states;

    /**
     * @brief 로봇 전체 상태 데이터 (RobotStateData struct)
     */
    RobotStateData robot_state;

    /**
     * @brief 조인트 개수 (n)
     */
    int nj;

    /**
     * @brief 관절 위치 벡터 크기 (플로팅 베이스 포함, nq)
     */
    int nq;

    /**
     * @brief 관절 속도 벡터 크기 (플로팅 베이스 포함, nv)
     */
    int nv;
  };
}

#endif // ALICE4_KINEMATICS_DYNAMICS_H_
