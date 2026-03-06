/**
 * @file linear_algebra.hpp
 * @brief 벡터 및 행렬 기반 기하학적 변환, 회전, 관성 및 자세 연산 함수 선언
 */
#ifndef AEIROBOT_LINEAR_ALGEBRA_HPP
#define AEIROBOT_LINEAR_ALGEBRA_HPP

#include <aeirobot_toolbox/pose6d.hpp>

#include "aeirobot_math/math_tool.hpp"

namespace aeirobot
{
    /**
     * @brief 3축 평행이동 벡터 생성
     * @param position_x x축 이동량
     * @param position_y y축 이동량
     * @param position_z z축 이동량
     * @return 이동 벡터 [x, y, z]
     */
    Eigen::Vector3d getTransitionXYZ(double position_x, double position_y, double position_z);

    /**
     * @brief 6DOF 변환 행렬(RPY 회전 포함) 생성
     * @param position_x x축 이동량
     * @param position_y y축 이동량
     * @param position_z z축 이동량
     * @param roll X축 회전(롤)
     * @param pitch Y축 회전(피치)
     * @param yaw Z축 회전(요)
     * @return 4x4 변환 행렬
     */
    Eigen::Matrix4d getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw);
    Eigen::Matrix4d getTransformationXYZRPY(Pose6D pose);

    /**
     * @brief 변환 행렬의 역행렬 계산
     * @param transform 원본 4x4 변환 행렬
     * @return 역변환 행렬
     */
    // Eigen::Matrix4d getInverseTransformation(const Eigen::MatrixXd &transform);
    Eigen::Matrix4d getInverseTransformation(const Eigen::Matrix4d &transform);

    /**
     * @brief 3축 관성 모멘트 행렬 생성
     * @param ixx xx 성분 관성 모멘트
     * @param ixy xy 성분 관성 모멘트
     * @param ixz xz 성분 관성 모멘트
     * @param iyy yy 성분 관성 모멘트
     * @param iyz yz 성분 관성 모멘트
     * @param izz zz 성분 관성 모멘트
     * @return 3x3 관성 모멘트 행렬
     */
    Eigen::Matrix3d getInertiaXYZ(double ixx, double ixy, double ixz, double iyy, double iyz, double izz);

    /**
     * @brief X축 회전 행렬 생성
     * @param angle 회전 각도
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d getRotationX(double angle);

    /**
     * @brief Y축 회전 행렬 생성
     * @param angle 회전 각도
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d getRotationY(double angle);

    /**
     * @brief Z축 회전 행렬 생성
     * @param angle 회전 각도
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d getRotationZ(double angle);

    /**
     * @brief RPY 순서로 4x4 회전 행렬 생성
     * @param roll X축 회전(롤)
     * @param pitch Y축 회전(피치)
     * @param yaw Z축 회전(요)
     * @return 4x4 회전 변환 행렬
     */
    Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw);

    /**
     * @brief 4D 평행이동 행렬 생성
     * @param position_x x축 이동량
     * @param position_y y축 이동량
     * @param position_z z축 이동량
     * @return 4x4 평행이동 행렬
     */
    Eigen::Matrix4d getTranslation4D(double position_x, double position_y, double position_z);

    /**
     * @brief 회전 행렬을 RPY 벡터로 변환
     * @param rotation 3x3 회전 행렬
     * @return RPY 벡터 [roll, pitch, yaw]
     */
    Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d &rotation);

    /**
     * @brief RPY 값을 회전 행렬로 변환
     * @param roll X축 회전(롤)
     * @param pitch Y축 회전(피치)
     * @param yaw Z축 회전(요)
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw);

    /**
     * @brief RPY 값을 쿼터니언으로 변환
     * @param roll X축 회전(롤)
     * @param pitch Y축 회전(피치)
     * @param yaw Z축 회전(요)
     * @return 회전 쿼터니언
     */
    Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);

    /**
     * @brief 회전 행렬을 쿼터니언으로 변환
     * @param rotation 3x3 회전 행렬
     * @return 회전 쿼터니언
     */
    Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d &rotation);

    /**
     * @brief 쿼터니언을 RPY 벡터로 변환
     * @param quaternion 입력 쿼터니언
     * @return RPY 벡터 [roll, pitch, yaw]
     */
    Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond &quaternion);

    /**
     * @brief 쿼터니언을 회전 행렬로 변환
     * @param quaternion 입력 쿼터니언
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond &quaternion);

    /**
     * @brief 벡터를 반대칭 행렬로 변환 (hat 연산)
     * @param matrix3d 3x1 벡터
     * @return 3x3 반대칭 행렬
     */
    Eigen::Matrix3d calcHatto(const Eigen::Vector3d &matrix3d);

    /**
     * @brief 로드리게스 공식을 이용해 회전 행렬 계산
     * @param hat_matrix 벡터의 hat 행렬
     * @param angle 회전 각도
     * @return 3x3 회전 행렬
     */
    Eigen::Matrix3d calcRodrigues(const Eigen::Matrix3d &hat_matrix, double angle);

    /**
     * @brief 회전 행렬을 축 벡터(omega)로 변환
     * @param rotation 3x3 회전 행렬
     * @return 축 벡터 ω
     */
    Eigen::Vector3d convertRotToOmega(const Eigen::Matrix3d &rotation);

    /**
     * @brief 두 벡터의 외적 계산
     * @param vector3d_a 첫 번째 벡터
     * @param vector3d_b 두 번째 벡터
     * @return 외적 벡터
     */
    Eigen::Vector3d calcCross(const Eigen::Vector3d &vector3d_a, const Eigen::Vector3d &vector3d_b);

    /**
     * @brief 두 벡터의 내적 계산
     * @param a 첫 번째 벡터
     * @param b 두 번째 벡터
     * @return 내적 값
     */
    double calcInner(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    /**
     * @brief 변환 행렬에서 Pose3D 정보 추출
     * @param transform 4x4 변환 행렬
     * @return Pose3D 구조체 (x,y,z,roll,pitch,yaw)
     */
    Pose3D getPose3DfromTransformMatrix(const Eigen::Matrix4d &transform);

    ////////////////////////////////////

    /**
     * @brief ROS 메시지용 PoseXYZRPY 형식으로 변환
     * @param transform 4x4 변환 행렬
     * @return PoseXYZRPY 메시지
     */
    aeirobot_msgs::msg::PoseXYZRPY GetPose3DfromTransformMatrix(const Eigen::Matrix4d &transform);

    /**
     * @brief 회전 행렬을 RPY로 변환 (특이 사례 처리 포함)
     * @param rotationMatrix 3x3 회전 행렬
     * @return RPY 벡터
     */
    Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d &rotationMatrix);

    /**
     * @brief 위치 및 RPY를 이용해 변환 행렬 생성 (Z-Y-X 순서)
     * @param x x축 이동량
     * @param y y축 이동량
     * @param z z축 이동량
     * @param roll X축 회전(롤)
     * @param pitch Y축 회전(피치)
     * @param yaw Z축 회전(요)
     * @return 4x4 변환 행렬
     */
    Eigen::Matrix4d GetTransformationMatrix(const double x, double y, double z, double roll, double pitch, double yaw);

    /**
     * @brief 벡터를 반대칭 행렬로 변환 (외적 연산 지원)
     * @param v 3x1 벡터
     * @return 3x3 skew 대칭 행렬
     */
    Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d &v);

    /**
     * @brief DH 파라미터로부터 변환 행렬 계산
     * @param alpha 트위스트 각도
     * @param a 링크 길이
     * @param d 오프셋 거리
     * @param theta 관절 각도
     * @return 4x4 DH 변환 행렬
     */
    Eigen::Matrix4d DHMatrix(double alpha, double a, double d, double theta);

    /**
     * @brief Modified DH 파라미터로부터 변환 행렬 계산
     * @param alpha 트위스트 각도
     * @param a 링크 길이
     * @param d 오프셋 거리
     * @param theta 관절 각도
     * @return 4x4 Modified DH 변환 행렬
     */
    Eigen::Matrix4d ModifiedDHMatrix(double alpha, double a, double d, double theta);

    /**
     * @brief 감쇠 최소제곱법을 사용한 역자코비안 계산
     * @param J 야코비안 행렬
     * @param epsilon 정규화 파라미터 (기본값 1e-6)
     * @return DLS 역자코비안 행렬
     */
    // Eigen::MatrixXd DampedLeastSquaresInverse(const Eigen::MatrixXd &J, double epsilon);
    Eigen::Matrix<double, 6, Eigen::Dynamic> DampedLeastSquaresInverse(
                        const Eigen::Matrix<double, Eigen::Dynamic, 6> &J, double epsilon);
    Eigen::Matrix4d PoseToTransformMatrix(const geometry_msgs::msg::Pose & pose);
}

#endif // AEIROBOT_LINEAR_ALGEBRA_HPP
