/**
 * @file lie_algebra.hpp
 * @brief Lie 대수 관련 연산을 제공하는 유틸리티 클래스 정의
 */
#ifndef LIE_ALGEBRA_HPP_
#define LIE_ALGEBRA_HPP_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

namespace aeirobot
{
    /**
     * @class LieAlgebra
     * @brief SE(3) 및 se(3) 관련 반대칭 매핑과 Adjoint 연산 제공
     */
    class LieAlgebra
    {
    public:
        /**
         * @brief 3x1 벡터를 3x3 반대칭 행렬로 변환
         * @param omega 회전 벡터 (각속도)
         * @return 반대칭 행렬
         */
        static Eigen::Matrix3d Ceil3DVectorOperator(const Eigen::Vector3d &omega);

        /**
         * @brief 3x3 반대칭 행렬을 3x1 벡터로 변환
         * @param W 반대칭 행렬
         * @return 벡터 (각속도)
         */
        static Eigen::Vector3d Floor3DVectorOperator(const Eigen::Matrix3d &W);

        /**
         * @brief 6x1 벡터를 4x4 반대칭 행렬 (Twist hat)로 변환
         * @param V 트위스트 벡터 [v; ω]
         * @return 4x4 반대칭 행렬
         */
        // static Eigen::Matrix4d Ceil6DVectorOperator(const Eigen::VectorXd &V);
        static Eigen::Matrix<double, 4, 4> Ceil6DVectorOperator(const Eigen::Matrix<double, 6, 1> &V);
        /**
         * @brief 4x4 반대칭 행렬을 6x1 벡터 (Twist)로 변환
         * @param M 4x4 반대칭 행렬
         * @return 트위스트 벡터 [v; ω]
         */
        static Eigen::Matrix<double,6,1> Floor6DVectorOperator(const Eigen::Matrix4d &M);

        /**
         * @brief SE(3) 변환 행렬을 6x6 Adjoint 행렬로 변환
         * @param T 4x4 변환 행렬
         * @return 6x6 Adjoint 행렬
         */
        // static Eigen::MatrixXd AdjointTransform(const Eigen::Matrix4d &T);
        static Eigen::Matrix<double, 6, 6> AdjointTransform(const Eigen::Matrix4d &T);

        /**
         * @brief SE(3) 변환 행렬의 역행렬 전치에 해당하는 Adjoint 행렬을 생성
         * @param T 4x4 변환 행렬
         * @return 6x6 역행렬 전치 Adjoint 행렬
         */
        static Eigen::MatrixXd AdjointTransformInverseTranspose(const Eigen::Matrix4d &T);
        // static Eigen::Matrix<double, 6, 6> AdjointTransformInverseTranspose(const Eigen::Matrix4d &T);

        /**
         * @brief SE(3) 변환 행렬의 전치 Adjoint 행렬을 생성
         * @param T 4x4 변환 행렬
         * @return 6x6 전치 Adjoint 행렬
         */
        // static Eigen::MatrixXd AdjointTransformTranspose(const Eigen::Matrix4d &T);
        static Eigen::Matrix<double, 6, 6> AdjointTransformTranspose(const Eigen::Matrix4d &T);

        /**
         * @brief 6x1 벡터로부터 Adjoint 연산자 행렬(adjoint operator) 생성
         * @param V 트위스트 벡터 [v; ω]
         * @return 6x6 Adjoint operator 행렬
         */
        // static Eigen::MatrixXd CreateAdjointMatrix(const Eigen::VectorXd &V);
        static Eigen::Matrix<double, 6, 6> CreateAdjointMatrix(const Eigen::Matrix<double, 6, 1> &V);

        /**
         * @brief SE(3) 변환 행렬의 역행렬(Adjoint inverse) 생성
         * @param T 4x4 변환 행렬
         * @return 6x6 Adjoint 역행렬
         */
        // static Eigen::MatrixXd InverseAdjoint(const Eigen::MatrixXd &T);
        static Eigen::Matrix<double, 6, 6> InverseAdjoint(const Eigen::Matrix4d &T);

        /**
         * @brief adjoint operator 계산
         * @param twist 트위스트 벡터 [v; ω]
         * @return 6x6 adjoint operator 행렬
         */
        // static Eigen::MatrixXd AdOperator(const Eigen::VectorXd &twist);
        static Eigen::Matrix<double, 6, 6> AdOperator(const Eigen::Matrix<double, 6, 1> &twist);

        /**
         * @brief co-adjoint operator 계산
         * @param twist 트위스트 벡터 [v; ω]
         * @return 6x6 co-adjoint operator 행렬
         */
        // static Eigen::MatrixXd CoAdOperator(const Eigen::VectorXd &twist);
        static Eigen::Matrix<double, 6, 6> CoAdOperator(const Eigen::VectorXd &twist);  // 현재 사용 X 
    };

} // namespace aeirobot

#endif // LIE_ALGEBRA_HPP_
