/**
 * @file incremental_jacobian.hpp
 * @brief 기구학 체인에 대한 증분 야코비안과 그 시간 도함수를 계산하는 클래스 정의
 */
#ifndef INCREMENTAL_JACOBIAN_HPP_
#define INCREMENTAL_JACOBIAN_HPP_

#include "aeirobot_math/algebra/lie_algebra.hpp"
#include <vector>
#include <iomanip>

namespace aeirobot
{
    /**
     * @class IncrementalJacobian
     * @brief 기구학 체인의 각 링크에 대해 증분 야코비안과 도함수를 계산
     */
    class IncrementalJacobian
    {
    public:
        /**
         * @struct Body
         * @brief 링크별 기구학 데이터 저장용 구조체
         */
        struct Body
        {
            /** @brief 바디 공간 야코비안 행렬 (6 x n) */
            // Eigen::MatrixXd jacobian_matrix;
            Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_matrix;
            /** @brief 야코비안 행렬의 시간 도함수 (6 x n) */
            // Eigen::MatrixXd derivative_jacobian_matrix;
            Eigen::Matrix<double, 6, Eigen::Dynamic> derivative_jacobian_matrix;
            /** @brief 이전 링크 프레임 대비 현재 링크의 트위스트 */
            // Eigen::MatrixXd between_twist_prev_body;
            Eigen::Matrix<double, 6, 1> between_twist_prev_body;
            /** @brief 부모 프레임에서 현재 링크로의 동차 변환 행렬 */
            Eigen::Matrix4d adjacent_transfom_matrix;
            /** @brief 관절 모션 행렬 (회전 또는 병진) */
            // Eigen::MatrixXd joint_matrix_e;
            Eigen::Matrix<double, 6, 1> joint_matrix_e;
            /** @brief 이 관절 기여를 선택하는 선택 행렬 */
            // Eigen::MatrixXd select_joint_number_matrix;
            Eigen::Matrix<double, Eigen::Dynamic, 1> select_joint_number_matrix;
            /** @brief 현재 링크의 바디 속도 벡터 (6 x 1) */
            // Eigen::MatrixXd current_body_velocity;
            Eigen::Matrix<double, 6, 1> current_body_velocity;
            /** @brief 현재 링크의 바디 가속도 벡터 (6 x 1) */
            // Eigen::MatrixXd current_body_acceleration;
            Eigen::Matrix<double, 6, 1> current_body_acceleration;
            /** @brief 관절 각도 θ */
            double theta;
            /** @brief 관절 속도 θ̇ */
            double theta_dot;
            /** @brief 관절 가속도 θ̈ */
            double theta_ddot;
            /** @brief 이전 단계 관절 속도 (유한 차분 계산용) */
            double prev_theta_dot;
        };

        /** @brief 베이스 및 모든 관절에 대한 Body 구조체 벡터 */
        std::vector<Body> bodies;
        /** @brief 작동 관절 수 */
        int numjoint_;
        /** @brief 적분 주파수 (Hz), dt = 1/delta_time_ */
        int delta_time_;

        /**
         * @brief 생성자
         * @param numjoint 기구학 체인의 관절 수
         * @param delta_time 유한 차분 계산을 위한 샘플링 주파수 (Hz)
         */
        IncrementalJacobian(int numjoint, int delta_time);

        /**
         * @brief 소멸자
         */
        ~IncrementalJacobian();

        /**
         * @brief 기구학을 체인에 전파하고 야코비안 및 도함수를 갱신
         *
         * 유한 차분으로 관절 가속도를 계산하고,
         * 관절 트위스트 및 그 도함수를 누적하여
         * 각 링크의 야코비안과 그 시간 도함수를 업데이트한 후,
         * 링크별 속도와 가속도를 계산한다.
         */
        void PropagateKinematics();
    };

} // namespace aeirobot

#endif // INCREMENTAL_JACOBIAN_HPP_
