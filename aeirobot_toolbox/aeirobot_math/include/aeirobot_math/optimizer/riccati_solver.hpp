/**
 * @file riccati_solver.hpp
 * @brief 연속 및 이산 대수적 Riccati 방정식 해법 함수 선언
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

namespace aeirobot
{
    /**
     * @brief 연속시간 모델용 반복법으로 Riccati 방정식 해결
     * @param A 시스템 동역학 행렬
     * @param B 입력 행렬
     * @param Q 상태 비용 행렬
     * @param R 제어 비용 행렬
     * @param P 초기 추정용 Riccati 행렬 (출력)
     * @param dt 시간 스텝(초)
     * @param tolerance 수렴 허용 오차
     * @param iter_max 최대 반복 횟수
     * @return 수렴 성공 시 true, 실패 시 false
     */

    /* Before */ 
    // bool SolveRiccatiIterationC(
    //     const Eigen::MatrixXd &A,
    //     const Eigen::MatrixXd &B,
    //     const Eigen::MatrixXd &Q,
    //     const Eigen::MatrixXd &R,
    //     Eigen::MatrixXd &P,
    //     const double dt = 0.001,
    //     const double &tolerance = 1e-5,
    //     const uint iter_max = 100000);
    
    /* After */
    template <int NX, int NU>
    bool SolveRiccatiIterationC(
        const Eigen::Matrix<double, NX, NX> &A,
        const Eigen::Matrix<double, NX, NU> &B,
        const Eigen::Matrix<double, NX, NX> &Q,
        const Eigen::Matrix<double, NU, NU> &R,
        Eigen::Matrix<double, NX, NX> &P,
        const double &dt = 0.001,
        const double &tolerance = 1e-5,
        const uint iter_max = 100000)
    {
        P = Q; // initialize

        Eigen::Matrix<double, NX, NX> P_next;
        auto AT = A.transpose();
        auto BT = B.transpose();
        auto Rinv = R.inverse();

        double diff;
        for (uint i = 0; i < iter_max; ++i)
        {
            P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
            diff = fabs((P_next - P).maxCoeff());
            P = P_next;
            if (diff < tolerance)
            {
                // std::cout << "iteration mumber = " << i << std::endl;
                return true;
            }
        }
        return false; // over iteration limit
    }

    /**
     * @brief 이산시간 모델용 반복법으로 Riccati 방정식 해결 (템플릿 · 헤더 only)
     * @tparam NX 상태 차원
     * @tparam NU 입력 차원
     * @param Ad 이산 시스템 동역학 행렬
     * @param Bd 이산 입력 행렬
     * @param Q 상태 비용 행렬
     * @param R 제어 비용 행렬
     * @param P 초기 추정용 Riccati 행렬 (출력)
     * @param tolerance 수렴 허용 오차
     * @param iter_max 최대 반복 횟수
     * @return 수렴 성공 시 true, 실패 시 false
     */
    /* Before */ 
    // bool SolveRiccatiIterationD(
    //     const Eigen::MatrixXd &Ad,
    //     const Eigen::MatrixXd &Bd,
    //     const Eigen::MatrixXd &Q,
    //     const Eigen::MatrixXd &R,
    //     Eigen::MatrixXd &P,
    //     const double &tolerance = 1e-5,
    //     const uint iter_max = 100000);

    /* After */
    template <int NX, int NU>
    bool SolveRiccatiIterationD(
        const Eigen::Matrix<double, NX, NX> &Ad,
        const Eigen::Matrix<double, NX, NU> &Bd,
        const Eigen::Matrix<double, NX, NX> &Q,
        const Eigen::Matrix<double, NU, NU> &R,
        Eigen::Matrix<double, NX, NX> &P,
        const double &tolerance = 1e-5,
        const uint &iter_max = 100000)
    {
        // 1) 초기화
        P = Q;

        // 2) 전치, 중간 저장용
        Eigen::Matrix<double, NX, NX> P_next;
        auto AdT = Ad.transpose();
        auto BdT = Bd.transpose();
        [[maybe_unused]] auto Rinv = R.inverse();

        double diff;
        // 3) 반복 계산
        for (uint i = 0; i < iter_max; ++i)
        {
            // -- discrete solver --
            P_next = AdT * P * Ad - 
                     AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad + Q;

            diff = fabs((P_next - P).maxCoeff());
            P = P_next;
            if (diff < tolerance)
            {
                // std::cout << "iteration mumber = " << i << std::endl;
                return true;
            }
        }
        return false;  // over iteration limit
    }

    /**
     * @brief Arimoto-Potter 방법으로 연속시간 Riccati 방정식 해결
     * @param A 시스템 동역학 행렬
     * @param B 입력 행렬
     * @param Q 상태 비용 행렬
     * @param R 제어 비용 행렬
     * @param P 계산된 Riccati 행렬 (출력)
     * @return 항상 true
     */

    /* Before */
    // bool SolveRiccatiArimotoPotter(
    //     const Eigen::MatrixXd &A,
    //     const Eigen::MatrixXd &B,
    //     const Eigen::MatrixXd &Q,
    //     const Eigen::MatrixXd &R,
    //     Eigen::MatrixXd &P);

    /* After */
    template <int NX, int NU>
    bool SolveRiccatiArimotoPotter(
        const Eigen::Matrix<double, NX, NX> &A,
        const Eigen::Matrix<double, NX, NU> &B,
        const Eigen::Matrix<double, NX, NX> &Q,
        const Eigen::Matrix<double, NU, NU> &R,
        Eigen::Matrix<double, NX, NX> &P)
    {

        // const uint dim_x = NX;
        // const uint dim_u = B.cols();

        // set Hamilton matrix
        Eigen::Matrix<double, 2 * NX, 2 * NX> Ham = Eigen::Matrix<double, 2 * NX, 2 * NX>::Zero();
        Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

        // calc eigenvalues and eigenvectors
        Eigen::EigenSolver<Eigen::Matrix<double, 2 * NX, 2 * NX>> Eigs(Ham);

        // check eigen values
        // std::cout << "eigen values：\n" << Eigs.eigenvalues() << std::endl;
        // std::cout << "eigen vectors：\n" << Eigs.eigenvectors() << std::endl;

        // extract stable eigenvectors into 'eigvec'
        Eigen::Matrix<std::complex<double>, 2 * NX, NX> eigvec = Eigen::Matrix<std::complex<double>, 2 * NX, NX>::Zero();
        int j = 0;
        for (unsigned int i = 0; i < 2 * NX; ++i)
        {
            if (Eigs.eigenvalues()[i].real() < 0.)
            {
                eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * NX, 1);
                ++j;
            }
        }

        // calc P with stable eigen vector matrix
        Eigen::Matrix<std::complex<double>, NX, NX> Vs_1, Vs_2;
        Vs_1 = eigvec.block(0, 0, NX, NX);
        Vs_2 = eigvec.block(NX, 0, NX, NX);
        P = (Vs_2 * Vs_1.inverse()).real();  // real part of the complex matrix

        return true;
    }

} // namespace aeirobot
