#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <time.h>
#include <vector>
#include <osqp.h>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace aeirobot
{

    class SolverQPWithOSQP
    {
    public:
        /// @brief  'section' 인자 받아 yaml 파일 내 해당 섹션 지정
        /// @param section: "stepping_controller" or "whole_body_controller" or "balance_controller_foot", ...
        SolverQPWithOSQP(const std::string &section, const std::string &yaml_path);
        ~SolverQPWithOSQP();

        /// @brief  파라미터 초기화
        void InitializeParameters();

        /// @brief OSQP 설정 로드
        /// @param yaml_path: yaml 파일 경로
        /// @param settings: OSQPSettings 참조
        void LoadOSQPSettingsFromYAML(OSQPSettings &settings);

        /// @brief YAML 파일에서 OSQP 설정값 로드 및 셋업
        void SetupOSQPSettings();

        /// @brief 워크스페이스 및  OSQPData · CSC 버퍼  해제
        void FreeOSQPData();
        void FreeOSQPWorkspace();

        /// @brief FreeOSQPData 및 FreeOSQPWorkspace 호출 & first_qp_setup_ 및 prev_solution_ 초기화
        void ResetData();

        /// @brief 주기마다 q, l, u만 업데이트하고 해를 반환
        /// @param P: 비용 행렬
        /// @param q: 비용 벡터
        /// @param A: 제약 행렬
        /// @param l: 제약 하한 벡터
        /// @param u: 제약 상한 벡터
        /// @param is_A_target: A 행렬 업데이트 여부
        /// @return QP 해(solution)
        Eigen::VectorXd SolveQp(const Eigen::MatrixXd &P, const Eigen::VectorXd &q,
                                const Eigen::MatrixXd &A, const Eigen::VectorXd &l,
                                const Eigen::VectorXd &u, bool is_A_target);

        /// @brief csc* → Eigen::MatrixXd 변환 함수 (디버깅용)
        /// @param mat: csc* 포인터
        /// @return Eigen::MatrixXd 행렬
        [[maybe_unused]] Eigen::MatrixXd CscToEigen(const csc *mat);

        /// @brief 컨트롤러 생성자 함수에서 호출 시, 처음 노드 시작한 경우(first_qp_setup_ == true)
        ///       - OSQPData 생성 & OSQPSettings 셋업 & OSQPWorkspace 생성
        /// @param data: OSQPData 참조
        void SetupOSQPAtFirst(OSQPData &data);

        bool is_solved_; // QP 해 구해졌는지 여부
    private:
        // 내부 인스턴스
        std::string section_;   // 컨트롤러 이름 -> Yaml 파일에서 파싱하여 OSQP 설정값 로드 시 사용
        std::string yaml_path_; // Yaml 파일 경로
        bool pattern_changed_;  // CSC 패턴 변경 여부
        bool first_qp_setup_;   // OSQPData, OSQPSettings, OSQPWorkspace 재생성 필요 여부
        bool user_initialized_; // 사용자가 이미 초기화했는지 여부

        c_int nnz_P_, nnz_A_;                      // 비영(非零) 원소 개수
        c_int n_, m_;                              // 기본 파라미터 [q.size(), A.rows()]
        std::vector<c_int> P_i_, P_p_, A_i_, A_p_; // CSC 구조(고정)
        std::vector<c_float> P_x_, A_x_;           // CSC 값(매 주기 갱신)
        Eigen::VectorXd prev_solution_;            // 이전 QP 해(solution)를 저장

        // 내부 유틸
        /// @brief NaN/Inf 체크 여부 반환
        /// @param M: SolveQp 함수의 입력인자 행렬 및 벡터(P, A, q, l, u)
        /// @return 체크 여부
        bool CheckNaNInf(const auto &M);

        /// @brief 행렬 A의 r번째 행 중 가장 큰 값이 TH(1e-12) 보다 작은지 여부 반환
        /// @param A: 행렬
        /// @param r: 행 인덱스
        /// @return 체크 여부
        bool CheckZeroRow(const Eigen::MatrixXd &A, int r);

        /// @brief 플래그(pattern_changed_, first_qp_setup_, user_initialized_) 초기화 및 플래그에 따른
        ///        OSQPData 생성 & OSQPSettings 셋업 & OSQPWorkspace 생성 필요 여부 확인 및 설정
        /// @param P: 비용 행렬
        /// @param A: 제약 행렬
        /// @param q: 비용 벡터
        void SetupOSQPIfNeeded(const Eigen::MatrixXd &P, const Eigen::MatrixXd &A,
                               const Eigen::VectorXd &q, const Eigen::VectorXd &l, const Eigen::VectorXd &u);

        /// @brief CSC 구조 생성 (col 기준)
        /// @param M: [row 기준] 행렬(P, A)
        /// @param upper_only: 상삼각 행렬 여부
        /// @return i_out: [col 기준] 비영(非零) 원소의 행 인덱스 벡터
        /// @return p_out: [col 기준] 비영(非零) 원소의 열 포인터 벡터
        /// @return 비영(非零) 원소 개수
        c_int ToCSCStructure(const Eigen::MatrixXd &M, bool upper_only,
                             std::vector<c_int> &i_out, std::vector<c_int> &p_out) const;

        /// @brief CSC 값들 vecotr에 채우기
        /// @param M: [row 기준] 행렬(P, A)
        /// @param upper_only: 상삼각 행렬 여부
        /// @return x_vec: [col 기준] 비영(非零) 원소의 값 담은 벡터
        void FillCSCValues(const Eigen::MatrixXd &M, bool upper_only,
                           std::vector<c_float> &x_vec) const;

        /// @brief 행렬 패턴 변경 여부 확인(ex. 0이 아닌 원소의 위치 변화) 검사.
        //     - CSC 등 sparse 구조에서 패턴이 바뀌면 내부 메모리 구조 재할당 필요.
        /// @param A: [row 기준] 행렬(zero + non-zero)
        /// @param A_i: [col 기준] 비영(非零) 원소의 행 인덱스 벡터
        /// @param A_p: [col 기준] 비영(非零) 원소의 열 포인터 벡터
        /// @param upper_only: 상삼각 행렬 여부
        /// @return 패턴 변경 여부
        bool PatternChanged(const Eigen::MatrixXd &A,
                            const std::vector<c_int> &A_i,
                            const std::vector<c_int> &A_p,
                            bool upper_only) const;

        /// @brief 상삼각(upper_only == true) 행렬에서 0이 아닌 원소 개수 count.
        ///   - QP(Quadratic Programming)에서 행렬을 sparse(희소) 형식으로 변환할 때,
        ///     대각선 기준 상삼각만 저장하는 경우, 실제로 필요한 저장 공간을 계산하는데 사용.
        ///   - OSQP 등 QP solver의 내부 sparse matrix 구조(CSC 등)에서
        ///     상삼각만 저장하는 경우가 많음.
        /// @param M: [row 기준] 행렬(P)
        /// @return 0이 아닌 원소 개수
        c_int CountNnzUpper(const Eigen::MatrixXd &M) const;

        // OSQP 관련
        OSQPData *osqp_data_;
        OSQPWorkspace *osqp_work_;
        OSQPSettings osqp_settings_;
    };
}
