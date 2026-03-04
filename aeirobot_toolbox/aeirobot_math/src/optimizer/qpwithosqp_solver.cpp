#include "aeirobot_math/optimizer/qpwithosqp_solver.hpp"

namespace aeirobot
{
    SolverQPWithOSQP::SolverQPWithOSQP(const std::string& section, const std::string& yaml_path)
    : section_(section), yaml_path_(yaml_path)
    {
        // std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("alice4_parameters");
        // std::string yaml_path = pkg_share_dir + "/config/controller_osqp_setting.yaml";

        InitializeParameters();
    }

    SolverQPWithOSQP::~SolverQPWithOSQP()
    {
        FreeOSQPWorkspace();
        FreeOSQPData();
    }

    void SolverQPWithOSQP::InitializeParameters()
    {
        osqp_data_ = nullptr;
        osqp_work_ = nullptr;
        first_qp_setup_ = true;
        pattern_changed_ = false;
        is_solved_ = false;
        user_initialized_ = false;
        nnz_P_ = 0;
        nnz_A_ = 0;
        P_i_.clear(); P_p_.clear(); A_i_.clear(); A_p_.clear();
        P_x_.clear(); A_x_.clear();
        prev_solution_ = Eigen::VectorXd();
    }

    void SolverQPWithOSQP::FreeOSQPWorkspace() {
        if (osqp_work_) {
            osqp_cleanup(osqp_work_);
            osqp_work_ = nullptr;
        }
    }
    void SolverQPWithOSQP::FreeOSQPData() {
        if (osqp_data_) {
            c_free(osqp_data_->P);
            c_free(osqp_data_->A);
            c_free(osqp_data_);
            osqp_data_ = nullptr;
        }
    }

    void SolverQPWithOSQP::LoadOSQPSettingsFromYAML(OSQPSettings& settings) 
    {
        if (yaml_path_.empty()) {
            throw std::runtime_error("YAML path is empty");
        }
        YAML::Node config = YAML::LoadFile(yaml_path_);
        if (!config[section_]) {
            throw std::runtime_error("Section " + section_ + " not found in OSQP settings YAML");
        }
        auto node = config[section_];
        if (node["verbose"]) settings.verbose = node["verbose"].as<bool>();
        if (node["warm_start"]) settings.warm_start = node["warm_start"].as<bool>();
        if (node["alpha"]) settings.alpha = node["alpha"].as<double>();
        if (node["max_iter"]) settings.max_iter = node["max_iter"].as<int>();
        if (node["eps_abs"]) settings.eps_abs = node["eps_abs"].as<double>();
        if (node["eps_rel"]) settings.eps_rel = node["eps_rel"].as<double>();
        if (node["eps_prim_inf"]) settings.eps_prim_inf = node["eps_prim_inf"].as<double>();
        if (node["eps_dual_inf"]) settings.eps_dual_inf = node["eps_dual_inf"].as<double>();
        if (node["check_termination"]) settings.check_termination = node["check_termination"].as<int>();
        if (node["time_limit"]) settings.time_limit = node["time_limit"].as<double>();
        if (node["adaptive_rho"]) settings.adaptive_rho = node["adaptive_rho"].as<bool>();
        if (node["polish"]) settings.polish = node["polish"].as<bool>();
        if (node["scaling"]) settings.scaling = node["scaling"].as<int>();
        if (node["scaled_termination"]) settings.scaled_termination = node["scaled_termination"].as<bool>();
        if (node["rho"]) settings.rho = node["rho"].as<double>();
        if (node["sigma"]) settings.sigma = node["sigma"].as<double>();

        // std::cout << "OSQP settings loaded from YAML" << std::endl;
        // std::cout << "section_: " << section_ << std::endl;
        // std::cout << "verbose: " << settings.verbose << std::endl;
        // std::cout << "warm_start: " << settings.warm_start << std::endl;
        // std::cout << "alpha: " << settings.alpha << std::endl;
        // std::cout << "max_iter: " << settings.max_iter << std::endl;
        // std::cout << "eps_abs: " << settings.eps_abs << std::endl;
        // std::cout << "eps_rel: " << settings.eps_rel << std::endl;
        // std::cout << "eps_prim_inf: " << settings.eps_prim_inf << std::endl;
        // std::cout << "eps_dual_inf: " << settings.eps_dual_inf << std::endl;
        // std::cout << "check_termination: " << settings.check_termination << std::endl;
        // std::cout << "time_limit: " << settings.time_limit << std::endl;
        // std::cout << "adaptive_rho: " << settings.adaptive_rho << std::endl;
        // std::cout << "polish: " << settings.polish << std::endl;
        // std::cout << "scaling: " << settings.scaling << std::endl;
        // std::cout << "scaled_termination: " << settings.scaled_termination << std::endl;
        // std::cout << "rho: " << settings.rho << std::endl;
        // std::cout << "sigma: " << settings.sigma << std::endl;
        // std::cout << "--------------------------------" << std::endl;
    }

    void SolverQPWithOSQP::SetupOSQPSettings()
    {
        if(!first_qp_setup_) return;

        /* 설정값 ‑ 한 번만 셋업 ------------------------------- */
        osqp_set_default_settings(&osqp_settings_); 

        // 설정값 로드 
        LoadOSQPSettingsFromYAML(osqp_settings_);       
    }

    void SolverQPWithOSQP::ResetData() {
        FreeOSQPWorkspace();
        FreeOSQPData();
        first_qp_setup_ = true;
        prev_solution_ = Eigen::VectorXd();
    }

    [[maybe_unused]] Eigen::MatrixXd SolverQPWithOSQP::CscToEigen(const csc* mat) {
        Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(mat->m, mat->n);
        for (c_int j = 0; j < mat->n; ++j) {
            for (c_int idx = mat->p[j]; idx < mat->p[j + 1]; ++idx) {
                c_int i = mat->i[idx];
                dense(i, j) = mat->x[idx];
            }
        }
        return dense;
    }

    bool SolverQPWithOSQP::CheckNaNInf(const auto& M)
    {
        return !(M.array().isFinite().all());
    }

    bool SolverQPWithOSQP::CheckZeroRow(const Eigen::MatrixXd& A, int r)
    {   
        // A.row(r) 은 행렬 A의 r번째 행을 추출한 벡터
        // cwiseAbs: 행렬의 각 원소를 절댓값으로 변환
        // maxCoeff: 행렬의 각 행에서 가장 큰 값을 찾음
        return A.row(r).cwiseAbs().maxCoeff() < 1e-12;
    }

    Eigen::VectorXd SolverQPWithOSQP::SolveQp(const Eigen::MatrixXd& P, const Eigen::VectorXd& q,
                                          const Eigen::MatrixXd& A, const Eigen::VectorXd& l,
                                          const Eigen::VectorXd& u, bool is_A_target)
    {   
        // 0. 기본 파라미터 설정 
        n_ = static_cast<c_int>(q.size());
        m_ = static_cast<c_int>(A.rows());

        // 1. NaN/Inf 체크
        if (CheckNaNInf(P) || CheckNaNInf(A) || CheckNaNInf(q) || CheckNaNInf(l) || CheckNaNInf(u))
        {
            std::cerr << "[QP-CHECK] NaN/Inf 발견 ― solve 건너뜀" << std::endl;
            return prev_solution_.size() == n_ ? prev_solution_
                                                    : Eigen::VectorXd::Zero(n_);
        }
        for (int r = 0; r < m_; ++r)
        {
            if (CheckZeroRow(A, r) && std::abs(l[r] - u[r]) > 1e-12)
            {
                // (기존) l = -INF, u = +INF   <-- LDL pivot 이 사라짐
                // (수정) 0 = 0 으로 만들어 행 자체를 완전히 죽인다
                const_cast<Eigen::VectorXd &>(l)[r] = 0.0;
                const_cast<Eigen::VectorXd &>(u)[r] = 0.0;
            }
        }

        // 2. 
        //  1) CSC 패턴이 바뀌었거나 첫 호출 -> CSC 구조 및 데이터 값 & OSQP Settings & OSQP Workspace 초기화
        //  2) 매 주기 ->  CSC 데이터 값 갱신
        SetupOSQPIfNeeded(P, A, q, l, u);

        // 3. OSQP 업데이트
        // P_x_, A_x_ vector [col 기준]
        osqp_update_P(osqp_work_, P_x_.data(), OSQP_NULL, nnz_P_);
        if (is_A_target) 
        {
            osqp_update_A(osqp_work_, A_x_.data(), OSQP_NULL, nnz_A_);
        }
        // q 벡터(선형 비용)를 워크스페이스에 복사
        osqp_update_lin_cost(osqp_work_, const_cast<c_float*>(reinterpret_cast<const c_float*>(q.data())));
        // l, u 벡터(제약 하한/상한)를 워크스페이스에 복사
        osqp_update_bounds(osqp_work_,
                        const_cast<c_float*>(reinterpret_cast<const c_float*>(l.data())),
                        const_cast<c_float*>(reinterpret_cast<const c_float*>(u.data())));

        // 4. 풀이
        osqp_solve(osqp_work_);

        // 5. 결과 매핑
        Eigen::VectorXd qp_sol(n_);
        if (osqp_work_->info->status_val == OSQP_SOLVED ||
            osqp_work_->info->status_val == OSQP_SOLVED_INACCURATE)
        {
            std::memcpy(qp_sol.data(), osqp_work_->solution->x, sizeof(c_float) * n_);
            is_solved_ = true;
        }
        else
        {   
            qp_sol = (prev_solution_.size() == n_) ? prev_solution_ : Eigen::VectorXd::Zero(n_);
            is_solved_ = false;
        }
        prev_solution_ = qp_sol;
        osqp_warm_start_x(osqp_work_, prev_solution_.data());
        return qp_sol;
    }

    void SolverQPWithOSQP::SetupOSQPAtFirst(OSQPData& data)
    {
        // OSQPData 생성
        osqp_data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
        osqp_data_->n = data.n;
        osqp_data_->m = data.m;
        osqp_data_->P = data.P;
        osqp_data_->A = data.A;
        osqp_data_->q = data.q;
        osqp_data_->l = data.l;
        osqp_data_->u = data.u;

        // 설정값 ‑ 한 번만 셋업
        SetupOSQPSettings();

        // 워크스페이스 생성
        if (osqp_setup(&osqp_work_, osqp_data_, &osqp_settings_) != 0)
        {
            throw std::runtime_error(
                std::string("OSQP 초기화 실패! In ") + section_ + "\n\n"
            );
        }
        // else
        // {
        //     std::cout << "osqp_work_->info->status_val : " << osqp_work_->info->status_val << std::endl;
        //     std::cout << "OSQP 초기화 성공!\n" << std::endl;
        // }       

        // 사용자가 이미 초기화했음을 표시
        user_initialized_ = true;
    }
    
    void SolverQPWithOSQP::SetupOSQPIfNeeded(const Eigen::MatrixXd& P, const Eigen::MatrixXd& A,
                                                const Eigen::VectorXd& q, const Eigen::VectorXd& l, const Eigen::VectorXd& u)
    {
        pattern_changed_ = false;

        /* sparsity 가 달라졌으면 워크스페이스 재생성. */
        if (!first_qp_setup_) 
        {
            pattern_changed_ = (CountNnzUpper(P) != nnz_P_) || PatternChanged(A, A_i_, A_p_,  /*upper_only=*/false);
            if (pattern_changed_)
            {
                // 1) 워크스페이스 해제
                FreeOSQPWorkspace();
                // 2) OSQPData · CSC 버퍼 해제
                FreeOSQPData();
                // 3) 초기화 플래그 초기화
                first_qp_setup_ = true;
                user_initialized_ = false;
            }
        }

        /* 첫 루프 */
        // OSQPWorkspace 및 OSQPData * CSC 버퍼 해제 -> CSC 구조 생성 + OSQPWorkspace 초기화 
        if (first_qp_setup_) {
            // CSC 구조 생성
            nnz_P_ = ToCSCStructure(P, /*upper_only=*/true, P_i_, P_p_);
            nnz_A_ = ToCSCStructure(A, /*upper_only=*/false, A_i_, A_p_);

            // 값 배열 크기 확보
            P_x_.resize(nnz_P_);
            A_x_.resize(nnz_A_);

            FillCSCValues(P, /*upper_only=*/true, P_x_);
            FillCSCValues(A, /*upper_only=*/false, A_x_);

            if (!user_initialized_)
            {
                // OSQPData 생성
                osqp_data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
                osqp_data_->n = q.size();
                osqp_data_->m = A.rows();
                osqp_data_->P = csc_matrix(P.rows(), P.cols(), nnz_P_, P_x_.data(), P_i_.data(), P_p_.data());
                osqp_data_->A = csc_matrix(A.rows(), A.cols(), nnz_A_, A_x_.data(), A_i_.data(), A_p_.data());
                osqp_data_->q = const_cast<c_float*>(reinterpret_cast<const c_float*>(q.data()));
                osqp_data_->l = const_cast<c_float*>(reinterpret_cast<const c_float*>(l.data()));
                osqp_data_->u = const_cast<c_float*>(reinterpret_cast<const c_float*>(u.data()));

                // 설정값 ‑ 한 번만 셋업
                SetupOSQPSettings();

                // 워크스페이스 생성
                if (osqp_setup(&osqp_work_, osqp_data_, &osqp_settings_) != 0)
                {
                    throw std::runtime_error(
                        std::string("OSQP 초기화 실패! In ") + section_ + "\n\n"
                    );
                }
                // else
                // {
                //     std::cout << "osqp_work_->info->status_val : " << osqp_work_->info->status_val << std::endl;
                //     std::cout << "OSQP 초기화 성공!\n" << std::endl;
                // }
            }

            first_qp_setup_ = false;
        }
        /* 매주기 마다 */
        // CSC 값 갱신 후 osqp_update_* 호출
        else 
        {
            // CSC 구조는 그대로, 값만 갱신
            FillCSCValues(P, /*upper_only=*/true, P_x_);
            FillCSCValues(A, /*upper_only=*/false, A_x_);
        }
    }

    bool SolverQPWithOSQP::PatternChanged(const Eigen::MatrixXd &M,
                            const std::vector<c_int> &old_i,
                            const std::vector<c_int> &old_p,
                            bool upper_only) const
    {
        std::vector<c_int> new_i, new_p;
        c_int nnz_now = 0;
        c_int rows = static_cast<c_int>(M.rows());
        c_int cols = static_cast<c_int>(M.cols());
        new_p.assign(cols + 1, 0);

        for (c_int j = 0; j < cols; ++j)
        {
            for (c_int i = 0; i < rows; ++i)
            {
                if (upper_only && i > j)
                    continue;
                if (M(i, j) == 0.0)
                    continue;
                new_i.push_back(i);
                ++nnz_now;
            }
            new_p[j + 1] = nnz_now;
        }

        return !(new_i == old_i && new_p == old_p);
    }

    c_int SolverQPWithOSQP::CountNnzUpper(const Eigen::MatrixXd &M) const
    {
        c_int nnz = 0;
        for (c_int j = 0; j < M.cols(); ++j)
            for (c_int i = 0; i <= j; ++i)
                if (M(i, j) != 0.0)
                    ++nnz;
        return nnz;
    }

    c_int SolverQPWithOSQP::ToCSCStructure(const Eigen::MatrixXd& M, bool upper_only, 
                            std::vector<c_int>& i_out, std::vector<c_int>& p_out) const 
    {
        const c_int rows = static_cast<c_int>(M.rows());
        const c_int cols = static_cast<c_int>(M.cols());

        i_out.clear();
        i_out.reserve(M.nonZeros());
        p_out.assign(cols + 1, 0);

        c_int nnz = 0;
        for (c_int j = 0; j < cols; ++j) 
        {
            for (c_int i = 0; i < rows; ++i) 
            {
                if (upper_only && i > j) 
                    continue;
                if (M(i, j) == 0.0) 
                    continue;
                
                i_out.push_back(i);
                ++nnz;
            }
            p_out[j + 1] = nnz;
        }
        return nnz;
    }

    void SolverQPWithOSQP::FillCSCValues(const Eigen::MatrixXd &M,
                                         bool upper_only,
                                         std::vector<c_float> &x_vec) const
    {
        c_int k = 0;
        for (c_int j = 0; j < M.cols(); ++j)
            for (c_int i = 0; i < M.rows(); ++i)
            {
                if (upper_only && i > j)
                    continue;
                if (M(i, j) == 0.0)
                    continue;
                x_vec[k++] = static_cast<c_float>(M(i, j));
            }
    }
}