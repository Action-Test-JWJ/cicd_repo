#include "aeirobot_robotics/linear_rotary_converter/linear_rotary_converter.hpp"

namespace aeirobot
{
    VirtualJointMap::VirtualJointMap()
    {
        alice4_version = aeirobot::GetParameter<int>("alice4_version", "version_name");
    }

    std::tuple<double, double, double, double, double, double>
    VirtualJointMap::CalculateAnkleLinearMap(double linear_1, double linear_2,
                                             double linear_1_dot, double linear_2_dot,
                                             double linear_1_effort, double linear_2_effort)
    {
        if (alice4_version == 1)
        {

            // ---------------------------------------------
            // 피치와 롤의 각도 계산
            // ---------------------------------------------
            double linear_average = std::fabs(linear_1 + linear_2) / 2.0 + 0.2735;
            double delta_linear = std::fabs(linear_1 - linear_2) / 2.0;

            // double pitch_len2 = std::sqrt(linear_average * linear_average + std::pow(0.048 - 0.027, 2));
            double pitch_len2 = std::sqrt(linear_average * linear_average + 0.000441);
            if (pitch_len2 < 0)
            {
                std::cerr << "Error: pitch_len2 is negative: " << pitch_len2 << std::endl;
                pitch_len2 = 0; // NaN 방지
            }
            double roll_pitch_offset_theta = std::acos(linear_average / pitch_len2);
            if (roll_pitch_offset_theta < -1.0 || roll_pitch_offset_theta > 1.0)
            {
                std::cerr << "Error: roll_pitch_offset_theta out of range: " << roll_pitch_offset_theta << std::endl;
                roll_pitch_offset_theta = std::max(-1.0, std::min(1.0, roll_pitch_offset_theta));
            }

            double roll_input = (delta_linear * std::cos(roll_pitch_offset_theta)) / 0.027;
            if (roll_input < -1.0 || roll_input > 1.0)
            {
                std::cerr << "Error: roll_input out of range: " << roll_input << std::endl;
                roll_input = std::max(-1.0, std::min(1.0, roll_input));
            }
            double roll = std::asin(roll_input);

            // double under_line = std::sqrt(std::pow(0.045, 2));
            double under_line = 0.045;

            // double line_2 = std::sqrt(std::pow((0.355 - 0.05), 2) + std::pow(0.04, 2));
            double line_2 = 0.307611768;

            double cos_theta = (under_line * under_line + line_2 * line_2 - linear_average * linear_average) / (2 * under_line * line_2);
            if (cos_theta < -1.0 || cos_theta > 1.0)
            {
                std::cerr << "Error: cos_theta out of range: " << cos_theta << std::endl;
                cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            }
            // double angle_radians = std::atan(0.015 / 0.045) + std::atan(0.04 / (0.355 - 0.05)) - aeirobot::DegToRad(90.0) -  std::atan(0.005 / (0.355-0.05+0.015));
            double angle_radians = 0.32175055 + 0.13040331 - 1.5707963 - 0.01562373;

            double pitch = (angle_radians + std::acos(cos_theta)) * -1.0;

            // ---------------------------------------------
            // 피치와 롤의 속도 계산 (미분)
            // ---------------------------------------------
            // return {pitch, roll};
            // 1. pitch의 linear_average에 대한 미분
            double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
            if (sin_theta == 0.0)
            {
                sin_theta = 1e-8; // 0을 방지하기 위해 작은 값으로 대체
            }
            double dpitch_dlinear_average = linear_average / (under_line * line_2 * sin_theta);

            // 2. linear_average에 대한 linear_1과 linear_2의 미분
            double sign_sum = ((linear_1 + linear_2) >= 0) ? 1.0 : -1.0;
            double dlinear_average_dlinear_1 = 0.5 * sign_sum;
            double dlinear_average_dlinear_2 = 0.5 * sign_sum;

            // 3. pitch의 linear_1과 linear_2에 대한 미분
            double dpitch_dlinear_1 = dpitch_dlinear_average * dlinear_average_dlinear_1;
            double dpitch_dlinear_2 = dpitch_dlinear_average * dlinear_average_dlinear_2;

            // 4. pitch_dot 계산
            double pitch_dot = (dpitch_dlinear_1 * linear_1_dot + dpitch_dlinear_2 * linear_2_dot) * -1.0;

            // 롤의 미분 계산
            // roll = asin(f), f = (delta_linear * cos(theta_offset)) / 0.027
            // 따라서, droll/dx = (1 / sqrt(1 - f^2)) * df/dx

            // 1. f 계산
            double f = (delta_linear * std::cos(roll_pitch_offset_theta)) / 0.027;
            f = std::max(-1.0, std::min(1.0, f)); // 안전을 위해 범위 제한

            // 먼저, linear_average와 pitch_len2의 미분을 계산
            // double dpitch_len2_dlinear_average = (linear_average) / pitch_len2;

            // To avoid complexity, we can use:
            double dcos_theta_offset_dlinear_average = (pitch_len2 * 1 - linear_average * (linear_average / pitch_len2)) / (pitch_len2 * pitch_len2);
            // Simplifies to sin_theta^2 / pitch_len2^3

            // Now, df/dlinear_average
            double df_dlinear_average = (delta_linear / 0.027) * dcos_theta_offset_dlinear_average;

            // df/ddelta_linear
            double df_ddelta_linear = std::cos(roll_pitch_offset_theta) / 0.027;

            // Now, ddelta_linear/dlinear_1 and ddelta_linear/dlinear_2
            double ddelta_linear_dlinear_1 = (linear_1 - linear_2) >= 0 ? 0.5 : -0.5;
            double ddelta_linear_dlinear_2 = (linear_1 - linear_2) >= 0 ? -0.5 : 0.5;

            // Now, df/dlinear_1 and df/dlinear_2 using chain rule
            double df_dlinear_1 = df_dlinear_average * dlinear_average_dlinear_1 + df_ddelta_linear * ddelta_linear_dlinear_1;
            double df_dlinear_2 = df_dlinear_average * dlinear_average_dlinear_2 + df_ddelta_linear * ddelta_linear_dlinear_2;

            // Now, droll/dlinear_1 and droll/dlinear_2
            double denominator_roll = std::sqrt(1.0 - f * f);
            if (denominator_roll == 0.0)
            {
                denominator_roll = 1e-8; // 0을 방지하기 위해 작은 값으로 대체
            }
            double droll_dlinear_1 = (1.0 / denominator_roll) * df_dlinear_1;
            double droll_dlinear_2 = (1.0 / denominator_roll) * df_dlinear_2;

            // 롤의 시간 미분 (roll_dot) 계산
            double roll_dot = droll_dlinear_1 * linear_1_dot + droll_dlinear_2 * linear_2_dot;

            // 롤의 부호 조정
            if (linear_1 >= linear_2)
            {
                roll = roll * -1.0;
                roll_dot = roll_dot * -1.0;
            }
            // ---------------------------------------------
            // 피치 토크 계산
            // ---------------------------------------------

            // Constants (meters)
            const double ankle_roll_length = 0.027;
            // const double ankle_pitch_length_top = 0.096;    // Top spacing between linear actuators
            // const double ankle_pitch_length_bottom = 0.054; // Bottom spacing between linear actuators

            // Calculate half lengths
            // const double ankle_pitch_length_top_half = ankle_pitch_length_top / 2.0;       // 0.048 m
            // const double ankle_pitch_length_bottom_half = ankle_pitch_length_bottom / 2.0; // 0.027 m
            // const double ankle_pitch_length_top_half = 0.048;    // 0.048 m
            // const double ankle_pitch_length_bottom_half = 0.027; // 0.027 m

            // Calculate the pitch angle based on the trapezoidal arrangement (half)
            // double ankle_pitch_angle_half = std::atan((ankle_pitch_length_top_half - ankle_pitch_length_bottom_half) / ankle_pitch_length_bottom_half);
            // double ankle_pitch_angle_half = 0.66104317
            // double ankle_cos_pitch_angle_offset = std::cos(ankle_pitch_angle_half);
            double ankle_cos_pitch_angle_offset = 0.78935222;

            // Calculate the vertical component of forces from each linear actuator (half)
            double ankle_force_1_vertical = linear_1_effort * ankle_cos_pitch_angle_offset;
            double ankle_force_2_vertical = linear_2_effort * ankle_cos_pitch_angle_offset;

            // Total vertical force acting on the ankle
            double ankle_force = ankle_force_1_vertical + ankle_force_2_vertical;
            double ankle_roll_torque = (ankle_force_1_vertical - ankle_force_2_vertical) * ankle_roll_length;

            // double offset_len = std::sqrt(0.045*0.045+0.015*0.015);
            double offset_len = 0.047434165;
            // double offset_theta = aeirobot::DegToRad(90.0) + std::atan(0.015/0.045) + pitch;
            double offset_theta = 1.89254685 + pitch;

            double ankle_pitch_torque = ankle_force * offset_len * std::sin(offset_theta);

            // std::cout << "ankle_roll_dot : " << aeirobot::RadToDeg(roll) << std::endl;
            // std::cout << "ankle_pitch_dot : " << aeirobot::RadToDeg(pitch) << std::endl;

            // 반환: {pitch, roll, pitch_dot, roll_dot}
            // std::cout << "ankle_roll_dot : " << aeirobot::RadToDeg(roll_dot) << std::endl;
            // std::cout << "ankle_pitch_torque : " << ankle_pitch_torque << std::endl;
            return std::make_tuple(pitch, roll, pitch_dot, roll_dot, ankle_pitch_torque, ankle_roll_torque);
        }
        else
        {
            // ---------------------------------------------
            // 피치와 롤의 각도 계산
            // ---------------------------------------------
            double linear_average = std::fabs(linear_1 + linear_2) / 2.0 + 0.3;
            double delta_linear = std::fabs(linear_1 - linear_2) / 2.0;

            double pitch_len2 = linear_average;
            if (pitch_len2 < 0)
            {
                std::cerr << "Error: pitch_len2 is negative: " << pitch_len2 << std::endl;
                pitch_len2 = 0; // NaN 방지
            }
            double roll_pitch_offset_theta = std::acos(linear_average / pitch_len2);
            if (roll_pitch_offset_theta < -1.0 || roll_pitch_offset_theta > 1.0)
            {
                std::cerr << "Error: roll_pitch_offset_theta out of range: " << roll_pitch_offset_theta << std::endl;
                roll_pitch_offset_theta = std::max(-1.0, std::min(1.0, roll_pitch_offset_theta));
            }

            double roll_input = (delta_linear * std::cos(roll_pitch_offset_theta)) / 0.04;
            if (roll_input < -1.0 || roll_input > 1.0)
            {
                std::cerr << "Error: roll_input out of range: " << roll_input << std::endl;
                roll_input = std::max(-1.0, std::min(1.0, roll_input));
            }
            // double roll = std::asin(roll_input);

            // double under_line = std::sqrt(std::pow(0.044, 2)+std::pow(0.016, 2));
            double under_line = 0.0468188;

            // double line_2 = std::sqrt(std::pow((0.37 - 0.038), 2) + std::pow(0.028, 2));
            double line_2 = 0.333178631;

            double cos_theta = (under_line * under_line + line_2 * line_2 - linear_average * linear_average) / (2 * under_line * line_2);
            if (cos_theta < -1.0 || cos_theta > 1.0)
            {
                std::cerr << "Error: cos_theta out of range: " << cos_theta << std::endl;
                cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            }
            // double angle_radians = std::atan(0.016 / 0.044) + std::atan(0.028 / (0.37 - 0.038)) - aeirobot::DegToRad(90.0);
            double angle_radians = 0.348771 + 0.08413824 - 1.5707963;

            // double pitch = (angle_radians + std::acos(cos_theta)) * -1.0;

            // 링크 관절부분 점으로 해서 ik 풀고
            // 리니어 길이로 자코비안 행렬 만들어서 fk
////////////////////////////////////////////////////////////////////////////////
            // ----- LM 역해 -----
            const int    kMaxIters = 30;
            const double kTol      = 1e-6;   // m, RMS 정지 기준
            const double kEps      = 1e-6;   // rad, 수치미분 스텝
            double       lambda    = 1e-4;

            // 초기값 
            double pitch0 = 0.0, roll0 = 0.0;
            Eigen::Vector2d theta(pitch0, roll0);

            // 측정 길이
            const Eigen::Vector2d y_meas(linear_1, linear_2);

            // ===== LM 루프 =====
            int iter = 0;
            for (; iter < kMaxIters; ++iter) {
                // f = 예측 길이 (MapLinearPosAnkle는 std::vector<double> 반환)
                const auto f = MapLinearPosAnkle(theta[0], theta[1]);
                if (f.size() < 2) { break;}

                Eigen::Vector2d y(f[0], f[1]);
                Eigen::Vector2d r = y - y_meas;
                const double rms = std::sqrt(0.5 * r.squaredNorm());

                // 수렴 조건 (RMS가 충분히 작으면 종료)
                if (rms < kTol) break;

                // 수치 미분으로 Jacobian J 계산
                Eigen::Matrix<double,2,2> J;
                for (int j = 0; j < 2; ++j) {
                    Eigen::Vector2d theta_p = theta, theta_m = theta;
                    theta_p[j] += kEps; 
                    theta_m[j] -= kEps;

                    const auto fp = MapLinearPosAnkle(theta_p[0], theta_p[1]);
                    const auto fm = MapLinearPosAnkle(theta_m[0], theta_m[1]);

                    // 사이즈 확인
                    if (fp.size() < 2 || fm.size() < 2) { break; }

                    J(0,j) = (fp[0] - fm[0]) / (2.0 * kEps);
                    J(1,j) = (fp[1] - fm[1]) / (2.0 * kEps);
                }

                // LM 스텝
                Eigen::Matrix2d H = J.transpose() * J + lambda * Eigen::Matrix2d::Identity();
                Eigen::Vector2d g = J.transpose() * r;
                Eigen::Vector2d dx = -H.ldlt().solve(g);

                // 각도 업데이트 + 각도 제한
                Eigen::Vector2d theta_new = theta + dx;

                // 새 오차 평가
                const auto f_new = MapLinearPosAnkle(theta_new[0], theta_new[1]);
                if (f_new.size() < 2) { break;}
                Eigen::Vector2d r_new(f_new[0], f_new[1]); 
                r_new -= y_meas;
                const double rms_new = std::sqrt(0.5 * r_new.squaredNorm());

                // 수용/거부 + 람다 조정
                if (rms_new < rms) { 
                    theta = theta_new; 
                    lambda = std::max(lambda * 0.33, 1e-8); 
                    // dx가 매우 작으면 종료 (각도 변화량 기준)
                    if (dx.norm() < 1e-9) break;
                } else { 
                    lambda = std::min(lambda * 4.0,  1e6); 
                }
            }

            // RMS 계산
            const auto f_last = MapLinearPosAnkle(theta[0], theta[1]);
            Eigen::Vector2d r_last(0.0, 0.0);
            if (f_last.size() >= 2) {
            r_last = Eigen::Vector2d(f_last[0], f_last[1]) - y_meas;
            }
            const double rms_last = std::sqrt(0.5 * r_last.squaredNorm());

            double roll_mag  = theta[1];  
            double pitch     = theta[0];

            // --- 부호 결정 ---
            // 너무 미세한 차이는 노이즈일 수 있으니 데드밴드 사용
            const double eps_len = 1e-5;  // 길이 비교 데드밴드 (m 단위로 적절히 조정)
            double sign_roll = +1.0;

            double d12 = linear_1 - linear_2;
            if (d12 >  eps_len) sign_roll = -1.0; // l1이 더 길면 -롤
            else if (d12 < -eps_len) sign_roll = +1.0; // l2가 더 길면 +롤
            else {
            // 거의 같은 경우: 이전 부호를 유지해 점프 방지 (히스테리시스)
            static double last_sign = +1.0;     
            sign_roll = last_sign;
            }

            // 부호 적용
            double roll = roll_mag * sign_roll;

            // (선택) 디버그 로그
            // std::cerr << std::fixed << std::setprecision(6)
            //           << "[LM] pitch=" << pitch * 180.0/M_PI
            //           << " roll="      << roll  * 180.0/M_PI
            //           << " | d12="     << d12
            //           << " | sign="    << sign_roll
            //           << " | eps_len=" << eps_len
            //           << std::endl;

///////////////////////////////////////////////////////////////////////////////////////

            // ---------------------------------------------
            // 피치와 롤의 속도 계산 (미분)
            // ---------------------------------------------
            // return {pitch, roll};
            // 1. pitch의 linear_average에 대한 미분
            double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
            if (sin_theta == 0.0)
            {
                sin_theta = 1e-8; // 0을 방지하기 위해 작은 값으로 대체
            }
            double dpitch_dlinear_average = linear_average / (under_line * line_2 * sin_theta);

            // 2. linear_average에 대한 linear_1과 linear_2의 미분
            double sign_sum = ((linear_1 + linear_2) >= 0) ? 1.0 : -1.0;
            double dlinear_average_dlinear_1 = 0.5 * sign_sum;
            double dlinear_average_dlinear_2 = 0.5 * sign_sum;

            // 3. pitch의 linear_1과 linear_2에 대한 미분
            double dpitch_dlinear_1 = dpitch_dlinear_average * dlinear_average_dlinear_1;
            double dpitch_dlinear_2 = dpitch_dlinear_average * dlinear_average_dlinear_2;

            // 4. pitch_dot 계산
            double pitch_dot = (dpitch_dlinear_1 * linear_1_dot + dpitch_dlinear_2 * linear_2_dot) * -1.0;

            // 롤의 미분 계산
            // roll = asin(f), f = (delta_linear * cos(theta_offset)) / 0.04
            // 따라서, droll/dx = (1 / sqrt(1 - f^2)) * df/dx

            // 1. f 계산
            double f = (delta_linear * std::cos(roll_pitch_offset_theta)) / 0.04;
            f = std::max(-1.0, std::min(1.0, f)); // 안전을 위해 범위 제한

            // 먼저, linear_average와 pitch_len2의 미분을 계산
            // double dpitch_len2_dlinear_average = (linear_average) / pitch_len2;

            // To avoid complexity, we can use:
            double dcos_theta_offset_dlinear_average = (pitch_len2 * 1 - linear_average * (linear_average / pitch_len2)) / (pitch_len2 * pitch_len2);
            // Simplifies to sin_theta^2 / pitch_len2^3

            // Now, df/dlinear_average
            double df_dlinear_average = (delta_linear / 0.04) * dcos_theta_offset_dlinear_average;

            // df/ddelta_linear
            double df_ddelta_linear = std::cos(roll_pitch_offset_theta) / 0.04;

            // Now, ddelta_linear/dlinear_1 and ddelta_linear/dlinear_2
            double ddelta_linear_dlinear_1 = (linear_1 - linear_2) >= 0 ? 0.5 : -0.5;
            double ddelta_linear_dlinear_2 = (linear_1 - linear_2) >= 0 ? -0.5 : 0.5;

            // Now, df/dlinear_1 and df/dlinear_2 using chain rule
            double df_dlinear_1 = df_dlinear_average * dlinear_average_dlinear_1 + df_ddelta_linear * ddelta_linear_dlinear_1;
            double df_dlinear_2 = df_dlinear_average * dlinear_average_dlinear_2 + df_ddelta_linear * ddelta_linear_dlinear_2;

            // Now, droll/dlinear_1 and droll/dlinear_2
            double denominator_roll = std::sqrt(1.0 - f * f);
            if (denominator_roll == 0.0)
            {
                denominator_roll = 1e-8; // 0을 방지하기 위해 작은 값으로 대체
            }
            double droll_dlinear_1 = (1.0 / denominator_roll) * df_dlinear_1;
            double droll_dlinear_2 = (1.0 / denominator_roll) * df_dlinear_2;

            // 롤의 시간 미분 (roll_dot) 계산
            double roll_dot = droll_dlinear_1 * linear_1_dot + droll_dlinear_2 * linear_2_dot;

            // 롤의 부호 조정
            if (linear_1 >= linear_2)
            {
                roll = roll * -1.0;
                roll_dot = roll_dot * -1.0;
            }
            // ---------------------------------------------
            // 피치 토크 계산
            // ---------------------------------------------

            // Constants (meters)
            const double ankle_roll_length = 0.04;
            // const double ankle_pitch_length_top = 0.04;    // Top spacing between linear actuators
            // const double ankle_pitch_length_bottom = 0.04; // Bottom spacing between linear actuators

            // Calculate half lengths
            // const double ankle_pitch_length_top_half = ankle_pitch_length_top / 2.0;       // 0.04 m
            // const double ankle_pitch_length_bottom_half = ankle_pitch_length_bottom / 2.0; // 0.04 m
            // const double ankle_pitch_length_top_half = 0.04;    // 0.04 m
            // const double ankle_pitch_length_bottom_half = 0.04; // 0.04 m

            // Calculate the vertical component of forces from each linear actuator (half)
            double ankle_force_1_vertical = linear_1_effort;
            double ankle_force_2_vertical = linear_2_effort;

            // Total vertical force acting on the ankle
            double ankle_force = ankle_force_1_vertical + ankle_force_2_vertical;
            double ankle_roll_torque = (ankle_force_1_vertical - ankle_force_2_vertical) * ankle_roll_length;

            // double offset_len = std::sqrt(0.044*0.044+0.016*0.016);
            double offset_len = 0.0468188;
            // double offset_theta = aeirobot::DegToRad(90.0) + std::atan(0.016/0.044) + pitch;
            double offset_theta = 1.5707963 + 0.34877100 + pitch;

            double ankle_pitch_torque = ankle_force * offset_len * std::sin(offset_theta);

            // std::cout << "ankle_roll_dot : " << aeirobot::RadToDeg(roll) << std::endl;
            // std::cout << "ankle_pitch_dot : " << aeirobot::RadToDeg(pitch) << std::endl;

            // 반환: {pitch, roll, pitch_dot, roll_dot}
            // std::cout << "ankle_roll_dot : " << aeirobot::RadToDeg(roll_dot) << std::endl;
            // std::cout << "ankle_pitch_torque : " << ankle_pitch_torque << std::endl;
            return std::make_tuple(pitch, roll, pitch_dot, roll_dot, ankle_pitch_torque, ankle_roll_torque);
        }
    }

    std::tuple<double, double, double, double, double, double>
    VirtualJointMap::CalculateHipLinearMap(double linear_1, double linear_2,
                                           double linear_1_dot, double linear_2_dot,
                                           double linear_1_effort, double linear_2_effort)
    {
        if (alice4_version == 1)
        {
            // ---------------------------------------------
            // 피치와 롤의 각도 계산
            // ---------------------------------------------
            double linear_average = std::fabs(linear_1 + linear_2) / 2.0 + 0.2735;
            double delta_linear = std::fabs(linear_1 - linear_2) / 2.0;

            // double pitch_len2 = std::sqrt(linear_average * linear_average + std::pow(0.055 - 0.04, 2));
            double pitch_len2 = std::sqrt(linear_average * linear_average + 0.000225);
            if (pitch_len2 < 0)
            {
                std::cerr << "Error: pitch_len2 is negative: " << pitch_len2 << std::endl;
                pitch_len2 = 0; // NaN 방지
            }
            double roll_pitch_offset_theta = std::acos(linear_average / pitch_len2);
            if (roll_pitch_offset_theta < -1.0 || roll_pitch_offset_theta > 1.0)
            {
                std::cerr << "Error: roll_pitch_offset_theta out of range: " << roll_pitch_offset_theta << std::endl;
                roll_pitch_offset_theta = std::max(-1.0, std::min(1.0, roll_pitch_offset_theta));
            }

            // roll 계산 및 검증
            double roll_input = (delta_linear * std::cos(roll_pitch_offset_theta)) / 0.04;
            if (roll_input < -1.0 || roll_input > 1.0)
            {
                std::cerr << "Error: roll_input out of range: " << roll_input << std::endl;
                roll_input = std::max(-1.0, std::min(1.0, roll_input));
            }
            double roll = std::asin(roll_input);

            // double under_line = std::sqrt(std::pow((0.061 - 0.015), 2) + std::pow(0.03, 2));
            double under_line = 0.054918121;
            // double line_2 = std::sqrt(std::pow((0.355 - 0.064), 2) + std::pow(0.05, 2));
            double line_2 = 0.286672287;

            double cos_theta = (under_line * under_line + line_2 * line_2 - linear_average * linear_average) / (2 * under_line * line_2);
            if (cos_theta < -1.0 || cos_theta > 1.0)
            {
                std::cerr << "Error: cos_theta out of range: " << cos_theta << std::endl;
                cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            }
            // double angle_radians = aeirobot::DegToRad(180.0) - std::atan2(0.03, 0.061 - 0.015);
            double angle_radians = 2.565634;

            double pitch = (angle_radians - std::acos(cos_theta)) * -1.0;

            // if (linear_1 >= linear_2)
            // {
            //     roll = roll * -1.0;
            // }

            // // std::cout << "hip roll : " << aeirobot::RadToDeg(roll) << std::endl;
            // // std::cout << "hip pitch : " << aeirobot::RadToDeg(pitch) << std::endl;

            // return {pitch, roll};
            // ---------------------------------------------
            // 피치와 롤의 속도 계산 (미분)
            // ---------------------------------------------

            // 상수값 정의
            // const double a = 0.045;       // under_line
            // const double b = 0.286672287; // line_2

            // 피치 각도의 미분 계산
            double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
            if (sin_theta == 0.0)
            {
                sin_theta = 1e-8; // 0을 방지하기 위해 작은 값으로 대체
            }
            double dpitch_dlinear_average = linear_average / (under_line * line_2 * sin_theta);

            // linear_average에 대한 linear_1과 linear_2의 미분
            double sum_linear = linear_1 + linear_2;
            double sign_sum = (sum_linear >= 0) ? 1.0 : -1.0;
            double dlinear_average_dlinear_1 = 0.5 * sign_sum;
            double dlinear_average_dlinear_2 = 0.5 * sign_sum;

            // 피치의 linear_1과 linear_2에 대한 미분 (체인 룰 적용)
            double dpitch_dlinear_1 = dpitch_dlinear_average * dlinear_average_dlinear_1;
            double dpitch_dlinear_2 = dpitch_dlinear_average * dlinear_average_dlinear_2;

            // 피치 속도(pitch_dot) 계산
            double pitch_dot = dpitch_dlinear_1 * linear_1_dot + dpitch_dlinear_2 * linear_2_dot;

            // 롤 각도의 미분 계산
            // roll = asin(f), f = (delta_linear * cos(theta_offset)) / 0.04
            double f = (delta_linear * std::cos(roll_pitch_offset_theta)) / 0.04;
            // 안전을 위해 asin의 인수 범위를 [-1, 1]로 제한
            f = std::max(-1.0, std::min(1.0, f));
            double droll_df = 1.0 / std::sqrt(1.0 - f * f);

            // cos(theta_offset)의 미분: dcos(theta_offset)/dlinear_average
            double dcos_theta_offset_dlinear_average = (pitch_len2 - linear_average * (linear_average / pitch_len2)) / (pitch_len2 * pitch_len2);

            // df/dlinear_average = (delta_linear / 0.04) * dcos_theta_offset_dlinear_average
            double df_dlinear_average = (delta_linear / 0.04) * dcos_theta_offset_dlinear_average;

            // df/ddelta_linear = cos(theta_offset) / 0.04
            double df_ddelta_linear = std::cos(roll_pitch_offset_theta) / 0.04;

            // ddelta_linear/dlinear_1와 ddelta_linear/dlinear_2 계산
            double delta_linear_sign = (linear_1 - linear_2) >= 0 ? 1.0 : -1.0;
            double ddelta_linear_dlinear_1 = 0.5 * delta_linear_sign;
            double ddelta_linear_dlinear_2 = -0.5 * delta_linear_sign;

            // df/dlinear_1 = df/dlinear_average * dlinear_average/dlinear_1 + df/ddelta_linear * ddelta_linear/dlinear_1
            double df_dlinear_1 = df_dlinear_average * dlinear_average_dlinear_1 + df_ddelta_linear * ddelta_linear_dlinear_1;

            // df/dlinear_2 = df/dlinear_average * dlinear_average/dlinear_2 + df/ddelta_linear * ddelta_linear/dlinear_2
            double df_dlinear_2 = df_dlinear_average * dlinear_average_dlinear_2 + df_ddelta_linear * ddelta_linear_dlinear_2;

            // droll/dlinear_1 = droll/df * df/dlinear_1
            double droll_dlinear_1 = droll_df * df_dlinear_1;

            // droll/dlinear_2 = droll/df * df/dlinear_2
            double droll_dlinear_2 = droll_df * df_dlinear_2;

            // 롤 속도(roll_dot) 계산
            double roll_dot = droll_dlinear_1 * linear_1_dot + droll_dlinear_2 * linear_2_dot;

            // 롤의 부호 조정 (linear_1_dot >= linear_2_dot인 경우)
            if (linear_1 >= linear_2)
            {
                roll = roll * -1.0;
                roll_dot = roll_dot * -1.0;
            }

            // ---------------------------------------------
            // 피치 토크 계산
            // ---------------------------------------------

            double hip_roll_length = 0.055;
            // double hip_pitch_length_top = 0.055;   // Top spacing between linear actuators
            // double hip_pitch_length_bottom = 0.04; // Bottom spacing between linear actuators

            // double hip_pitch_angle = std::atan((hip_pitch_length_top - hip_pitch_length_bottom) / hip_pitch_length_bottom);
            // double hip_pitch_angle = 0.35877067;
            double ankle_cos_pitch_angle_offset = 0.93632918;
            // double ankle_cos_pitch_angle_offset = std::cos(hip_pitch_angle);
            double hip_force_1_vertical = linear_1_effort * ankle_cos_pitch_angle_offset;
            double hip_force_2_vertical = linear_2_effort * ankle_cos_pitch_angle_offset;
            double hip_force = hip_force_1_vertical + hip_force_2_vertical;

            double hip_roll_torque = (hip_force_1_vertical - hip_force_2_vertical) * hip_roll_length;

            // double offset_len = std::sqrt(0.03*0.03+0.046*0.046);
            double offset_len = 0.003016;
            // double offset_theta =  std::atan(0.046/0.03) + std::atan(0.02/(0.355-0.064))) + pitch;
            double offset_theta = 0.99289439 + 0.06862061 + pitch;
            double hip_pitch_torque = hip_force * offset_len * std::cos(offset_theta) * -1.0;
            // ---------------------------------------------
            // 반환: pitch, roll, pitch_dot, roll_dot
            // ---------------------------------------------
            // std::cout << "hip_roll_dot : " << aeirobot::RadToDeg(roll_dot) << std::endl;
            // std::cout << "hip_pitch_dot : " << aeirobot::RadToDeg(pitch_dot) << std::endl;
            // std::cout << "hip_pitch_torque : " << hip_pitch_torque << std::endl;
            // std::cout << "hip_roll_torque : " << hip_roll_torque << std::endl;
            return std::make_tuple(pitch, roll, pitch_dot, roll_dot, hip_pitch_torque, hip_roll_torque);
        }
        else
        {
            // ---------------------------------------------
            // 피치와 롤의 각도 계산
            // ---------------------------------------------
            double linear_average = std::fabs(linear_1 + linear_2) / 2.0 + 0.3;
            double delta_linear = std::fabs(linear_1 - linear_2) / 2.0;

            double pitch_len2 = linear_average;
            if (pitch_len2 < 0)
            {
                std::cerr << "Error: pitch_len2 is negative: " << pitch_len2 << std::endl;
                pitch_len2 = 0; // NaN 방지
            }

            double roll_input = (delta_linear) / 0.04;
            if (roll_input < -1.0 || roll_input > 1.0)
            {
                std::cerr << "Error: roll_input out of range: " << roll_input << std::endl;
                roll_input = std::max(-1.0, std::min(1.0, roll_input));
            }
            // double roll = std::asin(roll_input);

            double under_line = 0.047;
            // double line_2 = std::sqrt(std::pow((0.37 - 0.04), 2) + std::pow(0.05, 2));
            double line_2 = 0.33376639;

            double cos_theta = (under_line * under_line + line_2 * line_2 - linear_average * linear_average) / (2 * under_line * line_2);
            if (cos_theta < -1.0 || cos_theta > 1.0)
            {
                std::cerr << "Error: cos_theta out of range: " << cos_theta << std::endl;
                cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
            }
            // double angle_radians = aeirobot::DegToRad(90.0) - std::atan2(0.05, 0.37 - 0.04);
            double angle_radians = 1.5707963 - 0.15037018;

            // double pitch = (angle_radians - std::acos(cos_theta)) * -1.0 - aeirobot::DegToRad(30.0);

            // 링크 관절부분 점으로 해서 ik 풀고
            // 리니어 길이로 자코비안 행렬 만들어서 fk
////////////////////////////////////////////////////////////////////////////////
            // ----- LM 역해 -----
            const int    kMaxIters = 30;
            const double kTol      = 1e-6;   // m, RMS 정지 기준
            const double kEps      = 1e-6;   // rad, 수치미분 스텝
            double       lambda    = 1e-4;

            // 초기값 
            double pitch0 = 0.0, roll0 = 0.0;
            Eigen::Vector2d theta(pitch0, roll0);

            // 측정 길이
            const Eigen::Vector2d y_meas(linear_1, linear_2);

            // ===== LM 루프 =====
            int iter = 0;
            for (; iter < kMaxIters; ++iter) {
                // f = 예측 길이 (MapLinearPosPelvis는 std::vector<double> 반환)
                const auto f = MapLinearPosPelvis(theta[0], theta[1]);
                if (f.size() < 2) { break;}

                Eigen::Vector2d y(f[0], f[1]);
                Eigen::Vector2d r = y - y_meas;
                const double rms = std::sqrt(0.5 * r.squaredNorm());

                // 수렴 조건 (RMS가 충분히 작으면 종료)
                if (rms < kTol) break;

                // 수치 미분으로 Jacobian J 계산
                Eigen::Matrix<double,2,2> J;
                for (int j = 0; j < 2; ++j) {
                    Eigen::Vector2d theta_p = theta, theta_m = theta;
                    theta_p[j] += kEps; 
                    theta_m[j] -= kEps;

                    const auto fp = MapLinearPosPelvis(theta_p[0], theta_p[1]);
                    const auto fm = MapLinearPosPelvis(theta_m[0], theta_m[1]);

                    // 사이즈 확인
                    if (fp.size() < 2 || fm.size() < 2) { break; }

                    J(0,j) = (fp[0] - fm[0]) / (2.0 * kEps);
                    J(1,j) = (fp[1] - fm[1]) / (2.0 * kEps);
                }

                // LM 스텝
                Eigen::Matrix2d H = J.transpose() * J + lambda * Eigen::Matrix2d::Identity();
                Eigen::Vector2d g = J.transpose() * r;
                Eigen::Vector2d dx = -H.ldlt().solve(g);

                // 각도 업데이트 + 각도 제한
                Eigen::Vector2d theta_new = theta + dx;

                // 새 오차 평가
                const auto f_new = MapLinearPosPelvis(theta_new[0], theta_new[1]);
                if (f_new.size() < 2) { break;}
                Eigen::Vector2d r_new(f_new[0], f_new[1]); 
                r_new -= y_meas;
                const double rms_new = std::sqrt(0.5 * r_new.squaredNorm());

                // 수용/거부 + 람다 조정
                if (rms_new < rms) { 
                    theta = theta_new; 
                    lambda = std::max(lambda * 0.33, 1e-8); 
                    // dx가 매우 작으면 종료 (각도 변화량 기준)
                    if (dx.norm() < 1e-9) break;
                } else { 
                    lambda = std::min(lambda * 4.0,  1e6); 
                }
            }

            // RMS 계산
            const auto f_last = MapLinearPosPelvis(theta[0], theta[1]);
            Eigen::Vector2d r_last(0.0, 0.0);
            if (f_last.size() >= 2) {
            r_last = Eigen::Vector2d(f_last[0], f_last[1]) - y_meas;
            }
            const double rms_last = std::sqrt(0.5 * r_last.squaredNorm());

            double roll_mag  = theta[1];  
            double pitch     = theta[0];

            // --- 부호 결정 ---
            // 너무 미세한 차이는 노이즈일 수 있으니 데드밴드 사용
            const double eps_len = 1e-5;  // 길이 비교 데드밴드 (m 단위로 적절히 조정)
            double sign_roll = +1.0;

            double d12 = linear_1 - linear_2;
            if (d12 >  eps_len) sign_roll = -1.0; // l1이 더 길면 -롤
            else if (d12 < -eps_len) sign_roll = +1.0; // l2가 더 길면 +롤
            else {
            // 거의 같은 경우: 이전 부호를 유지해 점프 방지 (히스테리시스)
            static double last_sign = +1.0;     
            sign_roll = last_sign;
            }

            // 부호 적용
            double roll = roll_mag * sign_roll;

            // (선택) 디버그 로그
            // std::cerr << std::fixed << std::setprecision(6)
            //           << "[LM] pitch=" << pitch * 180.0/M_PI
            //           << " roll="      << roll  * 180.0/M_PI
            //           << " | d12="     << d12
            //           << " | sign="    << sign_roll
            //           << " | eps_len=" << eps_len
            //           << std::endl;

///////////////////////////////////////////////////////////////////////////////////////

            

            // if (linear_1 >= linear_2)
            // {
            //     roll = roll * -1.0;
            // }

            // // std::cout << "hip roll : " << aeirobot::RadToDeg(roll) << std::endl;
            // // std::cout << "hip pitch : " << aeirobot::RadToDeg(pitch) << std::endl;

            // return {pitch, roll};
            // ---------------------------------------------
            // 피치와 롤의 속도 계산 (미분)
            // ---------------------------------------------

            // 상수값 정의
            // const double a = 0.045;       // under_line
            // const double b = 0.286672287; // line_2

            // 피치 각도의 미분 계산
            double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
            if (sin_theta == 0.0)
            {
                sin_theta = 1e-8; // 0을 방지하기 위해 작은 값으로 대체
            }
            double dpitch_dlinear_average = linear_average / (under_line * line_2 * sin_theta);

            // linear_average에 대한 linear_1과 linear_2의 미분
            double sum_linear = linear_1 + linear_2;
            double sign_sum = (sum_linear >= 0) ? 1.0 : -1.0;
            double dlinear_average_dlinear_1 = 0.5 * sign_sum;
            double dlinear_average_dlinear_2 = 0.5 * sign_sum;

            // 피치의 linear_1과 linear_2에 대한 미분 (체인 룰 적용)
            double dpitch_dlinear_1 = dpitch_dlinear_average * dlinear_average_dlinear_1;
            double dpitch_dlinear_2 = dpitch_dlinear_average * dlinear_average_dlinear_2;

            // 피치 속도(pitch_dot) 계산
            double pitch_dot = dpitch_dlinear_1 * linear_1_dot + dpitch_dlinear_2 * linear_2_dot;

            // 롤 각도의 미분 계산
            // roll = asin(f), f = (delta_linear * cos(theta_offset)) / 0.04
            double f = delta_linear / 0.04;
            // 안전을 위해 asin의 인수 범위를 [-1, 1]로 제한
            f = std::max(-1.0, std::min(1.0, f));
            double droll_df = 1.0 / std::sqrt(1.0 - f * f);

            // cos(theta_offset)의 미분: dcos(theta_offset)/dlinear_average
            double dcos_theta_offset_dlinear_average = (pitch_len2 - linear_average * (linear_average / pitch_len2)) / (pitch_len2 * pitch_len2);

            // df/dlinear_average = (delta_linear / 0.04) * dcos_theta_offset_dlinear_average
            double df_dlinear_average = (delta_linear / 0.04) * dcos_theta_offset_dlinear_average;

            // df/ddelta_linear = cos(theta_offset) / 0.04
            double df_ddelta_linear = 1.0 / 0.04;

            // ddelta_linear/dlinear_1와 ddelta_linear/dlinear_2 계산
            double delta_linear_sign = (linear_1 - linear_2) >= 0 ? 1.0 : -1.0;
            double ddelta_linear_dlinear_1 = 0.5 * delta_linear_sign;
            double ddelta_linear_dlinear_2 = -0.5 * delta_linear_sign;

            // df/dlinear_1 = df/dlinear_average * dlinear_average/dlinear_1 + df/ddelta_linear * ddelta_linear/dlinear_1
            double df_dlinear_1 = df_dlinear_average * dlinear_average_dlinear_1 + df_ddelta_linear * ddelta_linear_dlinear_1;

            // df/dlinear_2 = df/dlinear_average * dlinear_average/dlinear_2 + df/ddelta_linear * ddelta_linear/dlinear_2
            double df_dlinear_2 = df_dlinear_average * dlinear_average_dlinear_2 + df_ddelta_linear * ddelta_linear_dlinear_2;

            // droll/dlinear_1 = droll/df * df/dlinear_1
            double droll_dlinear_1 = droll_df * df_dlinear_1;

            // droll/dlinear_2 = droll/df * df/dlinear_2
            double droll_dlinear_2 = droll_df * df_dlinear_2;

            // 롤 속도(roll_dot) 계산
            double roll_dot = droll_dlinear_1 * linear_1_dot + droll_dlinear_2 * linear_2_dot;

            // 롤의 부호 조정 (linear_1_dot >= linear_2_dot인 경우)
            if (linear_1 >= linear_2)
            {
                roll = roll * -1.0;
                roll_dot = roll_dot * -1.0;
            }

            // ---------------------------------------------
            // 피치 토크 계산
            // ---------------------------------------------

            double hip_roll_length = 0.04;
            double hip_force_1_vertical = linear_1_effort;
            double hip_force_2_vertical = linear_2_effort;
            double hip_force = hip_force_1_vertical + hip_force_2_vertical;

            double hip_roll_torque = (hip_force_1_vertical - hip_force_2_vertical) * hip_roll_length;

            double offset_len = 0.047;
            // double offset_theta =  std::atan(0.003/(0.37-0.064))) + pitch;
            double offset_theta = 0.00980361 + pitch;
            double hip_pitch_torque = hip_force * offset_len * std::cos(offset_theta) * -1.0;
            // ---------------------------------------------
            // 반환: pitch, roll, pitch_dot, roll_dot
            // ---------------------------------------------
            // std::cout << "hip_roll_dot : " << aeirobot::RadToDeg(roll_dot) << std::endl;
            // std::cout << "hip_pitch_dot : " << aeirobot::RadToDeg(pitch_dot) << std::endl;
            // std::cout << "hip_pitch_torque : " << hip_pitch_torque << std::endl;
            // std::cout << "hip_roll_torque : " << hip_roll_torque << std::endl;
            return std::make_tuple(pitch, roll, pitch_dot, roll_dot, hip_pitch_torque, hip_roll_torque);
        }
    }

    std::tuple<double, double, double>
    VirtualJointMap::CalculateKneeLinearMap(double linear_1, double linear_1_dot, double linear_1_effort)
    {
        if (alice4_version == 1)
        {
            // ---------------------------------------------
            // 피치 각도 계산
            // ---------------------------------------------
            // 1) 기존 상수 정의
            // const double knee_offset = std::atan(0.04 / 0.05);
            // const double r = std::sqrt(0.05 * 0.05 + 0.04 * 0.04);   // 대각선 링크 길이 0.064031
            // const double K = std::sqrt(0.378 * 0.378 + 0.04 * 0.04); // 0.378^2 + 0.04^2 의 루트 0.38011
            // const double alpha = std::atan2(-0.04, 0.378);           // 위에서 말한 -0.04 / 0.378

            const double knee_offset = 0.674741;
            const double alpha = 0.105480; // std::atan2(0.04, 0.378)

            // 2) distance(d) = linear + 0.3285(원래 값)
            double distance = linear_1 + 0.40787876; // 빗변 길이가 다른 두 선분 길이합보다 큰 상황 방지 위해 값 조정
            // 3) C = d^2 - r^2 - (0.378^2 + 0.04^2)
            // double C = - d * d + r * r + (0.378 * 0.378 + 0.04 * 0.04);
            double C = -distance * distance + 0.148584;
            // 4) val = C / (2*r*K)
            // double val = C / (2.0 * r * K);
            double val = C / 0.048677647;
            // arccos의 정의역(-1 ~ 1)을 벗어나는 지 체크(실제 구동범위에 따라 예외처리)
            if (val > 1.0 || val < -1.0)
            {
                // std::cerr << "Warning: val out of range for acos: " << val << std::endl;
                val = std::max(-1.0, std::min(1.0, val)); // 범위를 강제 조정
            }
            // 5) theta = alpha + acos(val)
            // double theta = alpha + std::acos(val);
            // 6) pitch = theta - knee_offset

            double pitch;
            try
            {
                pitch = aeirobot::DegToRad(180.0) - alpha - std::acos(val) - knee_offset;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error in acos calculation: " << e.what() << ", val: " << val << std::endl;
                pitch = 0.0; // 예외 처리 후 기본 값
            }
            // std::cout << "knee_pitch : " << aeirobot::RadToDeg(pitch) << std::endl;

            // ---------------------------------------------
            // 피치 속도 계산 (미분)
            // ---------------------------------------------

            // val = C / C_denominator
            // C = distance^2 - 0.148583969
            // dC/dt = 2 * distance * linear_1_dot
            double dC_dt = 2.0 * distance * linear_1_dot;

            // val = C / C_denominator
            // dval/dt = dC/dt / C_denominator
            // double dval_dt = dC_dt / C_denominator;
            double dval_dt = dC_dt / 0.048677647; //(2.0 * r * K);

            // theta = alpha + acos(val)
            // dtheta/dt = -1 / sqrt(1 - val^2) * dval/dt
            double dtheta_dt;
            if (val > 1.0 - 1e-8)
            {
                // val가 1에 가까워지면 미분값을 0으로 설정하여 수치적 안정성 확보
                dtheta_dt = 0.0;
            }
            else if (val < -1.0 + 1e-8)
            {
                // val가 -1에 가까워지면 미분값을 0으로 설정하여 수치적 안정성 확보
                dtheta_dt = 0.0;
            }
            else
            {
                double denominator = std::sqrt(1.0 - val * val);
                if (denominator < 1e-8)
                {
                    // 분모가 너무 작아지면 작은 값으로 대체하여 수치적 안정성 확보
                    denominator = 1e-8;
                }
                dtheta_dt = (-1.0 / denominator) * dval_dt;
            }

            // pitch = theta - knee_offset
            // dpitch/dt = dtheta/dt
            double pitch_dot = dtheta_dt;
            // std::cout << "knee_pitch_dot : " << aeirobot::RadToDeg(pitch_dot) << std::endl;

            // ---------------------------------------------
            // 피치 토크 계산
            // ---------------------------------------------
            // double offset_len = std::sqrt(0.04*0.04+0.05*0.05);
            double offset_len = 0.064031242;
            // double offset_theta = aeirobot::DegToRad(90.0) - std::atan(0.04/0.05) + pitch;
            double offset_theta = 0.89605536 + pitch;
            double knee_torque = linear_1_effort * offset_len * std::sin(offset_theta) * -1.0;

            // std::cout << "knee_torque : " << knee_torque << std::endl;

            // ---------------------------------------------
            // 반환: pitch, pitch_dot
            // ---------------------------------------------
            return std::make_tuple(pitch, pitch_dot, knee_torque);
        }
        else
        {
            // ---------------------------------------------
            // 피치 각도 계산
            // ---------------------------------------------
            // 1) 기존 상수 정의
            // const double knee_offset = std::atan(0.028 / 0.038);
            // const double r = std::sqrt(0.038 * 0.038 + 0.028 * 0.028);   // 대각선 링크 길이 0.047201695
            // const double K = std::sqrt(0.337 * 0.337 + 0.036 * 0.036); // 0.337^2 + 0.036^2 의 루트 0.338917394
            // const double alpha = std::atan2(-0.036, 0.337);           // 위에서 말한 -0.04 / 0.378

            const double knee_offset = 0.63502674;
            const double alpha = 0.10642189; // std::atan2(-0.036, 0.337)

            // 2) distance(d) = linear + 0.306(원래  길이)
            double distance = linear_1 + 0.306; // 빗변 길이가 다른 두 선분 길이합보다 큰 상황 방지 위해 값 조정
            // 3) C = -d^2 + r^2 + (0.337^2 + 0.036^2)
            // double C = -distance * distance + r * r + K * K;
            double C = -distance * distance + 0.117093;
            // 4) val = C / (2*r*K)
            // double val = C / (2.0 * r * K);
            double val = C / 0.031994951;
            // arccos의 정의역(-1 ~ 1)을 벗어나는 지 체크(실제 구동범위에 따라 예외처리)
            if (val > 1.0 || val < -1.0)
            {
                // std::cerr << "Warning: val out of range for acos: " << val << std::endl;
                val = std::max(-1.0, std::min(1.0, val)); // 범위를 강제 조정
            }
            double pitch;
            try
            {
                pitch = aeirobot::DegToRad(180.0) - alpha - std::acos(val) - knee_offset;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error in acos calculation: " << e.what() << ", val: " << val << std::endl;
                pitch = 0.0; // 예외 처리 후 기본 값
            }
            // std::cout << "knee_pitch : " << aeirobot::RadToDeg(pitch) << std::endl;

            // ---------------------------------------------
            // 피치 속도 계산 (미분)
            // ---------------------------------------------

            // val = C / C_denominator
            // C = distance^2 - 0.148583969
            // dC/dt = 2 * distance * linear_1_dot
            double dC_dt = 2.0 * distance * linear_1_dot;

            // val = C / C_denominator
            // dval/dt = dC/dt / C_denominator
            // double dval_dt = dC_dt / C_denominator;
            double dval_dt = dC_dt / 0.031994951; //(2.0 * r * K);

            // theta = alpha + acos(val)
            // dtheta/dt = -1 / sqrt(1 - val^2) * dval/dt
            double dtheta_dt;
            if (val > 1.0 - 1e-8)
            {
                // val가 1에 가까워지면 미분값을 0으로 설정하여 수치적 안정성 확보
                dtheta_dt = 0.0;
            }
            else if (val < -1.0 + 1e-8)
            {
                // val가 -1에 가까워지면 미분값을 0으로 설정하여 수치적 안정성 확보
                dtheta_dt = 0.0;
            }
            else
            {
                double denominator = std::sqrt(1.0 - val * val);
                if (denominator < 1e-8)
                {
                    // 분모가 너무 작아지면 작은 값으로 대체하여 수치적 안정성 확보
                    denominator = 1e-8;
                }
                dtheta_dt = (-1.0 / denominator) * dval_dt;
            }

            // pitch = theta - knee_offset
            // dpitch/dt = dtheta/dt
            double pitch_dot = dtheta_dt;
            // std::cout << "knee_pitch_dot : " << aeirobot::RadToDeg(pitch_dot) << std::endl;

            // ---------------------------------------------
            // 피치 토크 계산
            // ---------------------------------------------
            // double offset_len = std::sqrt(0.028*0.028+0.038*0.038);
            double offset_len = 0.047201695;
            // double offset_theta = aeirobot::DegToRad(90.0) - std::atan(0.028/0.038) + pitch;
            double offset_theta = 0.93576956 + pitch;
            double knee_torque = linear_1_effort * offset_len * std::sin(offset_theta) * -1.0;

            // std::cout << "knee_torque : " << knee_torque << std::endl;

            // ---------------------------------------------
            // 반환: pitch, pitch_dot
            // ---------------------------------------------
            return std::make_tuple(pitch, pitch_dot, knee_torque);
        }
    }

    // Pos Linear Convert
    double VirtualJointMap::MapLinearPosKnee(double pitch) const
    {
        if (alice4_version == 1)
        {
            double knee_offset = std::atan(0.04 / 0.05);                 // atan to calculate the knee offset
            double r = std::sqrt(std::pow(0.05, 2) + std::pow(0.04, 2)); // Calculate the radius 'r'
            double point1_x = r * std::sin(knee_offset + pitch);
            double point1_y = -1 * r * std::cos(knee_offset + pitch);
            double linear = std::sqrt(std::pow((point1_y - 0.378), 2) + std::pow((point1_x - 0.04), 2)) - 0.3285;
            return linear;
        }
        else if (alice4_version == 2)
        {
            double knee_offset = std::atan(0.028 / 0.038);                 // atan to calculate the knee offset
            double r = std::sqrt(std::pow(0.038, 2) + std::pow(0.028, 2)); // Calculate the radius 'r'
            double point1_x = r * std::sin(knee_offset + pitch);
            double point1_y = -1 * r * std::cos(knee_offset + pitch);
            double linear = std::sqrt(std::pow((point1_y - 0.337), 2) + std::pow((point1_x - 0.036), 2)) - 0.306;
            return linear;
        }
        else
        {
            return 0.0;
        }
    }

    std::vector<double> VirtualJointMap::MapLinearPosPelvis(double pitch, double roll) const
    {
        if (alice4_version == 1)
        {
            //  ---------------------------
            //  (1) pitch *= -1 처리
            //  ---------------------------
            pitch *= -1.0;

            // ---------------------------
            // (2) 점 B를 "한 번에" 계산
            // ---------------------------
            const double r = 0.355;

            // p_b_x = (-r + 0.064)*sin(pitch) - 0.05*cos(pitch)
            // p_b_y = (-r + 0.064)*cos(pitch) + 0.05*sin(pitch)
            const double p_b_x =
                (-r + 0.064) * std::sin(pitch) - 0.05 * std::cos(pitch);

            const double p_b_y =
                (-r + 0.064) * std::cos(pitch) + 0.05 * std::sin(pitch);

            // ---------------------------
            // (3) BC = BO + OC
            //     B->O = -p_b, O->C = (oc_x, oc_y)
            // ---------------------------
            // oc_x = -0.03, oc_y = 0.046 (원본 그대로)
            const double oc_x = -0.03;
            const double oc_y = 0.046;

            // B->O = (-p_b_x, -p_b_y)
            const double bo_x = -p_b_x;
            const double bo_y = -p_b_y;

            // BC = BO + OC
            const double bc_x = bo_x + oc_x;
            const double bc_y = bo_y + oc_y;

            // ---------------------------
            // (4) pitch_len = ||BC||
            // ---------------------------
            const double pitch_len =
                std::sqrt(bc_x * bc_x + bc_y * bc_y);

            // ---------------------------
            // (5) pitch_len2 계산
            //     = sqrt( pitch_len^2 + (0.055 - 0.04)^2 )
            // ---------------------------
            // 즉, sqrt( pitch_len^2 + 0.015^2 )
            const double diff = 0.055 - 0.04; // 0.015
            const double pitch_len2 =
                std::sqrt(pitch_len * pitch_len + diff * diff);

            // ---------------------------
            // (6) roll_pitch_offset_theta = acos(pitch_len / pitch_len2)
            // ---------------------------
            const double roll_pitch_offset_theta =
                std::acos(pitch_len / pitch_len2);

            // ---------------------------
            // (7) roll_cal = |0.04 * sin(roll) / cos(roll_pitch_offset_theta)|
            // ---------------------------
            const double roll_cal =
                std::abs(0.04 * std::sin(roll) / std::cos(roll_pitch_offset_theta));

            // ---------------------------
            // (8) 최종 선형 액추에이터 길이 반환
            //     (기본 오프셋 0.2735를 빼주는 부분은 동일)
            // ---------------------------
            if (roll >= 0.0)
            {
                return {
                    pitch_len2 - roll_cal - 0.2735,
                    pitch_len2 + roll_cal - 0.2735};
            }
            else
            {
                return {
                    pitch_len2 + roll_cal - 0.2735,
                    pitch_len2 - roll_cal - 0.2735};
            }
        }
        else if (alice4_version == 2)
        {
            // ===== 함수 내부에 정의 =====
            auto Rx = [](double th) -> Eigen::Matrix3d {
            const double c = std::cos(th), s = std::sin(th);
            Eigen::Matrix3d R; R <<
                1, 0, 0,
                0, c,-s,
                0, s, c;
            return R;
            };
            auto Ry = [](double th) -> Eigen::Matrix3d {
            const double c = std::cos(th), s = std::sin(th);
            Eigen::Matrix3d R; R <<
                c, 0, s,
                0, 1, 0,
                -s, 0, c;
            return R;
            };
            auto Rodrigues = [](const Eigen::Vector3d& u_in, double th) -> Eigen::Matrix3d {
            Eigen::Vector3d u = u_in.normalized();
            const double c = std::cos(th), s = std::sin(th);
            Eigen::Matrix3d Ux;
            Ux <<     0, -u.z(),  u.y(),
                    u.z(),     0, -u.x(),
                    -u.y(),  u.x(),     0;
            return c * Eigen::Matrix3d::Identity()
                + (1.0 - c) * (u * u.transpose())
                + s * Ux;
            };

            // ===== (1) pitch 보정: pitch = pitch + (30deg) =====
            const double pitch_m = pitch + aeirobot::DegToRad(30.0);

            const double c_to_c_distance = 0.04; // 축과 축 사이의 거리 

            // ===== (2) 고정 지오메트리  =====
            const Eigen::Vector3d b1(0.047,  +c_to_c_distance,  0.00);      // base anchor 1 (world)
            const Eigen::Vector3d b2(0.047,  -c_to_c_distance,  0.00);      // base anchor 2 (world)
            const Eigen::Vector3d c1_local(0.05,  +c_to_c_distance, -0.33); // platform anchor 1 (local)
            const Eigen::Vector3d c2_local(0.05,  -c_to_c_distance, -0.33); // platform anchor 2 (local)

            // 중립에서 플랫폼 원점의 월드 위치 
            const Eigen::Vector3d t_platform_world = Eigen::Vector3d::Zero();

            // 순수 롤 축과 축이 지나는 점
            const Eigen::Vector3d u_axis_world = (Ry(pitch_m) * Eigen::Vector3d::UnitX()).normalized();
            const Eigen::Vector3d p0 = Eigen::Vector3d::Zero();  // 축이 지나는 월드상의 점

            // ===== (3) pitch만 적용: Rp = Ry(pitch_m) =====
            const Eigen::Matrix3d Rp = Ry(pitch_m);

            // 중립에서 플랫폼 앵커의 월드 좌표(평행이동 포함)
            const Eigen::Vector3d c1_w0 = Rp * c1_local + t_platform_world;
            const Eigen::Vector3d c2_w0 = Rp * c2_local + t_platform_world;

            // ===== (4) 순수 롤 적용 (Rodrigues): p' = p0 + Rr * (p - p0) =====
            const Eigen::Matrix3d Rr = Rodrigues(u_axis_world, roll);
            const Eigen::Vector3d c1w = p0 + Rr * (c1_w0 - p0);
            const Eigen::Vector3d c2w = p0 + Rr * (c2_w0 - p0);

            // ===== (5) 각 액추에이터의 절대 길이 =====
            const double l1_abs = (c1w - b1).norm();
            const double l2_abs = (c2w - b2).norm();

            // ===== (6) 장착 기준 옵셋 보정(기존 0.3 m 차감) =====
            constexpr double L0 = 0.300;
            double l1 = l1_abs - L0;
            double l2 = l2_abs - L0;
            double l1_clamped = std::clamp(l1, 0.0, 0.07);
            double l2_clamped = std::clamp(l2, 0.0, 0.07);
            

            // ===== (7) 로깅 =====
            // RCLCPP_INFO(
            // rclcpp::get_logger("linear_rotary_converter_Pelvis"),
            // "pitch_in=%.6f rad, pitch_m=%.6f rad, roll=%.6f rad | "
            // "l1_abs=%.6f m, l2_abs=%.6f m | l1_Pelvis=%.6f, l2_Pelvis=%.6f (offset=%.3f)",
            // pitch, pitch_m, roll, l1_abs, l2_abs, l1, l2, L0);

            // ===== (8) 결과 반환 =====
            return {l1_clamped, l2_clamped};
        }
    }

    std::vector<double> VirtualJointMap::MapLinearPosAnkle(double pitch, double roll) const
    {
        if (alice4_version == 1)
        {
            //  ---------------------------
            //  (1) 점 B를 한 번에 계산
            //  ---------------------------
            const double r = 0.355;
            const double p_b_x =
                (r - 0.05) * std::sin(pitch) + 0.04 * std::cos(pitch);
            const double p_b_y =
                (r - 0.05) * std::cos(pitch) - 0.04 * std::sin(pitch);

            // ---------------------------
            // (2) BO, OC, BC 벡터
            // ---------------------------
            //  - B에서 O까지: bo = O - B
            const double bo_x = -p_b_x;
            const double bo_y = -p_b_y;

            //  - O에서 C까지: oc
            const double oc_x = 0.045;
            const double oc_y = 0.015;

            //  - B->C = (B->O) + (O->C) = bo + oc
            const double bc_x = bo_x + oc_x;
            const double bc_y = bo_y + oc_y;

            // ---------------------------
            // (3) pitch_len = ||BC||
            // ---------------------------
            const double pitch_len =
                std::sqrt(bc_x * bc_x + bc_y * bc_y);

            // (4) pitch_len2 = sqrt( pitch_len^2 + (0.048 - 0.027)^2 )
            const double diff = 0.048 - 0.027; // 0.021
            const double pitch_len2 =
                std::sqrt(pitch_len * pitch_len + diff * diff);

            // (5) rp_theta = acos(pitch_len / pitch_len2)
            const double rp_theta =
                std::acos(pitch_len / pitch_len2);

            // (6) roll_cal = |0.027 * sin(roll) / cos(rp_theta)|
            const double roll_cal =
                std::abs(0.027 * std::sin(roll) / std::cos(rp_theta));

            // ---------------------------
            // (7) 최종 선형 액추에이터 길이
            // ---------------------------
            //   - 원본 코드와 동일하게 0.2735를 빼줌
            if (roll >= 0.0)
            {
                return {
                    pitch_len2 - roll_cal - 0.2735,
                    pitch_len2 + roll_cal - 0.2735};
            }
            else
            {
                return {
                    pitch_len2 + roll_cal - 0.2735,
                    pitch_len2 - roll_cal - 0.2735};
            }
        }
        if (alice4_version == 2)
        {
            // ===== 함수 내부에 정의 =====
            auto Rx = [](double th) -> Eigen::Matrix3d {
            const double c = std::cos(th), s = std::sin(th);
            Eigen::Matrix3d R; R <<
                1, 0, 0,
                0, c,-s,
                0, s, c;
            return R;
            };
            auto Ry = [](double th) -> Eigen::Matrix3d {
            const double c = std::cos(th), s = std::sin(th);
            Eigen::Matrix3d R; R <<
                c, 0, s,
                0, 1, 0,
                -s, 0, c;
            return R;
            };
            // Rodrigues 회전 (축 u, 각도 th)
            auto Rodrigues = [](const Eigen::Vector3d& u_in, double th) -> Eigen::Matrix3d {
            Eigen::Vector3d u = u_in.normalized();
            const double c = std::cos(th), s = std::sin(th);
            Eigen::Matrix3d Ux;
            Ux <<     0, -u.z(),  u.y(),
                    u.z(),     0, -u.x(),
                    -u.y(),  u.x(),     0;
            return c * Eigen::Matrix3d::Identity() + (1.0 - c) * (u * u.transpose()) + s * Ux;
            };

            // ===== (1) pitch 보정 =====
            const double pitch_m = pitch;  
            
            const double c_to_c_distance = 0.04; // 축과 축 사이의 거리 

            // ===== (2) 고정 지오메트리 =====
            const Eigen::Vector3d c1_local(-0.044,  +c_to_c_distance,  0.016);
            const Eigen::Vector3d c2_local(-0.044,  -c_to_c_distance,  0.016);
            const Eigen::Vector3d b1(-0.028,  +c_to_c_distance, 0.332);
            const Eigen::Vector3d b2(-0.028,  -c_to_c_distance, 0.332);

            // 플랫폼 원점의 월드 위치(중립). 없으면 (0,0,0)
            const Eigen::Vector3d t_platform_world = Eigen::Vector3d::Zero();

            // 롤 축 정의: 월드 고정 x축, 그리고 그 축이 지나는 점 p0
            const Eigen::Vector3d u_axis_world = (Ry(pitch_m) * Eigen::Vector3d::UnitX()).normalized();
            const Eigen::Vector3d p0 = Eigen::Vector3d::Zero(); // 필요 시 z 몇 cm 조정해 캘리브

            // ===== (3) pitch만 적용: Rp = Ry(pitch_m) =====
            const Eigen::Matrix3d Rp = Ry(pitch_m);

            // 중립에서 플랫폼 앵커의 월드 좌표(평행이동 포함)
            const Eigen::Vector3d c1_w0 = Rp * c1_local + t_platform_world;
            const Eigen::Vector3d c2_w0 = Rp * c2_local + t_platform_world;

            // ===== (4) 순수 롤 적용: p' = p0 + Rr * (p - p0) =====
            const Eigen::Matrix3d Rr = Rodrigues(u_axis_world, roll);
            const Eigen::Vector3d c1w = p0 + Rr * (c1_w0 - p0);
            const Eigen::Vector3d c2w = p0 + Rr * (c2_w0 - p0);

            // ===== (5) 각 액추에이터의 절대 길이 =====
            const double l1_abs = (c1w - b1).norm();
            const double l2_abs = (c2w - b2).norm();

            // ===== (6) 장착 기준 옵셋 보정 =====
            constexpr double L0 = 0.300;
            const double l1 = l1_abs - L0;
            const double l2 = l2_abs - L0;
            const double l1_clamped = std::clamp(l1, 0.0, 0.07);
            const double l2_clamped = std::clamp(l2, 0.0, 0.07);

            // ===== (7) 로깅 =====
            // RCLCPP_INFO(
            // rclcpp::get_logger("linear_rotary_converter_Ankle"),
            // "pitch_in=%.6f rad, pitch_m=%.6f rad, roll=%.6f rad | "
            // "l1_abs=%.6f m, l2_abs=%.6f m | l1_Ankle=%.6f, l2_Ankle=%.6f (offset=%.3f)",
            // pitch, pitch_m, roll, l1_abs, l2_abs, l1, l2, L0);

            // ===== (8) 결과 반환 =====
            return {l1_clamped, l2_clamped};   
        }
    }

    std::vector<double> VirtualJointMap::MapLinearEffortPelvis(double pitch, double hip_pitch_torque, double hip_roll_torque)
    {
        if (alice4_version == 1)
        {
            // --- version 1 상수 (CalculateHipLinearMap 버전1 대응) ---
            const double offset_len = 0.003016;
            const double offset_theta = 0.99289439 + 0.06862061 + pitch;
            const double hip_roll_length = 0.055;
            const double ankle_cos_pitch_angle_offset = 0.93632918;

            // --- 피치 토크 → 전체 힘 F ---
            double hip_force = hip_pitch_torque / (offset_len * std::cos(offset_theta));

            // --- 롤 토크 분배 (F1, F2) ---
            double force1_vertical = (hip_force + hip_roll_torque / hip_roll_length) / 2.0;
            double force2_vertical = (hip_force - hip_roll_torque / hip_roll_length) / 2.0;

            // --- 세로 성분 보정 해제 (리니어 힘 반환) ---
            double linear_effort1 = force1_vertical / ankle_cos_pitch_angle_offset;
            double linear_effort2 = force2_vertical / ankle_cos_pitch_angle_offset;

            return {linear_effort1, linear_effort2};
        }
        else
        {
            // --- version 2 상수 (CalculateHipLinearMap 버전2 대응) ---
            // version2 forward 매핑에선 추가 cos 보정 없음
            const double hip_roll_length = 0.04;
            const double offset_len = 0.047;

            double theta =  pitch;

            // 상수 오프셋
            const double offset = 1.42951;

            // 오프셋이 포함된 각도
            double theta_offset = offset - theta;

            // 분자와 분모 계산
            double numerator   = 0.333766 * std::sin(theta_offset); //분자
            double denominator = -0.047 + 0.333766 * std::cos(theta_offset); //분모

            // 각도 차이 계산
            double angle_diff = std::atan2(numerator, denominator);
            if (angle_diff > 1.5708)
            {   angle_diff = 2 * 1.5708 - angle_diff; //둔각시 예각 처리   
            }    

            // --- 피치 토크 → 전체 힘 F ---
            double hip_force = hip_pitch_torque / (offset_len * std::cos(angle_diff));

            // --- 롤 토크 분배 (F1, F2) ---
            double force1_vertical = (hip_force - hip_roll_torque / hip_roll_length) / 2.0;
            double force2_vertical = (hip_force + hip_roll_torque / hip_roll_length) / 2.0;

            // --- version2에선 vertical 힘 그대로 리니어 이포트로 사용 ---
            double linear_effort1 = force1_vertical;
            double linear_effort2 = force2_vertical;

            return {linear_effort1, linear_effort2};
        }
    }

    std::vector<double> VirtualJointMap::MapLinearEffortAnkle(double pitch, double ankle_pitch_torque, double ankle_roll_torque)
    {
        if (alice4_version == 1)
        {
            // --- version 1 (CalculateAnkleLinearMap v1 대응) ---
            const double offset_len = 0.047434165;
            const double offset_theta_base = 1.89254685;
            const double ankle_roll_length = 0.027;
            const double cos_offset = 0.78935222;

            double offset_theta = offset_theta_base + pitch;

            // (1) 피치 토크 → 전체 힘 F
            double ankle_force = -ankle_pitch_torque / (offset_len * std::sin(offset_theta));

            // (2) 롤 토크 분배 (F1, F2)
            double force1_vertical = (ankle_force + ankle_roll_torque / ankle_roll_length) / 2.0;
            double force2_vertical = (ankle_force - ankle_roll_torque / ankle_roll_length) / 2.0;

            // (3) 세로 성분 보정 해제
            double linear_effort1 = force1_vertical / cos_offset;
            double linear_effort2 = force2_vertical / cos_offset;

            return {linear_effort1, linear_effort2};
        }
        else
        {
            // --- version 2 (CalculateAnkleLinearMap v2 대응) ---
            // version2 forward 매핑에선 추가 cos 보정 없음
            const double ankle_roll_length = 0.04; // 레버 암 길이
            const double offset_len = 0.0468188;   // 레버 암 길이

            double theta =  pitch;

            // 상수 오프셋
            const double offset = 0.34877;

            // 오프셋이 포함된 각도
            double theta_offset = theta + offset;

            // 분자와 분모 계산
            double numerator   = 0.0468 * std::sin(theta_offset) - 0.0332; //분자
            double denominator = 0.0468 * std::cos(theta_offset) - 0.028; //분모

            // 각도 차이 계산
            double angle_diff = std::atan2(numerator, denominator) - theta_offset;

            // (1) 피치 토크 → 전체 힘 F
            double ankle_force = ankle_pitch_torque / (offset_len * std::sin(angle_diff));

            // (2) 롤 토크 분배 (F1, F2)
            double force1_vertical = (ankle_force - ankle_roll_torque / ankle_roll_length) / 2.0;
            double force2_vertical = (ankle_force + ankle_roll_torque / ankle_roll_length) / 2.0;

            // (3) vertical 힘 그대로 리니어 이포트로 사용
            double linear_effort1 = force1_vertical;
            double linear_effort2 = force2_vertical;

            return {linear_effort1, linear_effort2};
        }
    }

    double VirtualJointMap::MapLinearEffortKnee(double pitch, double knee_torque)
    {
        if (alice4_version == 1)
        {
            // --- version 1 상수 (CalculateKneeLinearMap v1 대응) ---
            const double offset_len = 0.064031242;
            const double offset_theta_base = 0.89605536;

            double offset_theta = offset_theta_base + pitch;

            // 피치 토크 → 리니어 힘
            double linear_effort = -knee_torque / (offset_len * std::sin(offset_theta));

            return linear_effort;
        }
        else
        {
            // --- version 2 상수 (CalculateKneeLinearMap v2 대응) ---
            const double offset_len = 0.047201695;

            double theta =  pitch;

            // 상수 오프셋
            const double offset = 0.93577;

            // 오프셋이 포함된 각도
            double theta_offset = theta - offset;

            // 분자와 분모 계산
            double numerator   = 0.337 - 0.047202 * std::sin(theta_offset); //분자
            double denominator = 0.036 - 0.047202 * std::cos(theta_offset); //분모

            // 각도 차이 계산
            double angle_diff = std::atan2(numerator, denominator) - theta_offset;

            // 피치 토크 → 리니어 힘
            double linear_effort = -knee_torque / (offset_len * std::sin(angle_diff));

            return linear_effort;
        }
    }
}