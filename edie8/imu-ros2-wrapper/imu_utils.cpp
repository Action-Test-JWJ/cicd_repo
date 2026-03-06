#include "imu_utils.hpp"

ImuUtils::ImuUtils()
    : gyro_medians{0.0, 0.0, 0.0}, 
      accel_medians{0.0, 0.0, 0.0},
      ideal_gravity{0.0, 0.0, 1.0},
      rotation_matrix_sum{{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}},
      buffer_size(1000),
      one_gravity_force(1.0)
{

}

ImuUtils::~ImuUtils() {}

std::array<double, 3> ImuUtils::ApplyGyroOffsets(const std::array<double, 3>& gyro_data)
{
    if (rotation_matrix_sum[0][0] == 0.0f)
    {
        std::cout << "Rotation matrix is not initialized. Cannot apply offsets." << std::endl;
        return {}; 
    }

    std::array<double, 3> calibrated_data = gyro_data;
    std::array<double, 3> temp = {0.0f, 0.0f, 0.0f};

    // 회전 행렬 적용
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            temp[i] += rotation_matrix_sum[i][j] * gyro_data[j];     
        }
    }
    calibrated_data = temp;

    // Bias 제거: KF-ed + Rotation Matrix 데이터에서 gyro_medians 적용
    if (gyro_medians[0] != 0.0 && gyro_medians[1] != 0.0 && gyro_medians[2] != 0.0)
    {
        for (int i = 0; i < 3; i++)
        {
            calibrated_data[i] -= gyro_medians[i];
        }
    }
    return calibrated_data;
}

std::array<double, 3> ImuUtils::ApplyAccelOffsets(const std::array<double, 3>& accel_data)
{   
    if (rotation_matrix_sum[0][0] == 0.0f)
    {
        std::cout << "Rotation matrix is not initialized. Cannot apply offsets." << std::endl;
        return {}; 
    }

    std::array<double, 3> calibrated_data = accel_data;
    std::array<double, 3> temp = {0.0f, 0.0f, 0.0f};

    // 회전 행렬 적용
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            temp[i] += rotation_matrix_sum[i][j] * accel_data[j];     
        }
    }
    calibrated_data = temp;

    // Bias 제거: KF-ed + Rotation Matrix 데이터에서 accel_medians 적용
    if (accel_medians[0] != 0.0 && accel_medians[1] != 0.0 && accel_medians[2] != 0.0)
    {
        // std::cout << "accel_medians : " << accel_medians[0] << ", " << accel_medians[1] << ", " << accel_medians[2] << std::endl;
        // std::cout << "################################" << std::endl;
        for (int i = 0; i < 3; i++)
        {
            calibrated_data[i] -= accel_medians[i];
        }
    }

    return calibrated_data;
}

// 중앙값 계산을 위한 헬퍼 함수 추가
double ImuUtils::CalculateMedian(std::vector<double>& data) {
    if (data.empty()) return 0.0f;
    
    // 데이터 정렬
    std::sort(data.begin(), data.end());
    
    // 중앙값 계산
    size_t size = data.size();
    if (size % 2 == 0) {
        return (data[size/2 - 1] + data[size/2]) / 2.0f;
    } else {
        return data[size/2];
    }
}

bool ImuUtils::CalibrateGyro(const std::vector<std::array<double, 3>>& gyro_data)
{
    std::cout << "Calibrating gyroscope..." << std::endl;

    // 보정 불가(1): buffer_size 이상의 데이터가 없음
    if (int(gyro_data.size()) < buffer_size) {
        std::cerr << "Not enough data for calibration. Need at least 100 samples." << std::endl;
        return false;
    }
    // 보정 불가(2): 회전 행렬이 초기화되지 않음
    if (rotation_matrix_sum[0][0] == 0.0f)
    {
        std::cout << "Rotation matrix is not initialized. Cannot apply offsets." << std::endl;
        return false; 
    }

    // 회전 행렬을 적용하여 gyro_data 보정
    std::vector<std::array<double, 3>> calibrated_gyro_data;
    for (const auto& sample : gyro_data) {
        std::array<double, 3> calibrated_sample = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                calibrated_sample[i] += rotation_matrix_sum[i][j] * sample[j];
            }
        }
        calibrated_gyro_data.push_back(calibrated_sample);
    }

    // 보정된 데이터로 gyro_medians 계산 (중앙값 사용)
    for (int i = 0; i < 3; i++)
    {
        std::vector<double> axis_data;
        axis_data.reserve(calibrated_gyro_data.size());
        for (const auto& sample : calibrated_gyro_data) {
            axis_data.push_back(sample[i]);
        }
        gyro_medians[i] = CalculateMedian(axis_data);
    }

    std::cout << "Gyroscope medians: gx_median=" << gyro_medians[0]
              << ", gy_median=" << gyro_medians[1]
              << ", gz_median=" << gyro_medians[2] << std::endl;
              
    return true;
}

bool ImuUtils::CalibrateAccel(const std::vector<std::array<double, 3>>& accel_data) 
{
    std::cout << "Calibrating accelerometer..." << std::endl;

    if (int(accel_data.size()) < buffer_size) {
        std::cerr << "Not enough data for calibration. Need at least 100 samples." << std::endl;
        return false;
    }

    // 각 샘플에 대해 회전 행렬 계산
    for (const auto& sample : accel_data)
    {
        std::array<std::array<double, 3>, 3> rotation_matrix = CalculateRotationMatrix(sample);
        // 회전 행렬을 누적하여 합산
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation_matrix_sum[i][j] += rotation_matrix[i][j];
            }
        }
    }
    // 회전 행렬 평균화
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            rotation_matrix_sum[i][j] /= buffer_size;
        }
    }

    // 회전 행렬을 적용하여 accel_data 보정
    std::vector<std::array<double, 3>> calibrated_accel_data;
    for (const auto& sample : accel_data) 
    {
        std::array<double, 3> adjusted_sample = sample; 
        
        std::array<double, 3> calibrated_sample = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                calibrated_sample[i] += rotation_matrix_sum[i][j] * adjusted_sample[j];
            }
        }
        calibrated_accel_data.push_back(calibrated_sample);
    }

    // 보정된 데이터로 accel_medians 계산 (중앙값 사용)
    for (int i = 0; i < 3; i++)
    {
        std::vector<double> axis_data;
        axis_data.reserve(calibrated_accel_data.size());
        for (const auto& sample : calibrated_accel_data) 
        {
            axis_data.push_back(sample[i]);
        }
        accel_medians[i] = CalculateMedian(axis_data);

        if (i==2){
            accel_medians[i] -= one_gravity_force;
        }
    }
    
    // 결과 출력
    std::cout << "Final Rotation Matrix:" << std::endl;
    for (const auto& row : rotation_matrix_sum) 
    {
        for (double element : row) 
        {
            std::cout << std::fixed << std::setprecision(4) << element << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Accelerometer Medians: ax_median=" << accel_medians[0]
              << ", ay_median=" << accel_medians[1]
              << ", az_median=" << accel_medians[2] << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    return true;
}

std::array<std::array<double, 3>, 3> ImuUtils::CalculateRotationMatrix(const std::array<double, 3>& accel_data)
{
    // 목표: g_real 방향을 g_ideal(= +Z)로 보내는 최소 회전 R
    std::array<double, 3> a = accel_data;
    std::array<double, 3> b = ideal_gravity; // 보통 {0,0,1} 가정

    // 1) normalize a, b
    const double na = ComputeVectorNorm3(a);
    const double nb = ComputeVectorNorm3(b);
    if (na < 1e-12 || nb < 1e-12) 
    {
        return ComputeIdentityMatrix3x3(); // 데이터 이상이면 identity
    }
    for(int i = 0; i < 3; i++)
    { 
        a[i] /= na; 
        b[i] /= nb; 
    }

    // 2) sin, cos
    std::array<double,3> v = ComputeCrossProduct3(a, b);
    const double sin_theta = ComputeVectorNorm3(v); // = sin(theta) (a,b가 단위벡터일 때)
    double cos_theta = ComputeDotProduct3(a, b);    // = cos(theta)
    if (cos_theta >  1.0) cos_theta =  1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    // 3) [예외처리] 평행/반평행 처리
    if (sin_theta < 1e-8) // theta =  0  or pi 일 때
    {
        if (cos_theta > 0.0) // theta =  0 일 때
        {
            // 이미 정렬됨
            return ComputeIdentityMatrix3x3();
        }
        else // theta =  pi 일 때
        {
            // 180도 회전: 축은 a와 직교인 축(x, y) 중 하나 선택
            // 첫 번째 시도(tmp=x축)
            std::array<double,3> tmp{1,0,0};
            std::array<double,3> axis = ComputeCrossProduct3(a, tmp);
            // 두 번째 시도(tmp=y축)
            if (ComputeVectorNorm3(axis) < 1e-6) 
            {
                tmp = {0,1,0};
                axis = ComputeCrossProduct3(a, tmp);
            }
            const double n_axis = ComputeVectorNorm3(axis);
            for(int i=0;i<3;i++) axis[i] /= n_axis;

            // Rodrigues with theta = pi
            const double sin_t = 0.0;
            const double cos_t = -1.0;

            std::array<std::array<double,3>,3> K {{
                {{0, -axis[2], axis[1]}},
                {{axis[2], 0, -axis[0]}},
                {{-axis[1], axis[0], 0}}
            }};
            auto K2 = MultiplyMatrix3x3(K, K);
            auto I = ComputeIdentityMatrix3x3();

            std::array<std::array<double,3>,3> R = I;
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    R[i][j] += sin_t * K[i][j] + (1.0 - cos_t) * K2[i][j];
                }
            }
            return R;
        }
    }

    // 4) 일반 케이스
    std::array<double,3> axis{ v[0]/sin_theta, v[1]/sin_theta, v[2]/sin_theta };
    const double sin_t = sin_theta;   // 단위벡터 기준
    const double cos_t = cos_theta;

    std::array<std::array<double,3>,3> K {{
        {{0, -axis[2], axis[1]}},
        {{axis[2], 0, -axis[0]}},
        {{-axis[1], axis[0], 0}}
    }};
    auto K2 = MultiplyMatrix3x3(K, K);
    auto I = ComputeIdentityMatrix3x3();

    std::array<std::array<double,3>,3> R = I;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            R[i][j] += sin_t * K[i][j] + (1.0 - cos_t) * K2[i][j];
        }
    }
    return R;
}

void ImuUtils::SaveOffsetsToYaml(const std::string& file_path)
{
    // YAML Node 생성
    YAML::Node config;

    // Accelerometer medians 저장
    config["ax_median"] = this->accel_medians[0];
    config["ay_median"] = this->accel_medians[1];
    config["az_median"] = this->accel_medians[2];

    // Gyroscope medians 저장
    config["gx_median"] = this->gyro_medians[0];
    config["gy_median"] = this->gyro_medians[1];
    config["gz_median"] = this->gyro_medians[2];

    // Rotation matrix 저장
    YAML::Node rotation_matrix_node;
    rotation_matrix_node["rows"] = 3;
    rotation_matrix_node["cols"] = 3;
    
    // rotation_matrix_data를 한 줄로 저장
    std::vector<double> rotation_matrix_data;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // 대각선 성분이 1인 경우 1.0으로 저장
            if (i == j && double(rotation_matrix_sum[i][j]) == 1.0f) {
                rotation_matrix_data.push_back(1.0f);
            } else {
                rotation_matrix_data.push_back(static_cast<double>(rotation_matrix_sum[i][j]));
            }
        }
    }
    rotation_matrix_node["data"] = rotation_matrix_data;
    config["rotation_matrix"] = rotation_matrix_node;

    // 파일로 저장
    try
    {
        std::ofstream fout(file_path);
        fout << config;
        fout.close();
        std::cout << "Offsets saved to " << file_path << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to save offsets to file: " << e.what() << std::endl;
    }
}

void ImuUtils::LoadOffsetsFromYaml(const std::string& file_path)
{
    try
    {
        // YAML 파일 읽기
        YAML::Node config = YAML::LoadFile(file_path);

        // Accelerometer medians 읽어오기
        accel_medians[0] = config["ax_median"].as<double>();
        accel_medians[1] = config["ay_median"].as<double>();
        accel_medians[2] = config["az_median"].as<double>();

        // Gyroscope medians 읽어오기
        gyro_medians[0] = config["gx_median"].as<double>();
        gyro_medians[1] = config["gy_median"].as<double>();
        gyro_medians[2] = config["gz_median"].as<double>();

        // Rotation matrix 읽어오기
        if (config["rotation_matrix"]) {
            YAML::Node rotation_matrix_node = config["rotation_matrix"];
            int index = 0;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    rotation_matrix_sum[i][j] = rotation_matrix_node["data"][index++].as<double>();
                }
            }
        } else {
            std::cerr << "Rotation matrix not found in YAML file." << std::endl;
        }

        std::cout << "Loaded offsets from " << file_path << ":" << std::endl;
        std::cout << "Accelerometer medians: "
                  << "ax=" << accel_medians[0] << ", "
                  << "ay=" << accel_medians[1] << ", "
                  << "az=" << accel_medians[2] << std::endl;
        std::cout << "Gyroscope medians: "
                  << "gx=" << gyro_medians[0] << ", "
                  << "gy=" << gyro_medians[1] << ", "
                  << "gz=" << gyro_medians[2] << std::endl;
        std::cout << "Rotation matrix:" << std::endl;
        for (const auto& row : rotation_matrix_sum) {
            std::cout << "[ ";
            for (double element : row) {
                std::cout << std::fixed << std::setprecision(4) << element << " ";
            }
            std::cout << "]" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to load offsets from file: " << e.what() << std::endl;
    }
}

double ImuUtils::ComputeVectorNorm3(const std::array<double,3>& v) 
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
  
double ImuUtils::ComputeDotProduct3(const std::array<double,3>& a, const std::array<double,3>& b) 
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

std::array<double,3> ImuUtils::ComputeCrossProduct3(const std::array<double,3>& a, const std::array<double,3>& b) 
{
    return { a[1]*b[2] - a[2]*b[1],
             a[2]*b[0] - a[0]*b[2],
             a[0]*b[1] - a[1]*b[0] };
}   

std::array<std::array<double,3>,3> ImuUtils::ComputeIdentityMatrix3x3() 
{
    return {{{{1.0f,0.0f,0.0f}}, {{0.0f,1.0f,0.0f}}, {{0.0f,0.0f,1.0f}}}};
}

std::array<std::array<double,3>,3> ImuUtils::MultiplyMatrix3x3(const std::array<std::array<double,3>,3>& A,
                                                                const std::array<std::array<double,3>,3>& B)
{
    std::array<std::array<double,3>,3> C = {{
        {{0.0f,0.0f,0.0f}},
        {{0.0f,0.0f,0.0f}},
        {{0.0f,0.0f,0.0f}}
    }};

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            for(int k=0;k<3;k++)
            {
                C[i][j] += A[i][k]*B[k][j];
            }
        }
    }

    return C;
}

std::vector<std::pair<double, double>> ImuUtils::ComputeAllanVariance(const std::vector<double>& data, double dt) const
{
    const size_t N = data.size();
    if (N < 2) return {};

    // 1) 가능한 tau 값들: dt, 2*dt, 4*dt, ... (2^k * dt <= N/2*dt)
    std::vector<double> taus;
    for (size_t m = 1; m * 2 < N; m *= 2) {
        taus.push_back(m * dt);
    }

    std::vector<std::pair<double,double>> result;
    result.reserve(taus.size());

    for (double tau : taus) {
        size_t M = static_cast<size_t>(std::floor(N * dt / tau));  // 블록 개수
        if (M < 2) continue;

        size_t blockSize = static_cast<size_t>(std::floor(tau / dt));
        // 2) 블록 평균들 계산
        std::vector<double> blockAvg;
        blockAvg.reserve(M);
        for (size_t i = 0; i + blockSize <= N; i += blockSize) {
            double sum = 0;
            for (size_t j = 0; j < blockSize; ++j) {
                sum += data[i + j];
            }
            blockAvg.push_back(sum / static_cast<double>(blockSize));
        }

        size_t K = blockAvg.size();
        if (K < 2) continue;

        // 3) Allan 분산 계산
        double sigma2 = 0;
        for (size_t i = 0; i + 1 < K; ++i) {
            double diff = blockAvg[i+1] - blockAvg[i];
            sigma2 += diff * diff;
        }
        sigma2 /= (2.0 * (K - 1));

        result.emplace_back(tau, sigma2);
    }

    return result;
}