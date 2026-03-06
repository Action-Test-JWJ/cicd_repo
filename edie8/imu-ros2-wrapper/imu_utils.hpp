#ifndef IMU_CALIBRATION_HPP
#define IMU_CALIBRATION_HPP

#include <array>
#include <cmath>    // std::floor, std::pow
#include <vector>
#include <utility>  // std::pair
#include <iostream>
#include <sstream>
#include <numeric>
#include <thread>
#include <chrono>
#include <fstream> 
#include <algorithm> 
#include <yaml-cpp/yaml.h>
#include <iomanip>

#include "aeirobot_toolbox/basic_tools.hpp"
#include "aeirobot_toolbox/vector3.hpp" 

class ImuUtils
{
public:
    ImuUtils();
    ~ImuUtils();

    /**
    * @brief 가속도/각속도의 각 축별 오프셋(median) 계산하는 함수
    * @param data 가속도/각속도의 각 축별 데이터 담은 버퍼
    * @return 오프셋(median)
    */
    double CalculateMedian(std::vector<double>& data);  

    /**
    * @brief 가속도 측정값(중력 방향)을 중력 벡터([0,0,1]) 방향으로 정렬해 주는 3×3 회전 행렬(R)을 계산하는 함수
    * 처음 노드 ON 했을 때 사용.
    * @param accel_data 크기가 N(=buffer_size)이상의 버퍼에 담긴 가속도 데이터({ax, ay, az})
    * @return 회전 행렬
    */
    std::array<std::array<double, 3>, 3> CalculateRotationMatrix(const std::array<double, 3>& accel_data);

    /**
    * @brief 초기 정지 상태에서 가속도 데이터를 수집해 가속도 오프셋을 계산하는 함수
    * 처음 노드 ON 했을 때 사용.
    * @param accel_data N(=buffer_size)개 이상의 가속도 데이터({ax, ay, az})들 담은 버퍼 
    * @return 업데이트 성공 여부
    */
    bool CalibrateAccel(const std::vector<std::array<double, 3>>& accel_data); 

    /**
    * @brief 초기 정지 상태에서 각속도 데이터를 수집해 각속도 오프셋을 계산하는 함수
    * 처음 노드 ON 했을 때 사용.
    * @param gyro_data N(=buffer_size)개 이상의 각속도 데이터({gx, gy, gz})들 담은 버퍼 
    * @return 업데이트 성공 여부
    */
    bool CalibrateGyro(const std::vector<std::array<double, 3>>& gyro_data); 

    /**
    * @brief 회전행렬를 적용하여 기울기 맞춘 후, accel 오프셋(median)을 빼서 bias 제거하는 함수
    * @param accel_data raw 가속도 데이터({ax, ay, az})
    * @return 중력 벡터 방향으로 기울기 맞춰진 & bias 제거된 가속도 데이터
    */
    std::array<double, 3> ApplyAccelOffsets(const std::array<double, 3>& accel_data);

    /**
    * @brief 회전행렬를 적용하여 기울기 맞춘 후, gyro 오프셋(median)을 빼서 bias 제거하는 함수
    * @param gyro_data raw 각속도 데이터({gx, gy, gz})
    * @return 중력 벡터 방향으로 기울기 맞춰진 & bias 제거된 각속도 데이터
    */
    std::array<double, 3> ApplyGyroOffsets(const std::array<double, 3>& gyro_data);

    /**
    * @brief YAML 파일에 가속도/각속도 데이터 오프셋 저장하는 함수
    * @param file_path 저장할 YAML 파일의 경로
    */
    void SaveOffsetsToYaml(const std::string& file_path);

    /**
    * @brief YAML 파일로부터 가속도/각속도 데이터 오프셋 로드하는 함수
    * @param file_path 로드할 YAML 파일의 경로
    */
    void LoadOffsetsFromYaml(const std::string& file_path);

    /**
    * @brief         Compute Allan Variance for a 1-axis 정적 IMU 데이터 시퀀스
    * 현재 사용 X
    * @param data    정지 상태에서 수집된 가속도 또는 자이로 샘플 (m/s^2 또는 rad/s)
    * @param dt      샘플 간격 (초)
    * @return        pair<tau, allan_variance> 벡터
    */
    std::vector<std::pair<double, double>> ComputeAllanVariance(const std::vector<double>& data, double dt) const;

private:
    std::array<double, 3> gyro_medians;
    std::array<double, 3> accel_medians;
    std::array<double, 3> ideal_gravity;
    std::array<std::array<double, 3>, 3> rotation_matrix;
    std::array<std::array<double, 3>, 3> rotation_matrix_sum = {}; 

    int buffer_size;
    double one_gravity_force;

    static double ComputeVectorNorm3(const std::array<double,3>& v);
    static double ComputeDotProduct3(const std::array<double,3>& a, 
                                     const std::array<double,3>& b);
    static std::array<double,3> ComputeCrossProduct3(const std::array<double,3>& a, 
                                                     const std::array<double,3>& b);
    static std::array<std::array<double,3>,3> ComputeIdentityMatrix3x3();
    static std::array<std::array<double,3>,3> MultiplyMatrix3x3(const std::array<std::array<double,3>,3>& A,
                                                                const std::array<std::array<double,3>,3>& B);
};

#endif // IMU_CALIBRATION_HPP

