#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <iostream>

class KalmanFilter
{
public:
    KalmanFilter(double R, double Q, double Pt_prev, double Xt_init);
    void Update(double noisy_raw_data);
    double raw_data, k_f_data;       // 입력 데이터 및 필터링된 결과

private:
    double Xt, Xt_update, Xt_prev;  // 상태 변수
    double Pt, Pt_update, Pt_prev;  // 공분산 변수
    double Kt;                      // 칼만 이득
    double R, Q;                    // 노이즈 공분산
};

#endif // KALMAN_FILTER_HPP
