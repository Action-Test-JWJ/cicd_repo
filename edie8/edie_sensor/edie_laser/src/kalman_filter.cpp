#include "edie_laser/kalman_filter.hpp"

KalmanFilter::KalmanFilter(double R_value, double Q_value, double Pt_prev_value, double Xt_init)
    : raw_data(0), k_f_data(0),
      Xt(Xt_init), Xt_update(0), Xt_prev(Xt_init),
      Pt(Pt_prev_value), Pt_update(0), Pt_prev(Pt_prev_value),
      Kt(0), R(R_value), Q(Q_value)
{
}

void KalmanFilter::Update(double noisy_raw_data)
{
    raw_data = noisy_raw_data;

    // 1. 예측 단계 (Prediction Step)
    Xt_update = Xt_prev;
    Pt_update = Pt_prev + Q;

    // 2. 칼만 이득 계산
    Kt = Pt_update / (Pt_update + R);

    // 3. 업데이트 단계 (Update)
    Xt = Xt_update + (Kt * (raw_data - Xt_update));
    Pt = (1 - Kt) * Pt_update;

    // 4. 다음 반복을 위해 저장
    Xt_prev = Xt;
    Pt_prev = Pt;
    
    k_f_data = Xt;
}
