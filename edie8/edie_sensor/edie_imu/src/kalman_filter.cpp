#include "edie_imu/kalman_filter.hpp"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Initialize(int state_dim, int measurement_dim, double Q_cov, double R_cov) {
    state_dim_ = state_dim;
    measurement_dim_ = measurement_dim;
    state_ = Eigen::VectorXd::Zero(state_dim);
    // P_: Predicted Covariance Matrix. Sigma(x(n, pred)).
    P_ = Eigen::MatrixXd::Identity(state_dim, state_dim); // 가우시안 분포의 분산 값 가짐
    // Q_: Process Noise Covariance
    Q_ = Eigen::MatrixXd::Identity(state_dim, state_dim) * Q_cov; 
    // R_: Measurement Noise Covariance
    R_ = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * R_cov; 
}

void KalmanFilter::StatePrediction(const Eigen::MatrixXd& F) {
    // state_: Predicted State(각도, 속도)
    state_ = F * state_;

    P_ = F * P_ * F.transpose() + Q_;
}

// z: measurement(각속도, 가속도)
void KalmanFilter::Update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H) {
    // y = y(n,meas) - Cx(n,pred)
    Eigen::VectorXd y = z - H * state_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // left state_: Optimal State(t = n)
    // right state: Predicted State(t = n)
    state_ = state_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;
}

Eigen::VectorXd KalmanFilter::GetState() const {
    return state_;
}
