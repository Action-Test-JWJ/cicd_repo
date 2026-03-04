#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <cmath>
#include <array>
#include <vector>
#include <numeric>
#include "aeirobot_toolbox/vector3.hpp" 
#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter();
    ~KalmanFilter();

    void Initialize(int state_dim, int measurement_dim, double Q_cov, double R_cov);
    void StatePrediction(const Eigen::MatrixXd& F);
    void Update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H);

    Eigen::VectorXd GetState() const;

private:
    int state_dim_;
    int measurement_dim_;
    
    Eigen::VectorXd state_;  // State vector
    Eigen::MatrixXd P_;      // Covariance matrix
    Eigen::MatrixXd Q_;      // Process noise covariance
    Eigen::MatrixXd R_;      // Measurement noise covariance
};

#endif // KALMAN_FILTER_HPP