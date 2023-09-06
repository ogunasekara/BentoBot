#include <Eigen/Dense>
#include <cmath>
#include <bentobot_odom/KalmanFilter.h>

namespace Bentobot
{
KalmanFilter::KalmanFilter(
    Eigen::Matrix<double, 5, 5> process_noise_cov, 
    Eigen::Matrix<double, 2, 2> odom_noise_cov,
    Eigen::Matrix<double, 1, 1> imu_noise_cov): 
    process_noise_cov{process_noise_cov},
    odom_noise_cov{odom_noise_cov},
    imu_noise_cov{imu_noise_cov}
{}

Eigen::Matrix<double, 5, 5> KalmanFilter::linearizeStateTransitionMatrix(double dt) {
    double th = state(2);
    double v = state(3);
    double w = state(4);
    
    Eigen::Matrix<double, 5, 5> A;
    A.fill(0);

    A(0, 0) = 1;
    A(0, 2) = (dt / 6.0) * (-1 * v * sin(th) - 4 * v * sin(th + ((dt / 2.0) * w)) - v * sin(th + (dt * w)));
    A(0, 3) = (dt / 6.0) * (cos(th) + 4 * cos(th + ((dt / 2.0) * 2)) + cos(th + (dt * w)));
    A(0, 4) = (dt / 6.0) * (-2 * v * dt * sin(th + ((dt / 2.0) * w)) - v * dt * sin(th + (dt * w)));

    A(1, 1) = 1;
    A(1, 2) = (dt / 6.0) * (v * cos(th) + 4 * v * cos(th + ((dt / 2.0) * w)) + v * cos(th + (dt * w)));
    A(1, 3) = (dt / 6.0) * (sin(th) + 4 * sin(th + ((dt / 2.0) * 2)) + sin(th + (dt * w)));
    A(1, 4) = (dt / 6.0) * (2 * v * dt * cos(th + ((dt / 2.0) * w)) + v * dt * cos(th + (dt * w)));

    A(2, 2) = 1;
    A(2, 4) = dt;
    
    A(3, 3) = 1;

    A(4, 4) = 1;

    return A;
}

void KalmanFilter::stateTransitionUpdate(double dt) {
    double x = state(0);
    double y = state(1);
    double th = state(2);
    double v = state(3);
    double w = state(4);

    Eigen::Matrix<double, 5, 5> G = linearizeStateTransitionMatrix(dt);

    // state transition function derived from runge-kutta approximation
    double new_x = x + (dt / 6.0) * (v * cos(th) + 4 * v * cos(th + ((dt / 2.0) * w)) + v * cos(th + (dt * w)));
    double new_y = y + (dt / 6.0) * (v * sin(th) + 4 * v * sin(th + ((dt / 2.0) * w)) + v * sin(th + (dt * w)));
    double new_th = th + dt * w;

    state(0) = new_x;
    state(1) = new_y;
    state(2) = new_th;

    covariance = G * covariance * G.transpose() + process_noise_cov;
}

void KalmanFilter::measurementOdomUpdate(double v, double w) {
    Eigen::Matrix<double, 2, 2> S = C_odom * covariance * C_odom.transpose() + odom_noise_cov;
    Eigen::Matrix<double, 5, 2> K = covariance * C_odom.transpose() * S.inverse();
    Eigen::Matrix<double, 5, 5> I;
    I.setIdentity();

    Eigen::Matrix<double, 2, 1> z;
    z(0) = v;
    z(1) = w;

    state = state + K * (z - C_odom * state);
    covariance = (I - K * C_odom) * covariance;
}

void KalmanFilter::measurementIMUUpdate(double w) {
    Eigen::Matrix<double, 1, 1> S = C_imu * covariance * C_imu.transpose() + imu_noise_cov;
    Eigen::Matrix<double, 5, 1> K = covariance * C_imu.transpose() * S.inverse();
    Eigen::Matrix<double, 5, 5> I;
    I.setIdentity();

    Eigen::Matrix<double, 1, 1> z;
    z(0) = 1;

    state = state + K * (z - C_imu * state);
    covariance = (I - K * C_imu) * covariance;
}

Eigen::Matrix<double, 5, 1> KalmanFilter::getState() {
    return state;
}
}