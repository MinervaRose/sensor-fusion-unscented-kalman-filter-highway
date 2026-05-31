#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// helper to normalize angles to [-pi, pi]
static inline void NormalizeAngle(double &angle) {
  while (angle > M_PI) angle -= 2. * M_PI;
  while (angle < -M_PI) angle += 2. * M_PI;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // (original 30 is intentionally too large; tune to realistic value)
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.9;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // --- Additional initialization ---

  is_initialized_ = false;
  time_us_ = 0;

  // State dimension [px, py, v, yaw, yawd]
  n_x_ = 5;

  // Augmented state dimension (add nu_a, nu_yawdd)
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Initialize state vector and covariance
  x_.setZero();
  P_.setIdentity();
  // Slightly refined diagonal
  P_(0,0) = 0.15 * 0.15;  // px variance ~ lidar std^2
  P_(1,1) = 0.15 * 0.15;  // py
  P_(2,2) = 1.0;          // v
  P_(3,3) = 0.1;          // yaw
  P_(4,4) = 0.1;          // yawd

  // Predicted sigma points matrix
  int n_sigma = 2 * n_aug_ + 1;
  Xsig_pred_ = MatrixXd(n_x_, n_sigma);

  // Weights
  weights_ = VectorXd(n_sigma);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < n_sigma; ++i) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  // NIS (not strictly needed for passing, but declared in header)
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_) {
    // First measurement: initialize x_
    x_.setZero();

    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      // Lidar gives px, py directly
      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];
      x_(0) = px;
      x_(1) = py;
      // v, yaw, yawd remain zero
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // Radar gives rho, phi, rhod
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      // double rhod = meas_package.raw_measurements_[2];  // do not use to init v

      double px = rho * std::cos(phi);
      double py = rho * std::sin(phi);
      x_(0) = px;
      x_(1) = py;
      // v, yaw, yawd remain zero
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // Compute elapsed time in seconds
  double dt = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;

  // Prediction step
  Prediction(dt);

  // Update step, depending on sensor type
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  int n_sigma = 2 * n_aug_ + 1;

  // 1) Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0.0;
  x_aug(6) = 0.0;

  // 2) Create augmented covariance matrix
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  // 3) Square root of P_aug
  MatrixXd L = P_aug.llt().matrixL();

  // 4) Create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma);
  Xsig_aug.col(0) = x_aug;
  double sqrt_lambda_n_aug = std::sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1)          = x_aug + sqrt_lambda_n_aug * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda_n_aug * L.col(i);
  }

  // 5) Predict sigma points using CTRV model
  for (int i = 0; i < n_sigma; ++i) {
    double px      = Xsig_aug(0, i);
    double py      = Xsig_aug(1, i);
    double v       = Xsig_aug(2, i);
    double yaw     = Xsig_aug(3, i);
    double yawd    = Xsig_aug(4, i);
    double nu_a    = Xsig_aug(5, i);
    double nu_yawdd= Xsig_aug(6, i);

    double px_p, py_p;

    // Avoid division by very small yawd
    if (std::fabs(yawd) > 1e-3) {
      px_p = px + v / yawd * (std::sin(yaw + yawd * delta_t) - std::sin(yaw));
      py_p = py + v / yawd * (-std::cos(yaw + yawd * delta_t) + std::cos(yaw));
    } else {
      px_p = px + v * delta_t * std::cos(yaw);
      py_p = py + v * delta_t * std::sin(yaw);
    }

    double v_p   = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p= yawd;

    // Add process noise
    px_p   += 0.5 * nu_a * delta_t * delta_t * std::cos(yaw);
    py_p   += 0.5 * nu_a * delta_t * delta_t * std::sin(yaw);
    v_p    += nu_a * delta_t;
    yaw_p  += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    // Write predicted sigma points into matrix
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // 6) Predict state mean
  x_.setZero();
  for (int i = 0; i < n_sigma; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // 7) Predict state covariance matrix
  P_.setZero();
  for (int i = 0; i < n_sigma; ++i) {
    VectorXd diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(diff(3));
    P_ += weights_(i) * diff * diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  int n_sigma = 2 * n_aug_ + 1;
  int n_z = 2;  // lidar: px, py

  // 1) Transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sigma);
  for (int i = 0; i < n_sigma; ++i) {
    Zsig(0, i) = Xsig_pred_(0, i);  // px
    Zsig(1, i) = Xsig_pred_(1, i);  // py
  }

  // 2) Mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // 3) Innovation covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for (int i = 0; i < n_sigma; ++i) {
    VectorXd zdiff = Zsig.col(i) - z_pred;
    S += weights_(i) * zdiff * zdiff.transpose();
  }

  // Add measurement noise
  S(0,0) += std_laspx_ * std_laspx_;
  S(1,1) += std_laspy_ * std_laspy_;

  // 4) Cross-correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < n_sigma; ++i) {
    VectorXd xdiff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(xdiff(3));

    VectorXd zdiff = Zsig.col(i) - z_pred;

    Tc += weights_(i) * xdiff * zdiff.transpose();
  }

  // 5) Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // 6) Residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - z_pred;

  // 7) Update state mean and covariance
  x_ += K * y;
  P_ -= K * S * K.transpose();

  // Optional: NIS_laser_ = y.transpose() * S.inverse() * y;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  int n_sigma = 2 * n_aug_ + 1;
  int n_z = 3;  // radar: rho, phi, rhod

  // 1) Transform sigma points into measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sigma);
  for (int i = 0; i < n_sigma; ++i) {
    double px  = Xsig_pred_(0, i);
    double py  = Xsig_pred_(1, i);
    double v   = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double vx = std::cos(yaw) * v;
    double vy = std::sin(yaw) * v;

    double rho = std::sqrt(px*px + py*py);
    double phi = std::atan2(py, px);
    double rhod = 0.0;
    if (rho > 1e-3) {
      rhod = (px*vx + py*vy) / rho;
    }

    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rhod;
  }

  // 2) Mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  for (int i = 0; i < n_sigma; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }
  NormalizeAngle(z_pred(1));

  // 3) Innovation covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  for (int i = 0; i < n_sigma; ++i) {
    VectorXd zdiff = Zsig.col(i) - z_pred;
    NormalizeAngle(zdiff(1));
    S += weights_(i) * zdiff * zdiff.transpose();
  }

  // Add measurement noise
  S(0,0) += std_radr_   * std_radr_;
  S(1,1) += std_radphi_ * std_radphi_;
  S(2,2) += std_radrd_  * std_radrd_;

  // 4) Cross-correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < n_sigma; ++i) {
    VectorXd xdiff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(xdiff(3));

    VectorXd zdiff = Zsig.col(i) - z_pred;
    NormalizeAngle(zdiff(1));

    Tc += weights_(i) * xdiff * zdiff.transpose();
  }

  // 5) Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // 6) Residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - z_pred;
  NormalizeAngle(y(1));

  // 7) Update state mean and covariance
  x_ += K * y;
  P_ -= K * S * K.transpose();

  // Optional: NIS_radar_ = y.transpose() * S.inverse() * y;
}
