#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except for init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except for init)
  use_radar_ = true;

  // time when the state is true, in us
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;  // need to be optimized with NIS

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.65;  // need to be optimized with NIS

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;  // fixed value given by the manufacturer

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;  // fixed value given by the manufacturer

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;  // fixed value given by the manufacturer

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;  // fixed value given by the manufacturer

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;  // fixed value given by the manufacturer

  // State dimension: [pos1 pos2 vel_abs yaw_angle yaw_rate]
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Number of sigma points
  n_aug_sigma = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // the current NIS for radar
  NIS_radar_ = 0.;

  // the current NIS for laser
  NIS_laser_ = 0.;

  // Weights of sigma points
  weights_ = VectorXd(n_aug_sigma);
  double t = lambda_ + n_aug_;
  weights_(0) = lambda_ / t;
  weights_.tail(n_aug_sigma-1).fill(0.5/t);

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  x_ = VectorXd(n_x_);
  x_.fill(0);

  // state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_aug_sigma);

}


UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_){
    Initialization(meas_package);
    return;
  }
}

/**
 * Initialize the state, covariance matrix and timestamp using a radar measurement
 * @param meas_package The first measurement
 */
void UKF::Initialization(MeasurementPackage meas_package){

  // Initialize the state
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    double rho = meas_package.raw_measurements_[0];      // range: radial distance from origin
    double phi = meas_package.raw_measurements_[1];      // bearing: angle between rho and x axis
//    double rho_dot = meas_package.raw_measurements_[2];  // radial velocity: change of rho
    double px = cos(phi) * rho;
    double py = sin(phi) * rho;

    x_ << px, py, 0, 0, 0;
  }
  else if  (meas_package.sensor_type_ == MeasurementPackage::LASER){
    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
  }


  // Initialize the timestamp
  time_us_ = meas_package.timestamp_;


  // Initialize the covariance
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  is_initialized_ = true;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
