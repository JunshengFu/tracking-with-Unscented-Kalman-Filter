#include <iostream>
#include "ukf.h"

#define DEBUG 0 // toggle debug

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


// constant value 2*pi
const double _2_PI = 2.*M_PI;

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
  std_a_ = 0.8;  // need to be optimized based on NIS

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.55;  // need to be optimized based on NIS

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
  n_aug_sigma_ = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // the current NIS for radar
  NIS_radar_ = 0.;

  // the current NIS for laser
  NIS_laser_ = 0.;

  // Weights of sigma points
  weights_ = VectorXd(n_aug_sigma_);
  double t = lambda_ + n_aug_;
  weights_(0) = lambda_ / t;
  weights_.tail(n_aug_sigma_-1).fill(0.5/t);

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate]
  x_ = VectorXd(n_x_);
  x_.fill(0);

  // state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_aug_sigma_);

  // agumented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, n_aug_sigma_);

}


UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * Initialization
   */
  if (!is_initialized_){
    Initialization(meas_package);
    return;
  }


  /**
   * Prediction
   */
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;  // convert unit from us to s.
  time_us_ = meas_package.timestamp_;

#if DEBUG
  std::cout << "delta_t = " << delta_t << std::endl;
  std::cout<< "before prediction" << std::endl;
  std::cout << "x= \n" << x_ << std::endl;
  std::cout << "P= \n" << P_ << std::endl;
#endif

  // when we apply Euler method for large steps, we are assuming that the derivative of the states is constant over
  // that whole interval, this leads to error in state propagation, so we divide the one big step to several small ones.
  // *See detail discussion* @ https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449/55
  while (delta_t > 0.1){
    const double dt = 0.05;
    Prediction(dt);
    delta_t -= dt;
  }
  Prediction(delta_t);


#if DEBUG
  std::cout<< "after prediction" << std::endl;
  std::cout << "x= \n" << x_ << std::endl;
  std::cout << "P= \n" << P_ << std::endl;
#endif

  /**
   * Update
   */
  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);
  }


}

/**
 * Initialize the state, covariance matrix and timestamp using the first measurement
 * @param meas_package The first measurement
 */
void UKF::Initialization(MeasurementPackage meas_package){

  /**
   * Initialize the state
   */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    double rho = meas_package.raw_measurements_[0];      // range: radial distance from origin
    double phi = meas_package.raw_measurements_[1];      // bearing: angle between rho and x axis
    //double rho_dot = meas_package.raw_measurements_[2];  // radial velocity: change of rho
    double px = cos(phi) * rho;
    double py = sin(phi) * rho;

    x_ << px, py, 0, 0, 0;
  }
  else if  (meas_package.sensor_type_ == MeasurementPackage::LASER){
    x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
  }
  else{
    return;  // if no valid measurement arrived, then wait for the next measurement
  }


  /**
   * Initialize the timestamp
   */
  time_us_ = meas_package.timestamp_;


  /**
   * Initialize the covariance matrix
   */
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // set Initialize flag to true
  is_initialized_ = true;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  *  Estimate the object's location. Modify the state
  *  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // calculate augmented sigma points: Xsig_agu_
  AugmentSigmaPoints();

#if DEBUG
  std::cout<< "agumented sigma points:\n" << Xsig_aug_ << std::endl;
#endif

  // predict the sigma points: Xsig_pred_
  PredictSigmaPoints(delta_t);

#if DEBUG
  std::cout<< "predicted sigma points:\n" << Xsig_pred_<< std::endl;
#endif

  // calculate the predicted mean and covariance: x_ and P_
  PredictMeanAndCovariance();

}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief about the object's
   * position. Modify the state vector, x_, and covariance, P_.
   * You'll also need to calculate the lidar NIS.
   */
  //set measurement dimension, lidar can measure px, py
  int n_z = 2;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_aug_sigma_);
  //predicted measurement mean: z_pred
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);


  //transform sigma points into radar measurement space
  for (int i = 0; i < n_aug_sigma_; i++) {

    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;                        //px
    Zsig(1,i) = p_y;                        //py
  }

  // measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_ * std_laspx_,  0,
       0,                        std_laspy_ * std_laspy_;

  // Predict radar measurement with given Sigma predictions
  PredictMeasurement(n_z, Zsig, z_pred, S, R);

  // Update the state
  VectorXd z = meas_package.raw_measurements_;
  UpdateState(z, z_pred, S, Zsig);

  // Calculate NIS
  NIS_laser_ = (z - z_pred).transpose() * S.inverse() * (z - z_pred);

}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief about the object's
   * position. Modify the state vector, x_, and covariance, P_.
   * You'll also need to calculate the radar NIS.
   */

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_aug_sigma_);
  //predicted measurement mean: z_pred
  VectorXd z_pred = VectorXd(n_z);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into radar measurement space
  for (int i = 0; i < n_aug_sigma_; i++) {

    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_,  0,                        0,
       0,                    std_radphi_*std_radphi_,  0,
       0,                    0,                        std_radrd_*std_radrd_;

  // Predict radar measurement with given Sigma predictions
  PredictMeasurement(n_z, Zsig, z_pred, S, R);

  // Update the state
  VectorXd z = meas_package.raw_measurements_;
  UpdateState(z, z_pred, S, Zsig);


  // Calculate NIS
  NIS_radar_ = (z - z_pred).transpose() * S.inverse() * (z - z_pred);

}

/**
 * Predict either lidar or radar measurement with given Sigma predictions
 * @param n_z The measurement dimension
 * @param Zsig The matrix for sigma points in measurement space
 * @param z_pred The predicted measurement mean
 * @param S The measurement covariance matrix
 * @param R The measurement noise covariance matrix
 */
void UKF::PredictMeasurement(int n_z, const MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, MatrixXd &R){

  //predicted measurement mean: z_pred
  z_pred.fill(0.0);
  for (int i=0; i < n_aug_sigma_; i++) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < n_aug_sigma_; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=_2_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=_2_PI;

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R;
}

/**
 * Updates the state with either lidar or radar measurement
 * @param z The measurement at k+1
 * @param z_pred The predictionof measurement at k+1
 * @param S The measurement covariance matrix
 * @param Zsig The matrix for sigma points in measurement space
 */
void UKF::UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, const MatrixXd &Zsig) {

  //The measurement dimension
  int n_z = z_pred.rows();  // check me!

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_aug_sigma_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=_2_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=_2_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=_2_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=_2_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*_2_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*_2_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}


/**
 * Calculate augmented sigma points: Xsig_agu_
 */
void UKF::AugmentSigmaPoints(){

  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.fill(0.0);
  x_aug_.head(n_x_) = x_;

#if DEBUG
  std::cout<<"x_aug_:\n"<<x_aug_<<std::endl;
#endif

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner( n_x_, n_x_ ) = P_;
  P_aug_(n_aug_-2, n_aug_-2) = std_a_ * std_a_;
  P_aug_(n_aug_-1, n_aug_-1) = std_yawdd_ * std_yawdd_;

#if DEBUG
  std::cout<< "agumented state covariance:\n" << P_aug_ << std::endl;
#endif

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i=0; i<n_aug_; i++)
  {
    Xsig_aug_.col(i+1)      = x_aug_ + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L.col(i);
  }


}


/**
 * Predict the sigma points: Xsig_pred_
 */
void UKF::PredictSigmaPoints(double delta_t){

  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}


/**
 * Predict the mean and covariance of the predicted state: x_ and P_
 */
void UKF::PredictMeanAndCovariance(){

  // weights are already initialed in Constructor: line 68

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_aug_sigma_; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_aug_sigma_; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalized to [-pi, pi]
    while (x_diff(3)> M_PI) x_diff(3)-=_2_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=_2_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
}