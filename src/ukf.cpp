#include "ukf.h"
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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Initialize sigma points
  Xsig_pred_ = MatrixXd(5, 11);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.4;

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

  // State space dimension
  n_x_ = 5;

  // Augmented space dimension
  n_aug_ = 7;

  // Measurement spread
  lambda_ = 3 - n_aug_;

  // Initialize weight vector
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i=1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights(i) = weight;
  }

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
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

  // Initialize state
  if (!is_initialized_) {
    UKF();

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "Laser Measurement coming in \n";
      VectorXd z_(3);
      z_ = meas_package.raw_measurements_;
      x_ << z_(0), z_(1), 3, 0, 0;
      P_ <<
              1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1000, 0, 0,
              0, 0, 0, 1000, 0,
              0, 0, 0, 0, 1000;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << "Radar Measurement coming in \n";
      VectorXd z_(3);
      z_ = meas_package.raw_measurements_;
      x_ << z_(0) * cos(z_(1)), z_(0) * sin(z_(1)), z_(2), 0, 0;
      P_ <<
              1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1000, 0, 0,
              0, 0, 0, 1000, 0,
              0, 0, 0, 0, 1000;
    }

    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;

    std::cout << "Initilizaed mean vector \n" << x_ << std::endl;
    std::cout << "Initialized covariance matrix \n" << P_ << std::endl;
  }

  // Calcuate time difference
  double dt = meas_package.timestamp_ - time_us_;
  std::cout << dt;


  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //std::cout << "Radar Measurement coming in \n";
    VectorXd z_(3);
    z_ = meas_package.raw_measurements_;
    //std::cout << z_ << std::endl << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    //std::cout << "Laser Measurement coming in \n";
    VectorXd z_(3);
    z_ = meas_package.raw_measurements_;
    //std::cout << z_ << std::endl << std::endl;
  }

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
