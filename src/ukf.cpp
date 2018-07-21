#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 0.5; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  // state dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;
  
  d_sig = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  // Create weights vector
  weights_ = VectorXd(d_sig);
  
  weights_(0) = lambda_/( lambda_ + n_aug_ );
  for( int i=1; i<2*n_aug_+1; i++ )weights_(i) = 0.5/( n_aug_ + lambda_ );

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, d_sig);
  
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

  // Initialization
  if (!is_initialized_) {
	  
    cout << "Initializing unscented Kalman filter" << endl;
	  
    // initial measurement
    x_ << 0.0,0.0,0.0,0.0,0.0;
    
    // initial covariance matrix
    P_.fill(0.);
    P_(0,0) = 0.15;
    P_(1,1) = 0.15;
    P_(2,2) = 1.; 
    P_(3,3) = 1.;
    P_(4,4) = 0.1;
	
     // initial timestamp
	time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		
		x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0.0, 0.0, 0.0;
		
        //check small value
		if (fabs(x_(0)) < 0.001 && fabs(x_(1)) < 0.001) {
		x_(0) = 0.001;
		x_(1) = 0.001;
	}}
	
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		
		/**
		//Convert polar to cartesian.
		*/

		float rho,phi,rhodot;

		rho = meas_package.raw_measurements_(0); // Radial distance from origin
		phi = meas_package.raw_measurements_(1);   // Angle between p and x
		rhodot = meas_package.raw_measurements_(2); // // chnge of p(range rate)
	    x_ << rho * cos(phi), rho * sin(phi), rhodot, 0.0, 0.0;
    }

    is_initialized_ = true;
  }
  
  else {
    if ((!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
        ||
        (!use_laser_ &&
          meas_package.sensor_type_ == MeasurementPackage::LASER)) {
      return;
    }

	// Compute the time from the previous measurement in seconds.
	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;
	
	Prediction(dt);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar update
		UpdateRadar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		// Laser update
		UpdateLidar(meas_package);
	}
}}

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


 /*******************************************************************************
 *    create Augment Sigma Points
 ******************************************************************************/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, d_sig);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

 //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  

/*******************************************************************************
 * Prediction
 ******************************************************************************/
   // Variable declaration
   double p_x, p_y, v, yaw, yawd, nu_a, nu_yawdd, delta_t2, yawd_dt;
   
  //predict sigma points
  for (int i = 0; i < d_sig; i++) {
	 
    p_x 	= Xsig_aug(0,i);
    p_y 	= Xsig_aug(1,i);
    v 		= Xsig_aug(2,i);
    yaw 	= Xsig_aug(3,i);
    yawd 	= Xsig_aug(4,i);
    nu_a 	= Xsig_aug(5,i);
    nu_yawdd = Xsig_aug(6,i);
    delta_t2 = delta_t* delta_t;
    yawd_dt = yawd * delta_t;

    //check for small valueand avoid division by zero
    if (fabs(yawd) > 0.001) {
        Xsig_pred_(0,i) = p_x + v/yawd * (sin(yaw + yawd_dt) - sin(yaw)) + (0.5 * delta_t2 * cos(yawd) * nu_a);
        Xsig_pred_(1,i) = p_y + v/yawd * (-cos(yaw + yawd_dt) + cos(yaw)) + (0.5 * delta_t2 * sin(yawd) * nu_a);
    }
    else {
        Xsig_pred_(0,i) = p_x + v*delta_t*cos(yaw) + (0.5 * delta_t2 * cos(yawd) * nu_a);
        Xsig_pred_(1,i) = p_y + v*delta_t*sin(yaw) + (0.5 * delta_t2 * sin(yawd) * nu_a);
    }

    Xsig_pred_(2,i) = v + nu_a * delta_t;
    Xsig_pred_(3,i) = yaw + yawd_dt + 0.5 * nu_yawdd * delta_t2;
    Xsig_pred_(4,i) = yawd + nu_yawdd * delta_t;
  }
  
/*******************************************************************************
 * Covariance mean predict
 ******************************************************************************/

  // State mean predict
  x_.fill(0.);
  for( int i = 0; i < d_sig; i++ ) {x_ = x_ + weights_(i)*Xsig_pred_.col(i);}

  //covariance matrix predict
  MatrixXd Xdiff = Xsig_pred_.array().colwise() - x_.array();

  //normalize angle
  for (int i=0; i < d_sig; i++) { Xdiff(3,i) = AngleNorm(Xdiff(3,i));}

  we_diff = Xdiff.array().rowwise() * weights_.transpose().array();
  P_ = we_diff * Xdiff.transpose();

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

/*******************************************************************************
 * Predicted Lidar Measurement
 ******************************************************************************/
  // Variable declaration
  int n_z = 2;
  
  //Generate sigma point matrix
  MatrixXd Zsig = MatrixXd(n_z, d_sig);

  //Mean prediction 
  VectorXd z_pred = VectorXd(n_z);
  VectorXd z = VectorXd(n_z);

  //Covariance Matrix X
  MatrixXd S = MatrixXd(n_z,n_z);

  //measurement model
  for (int i = 0; i < d_sig; ++i) {
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
  }

  //Mean prediction measurement
  MatrixXd zTot = Zsig.array().rowwise() * weights_.transpose().array();
  z_pred = zTot.rowwise().sum();

  //Covariance matrix measurement
  MatrixXd Zdiff = Zsig.array().colwise() - z_pred.array();

  //Normalize angle
  for (int i=0; i < d_sig; i++) {Zdiff(1,i) = AngleNorm(Zdiff(1,i));}
  
  MatrixXd we_z = Zdiff.array().rowwise() * weights_.transpose().array();
  S = we_z * Zdiff.transpose();

  //Incorporate noise measurement
  S(0,0) += std_laspx_ * std_laspx_;
  S(1,1) += std_laspy_ * std_laspy_;

/*******************************************************************************
 * UKF Lidar Update
 ******************************************************************************/
  //calculate cross correlation matrix
  MatrixXd Tc = we_diff * Zdiff.transpose();

  z = meas_package.raw_measurements_;
  
  //residual
  VectorXd z_diff = z - z_pred;

  //Normalize angle
  z_diff(1) = AngleNorm(z_diff(1));
  
  //calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
    
  //Kalman Gain K;
  MatrixXd K = Tc * S.inverse();

  //Checking P value
  if (NIS_laser_ > 100.0) { is_initialized_ = false;}
  else {     //Mean and covariance update
    x_ += K*z_diff;
    P_ -= K*S*K.transpose();
  }
}

/**
* Angle Normalization
*/

double UKF::AngleNorm(double angle) {
	const float T_PI = 2 * M_PI;   // Initialize 2π to Normalizing Angles

	// Normalize high angle measurement
	if (fabs(angle) > 100.0 * M_PI) {angle -= floor(angle / T_PI) * (T_PI);}

	while((angle > M_PI) || (angle < -M_PI)) {
	  if (angle> M_PI){angle -= T_PI;}
	  else {angle += T_PI;}
	}

  return angle;
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

  /*******************************************************************************
 * Predicted Radar Measurement
 ******************************************************************************/
 
  // variable declaration
 
  int n_z;
  double px,py,v,y;
  
  n_z = 3;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, d_sig);

  //Predict mean measurement
  VectorXd z_pred = VectorXd(n_z);
  VectorXd z = VectorXd(n_z);

  //covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //Sigma point measurement
  for (int i = 0; i < d_sig; ++i) {
      px = Xsig_pred_(0,i);
      py = Xsig_pred_(1,i);
      v = Xsig_pred_(2,i);
      y = Xsig_pred_(3,i);
	  
      // measurement model
      Zsig(0,i) = sqrt((px * px) + (py * py));
      Zsig(1,i) = atan2(py, px);
      Zsig(2,i) = (((px * cos(y)) + (py * sin(y))) * v) / Zsig(0,i);
  }

  //Mean measurement
  MatrixXd zTot = Zsig.array().rowwise() * weights_.transpose().array();
  z_pred = zTot.rowwise().sum();

  //Covariance matrix measurement
  MatrixXd Zdiff = Zsig.array().colwise() - z_pred.array();

  //Normalized angle
  for (int i=0; i < d_sig; i++) {
    Zdiff(1,i) = AngleNorm(Zdiff(1,i));
  }

  MatrixXd we_z = Zdiff.array().rowwise() * weights_.transpose().array();

  S = we_z * Zdiff.transpose();

  //add measurement with noise matrix
  S(0,0) += std_radr_ * std_radr_;
  S(1,1) += std_radphi_ * std_radphi_;
  S(2,2) += std_radrd_ * std_radrd_;


/*******************************************************************************
 * UKF Radar Update
 ******************************************************************************/

  MatrixXd Tc = we_diff * Zdiff.transpose();

  //Kalman K gain;
  MatrixXd K = Tc * S.inverse();

  z = meas_package.raw_measurements_;
  
  //residual
  VectorXd z_diff = z - z_pred;

  //Normalized angle
  z_diff(1) = AngleNorm(z_diff(1));

  //calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  // Checking P value
  if (NIS_radar_ > 100.0) { is_initialized_ = false;}
  else {
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
  }
}

