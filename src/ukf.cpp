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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
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
  // State dimension
  n_x_ = 5; 

  // Augmented State Dimension
  n_aug_=7;  

  // Lambda Spreading Factor
  lambda_ = 3-n_aug_;
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
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // Initialize previous timestap variable to zero 
    float previous_timestamp_ = 0;
    // first measurement
    cout << "UKF: " << endl;
    // fill state matrix with zeros
    x_ << 0.0,0.0,0.0,0.0,0.0;

    // Fill Covariance Matrix with rough starting point
	  P_ <<     0.1, 0.1, 0.1, 0.1, 0.1,
		          0.1, 0.1, 0.1, 0.1, 0.1,
              0.1, 0.1, 0.1, 0.1, 0.1,
			        0.1, 0.1, 0.1, 0.1, 0.1,
			        0.1, 0.1, 0.1, 0.1, 0.1; 

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      VectorXd meas = meas_package.raw_measurements_;
      float rho_=meas(0);
      float phi_=meas(1);
      x_(0) = rho_*cos(phi_);
      x_(1) = rho_*sin(phi_);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      VectorXd meas = meas_package.raw_measurements_;
      x_(0) = meas(0);
      x_(1) = meas(1);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Get new the delta t
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_==true ){
    float delta_t_ = (meas_package.timestamp_-previous_timestamp_)/1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    Prediction(delta_t_);
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_radar_==false){
    float delta_t_ = (meas_package.timestamp_-previous_timestamp_)/1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    Prediction(delta_t_);
    UpdateLidar(meas_package);
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

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5)=0;
  x_aug(6)=0;
 
 //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_ , n_x_) = P_;
  P_aug(5,5)=std_a_*std_a_;
  P_aug(6,6)=std_yawdd_*std_yawdd_;
  
  //create square root matrix
  MatrixXd A =  P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0)=x_aug;
  
  for(int i=0;i<n_aug_;i++){
      Xsig_aug.col(i+1)          =x_aug   + sqrt(lambda_+n_aug_)*A.col(i);
      Xsig_aug.col(i+1+n_aug_)    =x_aug - sqrt(lambda_+n_aug_)*A.col(i);
  }


float px;
float py;
float v;
float phi;
float phid;
float nu_a;
float nu_phidd;

VectorXd x_n = VectorXd(n_x_);


for(int i = 0; i<(2*n_aug_+1);i++){
    px=Xsig_aug(0,i);
    py=Xsig_aug(1,i);
    v=Xsig_aug(2,i);
    phi=Xsig_aug(3,i);
    phid=Xsig_aug(4,i);
    nu_a=Xsig_aug(5,i);
    nu_phidd=Xsig_aug(6,i);
    
    x_n(0)=0.5*delta_t*delta_t*cos(phi)*nu_a;
    x_n(1)=0.5*delta_t*delta_t*sin(phi)*nu_a;
    x_n(2)=delta_t*nu_a;
    x_n(3)=0.5*delta_t*delta_t*nu_phidd;
    x_n(4)=delta_t*nu_phidd;
    
    if(fabs(Xsig_aug(0,4))>0.0001){
        Xsig_pred_(0,i)=px+x_n(0)+((v/phid)*(sin(phi+phid*delta_t)-sin(phi)));
        Xsig_pred_(1,i)=py+x_n(1)+((v/phid)*(-cos(phi+phid*delta_t)+cos(phi)));
        Xsig_pred_(2,i)=v+x_n(2);
        Xsig_pred_(3,i)=phi+x_n(3)+(phid*delta_t);
        Xsig_pred_(4,i)=phid+x_n(4);
    }
    else{
        Xsig_pred_(0,i)=px+x_n(0)+(v*cos(phi)*delta_t);
        Xsig_pred_(1,i)=py+x_n(1)+(v*sin(phi)*delta_t);
        Xsig_pred_(2,i)=v+x_n(2);
        Xsig_pred_(3,i)=phi+x_n(3)+(phid*delta_t);
        Xsig_pred_(4,i)=phid+x_n(4);
    }
  }      

  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);

  //set weights
  weights_.fill(1/(2*(lambda_+n_aug_)));
  weights_(0)=lambda_/(lambda_+n_aug_);
  
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }


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

  MatrixXd Zsig = MatrixXd(3,(2*n_aug_+1));
  for(int i=0; i<(2*n_aug_+1);i++){
    double px=Xsig_pred_(0,i);
    double py=Xsig_pred_(1,i);
    double v=Xsig_pred_(2,i);
    double phi=Xsig_pred_(3,i);
    double vx=cos(phi)*v;
    double vy=sin(phi)*v;
    
    Zsig(0,i) = sqrt(px*px + py*py);                        //r
    Zsig(1,i) = atan2(py,px);                                 //phi
    Zsig(2,i) = (px*vx + py*vy ) / sqrt(px*px + py*py);   //r_dot 
  }

  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
}
