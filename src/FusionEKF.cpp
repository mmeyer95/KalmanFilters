#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "kalman_filter.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //Initialize H_laser_
  H_laser_ << 1,0,0,0,
              0,1,0,0;

  //Initialize Jacobian
  Hj_ << 0,0,0,0,
         0,0,0,0,
         0,0,0,0; 
  
  //initialize F matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
      

  //initialize Q
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 0,0,0,0,
        0,0,0,0,
        0,0,0,0,
        0,0,0,0;
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    
    //initialize covariance matrix
  	ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    
    // first measurement
    cout << "EKF: " << endl;    
    ekf_.x_ = VectorXd(4);

    
    //fill in estimate with current measurements if first
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_(0); //range 
      double phi = measurement_pack.raw_measurements_(1); //bearing 
      double rho_dot = measurement_pack.raw_measurements_(2);// range rate 
      double x = rho * cos(phi); 
      if (x < 0.001) { x = 0.001; } 
      double y = rho * sin(phi); 
      if (y < 0.001) { y = 0.001; } 
      double vx = rho_dot * cos(phi); 
      double vy = rho_dot * sin(phi); 
      ekf_.x_ << x, y, vx, vy;
      previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //Initialize state using measurement values
      ekf_.x_<< measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0,0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  ///////////////UPDATE F//////////////////////////////
  //change in time is new time minus old
  cout << "Time: " << measurement_pack.timestamp_ << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  //the new time is now old, since we have delta t
  previous_timestamp_ = measurement_pack.timestamp_;  
  //new F matrix values based on new delta T
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  //////////////UPDATE Q////////////////////////
 //calculate values for updating Q noise matrix
  if (dt<0.1){dt=0.1;}
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  int noise_ax = 9;
  int noise_ay = 9;
  //Update Q 
  ekf_.Q_(0,2) = dt_3/2*noise_ax;
  ekf_.Q_(0,0) = dt_4/4*noise_ax;
  ekf_.Q_(1,1) = dt_4/4*noise_ay;
  ekf_.Q_(1,3) = dt_3/2*noise_ay;
  ekf_.Q_(2,0) = dt_3/2*noise_ax; 
  ekf_.Q_(2,2) = dt_2*noise_ax;
  ekf_.Q_(3,1) = dt_3/2*noise_ay;
  ekf_.Q_(3,3) = dt_2*noise_ay;
  

  //Predict the next position
  ekf_.Predict();

  /**
   * Update
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //R matrix for radar
    ekf_.R_ = R_radar_;
    
    //H matrix should be the Jacobian
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;

    //Update
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    //R matrix for laser
    ekf_.R_ = R_laser_;
   
    //H matrix for laser
    ekf_.H_ = H_laser_;
   
    //update using the new measurement for laser
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
