#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  //Transpose of F matrix
  MatrixXd Ft = F_.transpose();
  //Predict state and 
  x_=F_*x_;
  cout << "P: "<< P_ << endl;
  P_=F_*P_*Ft+Q_;
  cout << "F: " << F_ << endl;
  cout << "P: "<< P_ << endl;
  cout << "Q_: " << Q_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht + R_;
  MatrixXd Si = S_.inverse();
  MatrixXd K =  P_ * Ht * Si;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //new state
  x_=x_+K*y;
  P_=(I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //calculate h(x') individual terms, normalize angle value
  float h1 = sqrt((x_(0)*x_(0)+x_(1)*x_(1)));
  float h2 = atan2(x_(1),x_(0));
  float h3 = (x_(0)*x_(2)+x_(1)*x_(3))/h1;
  VectorXd h_funct(3); 
  h_funct << h1,h2,h3;

  
  //calculate z using h(x')
  VectorXd y = z- h_funct;
  
  //normalize angle
  y(1) = atan2(sin(y(1)), cos(y(1)));
  //Using Jacobian for H matrix, update KF  
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht + R_;
  MatrixXd S_i = S_.inverse();
  MatrixXd K =  P_ * Ht * S_i;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  //new state
  x_=x_+K*y;
  P_=(I-K*H_)*P_;
}
