#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  //Predict
  x_=F_*x_+Q_;
  MatrixXd Ft = F_.transpose();
  P_=F_*P_*Ft;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht + R_;
  MatrixXd S_i = S_.inverse();
  MatrixXd K =  P_ * Ht * S_i;
  //new state
  x_=x_in+K*y;
  P_=(I-K*H_)*P_in;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
   //Update
  float p_x=VectorXd[0]
  float p_y = VectorXd[1]
  float v_x = VectorXd[2]
  float v_y = VectorXd[3]
   
  Vector h_x = [pow((pow(p_x,2)+pow(p_y,2)),0.5);atan(p_y/p_x);(p_x*v_x+p_y*v_y)/(pow((pow(p_x,2)+pow(p_y,2)),0.5);atan(p_y/p_x)]
  y=z- h_x
}
