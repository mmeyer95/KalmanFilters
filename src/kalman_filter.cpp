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
  x_=F_*x_+u;
  MatrixXd F_t = F_.transpose();
  P_=F_*P_*F_t;
}

void KalmanFilter::Update(const VectorXd &z) {
  //new state
  	x_=x_in+K*y;
	P_=(I-K*H_)*P_in;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
   //Update
    VectorXd y = z - H_ * x_;
    MatrixXd H_t = H_.transpose();
    MatrixXd S_ = H_ * P_ * H_t + R_;
    MatrixXd S_i = S_.inverse();
    MatrixXd K =  P_ * H_t * S_i;
}
