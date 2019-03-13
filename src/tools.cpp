#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  //initialize root mean square error
  VectorXd RMSE(4);
  RMSE << 0,0,0,0;
  
    for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i]-ground_truth[i];
    residual = residual.array()*residual.array();
    RMSE += residual;
    }
  //calculate the mean
  RMSE = RMSE/estimations.size();
  //calculate the squared root
  RMSE = RMSE.array().sqrt();

  //return
  return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  //Create floats of the matrix values
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  //pre-calculate terms
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  
  MatrixXd Hj_ (3,4);
  
  if (c2 < 0.01) {
    return Hj_;}

  //Calculate Jacobian
    Hj_ << (px/c2), (py/c2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, (px/c2), (py/c2);

  
  //return
  //cout << "Jacobian: " << Hj_ << endl;
  return Hj_;
}
