#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  //initialize root mean square error
	VectorXd rmse(4);
  	rmse << 0,0,0,0;
  
    for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i]-ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;

  //calculate the mean
  rmse = rmse/estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  //return
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
//Create floats for Jacobian calculation
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
//Calculate Jacobian
  Hj_ << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  //return
  return Hj_
}
