#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd sum(4);
  sum << 0,0,0,0;

  // check the validity of inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size())
  {
      return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd diff = estimations[i] - ground_truth[i];
    VectorXd squareddiff = diff.array() * diff.array();
    sum += squareddiff;
  }

  //mean calculation
  sum = sum / estimations.size();
  //square root calculation
  rmse << sum.array().sqrt();
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);


  //check division by zero
  float epsilon = 0.0001;
  float c1 = epsilon;

  if (fabs(px) < epsilon && fabs(py) < epsilon) {
    c1 = epsilon;
  }
  else
  {
    c1 = px*px + py*py;
  }

  float c2 = sqrt(c1);
  float c3 = c1*c2;

  //compute the Jacobian matrix
  Hj << px/c2, py/c2, 0, 0,
       -py/c1, px/c1, 0, 0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;

  return Hj;
}
