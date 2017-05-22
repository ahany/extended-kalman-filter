#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  VectorXd x_polar(3);
  float c1 = x_(0)*x_(0) + x_(1)*x_(1);
  float c2 = sqrt(c1);
  float c3 = x_(0)*x_(2) + x_(1)*x_(3);

  x_polar(0) = c2;
  if(c2 != 0)
  {
    x_polar(2) = c3/c2;
  }
  else
  {
    x_polar(2) = 0;
  }

  x_polar(1) = atan2(x_(1), x_(0));

  if(fabs(x_(0))< 0.0001 && fabs(x_(1)<0.0001))
  {
    x_polar(1) = 0;
  }

  VectorXd y = z - x_polar;

  // phi normalization
  if (y(1) > M_PI) {
     double temp = fmod((y(1) - M_PI), (2 * M_PI));
     y(1) = temp - M_PI;
  }
  if (y(1) < -M_PI) {
     double temp = fmod((y(1) + M_PI) ,(2 * M_PI));
     y(1) = temp + M_PI;
  }

  MatrixXd Ht = H_.transpose();

  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
