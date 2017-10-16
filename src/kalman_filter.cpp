#include "kalman_filter.h"
#include <limits>
#include <math.h>

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateCommon(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ -= K * H_ * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
	UpdateCommon(y);
}

float normalizeAngle(double phi) {
  while (phi < -M_PI) {
    phi += 2 * M_PI;
  }
  while (phi > M_PI) {
    phi -= 2 * M_PI;
  }
  return phi;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  VectorXd xp(3);
  const float eps = std::numeric_limits<float>::epsilon();
  float rho = sqrt(px * px + py * py);
  float phi = (px == 0.f && py==0.f) ? 0 : atan2(py, px);
  float rho_dot = (px * vx + py * vy) / std::max(rho, eps);
  xp << rho,
        phi,
        rho_dot;

  VectorXd y = z - xp;
  // Modify phi range
  y(1) = normalizeAngle(y(1));

	UpdateCommon(y);
}
