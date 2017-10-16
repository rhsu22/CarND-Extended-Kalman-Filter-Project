#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for (int i = 0; i < estimations.size(); i++) {
    auto diff = estimations[i] - ground_truth[i];
    VectorXd diffSq = diff.array() * diff.array();
    rmse += diffSq;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  const float eps = std::numeric_limits<float>::epsilon();
  float d0 = std::max(px * px + py * py, eps);
  float d1 = sqrt(d0);
  Hj << px/d1, py/d1, 0.f, 0.f,
        -py/d0, px/d0, 0.f, 0.f,
        py * (vx * py - vy * px) / (d0 * d1), px * (vy * px - vx * py) / (d0 * d1), px/d1, py/d1;

  return Hj;
}
