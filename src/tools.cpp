#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Initialize return
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check that input vectors are ok
  if(estimations.size() == 0 | estimations.size() != ground_truth.size()) {
    cout<<"Input for RMSE function is not good!";
    return rmse;
  }

  // Initialize residuals
  VectorXd residuals(4);

  for (int i=0; i<estimations.size(); i++) {

    residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();

    rmse += residuals;
  }

  // Calculate mean of rmse vector
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}