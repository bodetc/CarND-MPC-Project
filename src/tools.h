//
// Created by Cédric Bodet on 14.08.17.
//

#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H

#include <cppad/cppad.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

double polyeval(const Eigen::VectorXd& coeffs, double x);

double deriveval(const Eigen::VectorXd& coeffs, double x);

CppAD::AD<double> polyeval(const Eigen::VectorXd& coeffs, CppAD::AD<double> x);

CppAD::AD<double> deriveval(const Eigen::VectorXd& coeffs, CppAD::AD<double> x);

Eigen::MatrixXd coordinatesTransform(const std::vector<double> &ptsx, const std::vector<double> &ptsy, double x0, double y0,
                                     double theta);

#endif //MPC_TOOLS_H
