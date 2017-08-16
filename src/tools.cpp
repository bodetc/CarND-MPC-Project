//
// Created by Cédric Bodet on 14.08.17.
//

#include "tools.h"

using CppAD::AD;
using namespace std;

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial.
double deriveval(const Eigen::VectorXd& coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i-1);
  }
  return result;
}


AD<double> polyeval(const Eigen::VectorXd& coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

AD<double> deriveval(const Eigen::VectorXd& coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * CppAD::pow(x, i-1);
  }
  return result;
}

Eigen::MatrixXd coordinatesTransform(const vector<double> &ptsx, const vector<double> &ptsy, double x0, double y0,
                                     double theta) {
  const long N = ptsx.size();
  const double cosT = cos(theta);
  const double sinT = sin(theta);

  Eigen::MatrixXd A(N, 2);

  for(int i=0; i<N; i++){
    const double x=ptsx[i]-x0;
    const double y=ptsy[i]-y0;

    const double xp= x*cosT+y*sinT;
    const double yp=-x*sinT+y*cosT;

    A(i, 0) = xp;
    A(i, 1) = yp;
  }

  return A;
}