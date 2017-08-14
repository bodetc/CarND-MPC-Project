//
// Created by Cédric Bodet on 14.08.17.
//

#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

double polyeval(Eigen::VectorXd coeffs, double x);

#endif //MPC_TOOLS_H
