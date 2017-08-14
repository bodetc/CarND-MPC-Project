//
// Created by Cédric Bodet on 14.08.17.
//

#ifndef MPC_COMMON_H
#define MPC_COMMON_H

// Maximum steering angle, given in radiant
#define MAX_STEERING_RAD 0.436332

// Set the timestep length and duration
const size_t N = 20;
// Duration of the timestep, chosen to be in the same order of magnitude as the latency
const double dt = 0.1;

// Order of the polynomial for fitting the waypoints
const int POLY_ORDER = 3;

// Reference speed
const double ref_v =  30.;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

#endif //MPC_COMMON_H
