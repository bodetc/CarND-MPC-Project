# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

---

## Writeup

This is my implementation of the MPC controller for the second term of the Self-Driving Car Engineer Nanodegree Program.

### The Model
*Student describes their model in detail. This includes the state, actuators and update equations.*

The model used in this project is the model described in the lesson *Model Predictive Control*.
The state consists of the following variables:
* `px` and `py`: Position of the car in the 2D world
* `psi`: Orientation of the car
* `v`: Velocity of the car
* `cte`: Cross-track error, i.e. the distance between the position of the car and the desired waypoint.
* `epsi`: The angle between the actual and the desired car orientation

This model contains two actuators:
* `delta`: The steering angle
* `a`: The throttle position (also models braking with negative values)

The update equations of the model are given as follows:
* `x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt`
* `y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt`
* `psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt`
* `v_[t+1] = v[t] + a[t] * dt`
* `cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`
* `epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt`

In these equations, we defined the following symbols:
* `dt`: The timestep of the updates (see below)
* `f(x)`: The ideal trajectory, interpolated from the waypoints
* `Lf`: Distance between the front axle and the center of gravity, determining the turn radius.

The cost function was calculated by adding several individual costs with different weights.
Those weights were chosen to provide a smooth and safe ride in the simulator.

The following individual elements are minimized by the cost function:
* Cross Track Error: `2000.*CppAD::pow(vars[cte_start + t], 2)`
* Orientation error: `2000.*CppAD::pow(vars[epsi_start + t], 2)`
* Speed error: `CppAD::pow(vars[v_start + t] - ref_v, 2)`
* Use of steering: `100.*CppAD::pow(vars[delta_start + t], 2)`
* Use of throttle: `10.*CppAD::pow(vars[a_start + t], 2)`
* Value gap between sequential actuations:
`100.*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2)`
and `100.*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2)`


### Timestep Length and Elapsed Duration (N & dt)
*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

The final values of the timestep length and elapsed duration were chosen as
`dt=0.1s` and `N=10`.
The timestep is chosen to be in the order of the latency, a smaller timestep would imply controlling the car faster than is physically possible.
The total prediction time `N*dt=1 s` allows to negotiate the curve smoothly while keeping the prediction horizon within the provided waypoints.

Other values of the timestep were experimented, such as `0.05s` and `0.2s`, with simulation horizons ranging from `1s` to `2s`.

### Polynomial Fitting and MPC Preprocessing
*A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

For easier representation of the waypoints, the data is preprocessed by performing a translation/rotation of the points to the reference frame of the car.
First a translation `(-px, -py)` is performed,
followed by a rotation of angle `-psi`.

Afterwards, a polynomial fittting of order three was performed on the waypoints.

###Model Predictive Control with Latency
*The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

The latency is taken into account by using the update equations of the model (see above) for the duration of the latency.