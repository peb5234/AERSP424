#ifndef SIM_HEADER_H
#define SIM_HEADER_H

#include "helpers.h"
#include "datatypes.h"

/// @brief Forcing Function applied to the aircraft during simulation
/// @param time current time in the simulation
/// @param[out] omega angular velocities experienced by the aircraft at this time
void ffunc(double time, AngularVelocity *omega);

/// @brief Compute the Gimbal Equation multiplication of states (attitude angles) and the rates
/// @param x state of aircraft for use with (roll, pitch, yaw)
/// @param omega angular rates applied to the body
/// @return Qdot for derivative of the body Euler angles
vec3f gimbal_eqn(states x, AngularVelocity omega);

/// @brief Computer the Strapdown Equation multiplication of Cdot = DCM * strapdown
/// @param omega angular rates applied to the body
/// @param dcm Direction Cosine Matrix of current attitude
/// @return Cdot for derivatives of the Direction Cosine Matrix
mat3by3 strapdown_eqn(AngularVelocity omega, mat3by3 dcm);

/// @brief State Derivative function for the aircraft simulation
/// @param x state variable (Angles_{phi,theta,psi}, Vbody_{N,E,D},Position_{N,E,D}) as a 9 element array
/// @param dcm Direction Cositne Matrix as a 3x3 create_matrix() object
/// @param v_body velocity of the body in {N,E,D}
/// @param t current simulation time
/// @param[out] xdot state derivative
/// @param[out] Cdot strapdown equations for updating DCM
void dfunc(states x, mat3by3 dcm, vec3f v_body, double t, states *xdot, mat3by3 *Cdot);

/// @brief Aircraft Simulation main loop
/// @param dt timestep to use
/// @param max_time max simulation time to execute
void simulate(double dt, double max_time);

/// @brief Simulation step that performs Runge-Kutta numerical integration
/// @param x current state of the system
/// @param dcm current Direction Cosine Matrix of the system
/// @param v_body current velocity of the body of the system
/// @param time current time of the simulation
/// @param dt timestep used in simulation
/// @param[out] x_new udpated state vector
/// @param[out] dcm_new updated Direction Cosine Matrix
/// @param[out] xdot_new updated state vector derivative
void rk4_step(states x, mat3by3 dcm, vec3f v_body, double time, double dt, states *x_new, mat3by3 *dcm_new, states *xdot_new);

/// @brief Print the HEADER line for file that stores simulation results
void print_header(void);

/// @brief Print the current time, states, and state derivatives in a delimited format for plotting with secondary tool
/// @param t current time
/// @param x current state vector of aircraft
/// @param xdot current state derivative vector of aircraft
void print_current(double t, states *x, states *xdot);

#endif
