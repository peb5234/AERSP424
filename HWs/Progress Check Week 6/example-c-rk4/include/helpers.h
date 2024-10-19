#ifndef HELPERS_HEADER_H
#define HELPERS_HEADER_H

#include "datatypes.h"

/// @brief Quick method to initialize all the states to Zero
/// @param[out] a initialized state vector
void init_states(states *a);

/// @brief Quick method to initialize all the elements of the matrix to Zero
/// @param[out] m initialized matrix
void init_mat3by3(mat3by3 *m);

/// @brief Add two matrices of rows x cols together
/// @param a matrix A
/// @param b matrix B
// @return a new matrix that is the addition of the two
mat3by3 add_matrix(mat3by3 a, mat3by3 b);

/// @brief Elementwise mulitplication by a scalar
/// @param a matrix
/// @param s scaling factor for each element
/// @return a new matrix that is scaled
mat3by3 scale_matrix(mat3by3 a, double s);

/// @brief Multiplication of two matrices of rows x cols together
/// @param a matrix A
/// @param b matrix B
// @return a new matrix that is the multiplication of the two
mat3by3 mult_matrix(mat3by3 a, mat3by3 b);

/// @brief Add two vectors of size rows together
/// @param a state vector A
/// @param b state vector B
/// @return a new vector that is the addition of the two
states add_vector(states a, states b);

/// @brief Elementwise multiplication by a scalar
/// @param a state vector
/// @param s scaling factor for each element
/// @return a new vector that is scaled
states scale_vector(states a, double s);

/// @brief Multiple a matrix and a vector (3 x 3) * (3 x 1)
/// @param m left-side matrix such as DCM or Cdot
/// @param v right-side vector
/// @return new vector resulting from multiplication
vec3f mult_mat_vec(mat3by3 m, vec3f v);

/// @brief Print a named matrix to standard out
/// @param name a name prefix for printing
/// @param arr matrix of doubles
void print_matrix(const char *name, mat3by3 *arr);

/// @brief Print a named vector to standard out
/// @param name a name prefix for printing
/// @param v array of doubles
void print_vec3f(const char *name, vec3f *v);

/// @brief Print a named state vector to standard out
/// @param name a name prefix for printing
/// @param arr vector of doubles
void print_states(const char *name, states *arr);

/// @brief Print a named array to standard out
/// @param name a name prefix for printing
/// @param arr array of doubles
/// @param N size of the array arr
void print_array(const char *name, double *arr, int N);

/// @brief Euler Rotation Matrix for PHI
/// {{1,0,0},{0,C,-S},{0,S,C}}
/// @param phi
/// @return 3x3 rotation matrix for rotation
mat3by3 rotate_by_phi(double phi);

/// @brief Euler Rotation Matrix for PSI
/// {{C,0,S},{0,1,0},{-S,0,C}}
/// @param phi
/// @return 3x3 rotation matrix for rotation
mat3by3 rotate_by_psi(double psi);

/// @brief Euler Rotation Matrix for THETA
/// {{C,-S,0},{S,C,0},{0,0,1}}
/// @param phi
/// @return 3x3 rotation matrix for rotation
mat3by3 rotate_by_theta(double theta);
#endif
