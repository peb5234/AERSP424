#ifndef DATATYPES_HEADER_H
#define DATATYPES_HEADER_H

#define N_STATES 9 // Number of STATE variables

/// @brief double array representing the N_STATES of the simulation
typedef struct
{
    double x[N_STATES];
} states;

/// @brief double array of arrays for the 2D matrix of 3x3 (DCM, Cdot)
typedef struct
{
    double m[3][3];
} mat3by3;

/// @brief A structure to show the angular velocities applied to the aircraft during simulation
typedef struct
{
    double p;
    double q;
    double r;
} AngularVelocity;

/// @brief A small 3-element vector for storing (alternative to the AngularVelocity) or double[3] representation
typedef struct
{
    double x, y, z;
} vec3f;

#endif
