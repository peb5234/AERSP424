#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

void Gimbal();
// Define the derivative function prototype (you should implement this separately)
void derivative(const vector<double>& x, const vector<vector<double>>& DCM, double time, const vector<double>& V_body, vector<double>& xdot, vector<vector<double>>& Cdot);


int main(){
    const double PI=3.1415926;
    //double time;
    //double xdot[3][3] ;
    //double theta;
    //double phi;
    //double p=PI/16;
    //double q=cos(6/PI*time);
    //double r=3*sin(30/PI*time);


    //Setup
    double time = 0.0, endtime = 60.0, dt = 0.2; // Use dt for .2, .1, .025, .0125
    int cntr = 0; // iteration counter
    vector<double> V_body = {60, 0.0, 0.0}; // Velocity in the x direction of 60 knots

    // Assuming state vector `x` and DCM are 3x1 and 3x3 respectively
    vector<double> x(3, 0.0); // state vector
    vector<vector<double>> DCM(3, vector<double>(3, 0.0)); // DCM matrix

    vector<vector<double>> totalxdot(3, vector<double>(1000)); // stores results of xdot
    vector<double> time_arr(1000); // time array to store time values

    // Initialize DCM as identity matrix (if needed)
    DCM[0][0] = DCM[1][1] = DCM[2][2] = 1.0;

    // Main RK4 loop
    while (time < endtime) {
        // Allocate space for the derivatives
        vector<double> xdot1(3), xdot2(3), xdot3(3), xdot4(3);
        vector<vector<double>> Cdot1(3, vector<double>(3)), Cdot2(3, vector<double>(3)), Cdot3(3, vector<double>(3)), Cdot4(3, vector<double>(3));

        // First derivative estimate
        derivative(x, DCM, time, V_body, xdot1, Cdot1);

        // Second derivative estimate
        vector<double> x_temp = { x[0] + xdot1[0] * dt / 2, x[1] + xdot1[1] * dt / 2, x[2] + xdot1[2] * dt / 2 };
        vector<vector<double>> DCM_temp = { 
            { DCM[0][0] + Cdot1[0][0] * dt / 2, DCM[0][1] + Cdot1[0][1] * dt / 2, DCM[0][2] + Cdot1[0][2] * dt / 2 },
            { DCM[1][0] + Cdot1[1][0] * dt / 2, DCM[1][1] + Cdot1[1][1] * dt / 2, DCM[1][2] + Cdot1[1][2] * dt / 2 },
            { DCM[2][0] + Cdot1[2][0] * dt / 2, DCM[2][1] + Cdot1[2][1] * dt / 2, DCM[2][2] + Cdot1[2][2] * dt / 2 }
        };
        derivative(x_temp, DCM_temp, time + dt / 2, V_body, xdot2, Cdot2);

        // Third derivative estimate
        x_temp = { x[0] + xdot2[0] * dt / 2, x[1] + xdot2[1] * dt / 2, x[2] + xdot2[2] * dt / 2 };
        DCM_temp = { 
            { DCM[0][0] + Cdot2[0][0] * dt / 2, DCM[0][1] + Cdot2[0][1] * dt / 2, DCM[0][2] + Cdot2[0][2] * dt / 2 },
            { DCM[1][0] + Cdot2[1][0] * dt / 2, DCM[1][1] + Cdot2[1][1] * dt / 2, DCM[1][2] + Cdot2[1][2] * dt / 2 },
            { DCM[2][0] + Cdot2[2][0] * dt / 2, DCM[2][1] + Cdot2[2][1] * dt / 2, DCM[2][2] + Cdot2[2][2] * dt / 2 }
        };
        derivative(x_temp, DCM_temp, time + dt / 2, V_body, xdot3, Cdot3);

        // Fourth derivative estimate
        x_temp = { x[0] + xdot3[0] * dt, x[1] + xdot3[1] * dt, x[2] + xdot3[2] * dt };
        DCM_temp = { 
            { DCM[0][0] + Cdot3[0][0] * dt, DCM[0][1] + Cdot3[0][1] * dt, DCM[0][2] + Cdot3[0][2] * dt },
            { DCM[1][0] + Cdot3[1][0] * dt, DCM[1][1] + Cdot3[1][1] * dt, DCM[1][2] + Cdot3[1][2] * dt },
            { DCM[2][0] + Cdot3[2][0] * dt, DCM[2][1] + Cdot3[2][1] * dt, DCM[2][2] + Cdot3[2][2] * dt }
        };
        derivative(x_temp, DCM_temp, time + dt, V_body, xdot4, Cdot4);

        // Calculate RK4 estimate for xdot and Cdot
        for (int i = 0; i < 3; i++) {
            totalxdot[i][cntr] = (xdot1[i] + 2 * xdot2[i] + 2 * xdot3[i] + xdot4[i]) / 6.0;
        }

        vector<vector<double>> totalCdot(3, vector<double>(3));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                totalCdot[i][j] = (Cdot1[i][j] + 2 * Cdot2[i][j] + 2 * Cdot3[i][j] + Cdot4[i][j]) / 6.0;
            }
        }

        // Update the state vector x
        for (int i = 0; i < 3; i++) {
            x[i] += totalxdot[i][cntr] * dt;
        }

        // Update DCM
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                DCM[i][j] += totalCdot[i][j] * dt;
            }
        }

        // Increment time and counter
        cntr++;
        time += dt;
        time_arr[cntr] = time;
    }

    return 0;   
}


// Function to perform the matrix multiplication for gimbal angles and rates
void Gimbal(double theta, double phi, double p, double q, double r, double xdot[3]) {
    // Define the transformation matrix elements
    double M[3][3] = {
        {1, tan(theta) * sin(phi), tan(theta) * cos(phi)},
        {0, cos(phi), -sin(phi)},
        {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}
    };

    // Perform matrix-vector multiplication
    xdot[0] = M[0][0] * p + M[0][1] * q + M[0][2] * r;
    xdot[1] = M[1][0] * p + M[1][1] * q + M[1][2] * r;
    xdot[2] = M[2][0] * p + M[2][1] * q + M[2][2] * r;
}

void derivative(const vector<double>& x, const vector<vector<double>>& DCM, double time, const vector<double>& V_body, 
                vector<double>& xdot, vector<vector<double>>& Cdot) {
    // Assuming x is the state vector (position and velocity)
    // DCM is the direction cosine matrix (3x3 matrix)
    // V_body is the body-frame velocity vector (3x1 vector)
    // xdot will store the derivative of the state vector
    // Cdot will store the derivative of the DCM matrix

    // Placeholder values for the derivative calculations.
    // In reality, these derivatives will depend on the specific dynamics of the system being modeled.

    // Example: Calculate the time derivative of the position and velocity
    // For now, we assume simple motion where velocity is constant and affects position.
    // For a more sophisticated model, you may need to add forces, accelerations, etc.
    
    // Example dynamics (you would replace this with the actual dynamics of your system):
    for (int i = 0; i < 3; i++) {
        xdot[i] = V_body[i]; // Example: assuming the state vector derivative is velocity
    }

    // Placeholder for DCM derivative. This could involve angular velocities or rotation rates.
    // You may have to compute this using the angular velocity of the body (for example, using skew-symmetric matrices).
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Cdot[i][j] = 0.0; // Placeholder: Replace this with actual DCM derivative calculation
        }
    }
}
