#include <stdio.h>
#include <math.h>

#include "datatypes.h"
#include "helpers.h"
#include "sim.h"

void print_header(void)
{
    printf("time\tphi\ttheta\tpsi\tVbody_N\tVbody_E\tVbody_D\tP_N\tP_E\tP_D\tV_N\tV_E\tV_D\n");
}

void print_current(double t, states *x, states *xdot)
{
    printf("%f", t);
    for (int i = 0; i < N_STATES; i++)
        printf("\t%f", x->x[i]);
    for (int i = 0; i < N_STATES; i++)
        printf("\t%f", xdot->x[i]);
    printf("\n");
}

void ffunc(double time, AngularVelocity *omega)
{
    omega->p = M_PI / 6.0 + 0 * time;
    omega->q = cos(time * 6.0 / M_PI);
    omega->r = 3 * sin(time * 30 / M_PI);
}

vec3f gimbal_eqn(states x, AngularVelocity omega)
{
    double phi = x.x[0];
    double theta = x.x[1];
    mat3by3 gimbal = {{{1, tan(theta) * sin(phi), tan(theta) * cos(phi)},
                       {0, cos(phi), -sin(phi)},
                       {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}}};
    vec3f tmp;
    tmp.x = omega.p;
    tmp.y = omega.q;
    tmp.z = omega.r;
    vec3f qdot = mult_mat_vec(gimbal, tmp);

    return qdot;
}

mat3by3 strapdown_eqn(AngularVelocity omega, mat3by3 dcm)
{
    mat3by3 Cdot;
    init_mat3by3(&Cdot);
    mat3by3 strapdown = {{{0, -omega.r, omega.q},
                          {omega.r, 0, -omega.p},
                          {-omega.q, omega.p, 0}}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                Cdot.m[i][j] += (dcm.m[i][k] * strapdown.m[k][j]);
    return Cdot;
}
void dfunc(states x, mat3by3 dcm, vec3f v_body, double t, states *xdot, mat3by3 *Cdot)
{
    // In this case, p,q & r are given by an analytic formula
    AngularVelocity omega;
    ffunc(t, &omega);

    // Gimbal Equation
    vec3f qdot = gimbal_eqn(x, omega);

    // Strapdown Equation
    *Cdot = strapdown_eqn(omega, dcm);

    // Get Velocity in NED
    // this is position_dot for position measured in NED
    vec3f v_ned = mult_mat_vec(dcm, v_body);

    // Assemble the vector of state derivatives -- putting in zeros for the
    // derivative of velocity since it is a constant here
    xdot->x[0] = qdot.x;
    xdot->x[1] = qdot.y;
    xdot->x[2] = qdot.z;
    xdot->x[3] = 0.0;
    xdot->x[4] = 0.0;
    xdot->x[5] = 0.0;
    xdot->x[6] = v_ned.x;
    xdot->x[7] = v_ned.y;
    xdot->x[8] = v_ned.z;

#if DEBUG
    print_matrix("dcm", &dcm);
    print_matrix("cdot", Cdot);
    print_states("xdot", xdot);
    print_vec3f("v_body", &v_body);
    print_vec3f("v_ned", &v_ned);
#endif
}

void rk4_step(states x, mat3by3 dcm, vec3f v_body, double time, double dt, states *x_new, mat3by3 *dcm_new, states *xdot_new)
{
    states xdot1, xdot2, xdot3, xdot4;
    mat3by3 Cdot1, Cdot2, Cdot3, Cdot4;
    mat3by3 Cdot;
    dfunc(x, dcm, v_body, time, &xdot1, &Cdot1);
#if DEBUG
    print_states("xdot1", &xdot1);
    print_matrix("cdot1", &Cdot1);
#endif

    add_vector(x, scale_vector(x, 2.0));
    dfunc(
        add_vector(x, scale_vector(xdot1, dt / 2.0)),
        add_matrix(dcm, scale_matrix(Cdot1, dt / 2.0)),
        v_body, time + dt / 2.0,
        &xdot2,
        &Cdot2);
#if DEBUG
    print_states("xdot2", &xdot2);
#endif

    dfunc(
        add_vector(x, scale_vector(xdot2, dt / 2.0)),
        add_matrix(dcm, scale_matrix(Cdot2, dt / 2.0)),
        v_body, time + dt / 2.0,
        &xdot3,
        &Cdot3);
#if DEBUG
    print_states("xdot3", &xdot3);
#endif
    dfunc(
        add_vector(x, scale_vector(xdot3, dt)),
        add_matrix(dcm, scale_matrix(Cdot3, dt)),
        v_body, time + dt,
        &xdot4,
        &Cdot4);
#if DEBUG
    print_states("xdot4", &xdot4);
#endif

    /*
     * Runga-Kutta is now complete!! Perform the final integration step
     * Simple as doing the multiplications and the scaling
     */
    Cdot = scale_matrix(
        add_matrix(
            add_matrix(
                add_matrix(Cdot1, scale_matrix(Cdot2, 2.0)),
                scale_matrix(Cdot3, 2.0)),
            Cdot4),
        1 / 6.0);

    // UPDATE the states and return variables
    *dcm_new = add_matrix(
        dcm,
        scale_matrix(Cdot, dt));
#if DEBUG
    print_matrix("dcm", dcm_new);
#endif

    // UPDATE the states and return variables
    *xdot_new = scale_vector(
        add_vector(
            add_vector(
                add_vector(xdot1, scale_vector(xdot2, 2)),
                scale_vector(xdot3, 2)),
            xdot4),
        1 / 6.0);
#if DEBUG
    print_states("xdot_new", xdot_new);
#endif
    *x_new = add_vector(x, scale_vector(*xdot_new, dt));
#if DEBUG
    print_states("x_new", x_new);
#endif
}

void simulate(double dt, double max_time)
{
    // initial euler angles
    double x0[3] = {0, 0, 0};

    // initial velocity of the aircraft
    vec3f v_body = {60 * 6076 / 3600, 0, 0};

    // initial state
    states x;
    init_states(&x);
    x.x[0] = x0[0];
    x.x[1] = x0[1];
    x.x[2] = x0[2];
    x.x[3] = v_body.x;
    x.x[4] = v_body.y;
    x.x[5] = v_body.z;
    x.x[6] = 0;
    x.x[7] = 0;
    x.x[8] = 0;

    mat3by3 dcm;
    init_mat3by3(&dcm);
    dcm.m[0][0] = 1;
    dcm.m[1][1] = 1;
    dcm.m[2][2] = 1;

    dcm = mult_matrix(rotate_by_phi(x.x[0]), dcm);
    dcm = mult_matrix(rotate_by_psi(x.x[1]), dcm);
    dcm = mult_matrix(rotate_by_theta(x.x[2]), dcm);
#if DEBUG
    print_matrix("dcm0", &dcm);
#endif

    states xdot;
    init_states(&xdot);
    mat3by3 Cdot;
    init_mat3by3(&Cdot);
    double time = 0;
    print_header();
    print_current(time, &x, &xdot);
    do
    {
        rk4_step(x, dcm, v_body, time, dt, &x, &dcm, &xdot);
        // increment time by dt
        time += dt;
        // print to standard out for data recording
        print_current(time, &x, &xdot);

    } while (time < max_time);
}
