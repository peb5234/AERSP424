#include <stdio.h>
#include <math.h>

#include "helpers.h"
#include "datatypes.h"

void init_states(states *a)
{
    for (int i = 0; i < N_STATES; i++)
        a->x[i] = 0;
}

void init_mat3by3(mat3by3 *m)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m->m[i][j] = 0;
}

mat3by3 add_matrix(mat3by3 a, mat3by3 b)
{
    mat3by3 mat;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mat.m[i][j] = a.m[i][j] + b.m[i][j];
    return mat;
}

mat3by3 scale_matrix(mat3by3 a, double s)
{
    mat3by3 mat;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            mat.m[i][j] = a.m[i][j] * s;
    return mat;
}

mat3by3 mult_matrix(mat3by3 a, mat3by3 b)
{
    mat3by3 mat;
    init_mat3by3(&mat);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                mat.m[i][k] += (a.m[i][j] * b.m[j][k]);
    return mat;
}
states add_vector(states a, states b)
{
    states vec;
    for (int i = 0; i < N_STATES; i++)
        vec.x[i] = a.x[i] + b.x[i];
    return vec;
}

states scale_vector(states a, double s)
{
    states vec;
    for (int i = 0; i < N_STATES; i++)
        vec.x[i] = a.x[i] * s;
    return vec;
}

vec3f mult_mat_vec(mat3by3 m, vec3f v)
{
    vec3f r = {0, 0, 0};
    r.x = m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z;
    r.y = m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z;
    r.z = m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z;
    return r;
}

void print_matrix(const char *name, mat3by3 *arr)
{
    printf("%5s:\n", name);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            printf("  [%d][%d] = %f\n", i, j, arr->m[i][j]);
    printf("\n");
}
void print_vec3f(const char *name, vec3f *v)
{
    printf("%5s:\n", name);
    printf("  x = %f\n", v->x);
    printf("  y = %f\n", v->y);
    printf("  z = %f\n", v->z);
}

void print_states(const char *name, states *arr)
{
    printf("%5s:", name);
    for (int i = 0; i < N_STATES; i++)
        printf("% 5.3f\t", arr->x[i]);
    printf("\n");
}

void print_array(const char *name, double *arr, int N)
{
    printf("%5s:", name);
    for (int i = 0; i < N; i++)
        printf("% 5.3f\t", arr[i]);
    printf("\n");
}

mat3by3 rotate_by_phi(double phi)
{
    mat3by3 rot = {{{1, 0, 0}, {0, cos(phi), -sin(phi)}, {0, sin(phi), cos(phi)}}};
    return rot;
}
mat3by3 rotate_by_psi(double psi)
{
    mat3by3 rot = {{{cos(psi), 0, sin(psi)}, {0, 1, 0}, {-sin(psi), 0, cos(psi)}}};
    return rot;
}
mat3by3 rotate_by_theta(double theta)
{
    mat3by3 rot = {{{cos(theta), -sin(theta), 0}, {sin(theta), cos(theta), 0}, {0, 0, 1}}};
    return rot;
}
