#include "control.h"
#include "config.h"

/* saturación físico DAC */
static inline float sat_u(float u){
    if (u < SAT_MIN) u = SAT_MIN;
    if (u > SAT_MAX) u = SAT_MAX;
    return u;
}

void control_init(control_state_t *st){
    st->xhat.filas = 2; st->xhat.columnas = 1;
    st->xhat.d[0][0] = 0.0; st->xhat.d[1][0] = 0.0;
}

/* 1) u = K0*r - K*xhat ;  z = A*xhat + B*u  (todo AC) */
float current_u_and_z(control_state_t *st,
                      const control_model_t *m,
                      float ref_ac,
                      Mat *z)
{
    Mat R   = (Mat){.filas=1,.columnas=1,.d={{ref_ac}}};
    Mat KX  = (Mat){.filas=1,.columnas=1,.d={{0.0}}};
    Mat K0R = (Mat){.filas=1,.columnas=1,.d={{0.0}}};
    mat_mul(&m->K,  &st->xhat, &KX);
    mat_mul(&m->K0, &R,        &K0R);
    double u_ac = K0R.d[0][0] - KX.d[0][0];

    Mat Ax = (Mat){.filas=2,.columnas=1,.d={{0.0},{0.0}}};
    Mat Bu = (Mat){.filas=2,.columnas=1,.d={{0.0},{0.0}}};
    Mat U1 = (Mat){.filas=1,.columnas=1,.d={{u_ac}}};
    mat_mul(&m->Ad, &st->xhat, &Ax);
    mat_mul(&m->Bd, &U1,       &Bu);

    z->filas = 2; z->columnas = 1;
    z->d[0][0] = Ax.d[0][0] + Bu.d[0][0];
    z->d[1][0] = Ax.d[1][0] + Bu.d[1][0];

    return sat_u((float)(u_ac + (double)VDDA2));  /* físico para DAC */
}

/* 2) xhat^+ = z + Ke * ( y - C*z )  (Ke = m->L) */
void current_correct_with_y(control_state_t *st,
                            const control_model_t *m,
                            const Mat *z,
                            float y_ac)
{
    Mat Cz = (Mat){.filas=1,.columnas=1,.d={{0.0}}};
    mat_mul(&m->Cd, z, &Cz);
    double innov = (double)y_ac - Cz.d[0][0];

    Mat E  = (Mat){.filas=1,.columnas=1,.d={{innov}}};
    Mat Le = (Mat){.filas=2,.columnas=1,.d={{0.0},{0.0}}};
    mat_mul(&m->L, &E, &Le);  /* Ke */

    st->xhat.d[0][0] = z->d[0][0] + Le.d[0][0];
    st->xhat.d[1][0] = z->d[1][0] + Le.d[1][0];
}
