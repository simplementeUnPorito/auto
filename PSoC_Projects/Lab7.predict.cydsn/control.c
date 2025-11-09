#include "control.h"
#include "config.h"   /* SAT_MIN, SAT_MAX, VDDA2 */
#include <string.h>

/* Saturación de esfuerzo físico */
static inline float sat_u_phys(float u)
{
    if (u > SAT_MAX) u = SAT_MAX;
    if (u < SAT_MIN) u = SAT_MIN;
    return u;
}

void control_init(control_state_t *st)
{
    /* xhat = 0 (2x1) */
    st->xhat.filas = 2;
    st->xhat.columnas = 1;
    st->xhat.d[0][0] = 0.0;
    st->xhat.d[1][0] = 0.0;
}

/* u = K0*r - K*xhat (AC); observador según 'law' */
float control_step(control_state_t *st,
                   const control_model_t *m,
                   float ref_ac,
                   float y_volts,
                   uint8_t mode,
                   control_law_t law)
{
    /* Lazo abierto: el DAC debe seguir la referencia en físico */
    if (mode == 1 /* MODE_OPEN */) {
        float u_phys_ol = ref_ac + VDDA2;
        return sat_u_phys(u_phys_ol);
    }

    /* IMPORTANTE:
       Si tu ADC YA ENTREGA AC (centrado en 0), entonces y_volts es AC y
       NO hay que restar el offset. Si en tu hardware el CountsTo_Volts()
       devuelve absoluto, y querés usar AC, entonces y_ac = y_volts - VDDA2.
       Para respetar tu esquema “todo ya está en AC”, tomamos y_ac = y_volts. */
    double y_ac = (double)y_volts;  /* ya AC según tu circuito */

    /* u_ac = K0*r - K*xhat */
    Mat R   = (Mat){ .filas=1,.columnas=1,.d={{ ref_ac }} };
    Mat KX  = (Mat){ .filas=1,.columnas=1,.d={{ 0.0 }} };
    Mat K0R = (Mat){ .filas=1,.columnas=1,.d={{ 0.0 }} };

    mat_mul(&m->K,  &st->xhat, &KX);
    mat_mul(&m->K0, &R,        &K0R);

    double u_ac = K0R.d[0][0] - KX.d[0][0];

    /* Llevar a físico y saturar SOLO al salir */
    float u_phys = sat_u_phys((float)(u_ac + (double)VDDA2));

    /* u realmente aplicado en AC (post-sat) */
    double u_ac_apl = (double)u_phys - (double)VDDA2;

    /* Observador */
    Mat Cx  = (Mat){ .filas=1,.columnas=1,.d={{ 0.0 }} };
    mat_mul(&m->Cd, &st->xhat, &Cx);
    double e_y = y_ac - Cx.d[0][0];

    Mat Ax = (Mat){ .filas=2,.columnas=1,.d={{ 0.0 },{ 0.0 }} };
    Mat Bu = (Mat){ .filas=2,.columnas=1,.d={{ 0.0 },{ 0.0 }} };
    Mat Le = (Mat){ .filas=2,.columnas=1,.d={{ 0.0 },{ 0.0 }} };

    Mat U1 = (Mat){ .filas=1,.columnas=1,.d={{ 0.0 }} };
    Mat E1 = (Mat){ .filas=1,.columnas=1,.d={{ e_y }} };

    switch (law) {

        case CL_PREDICTOR:
            /* x̂⁺ = A x̂ + B u + L (y - C x̂)  usando y[k] */
            U1.d[0][0] = u_ac_apl;
            mat_mul(&m->Ad, &st->xhat, &Ax);
            mat_mul(&m->Bd, &U1,       &Bu);
            mat_mul(&m->L,  &E1,       &Le);

            st->xhat.d[0][0] = Ax.d[0][0] + Bu.d[0][0] + Le.d[0][0];
            st->xhat.d[1][0] = Ax.d[1][0] + Bu.d[1][0] + Le.d[1][0];
            break;

        case CL_CURRENT:
            
        default:
            /* “current” típico:
               x̂_tmp = A x̂ + B u
               x̂⁺    = x̂_tmp + L (y - C x̂_tmp) */
            U1.d[0][0] = u_ac_apl;
            mat_mul(&m->Ad, &st->xhat, &Ax);
            mat_mul(&m->Bd, &U1,       &Bu);

            /* x̂_tmp en Ax := Ax + Bu */
            Ax.d[0][0] += Bu.d[0][0];
            Ax.d[1][0] += Bu.d[1][0];

            /* innovación con x̂_tmp */
            Mat Cx2 = (Mat){ .filas=1,.columnas=1,.d={{ 0.0 }} };
            mat_mul(&m->Cd, &Ax, &Cx2);
            {
                double e2 = y_ac - Cx2.d[0][0];
                Mat E2 = (Mat){ .filas=1,.columnas=1,.d={{ e2 }} };
                mat_mul(&m->L, &E2, &Le);
            }

            st->xhat.d[0][0] = Ax.d[0][0] + Le.d[0][0];
            st->xhat.d[1][0] = Ax.d[1][0] + Le.d[1][0];
            break;
    }

    return u_phys;
}
