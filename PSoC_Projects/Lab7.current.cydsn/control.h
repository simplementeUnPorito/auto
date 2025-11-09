#ifndef CONTROL_H
#define CONTROL_H
#include "mat.h"
#include <stdint.h>

/* Modelo/ganancias (discreto, AC) */
typedef struct {
    Mat Ad;   /* 2x2 */
    Mat Bd;   /* 2x1 */
    Mat Cd;   /* 1x2 */
    Mat K;    /* 1x2 */
    Mat K0;   /* 1x1 (Nbar) */
    Mat L;    /* 2x1 (Ke = acker(A',(C*A)',...).') */
} control_model_t;

/* Estado del estimador */
typedef struct { Mat xhat; } control_state_t;

void  control_init(control_state_t *st);

/* Paso 1: calcula u[k] (AC) y z[k+1] (AC); retorna u_phys (saturado) */
float current_u_and_z(control_state_t *st,
                      const control_model_t *m,
                      float ref_ac,
                      Mat *z_out);

/* Paso 2: corrige x̂ con y[k+1] (AC) */
void  current_correct_with_y(control_state_t *st,
                             const control_model_t *m,
                             const Mat *z,
                             float y_ac);
#endif
