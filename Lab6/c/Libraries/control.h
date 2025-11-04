#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "mat.h"


typedef enum {
    CL_PREDICTOR = 0,   
    CL_CURRENT   = 1    
} control_law_t;


typedef struct {
    Mat Ad;   /* 2x2 */
    Mat Bd;   /* 2x1 */
    Mat Cd;   /* 1x2 */
    Mat K;    /* 1x2 */
    Mat K0;   /* 1x1  (prefiltro / Nbar / K0) */
    Mat L;    /* 2x1  (ganancias del observador) */
} control_model_t;

/* Estado interno del estimador */
typedef struct {
    Mat xhat; /* 2x1 */
} control_state_t;

/* Inicializa el estado del estimador en cero */
void control_init(control_state_t *st);


float control_step(control_state_t *st,
                   const control_model_t *m,
                   float ref_ac,
                   float y_volts,
                   uint8_t mode,
                   control_law_t law);

#endif /* CONTROL_H */
