#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "mat.h"

/* Tipos de ley de control (cerrado) */
typedef enum {
    CL_PREDICTOR = 0,   /* x̂⁺ = A x̂ + B u + L(y - C x̂) */
    CL_CURRENT   = 1    /* “current”: x̂_tmp = A x̂ + B u; x̂⁺ = x̂_tmp + L(y - C x̂_tmp) */
} control_law_t;

/* Modelo y ganancias DISCRETAS (todas en AC) */
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

/* Paso de control:
   - ref_ac   : referencia en AC (centrada)
   - y_volts  : medición física del ADC (V). Si tu ADC ya entrega AC puro,
                pásalo igual (no se usará el offset).
   - mode     : 0 = cerrado, 1 = abierto (solo sigue ref en físico)
   - law      : CL_PREDICTOR o CL_CURRENT

   Devuelve u_phys (V) saturado en [SAT_MIN, SAT_MAX].
   NOTA: Esta función NO escribe el DAC ni toca UART; es “pura”. */
float control_step(control_state_t *st,
                   const control_model_t *m,
                   float ref_ac,
                   float y_volts,
                   uint8_t mode,
                   control_law_t law);

#endif /* CONTROL_H */
