#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* ===== Logging (placeholder) ===== */
#ifndef CONTROL_LOG_LEN
#define CONTROL_LOG_LEN (512u)  /* cantidad de muestras */
#endif

/* Elegí qué guardás por muestra (recomendado 3: r,y,u) */
#ifndef CONTROL_LOG_FIELDS
#define CONTROL_LOG_FIELDS (3u) /* 1=y ; 2=r,y ; 3=r,y,u */
#endif

/* ===== Saturación / offset físico (placeholders) =====
   Si tu control trabaja en AC alrededor de 0 y tu DAC es 0..4.08 con offset VDDA2,
   seteá CONTROL_U_OFFSET = VDDA2 y saturación a [0..4.08].
*/
#ifndef CONTROL_SAT_MIN
#define CONTROL_SAT_MIN (0.0f)
#endif

#ifndef CONTROL_SAT_MAX
#define CONTROL_SAT_MAX (4.08f)
#endif

#ifndef CONTROL_U_OFFSET
#define CONTROL_U_OFFSET (0.0f)
#endif

#ifndef CONTROL_STOP_TARGET
#define CONTROL_STOP_TARGET (0.0f) /* a dónde querés llevar el esfuerzo al parar */
#endif

/* ===== API usada por UARTP ===== */
bool control_stop_suave_step(void);
void control_start(float ref0);

void control_apply_tf(const float* c, uint16_t n);
void control_apply_ss(const float* c, uint16_t n);

void control_step(void);

/* ===== Dump de muestras (pull, placeholder) ===== */
void control_log_reset(void);
uint8_t control_log_ready(void);
const uint8_t* control_log_bytes(uint16_t* nbytes);

#endif
