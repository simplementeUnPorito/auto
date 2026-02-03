/* =========================
   control_app.h
   ========================= */
#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* =======================
   RANGO DEL ACTUADOR (PWM)
   ======================= */
/* u_phy final que sale a tu callback write_u().
   En tu caso: PWM en microsegundos */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (1000.0f)
#endif
#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (2000.0f)
#endif

/* Offset opcional (si quisieras centrar el comando).
   Para PWM no te hace falta: dejalo 0. */
#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (0.0f)
#endif

/* =======================
   RANGOS DE REFERENCIA
   ======================= */
/* --- OPEN LOOP: ref = PWM directo --- */
#ifndef CONTROL_OL_REF_MIN
    #define CONTROL_OL_REF_MIN (1000.0f)
#endif
#ifndef CONTROL_OL_REF_MAX
    #define CONTROL_OL_REF_MAX (2000.0f)
#endif

/* --- CLOSED LOOP: ref = altura (cm) --- */
#ifndef CONTROL_CL_REF_MIN
    #define CONTROL_CL_REF_MIN (5.0f)    /* cm */
#endif
#ifndef CONTROL_CL_REF_MAX
    #define CONTROL_CL_REF_MAX (80.0f)   /* cm */
#endif

/* =======================
   Stop suave (en unidades de u_cmd)
   ======================= */
#ifndef CONTROL_STOP_TARGET
    /* Para PWM: normalmente querés ir a 1000 us */
    #define CONTROL_STOP_TARGET (1000.0f)
#endif
#ifndef CONTROL_STOP_STEP
    /* paso en microsegundos por iteración */
    #define CONTROL_STOP_STEP (2.0f)
#endif

#ifndef CONTROL_STOP_TOL_REL
#define CONTROL_STOP_TOL_REL   (0.05f)   /* 5% */
#endif

#ifndef CONTROL_STOP_TOL_ABS
#define CONTROL_STOP_TOL_ABS   (0.5f)    /* 0.5 us (tolerancia mínima) */
#endif

#ifndef CONTROL_STOP_DELAY_US
#define CONTROL_STOP_DELAY_US  (1000u)   /* 1 ms entre pasos */
#endif

#ifndef CONTROL_STOP_MAX_ITERS
#define CONTROL_STOP_MAX_ITERS (20000u)  /* failsafe */
#endif

/* =======================
   Callbacks IO
   ======================= */
typedef void (*control_write_u_fn_t)(float u_phy);

/* Corre DENTRO del ISR de muestreo.
   Debe ser ultraliviano y terminar llamando:
     control_sample_isr_push(y);
*/
typedef void (*control_sample_isr_fn_t)(void);

/* =======================
   Variables ISR -> main
   ======================= */
extern volatile float   control_last_y;
extern volatile uint8_t control_sample_pending;
extern volatile float   control_ref_v;

/* =======================
   API
   ======================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u);

void control_on_sample_isr(void);
void control_sample_isr_push(float y);

void  control_set_reference(float r);
float control_get_reference(void);

void  control_set_sample_time(float Ts);
float control_get_sample_time(void);

void  control_start(float ref0);
bool  control_stop_suave_step(void);

void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

void  control_step(void);
void  control_force_min(void);

#endif /* CONTROL_APP_H */
