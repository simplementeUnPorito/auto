/* =========================
   control_app.h  (UNIDADES EN METROS PARA y/r)
   ========================= */
#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
   UNIDADES (Closed-loop)
   - y_phys: metros [m]
   - r_phys: metros [m]
   - y_rel, r_rel: metros [m]
   Open-loop:
   - referencia puede ser PWM absoluto [us] o Δu [us] según flags
   ========================================================= */

/* =======================
   RANGO DEL ACTUADOR (PWM)
   ======================= */

/* Máximo PWM SIEMPRE */
#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (1700.0f)
#endif

/* Mínimo PWM cuando HAY control (sesión activa) */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (1150.0f)
#endif

/* Mínimo PWM cuando NO hay control (idle) */
#ifndef CONTROL_SAT_MIN_IDLE
    #define CONTROL_SAT_MIN_IDLE (1000.0f)
#endif

#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (1300.0f)
#endif

/* =======================
   RANGOS DE REFERENCIA
   ======================= */
#ifndef CONTROL_CALIB_HOLD_SAMPLES
    #define CONTROL_CALIB_HOLD_SAMPLES (30u)
#endif

#ifndef CONTROL_CALIB_MIN_HITS
    #define CONTROL_CALIB_MIN_HITS     (2u)
#endif

#ifndef CONTROL_OL_REF_IS_DELTA
    #define CONTROL_OL_REF_IS_DELTA (1u)
#endif

/* Open-loop (PWM absoluto) si CONTROL_OL_REF_IS_DELTA = 0 */
#ifndef CONTROL_OL_REF_MIN
    #define CONTROL_OL_REF_MIN (1000.0f)
#endif
#ifndef CONTROL_OL_REF_MAX
    #define CONTROL_OL_REF_MAX (1700.0f)
#endif

/* Closed-loop (referencia en METROS) */
#ifndef CONTROL_CL_REF_MIN
    #define CONTROL_CL_REF_MIN (0.12f)   /* antes 12 cm */
#endif
#ifndef CONTROL_CL_REF_MAX
    #define CONTROL_CL_REF_MAX (1.20f)   /* antes 120 cm */
#endif

/* =======================
   Descenso / Stop suave
   ======================= */
#ifndef CONTROL_DESC_TARGET_CMD_US
    #define CONTROL_DESC_TARGET_CMD_US   (20.0f)
#endif

#ifndef CONTROL_DESC_CUTOFF_US
    #define CONTROL_DESC_CUTOFF_US   (1000.0f)
#endif

/* Altura mínima de referencia/offset en METROS */
#ifndef CONTROL_DESC_MIN_Y_M
    #define CONTROL_DESC_MIN_Y_M    (0.12f)  /* antes 12 cm */
#endif

#ifndef CONTROL_DESC_STEP_US
    #define CONTROL_DESC_STEP_US     (2.0f)
#endif

#ifndef CONTROL_DESC_DELAY_US
    #define CONTROL_DESC_DELAY_US    (20000u)
#endif

#ifndef CONTROL_DESC_MAX_ITERS
    #define CONTROL_DESC_MAX_ITERS   (600u)
#endif

#ifndef CONTROL_DESC_MIN_HITS
    #define CONTROL_DESC_MIN_HITS    (2u)
#endif

#ifndef CONTROL_DESC_HOLD_MS
    #define CONTROL_DESC_HOLD_MS     (500u)
#endif

#ifndef CONTROL_DESC_HOLD_POLL_US
    #define CONTROL_DESC_HOLD_POLL_US (20000u)
#endif

/* =======================
   Auto-calibración u0
   ======================= */
#ifndef CONTROL_ENABLE_AUTOCAL
    #define CONTROL_ENABLE_AUTOCAL (1u)
#endif

#ifndef CONTROL_CALIB_START_US
    #define CONTROL_CALIB_START_US (1350.0f)
#endif

#ifndef CONTROL_CALIB_STEP_US
    #define CONTROL_CALIB_STEP_US  (2.0f)
#endif

#ifndef CONTROL_CALIB_MAX_US
    #define CONTROL_CALIB_MAX_US   (1600.0f)
#endif

/* Umbral de movimiento en METROS (antes 0.30 cm -> 0.0030 m) */
#ifndef CONTROL_CALIB_DY_M
    #define CONTROL_CALIB_DY_M    (0.0030f)
#endif

#ifndef CONTROL_CALIB_N_HITS
    #define CONTROL_CALIB_N_HITS   (10u)
#endif

#ifndef CONTROL_CALIB_MAX_SAMPLES
    #define CONTROL_CALIB_MAX_SAMPLES (800u)
#endif

/* =======================
   TF (IIR) - orden máximo
   ======================= */
#ifndef CONTROL_TF_MAX_ORDER
    #define CONTROL_TF_MAX_ORDER (10u)
#endif

/* =======================
   Callbacks IO
   ======================= */
typedef void (*control_write_u_fn_t)(float u_phy);
typedef void (*control_sample_isr_fn_t)(void);

/* =======================
   Variables ISR -> main
   ======================= */
extern volatile float   control_last_y;        /* y en METROS */
extern volatile uint8_t control_sample_pending;
extern volatile float   control_ref_v;         /* r en METROS */

/* =======================
   API
   ======================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u);

void control_on_sample_isr(void);

/* y debe venir en METROS */
void control_sample_isr_push(float y_m);

void  control_set_reference(float r_m);
float control_get_reference(void);

void  control_set_sample_time(float Ts);
float control_get_sample_time(void);

void  control_start(float ref0_m);
bool  control_stop_suave_step(void);

void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

void  control_step(void);
void  control_force_min(void);

float control_get_u0_offset_us(void);

#endif /* CONTROL_APP_H */
