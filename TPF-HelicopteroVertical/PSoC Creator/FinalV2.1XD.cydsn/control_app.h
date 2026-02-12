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
/* u_phy final que sale a tu callback write_u(). En tu caso: PWM en microsegundos */
#ifndef CONTROL_SAT_MIN
    #define CONTROL_SAT_MIN (1000.0f)
#endif
#ifndef CONTROL_SAT_MAX
    #define CONTROL_SAT_MAX (1700.0f)
#endif

/* Offset default (fallback). El real se auto-calibra en runtime (g_u0_offset_us) */
#ifndef CONTROL_U_OFFSET
    #define CONTROL_U_OFFSET (1400.0f)
#endif

/* =======================
   RANGOS DE REFERENCIA
   ======================= */
#ifndef CONTROL_CALIB_HOLD_SAMPLES
    #define CONTROL_CALIB_HOLD_SAMPLES (30u)   /* N muestras por nivel */
#endif

#ifndef CONTROL_CALIB_MIN_HITS
    #define CONTROL_CALIB_MIN_HITS     (2u)    /* anti-ruido: hits consecutivos */
#endif

/* --- OPEN LOOP: ref = Δu (us) relativo a u0 --- */
#ifndef CONTROL_OL_REF_IS_DELTA
    #define CONTROL_OL_REF_IS_DELTA (1u)
#endif

/* Si OL_REF_IS_DELTA=0, entonces ref sería PWM absoluto y se usa esto: */
#ifndef CONTROL_OL_REF_MIN
    #define CONTROL_OL_REF_MIN (1000.0f)
#endif
#ifndef CONTROL_OL_REF_MAX
    #define CONTROL_OL_REF_MAX (1700.0f)
#endif

/* --- CLOSED LOOP: ref = altura (cm) --- */
#ifndef CONTROL_CL_REF_MIN
    #define CONTROL_CL_REF_MIN (12.0f)    /* cm */
#endif
#ifndef CONTROL_CL_REF_MAX
    #define CONTROL_CL_REF_MAX (120.0f)   /* cm */
#endif

/* =======================
   Descenso / Stop suave
   ======================= */
/* OJO: TARGET es Δu relativo (cmd), NO PWM absoluto */
#ifndef CONTROL_DESC_TARGET_CMD_US
    #define CONTROL_DESC_TARGET_CMD_US   (20.0f)   /* Δu (us) relativo a u0 */
#endif

/* PWM absoluto para "cortar" al final */
#ifndef CONTROL_DESC_CUTOFF_US
    #define CONTROL_DESC_CUTOFF_US   (1000.0f)   /* PWM absoluto de corte */
#endif

#ifndef CONTROL_DESC_MIN_Y_CM
    #define CONTROL_DESC_MIN_Y_CM    (12.0f)     /* altura mínima (cm) */
#endif

#ifndef CONTROL_DESC_STEP_US
    #define CONTROL_DESC_STEP_US     (2.0f)      /* rampa en microsegundos (Δu) */
#endif

#ifndef CONTROL_DESC_DELAY_US
    #define CONTROL_DESC_DELAY_US    (20000u)    /* 20ms -> 50Hz */
#endif

#ifndef CONTROL_DESC_MAX_ITERS
    #define CONTROL_DESC_MAX_ITERS   (600u)      /* 600*20ms = 12s */
#endif

#ifndef CONTROL_DESC_MIN_HITS
    #define CONTROL_DESC_MIN_HITS    (2u)        /* N lecturas seguidas <= min */
#endif

#ifndef CONTROL_DESC_HOLD_MS
    #define CONTROL_DESC_HOLD_MS     (500u)      /* esperar 0.5 segundos */
#endif

#ifndef CONTROL_DESC_HOLD_POLL_US
    #define CONTROL_DESC_HOLD_POLL_US (20000u)   /* chequeo cada 20ms */
#endif

/* =======================
   Auto-calibración u0 (runtime)
   ======================= */
#ifndef CONTROL_ENABLE_AUTOCAL
    #define CONTROL_ENABLE_AUTOCAL (1u)
#endif

#ifndef CONTROL_CALIB_START_US
    #define CONTROL_CALIB_START_US (1250.0f)
#endif

#ifndef CONTROL_CALIB_STEP_US
    #define CONTROL_CALIB_STEP_US  (2.0f)
#endif

#ifndef CONTROL_CALIB_MAX_US
    #define CONTROL_CALIB_MAX_US   (1500.0f)
#endif

#ifndef CONTROL_CALIB_DY_CM
    #define CONTROL_CALIB_DY_CM    (0.30f)   /* cm: umbral de movimiento real */
#endif

#ifndef CONTROL_CALIB_N_HITS
    #define CONTROL_CALIB_N_HITS   (10u)
#endif

#ifndef CONTROL_CALIB_MAX_SAMPLES
    #define CONTROL_CALIB_MAX_SAMPLES (800u)
#endif

/* =======================
   TF (IIR) - orden máximo soportado por el paquete TF
   ======================= */
#ifndef CONTROL_TF_MAX_ORDER
    #define CONTROL_TF_MAX_ORDER (10u) /* b0..b10 / a0..a10 */
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

/* START: inicia control + dispara autocalibración (si enabled) */
void  control_start(float ref0);

/* Stop suave: rampa hacia target y corta */
bool  control_stop_suave_step(void);

/* PAQUETE FIJO 25 floats:
   TF:
     c[0..10]  = b0..b10
     c[11..21] = a0..a10
     c[22]     = order (0..10)
     c[23]     = N
     c[24]     = FsHz
   SS:
     c[0..22]  = matrices/ganancias
     c[23]     = N
     c[24]     = FsHz
*/
void  control_apply_tf(const float* c, uint16_t n);
void  control_apply_ss(const float* c, uint16_t n);

void  control_step(void);
void  control_force_min(void);

/* Debug: leer u0 runtime */
float control_get_u0_offset_us(void);

#endif /* CONTROL_APP_H */