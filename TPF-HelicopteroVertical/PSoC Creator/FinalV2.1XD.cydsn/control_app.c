/* =========================
   control_app.c  (UNIDADES EN METROS PARA y/r)
   (PAQUETE FIJO 25 floats)
   =========================
   Meta SIEMPRE al final:
     c[23] = N
     c[24] = FsHz
   TF además usa:
     c[22] = order (0..10)
*/

#include "control_app.h"

#include "project.h"
#include "uartp_sw.h"
#include "arm_math.h"
#include <string.h>
#include <stdint.h>

/* =======================
   Defaults de estabilización post-calib (si no están en .h)
   ======================= */
#ifndef CONTROL_U0_SETTLE_SAMPLES
    #define CONTROL_U0_SETTLE_SAMPLES (50u)
#endif
/* antes 0.5 cm -> 0.005 m */
#ifndef CONTROL_U0_SETTLE_DY_M
    #define CONTROL_U0_SETTLE_DY_M   (0.005f)
#endif

/* =======================
   Variables ISR -> main
   ======================= */
volatile float   control_last_y = 0.0f;   /* y en METROS */
volatile uint8_t control_sample_pending = 0u;
volatile float   control_ref_v = 0.0f;    /* r en METROS */

/* =======================
   IO callbacks
   ======================= */
static control_sample_isr_fn_t s_sample_isr_cb = 0;
static control_write_u_fn_t    s_write_u_cb    = 0;

/* =======================
   Estado general
   ======================= */
static float g_ref = 0.0f;   /* r en METROS */
static float g_Ts  = 1.0f;

/* Interno: Δu (us) */
static float g_u_cmd = 0.0f;                /* comando interno (Δu) */
static float g_u_out = CONTROL_SAT_MIN_IDLE;/* físico (PWM us) post offset+sat */

/* integrador */
static float g_vint  = 0.0f;

/* Offset runtime (u0). Se calcula en calibración o cae al default */
static float g_u0_offset_us = CONTROL_U_OFFSET;

float control_get_u0_offset_us(void)
{
    return g_u0_offset_us;
}

/* =======================
   Sesión de control
   ======================= */
static uint8_t g_session_active = 0u; /* 0=idle (PWM=1000), 1=en sesión (min=1100) */
static uint8_t g_u0_valid       = 0u; /* 0=pendiente calibrar, 1=u0 fijo en esta sesión */

/* =======================
   NaN-aware: “últimos buenos”
   ======================= */
static float g_last_r_phys_ok   = 0.0f;  /* METROS */
static float g_last_y_phys_ok   = 0.0f;  /* METROS */

static float g_last_u_cmd_ok    = 0.0f;                 /* Δu válido */
static float g_last_u_out_ok    = CONTROL_SAT_MIN_IDLE; /* PWM válido */

static float g_last_y_rel_ok    = 0.0f; /* METROS */
static float g_last_r_rel_ok    = 0.0f; /* METROS */
static float g_last_e_ok        = 0.0f; /* METROS */

/* =======================
   Helpers NaN/Inf (sin libm)
   ======================= */
static inline uint8_t isfinite_f32(float x)
{
    uint32_t u;
    memcpy(&u, &x, sizeof(u));
    return (uint8_t)((u & 0x7F800000u) != 0x7F800000u); /* exp != 255 */
}

static inline uint8_t isnan_f32(float x)
{
    uint32_t u;
    memcpy(&u, &x, sizeof(u));
    return (uint8_t)(((u & 0x7F800000u) == 0x7F800000u) && ((u & 0x007FFFFFu) != 0u));
}

static inline float f32_hold_if_bad(float x, float prev_ok)
{
    return isfinite_f32(x) ? x : prev_ok;
}

static inline uint8_t vec3_all_finite(const float v[3])
{
    return (uint8_t)(isfinite_f32(v[0]) && isfinite_f32(v[1]) && isfinite_f32(v[2]));
}

static inline void vec3_hold_if_bad(float v[3], const float prev_ok[3])
{
    if (!vec3_all_finite(v)) {
        v[0] = prev_ok[0];
        v[1] = prev_ok[1];
        v[2] = prev_ok[2];
    }
}

static inline void vec3_update_ok_if_good(const float v[3], float prev_ok[3])
{
    if (vec3_all_finite(v)) {
        prev_ok[0] = v[0];
        prev_ok[1] = v[1];
        prev_ok[2] = v[2];
    }
}

/* =======================
   TF (IIR DF2T) hasta orden 10
   ======================= */
static float tf_b[CONTROL_TF_MAX_ORDER + 1u] = {0}; /* b0..b10 */
static float tf_a[CONTROL_TF_MAX_ORDER + 1u] = {0}; /* a0..a10 */
static float tf_w[CONTROL_TF_MAX_ORDER]      = {0}; /* w0..w9 */
static uint8  tf_order = 0u;                         /* orden efectivo 0..10 */
static float  tf_last_y_ok = 0.0f;                   /* último y TF válido */

/* =======================
   SS (3 estados)
   ======================= */
static float ss_A[9] = {0};   /* fila-major 3x3 */
static float ss_B[3] = {0};   /* 3x1 */
static float ss_C[3] = {0};   /* 1x3 */
static float ss_D    = 0.0f;
static float ss_L[3] = {0};   /* 3x1 */
static float ss_K[3] = {0};   /* 1x3 */
static float ss_Kx   = 0.0f;  /* Ki (modo I) o Kr (modo NOI) */

static float xhat[3] = {0.0f, 0.0f, 0.0f};  /* posterior */
static float zhat[3] = {0.0f, 0.0f, 0.0f};  /* prior (para ACT) */

/* backups “últimos buenos” */
static float xhat_ok[3] = {0.0f, 0.0f, 0.0f};
static float zhat_ok[3] = {0.0f, 0.0f, 0.0f};

/* =======================
   Auto-calibración u0 (estado)
   ======================= */
#if CONTROL_ENABLE_AUTOCAL
typedef enum {
    CALIB_IDLE = 0,
    CALIB_RAMP,
    CALIB_SETTLE,
    CALIB_DONE,
    CALIB_FAIL
} calib_state_t;

static calib_state_t g_calib_state   = CALIB_IDLE;

/* Calibración por niveles */
static float    g_calib_u_phy         = CONTROL_CALIB_START_US;
static float    g_calib_y_level0      = 0.0f;   /* METROS */
static uint16_t g_calib_hits          = 0u;
static uint16_t g_calib_level_samples = 0u;
static uint32_t g_calib_total_samples = 0u;

/* Estabilización post-calib */
static float    g_settle_y_prev       = 0.0f;   /* METROS */
static uint16_t g_settle_stable_cnt   = 0u;
static uint8_t  g_settle_first        = 1u;
#endif

/* =======================
   Helpers “clásicos” + NaN-aware
   ======================= */
static inline uint16_t u16_from_float_safe(float x, uint16_t prev_ok)
{
    if (!isfinite_f32(x)) return prev_ok;
    if (x <= 0.0f) return 0u;
    if (x >= 65535.0f) return 65535u;
    return (uint16_t)(x + 0.5f);
}

static inline uint8 tf_order_from_float(float x)
{
    if (!isfinite_f32(x)) return tf_order;
    int32_t o = (int32_t)(x + 0.5f);
    if (o < 0) o = 0;
    if (o > (int32_t)CONTROL_TF_MAX_ORDER) o = (int32_t)CONTROL_TF_MAX_ORDER;
    return (uint8)o;
}

static inline float dot3_cmsis(const float a[3], const float b[3])
{
    float out = 0.0f;
    arm_dot_prod_f32((float32_t*)a, (float32_t*)b, 3u, (float32_t*)&out);
    return out;
}

static inline float ss_yhat_from_xu(const float x[3], float u_cmd)
{
    float y = dot3_cmsis(ss_C, x) + ss_D * u_cmd;
    return isfinite_f32(y) ? y : 0.0f;
}

static inline void ss_predict_from_xu(const float x[3], float u_cmd, float z[3])
{
    if (!vec3_all_finite(x) || !isfinite_f32(u_cmd)) {
        z[0] = x[0]; z[1] = x[1]; z[2] = x[2];
        return;
    }

    float t0, t1, t2;

    arm_dot_prod_f32((float32_t*)&ss_A[0], (float32_t*)x, 3u, (float32_t*)&t0);
    arm_dot_prod_f32((float32_t*)&ss_A[3], (float32_t*)x, 3u, (float32_t*)&t1);
    arm_dot_prod_f32((float32_t*)&ss_A[6], (float32_t*)x, 3u, (float32_t*)&t2);

    t0 += ss_B[0] * u_cmd;
    t1 += ss_B[1] * u_cmd;
    t2 += ss_B[2] * u_cmd;

    if (!isfinite_f32(t0) || !isfinite_f32(t1) || !isfinite_f32(t2)) {
        z[0] = x[0]; z[1] = x[1]; z[2] = x[2];
        return;
    }

    z[0] = t0; z[1] = t1; z[2] = t2;
}

static inline float satf_safe(float u, float umin, float umax)
{
    if (!isfinite_f32(u)) {
        float h = g_last_u_cmd_ok;
        if (!isfinite_f32(h)) h = 0.0f;
        if (h > umax) return umax;
        if (h < umin) return umin;
        return h;
    }

    if (u > umax) return umax;
    if (u < umin) return umin;
    return u;
}

static inline uint8 ss_mode_has_integrator(uint8 impl)
{
    return (impl == UARTP_IMPL_SS_PRED_I) || (impl == UARTP_IMPL_SS_ACT_I);
}

static inline float ss_Kr_eff(void)
{
    return ss_mode_has_integrator((uint8)UARTP_Impl) ? 0.0f : ss_Kx;
}

static inline float ss_Ki_eff(void)
{
    return ss_mode_has_integrator((uint8)UARTP_Impl) ? ss_Kx : 0.0f;
}

/* Conversión: PWM físico (us) -> Δu (us) */
static inline float ucmd_from_uphy(float u_phy)
{
    if (!isfinite_f32(u_phy) || !isfinite_f32(g_u0_offset_us)) return 0.0f;
    return (u_phy - g_u0_offset_us);
}

/* Conversión: Δu (us) -> PWM físico (us) */
static inline float uphy_from_ucmd(float u_cmd)
{
    if (!isfinite_f32(u_cmd) || !isfinite_f32(g_u0_offset_us)) return CONTROL_SAT_MIN_IDLE;
    return (u_cmd + g_u0_offset_us);
}

/* Rango Δu permitido (dinámico) EN CONTROL */
static inline float cmd_min_dyn(void) { return (CONTROL_SAT_MIN - g_u0_offset_us); }
static inline float cmd_max_dyn(void) { return (CONTROL_SAT_MAX - g_u0_offset_us); }

/* y_rel y r_rel EN METROS */
static inline float y_rel_from_y(float y_phys_m) { return (y_phys_m - (float)CONTROL_DESC_MIN_Y_M); }
static inline float r_rel_from_r(float r_phys_m) { return (r_phys_m - (float)CONTROL_DESC_MIN_Y_M); }

/* --- clamp de referencia según modo --- */
static inline float clamp_ref(float r)
{
    r = f32_hold_if_bad(r, g_last_r_phys_ok);

    if (UARTP_Impl == UARTP_IMPL_OPENLOOP)
    {
#if CONTROL_OL_REF_IS_DELTA
        return satf_safe(r, cmd_min_dyn(), cmd_max_dyn());
#else
        if (r < CONTROL_OL_REF_MIN) return CONTROL_OL_REF_MIN;
        if (r > CONTROL_OL_REF_MAX) return CONTROL_OL_REF_MAX;
        return r;
#endif
    }
    else
    {
        /* CLOSED LOOP: r EN METROS */
        if (r < CONTROL_CL_REF_MIN) return CONTROL_CL_REF_MIN;
        if (r > CONTROL_CL_REF_MAX) return CONTROL_CL_REF_MAX;
        return r;
    }
}

/* =======================
   write_u (único actuador)
   Entrada: Δu (us)
   ======================= */
static inline void write_u(float u_cmd)
{
    if (!g_session_active)
    {
        g_u_cmd = ucmd_from_uphy(CONTROL_SAT_MIN_IDLE);
        g_u_out = CONTROL_SAT_MIN_IDLE;

        g_last_u_cmd_ok = g_u_cmd;
        g_last_u_out_ok = g_u_out;

        if (s_write_u_cb) s_write_u_cb(CONTROL_SAT_MIN_IDLE);
        return;
    }

    u_cmd = satf_safe(u_cmd, cmd_min_dyn(), cmd_max_dyn());

    g_u_cmd = u_cmd;
    if (isfinite_f32(g_u_cmd)) g_last_u_cmd_ok = g_u_cmd;
    else g_u_cmd = g_last_u_cmd_ok;

    float u_phy = uphy_from_ucmd(g_u_cmd);
    if (!isfinite_f32(u_phy)) {
        u_phy = g_last_u_out_ok;
    }

    if (u_phy > CONTROL_SAT_MAX) u_phy = CONTROL_SAT_MAX;
    if (u_phy < CONTROL_SAT_MIN) u_phy = CONTROL_SAT_MIN;

    g_u_out = u_phy;
    if (isfinite_f32(g_u_out)) g_last_u_out_ok = g_u_out;
    else g_u_out = g_last_u_out_ok;

    if (s_write_u_cb) s_write_u_cb(g_u_out);
}

void control_force_min(void)
{
    if (g_session_active) write_u(ucmd_from_uphy(CONTROL_SAT_MIN));
    else write_u(0.0f);
}

/* =======================
   Observadores (NaN-aware)
   ======================= */
static inline void ss_observer_pred_step(float y, float u_prev_cmd, float u_k_cmd)
{
    y          = f32_hold_if_bad(y, g_last_y_rel_ok);
    u_prev_cmd = f32_hold_if_bad(u_prev_cmd, g_last_u_cmd_ok);
    u_k_cmd    = f32_hold_if_bad(u_k_cmd, g_last_u_cmd_ok);

    vec3_hold_if_bad(xhat, xhat_ok);

    float yhat = ss_yhat_from_xu(xhat, u_prev_cmd);
    float innov = y - yhat;

    if (!isfinite_f32(innov)) return;

    float z[3];
    ss_predict_from_xu(xhat, u_k_cmd, z);

    float Linv[3];
    arm_scale_f32((float32_t*)ss_L, innov, (float32_t*)Linv, 3u);

    float xnew[3];
    arm_add_f32((float32_t*)z, (float32_t*)Linv, (float32_t*)xnew, 3u);

    vec3_hold_if_bad(xnew, xhat_ok);

    xhat[0] = xnew[0]; xhat[1] = xnew[1]; xhat[2] = xnew[2];
    vec3_update_ok_if_good(xhat, xhat_ok);
}

static inline void ss_observer_act_correct(float y, float u_prev_cmd)
{
    y          = f32_hold_if_bad(y, g_last_y_rel_ok);
    u_prev_cmd = f32_hold_if_bad(u_prev_cmd, g_last_u_cmd_ok);

    vec3_hold_if_bad(zhat, zhat_ok);

    float yzh = ss_yhat_from_xu(zhat, u_prev_cmd);
    float innov = y - yzh;

    if (!isfinite_f32(innov)) return;

    float Linv[3];
    arm_scale_f32((float32_t*)ss_L, innov, (float32_t*)Linv, 3u);

    float xnew[3];
    arm_add_f32((float32_t*)zhat, (float32_t*)Linv, (float32_t*)xnew, 3u);

    vec3_hold_if_bad(xnew, xhat_ok);

    xhat[0] = xnew[0]; xhat[1] = xnew[1]; xhat[2] = xnew[2];
    vec3_update_ok_if_good(xhat, xhat_ok);
}

static inline void ss_observer_act_predict(float u_k_cmd)
{
    u_k_cmd = f32_hold_if_bad(u_k_cmd, g_last_u_cmd_ok);
    vec3_hold_if_bad(xhat, xhat_ok);

    float znew[3];
    ss_predict_from_xu(xhat, u_k_cmd, znew);

    vec3_hold_if_bad(znew, zhat_ok);

    zhat[0] = znew[0]; zhat[1] = znew[1]; zhat[2] = znew[2];
    vec3_update_ok_if_good(zhat, zhat_ok);
}

/* =======================
   TF DF2T step (NaN-aware)
   ======================= */
static inline float tf_step(float x)
{
    const uint8 ord = tf_order;

    if (!isfinite_f32(x)) return tf_last_y_ok;

    if (ord == 0u) {
        float y0 = tf_b[0] * x;
        if (isfinite_f32(y0)) tf_last_y_ok = y0;
        return tf_last_y_ok;
    }

    float y = tf_b[0] * x + tf_w[0];

    if (!isfinite_f32(y)) return tf_last_y_ok;

    for (uint8 i = 0u; i < (uint8)(ord - 1u); i++) {
        float wi = tf_w[i + 1u] + tf_b[i + 1u] * x - tf_a[i + 1u] * y;
        if (!isfinite_f32(wi)) return tf_last_y_ok;
        tf_w[i] = wi;
    }

    {
        float wlast = tf_b[ord] * x - tf_a[ord] * y;
        if (!isfinite_f32(wlast)) return tf_last_y_ok;
        tf_w[ord - 1u] = wlast;
    }

    tf_last_y_ok = y;
    return y;
}

/* =======================
   IO / ISR glue
   ======================= */
void control_register_io(control_sample_isr_fn_t sample_isr,
                         control_write_u_fn_t    write_u_fn)
{
    s_sample_isr_cb = sample_isr;
    s_write_u_cb    = write_u_fn;
}

void control_on_sample_isr(void)
{
    if (s_sample_isr_cb) {
        s_sample_isr_cb();
    } else {
        controlPin_Write(1);
        control_sample_pending = 1u;
    }
}

/* y EN METROS */
void control_sample_isr_push(float y_m)
{
    controlPin_Write(1);
    control_last_y = y_m;
    control_sample_pending = 1u;
}

/* =======================
   Referencia y Ts (NaN-aware)
   ======================= */
void control_set_reference(float r_m)
{
    if (!isfinite_f32(r_m)) return;

    uint8 intr = CyEnterCriticalSection();
    g_ref = r_m;
    control_ref_v = r_m;
    CyExitCriticalSection(intr);

    g_last_r_phys_ok = r_m;
}

float control_get_reference(void)
{
    return g_ref;
}

void control_set_sample_time(float Ts)
{
    if (!isfinite_f32(Ts) || Ts <= 0.0f) Ts = 1.0f;
    g_Ts = Ts;
}

float control_get_sample_time(void)
{
    return g_Ts;
}

/* =======================
   START: 1x por sesión
   ======================= */
void control_start(float ref0_m)
{
    control_set_reference(ref0_m);

    if (g_session_active) return;
    g_session_active = 1u;

    xhat[0] = 0.0f; xhat[1] = 0.0f; xhat[2] = 0.0f;
    zhat[0] = 0.0f; zhat[1] = 0.0f; zhat[2] = 0.0f;

    xhat_ok[0]=0.0f; xhat_ok[1]=0.0f; xhat_ok[2]=0.0f;
    zhat_ok[0]=0.0f; zhat_ok[1]=0.0f; zhat_ok[2]=0.0f;

    memset(tf_w, 0, sizeof(tf_w));
    tf_last_y_ok = 0.0f;

    g_vint = 0.0f;

#if CONTROL_ENABLE_AUTOCAL
    if (!g_u0_valid)
    {
        g_calib_state          = CALIB_RAMP;
        g_calib_u_phy          = CONTROL_CALIB_START_US;
        g_calib_hits           = 0u;
        g_calib_level_samples  = 0u;
        g_calib_total_samples  = 0u;
        g_calib_y_level0       = 0.0f;

        g_u0_offset_us = CONTROL_U_OFFSET;

        write_u(ucmd_from_uphy(g_calib_u_phy));
        return;
    }
#endif

    /* bumpless desde u actual */
    {
        float u0_cmd = ucmd_from_uphy(g_u_out);

        if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
            float Ki = ss_Ki_eff();
            if (Ki != 0.0f && isfinite_f32(Ki)) g_vint = u0_cmd / Ki;
        }

        write_u(u0_cmd);

        if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
            ss_predict_from_xu(xhat, u0_cmd, zhat);
            vec3_update_ok_if_good(zhat, zhat_ok);
        }
    }
}

/* =======================
   STOP suave
   ======================= */
bool control_stop_suave_step(void)
{
    float u_target_cmd = (float)CONTROL_DESC_TARGET_CMD_US;
    u_target_cmd = satf_safe(u_target_cmd, cmd_min_dyn(), cmd_max_dyn());

    for (uint32_t i = 0u; i < (uint32_t)CONTROL_DESC_MAX_ITERS; i++)
    {
        float u = g_u_cmd;

        if (u < u_target_cmd) {
            break;
        } else if (u > u_target_cmd) {
            u -= CONTROL_DESC_STEP_US;
            if (u < u_target_cmd) u = u_target_cmd;
            write_u(u);
        } else {
            break;
        }

        CyDelayUs(CONTROL_DESC_DELAY_US);
    }

    write_u(u_target_cmd);

    uint32_t hold_loops = (CONTROL_DESC_HOLD_MS * 1000u) / CONTROL_DESC_HOLD_POLL_US;
    if (hold_loops < 1u) hold_loops = 1u;

    for (uint32_t k = 0u; k < hold_loops; k++) {
        write_u(u_target_cmd);
        CyDelayUs(CONTROL_DESC_HOLD_POLL_US);
    }

    g_session_active = 0u;
    g_u0_valid       = 0u;
#if CONTROL_ENABLE_AUTOCAL
    g_calib_state    = CALIB_IDLE;
#endif

    write_u(0.0f);
    return true;
}

/* =======================
   Carga coeficientes (PAQUETE FIJO 25 floats)
   ======================= */
void control_apply_tf(const float* c, uint16_t n)
{
    if (!c || n < 25u) return;

    for (uint8 i = 0u; i < (uint8)(CONTROL_TF_MAX_ORDER + 1u); i++) {
        if (isfinite_f32(c[i]))         tf_b[i] = c[i];
        if (isfinite_f32(c[i + 11u]))   tf_a[i] = c[i + 11u];
    }

    tf_order = tf_order_from_float(c[22]);

    for (uint8 i = (uint8)(tf_order + 1u); i < (uint8)(CONTROL_TF_MAX_ORDER + 1u); i++) {
        tf_b[i] = 0.0f;
        tf_a[i] = 0.0f;
    }

    if (!isfinite_f32(tf_a[0]) || tf_a[0] == 0.0f) {
        tf_a[0] = 1.0f;
    }
    if (tf_a[0] != 1.0f) {
        const float a0 = tf_a[0];
        if (isfinite_f32(a0) && a0 != 0.0f) {
            for (uint8 i = 0u; i <= tf_order; i++) tf_b[i] /= a0;
            for (uint8 i = 1u; i <= tf_order; i++) tf_a[i] /= a0;
            tf_a[0] = 1.0f;
        } else {
            tf_a[0] = 1.0f;
        }
    }

    UARTP_StreamN = u16_from_float_safe(c[23], UARTP_StreamN);
    if (isfinite_f32(c[24])) UARTP_StreamFsHz = c[24];

    memset(tf_w, 0, sizeof(tf_w));
    tf_last_y_ok = 0.0f;
}

void control_apply_ss(const float* c, uint16_t n)
{
    if (!c || n < 25u) return;

    for (uint8 i=0u; i<9u; i++)  if (isfinite_f32(c[i])) ss_A[i] = c[i];
    for (uint8 i=0u; i<3u; i++)  if (isfinite_f32(c[9u+i]))  ss_B[i] = c[9u+i];
    for (uint8 i=0u; i<3u; i++)  if (isfinite_f32(c[12u+i])) ss_C[i] = c[12u+i];
    if (isfinite_f32(c[15])) ss_D = c[15];
    for (uint8 i=0u; i<3u; i++)  if (isfinite_f32(c[16u+i])) ss_L[i] = c[16u+i];
    for (uint8 i=0u; i<3u; i++)  if (isfinite_f32(c[19u+i])) ss_K[i] = c[19u+i];
    if (isfinite_f32(c[22])) ss_Kx = c[22];

    UARTP_StreamN = u16_from_float_safe(c[23], UARTP_StreamN);
    if (isfinite_f32(c[24])) UARTP_StreamFsHz = c[24];

    xhat[0]=0.0f; xhat[1]=0.0f; xhat[2]=0.0f;
    zhat[0]=0.0f; zhat[1]=0.0f; zhat[2]=0.0f;
    xhat_ok[0]=0.0f; xhat_ok[1]=0.0f; xhat_ok[2]=0.0f;
    zhat_ok[0]=0.0f; zhat_ok[1]=0.0f; zhat_ok[2]=0.0f;

    g_vint = 0.0f;
}

/* =======================
   CONTROL STEP (main)
   ======================= */
void control_step(void)
{
    uint8 intr = CyEnterCriticalSection();

    float y_phys = control_last_y; /* METROS */
    control_sample_pending = 0u;

    float r_phys = g_ref;          /* METROS */

    float u_prev_cmd = ucmd_from_uphy(g_u_out);

    CyExitCriticalSection(intr);

    /* -------- NaN-aware INPUTS -------- */
    y_phys = f32_hold_if_bad(y_phys, g_last_y_phys_ok);
    r_phys = f32_hold_if_bad(r_phys, g_last_r_phys_ok);

    if (isfinite_f32(y_phys)) g_last_y_phys_ok = y_phys;
    if (isfinite_f32(r_phys)) g_last_r_phys_ok = r_phys;

    u_prev_cmd = f32_hold_if_bad(u_prev_cmd, g_last_u_cmd_ok);

    /* clamp ref ya NaN-aware */
    r_phys = clamp_ref(r_phys);

#if CONTROL_ENABLE_AUTOCAL
    if (g_calib_state == CALIB_RAMP)
    {
        g_calib_total_samples++;

        if (g_calib_level_samples == 0u) {
            g_calib_y_level0 = y_phys;
            g_calib_hits = 0u;
        }

        g_calib_level_samples++;

        float dy = y_phys - g_calib_y_level0;
        if (dy < 0) dy = -dy;

        if (dy >= CONTROL_CALIB_DY_M) {
            if (g_calib_hits < 65535u) g_calib_hits++;
        } else {
            g_calib_hits = 0u;
        }

        if (g_calib_hits >= (uint16_t)CONTROL_CALIB_MIN_HITS)
        {
            g_u0_offset_us = g_calib_u_phy;
            g_u0_valid     = 1u;

            write_u(0.0f);

            g_calib_state       = CALIB_SETTLE;
            g_settle_first      = 1u;
            g_settle_stable_cnt = 0u;

            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        if (g_calib_total_samples >= CONTROL_CALIB_MAX_SAMPLES || g_calib_u_phy >= CONTROL_CALIB_MAX_US)
        {
            g_calib_state   = CALIB_FAIL;
            g_u0_offset_us  = CONTROL_U_OFFSET;
            g_u0_valid      = 0u;

            write_u(0.0f);
            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        if (g_calib_level_samples < (uint16_t)CONTROL_CALIB_HOLD_SAMPLES)
        {
            write_u(ucmd_from_uphy(g_calib_u_phy));
            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        g_calib_u_phy += CONTROL_CALIB_STEP_US;
        if (g_calib_u_phy > CONTROL_CALIB_MAX_US) g_calib_u_phy = CONTROL_CALIB_MAX_US;

        g_calib_level_samples = 0u;

        write_u(ucmd_from_uphy(g_calib_u_phy));
        UARTP_Telemetry_Push(g_u_cmd, y_phys);
        controlPin_Write(0);
        return;
    }

    if (g_calib_state == CALIB_SETTLE)
    {
        write_u(0.0f);

        if (g_settle_first) {
            g_settle_y_prev = y_phys;
            g_settle_first = 0u;
            g_settle_stable_cnt = 0u;
            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        float dy = y_phys - g_settle_y_prev;
        if (dy < 0) dy = -dy;
        g_settle_y_prev = y_phys;

        if (dy <= (float)CONTROL_U0_SETTLE_DY_M) {
            if (g_settle_stable_cnt < 65535u) g_settle_stable_cnt++;
        } else {
            g_settle_stable_cnt = 0u;
        }

        if (g_settle_stable_cnt >= (uint16_t)CONTROL_U0_SETTLE_SAMPLES)
        {
            g_calib_state = CALIB_DONE;

            if (UARTP_Impl != UARTP_IMPL_TF && UARTP_Impl != UARTP_IMPL_OPENLOOP) {
                ss_predict_from_xu(xhat, 0.0f, zhat);
                vec3_update_ok_if_good(zhat, zhat_ok);
            }

            UARTP_Telemetry_Push(g_u_cmd, y_phys);
            controlPin_Write(0);
            return;
        }

        UARTP_Telemetry_Push(g_u_cmd, y_phys);
        controlPin_Write(0);
        return;
    }
#endif

    /* -------- rels (METROS) -------- */
    float y_rel = y_rel_from_y(y_phys);
    float r_rel = r_rel_from_r(r_phys);

    y_rel = f32_hold_if_bad(y_rel, g_last_y_rel_ok);
    r_rel = f32_hold_if_bad(r_rel, g_last_r_rel_ok);

    if (isfinite_f32(y_rel)) g_last_y_rel_ok = y_rel;
    if (isfinite_f32(r_rel)) g_last_r_rel_ok = r_rel;

    /* error en METROS */
    {
        float e = r_rel - y_rel;
        e = f32_hold_if_bad(e, g_last_e_ok);
        if (isfinite_f32(e)) g_last_e_ok = e;
    }

    switch (UARTP_Impl)
    {
        case UARTP_IMPL_OPENLOOP:
        {
#if CONTROL_OL_REF_IS_DELTA
            float u_cmd = satf_safe(r_phys, cmd_min_dyn(), cmd_max_dyn());
            write_u(u_cmd);
#else
            float u_cmd = ucmd_from_uphy(r_phys);
            write_u(u_cmd);
#endif
        } break;

        case UARTP_IMPL_TF:
        {
            float e = g_last_e_ok;
            float u_cmd = tf_step(e);
            write_u(u_cmd);
        } break;

        case UARTP_IMPL_SS_PRED_NOI:
        case UARTP_IMPL_SS_PRED_I:
        {
            uint8 has_i = ss_mode_has_integrator((uint8)UARTP_Impl);

            if (has_i) {
                float e = g_last_e_ok;
                if (isfinite_f32(e) && isfinite_f32(g_vint)) g_vint += e;
                else if (!isfinite_f32(g_vint)) g_vint = 0.0f;
            }

            vec3_hold_if_bad(xhat, xhat_ok);

            float u_cmd = ss_Kr_eff()*r_rel + ss_Ki_eff()*g_vint - dot3_cmsis(ss_K, xhat);
            u_cmd = f32_hold_if_bad(u_cmd, g_last_u_cmd_ok);

            write_u(u_cmd);

            float u_k_cmd = ucmd_from_uphy(g_u_out);
            ss_observer_pred_step(y_rel, u_prev_cmd, u_k_cmd);
        } break;

        case UARTP_IMPL_SS_ACT_NOI:
        case UARTP_IMPL_SS_ACT_I:
        default:
        {
            uint8 has_i = ss_mode_has_integrator((uint8)UARTP_Impl);

            ss_observer_act_correct(y_rel, u_prev_cmd);

            if (has_i) {
                float e = g_last_e_ok;
                if (isfinite_f32(e) && isfinite_f32(g_vint)) g_vint += e;
                else if (!isfinite_f32(g_vint)) g_vint = 0.0f;
            }

            vec3_hold_if_bad(xhat, xhat_ok);

            float u_cmd = ss_Kr_eff()*r_rel + ss_Ki_eff()*g_vint - dot3_cmsis(ss_K, xhat);
            u_cmd = f32_hold_if_bad(u_cmd, g_last_u_cmd_ok);

            write_u(u_cmd);

            float u_k_cmd = ucmd_from_uphy(g_u_out);
            ss_observer_act_predict(u_k_cmd);
        } break;
    }

    /* Telemetría NaN-aware: u = Δu [us], y = METROS */
    {
        float u_send = isfinite_f32(g_u_cmd) ? g_u_cmd : g_last_u_cmd_ok;
        float y_send = isfinite_f32(y_phys) ? y_phys : g_last_y_phys_ok;
        UARTP_Telemetry_Push(u_send, y_send);
    }

    controlPin_Write(0);
}
