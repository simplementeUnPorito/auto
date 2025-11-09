#include "project.h"
#include "config.h"   // SAT_MIN, SAT_MAX, VDDA2, A_AMPL

/* ========= Helpers ========= */
static inline uint8_t volt_to_dac(float v)
{
    if (v < SAT_MIN) v = SAT_MIN;
    if (v > SAT_MAX) v = SAT_MAX;
    float n = v / SAT_MAX; if (n < 0) n = 0; if (n > 1) n = 1;
    return (uint8_t)(255.0f * n);
}
static inline float sat_u_phys(float u_phys)
{
    if (u_phys > SAT_MAX) u_phys = SAT_MAX;
    if (u_phys < SAT_MIN) u_phys = SAT_MIN;
    return u_phys;
}

/* ========= Modelo discreto ========= */
static float a11 = 0.932581f, a12 = 0.000000f;
static float a21 = -1.145618f, a22 = 0.455938f;
static float b1  = -0.117824f, b2  = 0.080093f;
/* C = [0 1] */

/* ========= Ganancias =========
   u[k] = k0 * v[k]  -  (k1 * xhat1 + k2 * xhat2)
   v[k] = v[k-1] + (ref[k] - y[k])
   (esto es tu diseño con integrador)
*/
static float k1 = -1.0207f;    // sobre xhat1
static float k2 = 0.6242f;    // sobre xhat2
static float k0 = 0.228f;    // sobre el integrador

/* ========= Observador (actual) =========
   xhat = z + L*(y - C*z)
*/
static float l1 = -0.503374f;
static float l2 =  0.988519f;

/* ========= Estado global ========= */
static volatile uint8_t tick_flag = 0;
static volatile float   ref_ac    = 0.0f;
static float xhat1 = 0.0f, xhat2 = 0.0f;
static float v_int = 0.0f;                 // <-- integrador

static float ref_max =  0.5f * A_AMPL;
static float ref_min = -0.5f * A_AMPL;

/* ========= ISR ========= */
CY_ISR(adquirirMuestra)
{
    ref_ac = ref_state_Read() ? ref_max : ref_min;
    VDAC8_ref_SetValue(volt_to_dac(ref_ac + VDDA2));
    tick_flag = 1u;
}



/* ========= main ========= */
int main(void)
{
    CyGlobalIntEnable;

    /* Analógico */
    Opa_dac_Start();
    Opa_stage1_Start();
    Opa_vdda_2_Start();
    Opa_stage2_Start();

    VDAC8_Start();
    VDAC8_ref_Start();

    VADC_Start();  // firmware trigger

    isr_tick_StartEx(adquirirMuestra);
    timer_ref_Start();

    for(;;)
    {
        if (tick_flag)
        {
            tick_flag = 0u;

            /* ===== (a) Medir y[k] para el integrador ===== */
            float y_k = VADC_CountsTo_Volts(VADC_GetResult16());   // ya AC

            /* ===== (b) Actualizar integrador: v[k] = v[k-1] + (r - y) ===== */
            float err_ry = ref_ac - y_k;
            v_int += err_ry;
            /* opcional anti-windup
            if (v_int > 5.0f)  v_int = 5.0f;
            if (v_int < -5.0f) v_int = -5.0f;
            */

            /* ===== (c) Control con integrador =====
               u_ac = k0 * v_int  -  (k1 * xhat1 + k2 * xhat2)
            */
            float u_ac = k0 * v_int - (k1 * xhat1 + k2 * xhat2);

            /* ===== (d) Enviar al DAC (con offset) ===== */
            VDAC8_SetValue(volt_to_dac(sat_u_phys(u_ac + VDDA2)));


            /* ===== (e) Predicción: z = A*xhat + B*u ===== */
            float z1 = a11 * xhat1 + a12 * xhat2 + b1 * u_ac;
            float z2 = a21 * xhat1 + a22 * xhat2 + b2 * u_ac;


            /* ===== (g) Corrección: xhat = z + L*(y[k+1] - C*z) ===== */
            float innov = y_k - z2;         // C = [0 1]
            xhat1 = z1 + l1 * innov;
            xhat2 = z2 + l2 * innov;
        }
    }
}



