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

/* ========= Parámetros del modelo (DISCRETO, en AC) =========
   Copiados de tu export bueno (coherentes con la simulación):
   A = [ 0.932581   0
        -1.145618  0.455938 ]
   B = [ -0.117824
          0.080093 ]
   C = [ 0  1 ]
   K = [ 2.907022  1.636048 ]
   K0 = 0.569878
   Ke (L_current) = [ -0.503374
                       0.988519 ]
*/
static const float a11 = 0.932581f, a12 = 0.000000f;
static const float a21 = -1.145618f, a22 = 0.455938f;
static const float b1  = -0.117824f, b2  = 0.080093f;

static const float k1  = 2.907022f,  k2  = 1.636048f;
static const float k0  = 0.569878f;

static const float l1  = -0.503374f, l2  = 0.988519f;

/* ========= Estado global ========= */
static volatile uint8_t tick_flag = 0;
static volatile float   ref_ac    = 0.0f;   // referencia en AC (±A/2)
static float xhat1 = 0.0f, xhat2 = 0.0f;    // estimador 2×1

/* Para probar sin UART: amplitud fija A_AMPL */
static const float ref_max =  0.5f * A_AMPL;
static const float ref_min = -0.5f * A_AMPL;

/* ========= ISR de tick (NO des/clear dentro de la ISR) ========= */
CY_ISR(adquirirMuestra)
{
    // referencia por pin (AC)
    ref_ac = ref_state_Read() ? ref_max : ref_min;
    // DAC de referencia (para el osciloscopio)
    VDAC8_ref_SetValue(volt_to_dac(ref_ac + VDDA2));

    // marcar que hay que ejecutar un paso de control
    tick_flag = 1u;

    // NO llamar Disable/Enable/ClearPending aquí.
}

/* ========= main ========= */
int main(void)
{
    CyGlobalIntEnable;

    /* Analógico / DAC / ADC */
    Opa_dac_Start();
    Opa_stage1_Start();
    Opa_vdda_2_Start();
    Opa_stage2_Start();

    VDAC8_Start();
    VDAC8_ref_Start();

    VADC_Start();  // Configurar el ADC en "Firmware Trigger" (no free run)

    /* Timer + ISR */
    isr_tick_StartEx(adquirirMuestra);
    timer_ref_Start();     // fija tu Ts

    for (;;)
    {
        if (tick_flag)
        {
            tick_flag = 0u;

            /* ===== 1) Control “CURRENT”: u[k] con xhat[k] =====
               u_ac = k0*ref - K*xhat
            */
            float u_ac = k0 * ref_ac - (k1 * xhat1 + k2 * xhat2);

            /* ===== 2) Aplicar esfuerzo (físico) y saturar ===== */
            float u_phys = sat_u_phys(u_ac + VDDA2);
            VDAC8_SetValue(volt_to_dac(u_phys));

            /* u aplicado en AC (post-sat) */
            float u_ac_apl = u_phys - VDDA2;

            /* ===== 3) Predicción a k+1: z = A*xhat + B*u_aplicado ===== */
            float z1 = a11 * xhat1 + a12 * xhat2 + b1 * u_ac_apl;
            float z2 = a21 * xhat1 + a22 * xhat2 + b2 * u_ac_apl;

            /* ===== 4) Medición y[k+1] (AC) ===== */
            VADC_StartConvert();
            while (VADC_IsEndConversion(VADC_RETURN_STATUS) == 0) { /* busy wait corto */ }
            float y_ac = VADC_CountsTo_Volts(VADC_GetResult16());   // tu hardware ya entrega AC

            /* ===== 5) Corrección “CURRENT”: xhat = z + L*(y - C*z) =====
               C = [0 1] ⇒ y_hat = z2
            */
            float innov = y_ac - z2;
            xhat1 = z1 + l1 * innov;
            xhat2 = z2 + l2 * innov;
        }
    }
}
