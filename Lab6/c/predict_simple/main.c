#include "project.h"
#include "config.h"

/* ---------- Helpers ---------- */
static inline uint8_t volt_to_dac(float v)
{
    if (v < SAT_MIN) v = SAT_MIN;
    if (v > SAT_MAX) v = SAT_MAX;
    float n = v / SAT_MAX; if (n < 0) n = 0; if (n > 1) n = 1;
    return (uint8_t)(255.0f * n);
}
static inline float sat_u_phys(float u)
{
    if (u > SAT_MAX) u = SAT_MAX;
    if (u < SAT_MIN) u = SAT_MIN;
    return u;
}

/* ---------- Modelo discreto ---------- */
float a11 = 0.932581f, a12 = 0.000000f;
float a21 = -1.145618f, a22 = 0.455938f;
float b1  = -0.117824f, b2  = 0.080093f;
float c1  = 0.0f,       c2  = 1.0f;

float k1  = 2.907022f,  k2  = 1.636048f;
float k0  = 0.569878f;

float l1  = -0.503374f, l2  = 0.988519f;

/* ---------- Estado ---------- */
static volatile uint8_t tick_flag = 0;
static volatile float ref_ac = 0.0f;
static float xhat1 = 0.0f, xhat2 = 0.0f;

static const float ref_max =  0.5f * A_AMPL;
static const float ref_min = -0.5f * A_AMPL;

/* ---------- ISR de muestreo ---------- */
CY_ISR(adquirirMuestra)
{
    /* referencia AC según el switch */
    ref_ac = ref_state_Read() ? ref_max : ref_min;
    /* DAC auxiliar para scope */
    VDAC8_ref_SetValue(volt_to_dac(ref_ac + VDDA2));
    tick_flag = 1u;
}

/* ---------- MAIN ---------- */
int main(void)
{
    CyGlobalIntEnable;

    /* Front-end analógico */
    Opa_dac_Start();
    Opa_stage1_Start();
    Opa_vdda_2_Start();
    Opa_stage2_Start();

    VDAC8_Start();
    VDAC8_ref_Start();

    VADC_Start();  // Firmware trigger (NO free-run)

    muestra_disponible_StartEx(adquirirMuestra);
    timer_ref_Start(); // genera Ts

    for (;;)
    {
        if (tick_flag)
        {
            tick_flag = 0u;


            float u_ac = k0 * ref_ac - (k1 * xhat1 + k2 * xhat2);

            /* 2) Leer y[k] (AC) */
            VADC_StartConvert();
            while (VADC_IsEndConversion(VADC_RETURN_STATUS) == 0) {}
            float y_ac = VADC_CountsTo_Volts(VADC_GetResult16());


            float y_hat = c1 * xhat1 + c2 * xhat2;
            float err   = y_ac - y_hat;
            float x1n   = a11 * xhat1 + a12 * xhat2 + b1 * u_ac + l1 * err;
            float x2n   = a21 * xhat1 + a22 * xhat2 + b2 * u_ac + l2 * err;
            xhat1 = x1n;
            xhat2 = x2n;

            /* 4) DAC físico (con saturación y offset) */
            float u_phys = sat_u_phys(u_ac + VDDA2);
            VDAC8_SetValue(volt_to_dac(u_phys));
        }
    }
}
