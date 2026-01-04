#include "project.h"
#include <stdint.h>

/* ==============================
   CONFIG
   ============================== */
#define VDAC_FS_V        (4.08f)
#define VDAC_MAX_CODE    (255.0f)

/* >>> salida fija en lazo abierto <<< */
#define U_OPENLOOP_V     (1.00f)

/* muestreo / filtro simple para lectura */
#define ADC_AVG_ALPHA    (0.01f)   /* 0.01 => promedio suave */

/* ==============================
   Helpers
   ============================== */
static inline uint8 clamp_u8_from_volts(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > VDAC_FS_V) v = VDAC_FS_V;

    float code = (v * VDAC_MAX_CODE) / VDAC_FS_V;
    if (code < 0.0f)   code = 0.0f;
    if (code > 255.0f) code = 255.0f;

    return (uint8)(code + 0.5f);
}

/* Variables para “ver” en runtime (aunque no tengas debugger, te sirve luego para logs) */
volatile float g_y_adc_v      = 0.0f;  /* lectura instantánea */
volatile float g_y_adc_v_filt = 0.0f;  /* lectura filtrada */

int main(void)
{
    CyGlobalIntEnable;

    /* Start HW */
    VDAC8_esfuerzo_Start();
    follower_Start();              /* si tu esquema usa buffer/follower */
    ADC_muestra_Start();
    

    /* Poner el DAC fijo en 1V (open-loop) */
    VDAC8_esfuerzo_SetValue(clamp_u8_from_volts(U_OPENLOOP_V));

    /* LED “alive” */
    LED_BUILTIN_Write(1);

    for (;;)
    {
        
        CyDelay(200);
    }
}
