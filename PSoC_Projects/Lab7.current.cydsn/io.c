#include "project.h"
#include "config.h"
#include "io.h"

uint8_t io_volt_to_dac(float v)
{
    if (v < SAT_MIN) v = SAT_MIN;
    if (v > SAT_MAX) v = SAT_MAX;
    float norm = v / SAT_MAX;     /* 0..1 */
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return (uint8_t)(norm * 255.0f);
}

void io_init(void)
{
    Opa_dac_Start();
    Opa_stage1_Start();
    Opa_vdda_2_Start();
    Opa_stage2_Start();
    PGA_ref_Start();

    VDAC8_Start();
    VDAC8_ref_Start();

    VADC_Start();
}

float io_adc_read_volts(void)
{
    /* Bloqueante corto (ya está convertido por el timer/ISR) */
    return (float)VADC_CountsTo_Volts(VADC_GetResult16());
}

void io_dac_write(float volts)
{
    VDAC8_SetValue(io_volt_to_dac(volts));
}

void io_ref_write(float volts)
{
    VDAC8_ref_SetValue(io_volt_to_dac(volts));
}
