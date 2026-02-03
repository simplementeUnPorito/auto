#include "project.h"
#include "uartp_sw.h"
#include "control_app.h"
#include "tfmini_psoc.h"
#include <stdint.h>

#define TFMINI_FPS_DEFAULT (100u)



/* Actuador: por ahora NO DAC (implementalo a tu actuador real) */
static void my_write_u(float u)
{
    (void)u;
}

/* Callbacks para UARTP (ya no usan timer_sampling) */
static void my_sampling_enable(void)      { (void)tfmini_enable(); }
static void my_sampling_disable(void)     { (void)tfmini_disable(); }
static void my_sampling_clear_flags(void) { tfmini_clear_flags(); }

static void my_sampling_change_fs(float fs_hz)
{
    if (!(fs_hz > 0.0f)) return;

    uint16_t fps = (uint16_t)(fs_hz + 0.5f);
    if (fps < 1u) fps = 1u;
    if (fps > 1000u) fps = 1000u;

    if (tfmini_set_fps(fps)) {
        control_set_sample_time(1.0f / (float)fps);
    }
}

int main(void)
{
    CyGlobalIntEnable;

    //timer_led_Start();
    timer_uart_Start();

    control_register_io(NULL, my_write_u);
    control_set_sample_time(1.0f / (float)TFMINI_FPS_DEFAULT);

    tfmini_init();
    (void)tfmini_set_fps(TFMINI_FPS_DEFAULT);
    (void)tfmini_enable();

    uartp_cfg_t cfg = {
        .stop_step = control_stop_suave_step,
        .start     = control_start,
        .force_min = control_force_min,
        .apply_tf  = control_apply_tf,
        .apply_ss  = control_apply_ss,

        .sampling_enable      = my_sampling_enable,
        .sampling_disable     = my_sampling_disable,
        .sampling_change_fs   = my_sampling_change_fs,
        .sampling_clear_flags = my_sampling_clear_flags
    };
    UARTP_Init(&cfg);

    for (;;)
    {
        switch (UARTP_SysMode)
        {
            case UARTP_SYS_COMMAND:
                (void)UARTP_ProcessOnce();
                //registro_led_Write(1);
                break;

            case UARTP_SYS_CONTROL:
            {
                uint16_t y_cm;

                if (tfmini_pop_cm(&y_cm)) {
                    control_sample_isr_push((float)y_cm);
                }

                if (control_sample_pending) {
                    control_sample_pending = 0u;
                    control_step();
                }

                UARTP_ControlTick();
                //registro_led_Write(2);
                break;
            }

            default:
                UARTP_EnterCommandMode();
                //registro_led_Write(3);
                break;
        }
    }
}
