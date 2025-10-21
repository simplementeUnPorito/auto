#include "project.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "mat.h"       // <-- tu librería de matrices

/* ==== Prototipos ==== */
CY_ISR_PROTO(adquirirMuestra);
static void pc_process_rx(void);
static void handle_command(const char *line);
static void set_ref_value(float *target, float val, const char *name);
static void set_ref_period(uint16_t period);
static void ref_status(void);

/* ==== Config general / límites ==== */
#define SAT_MIN        (0.0f)
#define SAT_MAX        (4.048f)
#define MIDDLE_VOLTAGE ((SAT_MAX - SAT_MIN)/2.0f)
#define VDDA2          (2.5f)
#define RX_BUF_SZ 64
#define A 0.4f
/* ==== Estado global ==== */
volatile char  flag    = '0';
volatile double ref_max = (float)(A/2);//(MIDDLE_VOLTAGE + 0.1f);
volatile double ref_min = (float)(-1*A/2);//(MIDDLE_VOLTAGE - 0.1f);
volatile double ref     = 0.0f;
volatile Mat X;
volatile double e       = 0.0f;

/* Modo de operación: 0 = closed loop (PID), 1 = open loop (DAC = ref) */
enum { MODE_CLOSED = 0, MODE_OPEN = 1 };
volatile uint8 mode = MODE_CLOSED;

/* ==== UART RX line buffer ==== */
static char   rx_buf[RX_BUF_SZ];
static uint8  rx_len = 0;

/* ==== Helpers sin libm ==== */
static long my_lroundf(float x) { return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f); }

/* Mapear voltaje a 0..255 con clamps a [SAT_MIN, SAT_MAX] */
static inline uint8 volt_to_dac(float v)
{
    if (v < SAT_MIN) v = SAT_MIN;
    if (v > SAT_MAX) v = SAT_MAX;
    float norm = v / SAT_MAX;               /* 0..1 */
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return (uint8)(norm * 255.0f);
}

/* ==== ISR de muestra ==== */
CY_ISR(adquirirMuestra)
{
    /* Limpiar fuente de interrupción de “muestra disponible” */
    muestra_disponible_ClearPending();

    /* Conmutar referencia según pin/switch (1 -> max, 0 -> min) */
    ref = (ref_state_Read()) ? ref_max : ref_min;
    
    VDAC8_ref_SetValue(volt_to_dac(ref+VDDA2));

    /* Leer ADC y calcular error */
    float x1 = VADC_X1_CountsTo_Volts(VADC_X1_GetResult16());
    float x2 = VADC_X2_CountsTo_Volts(VADC_X2_GetResult16());
    X.d[0][0] = x1;
    X.d[1][0] = x2;
    //e = ref - muestra;

    flag = '1';

    /* Según tu diseño, deshabilitás y re-habilitás en el main */
    muestra_disponible_Disable();
}


/* Hook del esfuerzo: respeta el modo (open/closed) */
static inline void actualizarEsfuerzo(void)
{
    const Mat K = {.filas = 1, .columnas = 2,
             .d = {{4.0979, 6.4912}}};//.d = {{2.6952,6.5662}}};//
    Mat X_copy  = {.filas = 2, .columnas = 1,
                    .d = {{0.0},
                          {0.0}}};
    Mat U_unsat = {.filas = 1, .columnas = 1,
             .d = {{0.0}}};
    
    Mat K0 = {.filas = 1, .columnas = 1,
            .d = {{3.3618}}};//.d = {{4.8505}}};//
    Mat R = {.filas = 1, .columnas = 1,
             .d = {{0.0}}};
    Mat KX= {.filas = 1, .columnas = 1,
             .d = {{0.0}}};
    Mat K0R = {.filas = 1, .columnas = 1,
             .d = {{0.0}}};;
    R.d[0][0] = ref; //Calculamos todo en AC
    if (mode == MODE_OPEN) {
        /* === LAZO ABIERTO: el DAC sigue la referencia (ref_min / ref_max) === */
        VDAC8_SetValue(volt_to_dac(ref));
        return;
    }
   
    
    mat_copy_volatile(&X_copy,&X);
    mat_mul(&K,&X_copy,&KX);
    mat_mul(&K0,&R,&K0R);
    mat_sut(&K0R,&KX,&U_unsat);
    U_unsat.d[0][0]+=VDDA2;
    float u_sat;
    if (U_unsat.d[0][0] > SAT_MAX)      u_sat = SAT_MAX;
    else if (U_unsat.d[0][0] < SAT_MIN) u_sat = SAT_MIN;
    else                        u_sat = U_unsat.d[0][0];
    VDAC8_SetValue(volt_to_dac(u_sat));
//    /* === LAZO CERRADO: controlador PID discreto === */
//    static float u_1 = 0.0f, u_2 = 0.0f, u_3 = 0.0f;
//    static float e_1 = 0.0f, e_2 = 0.0f, e_3 = 0.0f;
//
//    /* Salida no saturada */
//    float u_unsat = (float)(-U1*u_1 -U2*u_2 -U3*u_3+ E0*e + E1*e_1 + E2*e_2+E3*e_3)/U0;
//
//    /* Saturación correcta (sobre u_unsat) */
//    float u_sat;
//    if (u_unsat > SAT_MAX)      u_sat = SAT_MAX;
//    else if (u_unsat < SAT_MIN) u_sat = SAT_MIN;
//    else                        u_sat = u_unsat;
//
//    /* DAC: escribir esfuerzo saturado */
//    VDAC8_SetValue(volt_to_dac(u_sat));
//
//    /* Correr estados (orden correcto) */
//    u_3 = u_2; u_2 = u_1; u_1 = u_sat;
//    e_3 = e_2; e_2 = e_1; e_1 = e;
}

/* ==== Helpers de parsing ==== */

/* Recorta espacios a la derecha (CR/LF/espacios) */
static void rstrip(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n-1]=='\r' || s[n-1]=='\n' || isspace((unsigned char)s[n-1])))
        s[--n] = '\0';
}

static void to_lower_str(char *s)
{
    for (; *s; ++s) *s = (char)tolower((unsigned char)*s);
}

/* Procesa bytes RX por líneas (eco por línea) */
static void pc_process_rx(void)
{
    int ch;
    while ((ch = PC_GetChar()) != 0) {
        if (ch == '\n' || ch == '\r') {
            if (rx_len > 0) {
                rx_buf[rx_len] = '\0';
                PC_PutString(rx_buf); PC_PutString("\r\n"); /* eco de línea */
                handle_command(rx_buf);
                rx_len = 0;
            }
        } else {
            if (rx_len < (RX_BUF_SZ-1)) {
                rx_buf[rx_len++] = (char)ch;
            } else {
                rx_len = 0;
                PC_PutString("ERR: line too long\r\n");
            }
        }
    }
}

/* ==== Status command (sin floats en printf; usa mV) ==== */
static void ref_status(void)
{
    long ref_min_mV = my_lroundf(ref_min * 1000.0f);
    long ref_max_mV = my_lroundf(ref_max * 1000.0f);
    long ref_mV     = my_lroundf(ref * 1000.0f);
    unsigned per    = (unsigned)timer_ref_ReadPeriod();

    char msg[160];
    snprintf(msg, sizeof(msg),
             "STATUS:\r\n"
             "  mode=%s\r\n"
             "  ref_min=%ld mV\r\n"
             "  ref_max=%ld mV\r\n"
             "  ref=%ld mV\r\n"
             "  ref_period=%u\r\n",
             (mode == MODE_OPEN) ? "open" : "closed",
             ref_min_mV, ref_max_mV, ref_mV, per);
    PC_PutString(msg);
}

/* ==== Parser de comandos ==== */
static void handle_command(const char *line_in)
{
    char line[RX_BUF_SZ];
    strncpy(line, line_in, sizeof(line));
    line[sizeof(line)-1] = '\0';
    rstrip(line);
    to_lower_str(line);

    /* Comandos sin valor */
    if (strcmp(line, "ref_status") == 0 || strcmp(line, "status") == 0) {
        ref_status(); return;
    }
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        PC_PutString("Commands:\r\n"
                     "  ref_max:<float 0..4.08>\r\n"
                     "  ref_min:<float 0..4.08>\r\n"
                     "  ref_period:<uint16 0..65535>\r\n"
                     "  mode:<open|closed|1|0>\r\n"
                     "  ref_status\r\n");
        return;
    }

    /* Clave:valor */
    char *colon = strchr(line, ':');
    if (!colon) { PC_PutString("ERR: expected key:value or ref_status\r\n"); return; }
    *colon = '\0';
    const char *key = line;
    const char *val_str = colon + 1;
    while (*val_str && isspace((unsigned char)*val_str)) val_str++;

    if (strcmp(key, "ref_max") == 0) {
        float v = (float)atof(val_str);
        set_ref_value((float*)&ref_max, v, "ref_max");
    } else if (strcmp(key, "ref_min") == 0) {
        float v = (float)atof(val_str);
        set_ref_value((float*)&ref_min, v, "ref_min");
    } else if (strcmp(key, "ref_period") == 0) {
        long v = strtol(val_str, NULL, 0);
        if (v < 0 || v > 65535) PC_PutString("ERR: ref_period out of range (0..65535)\r\n");
        else set_ref_period((uint16_t)v);
    } else if (strcmp(key, "mode") == 0) {
        if (strcmp(val_str, "open") == 0 || strcmp(val_str, "1") == 0) {
            mode = MODE_OPEN;  PC_PutString("OK: mode=open\r\n");
        } else if (strcmp(val_str, "closed") == 0 || strcmp(val_str, "0") == 0) {
            mode = MODE_CLOSED; PC_PutString("OK: mode=closed\r\n");
        } else {
            PC_PutString("ERR: mode must be open|closed|1|0\r\n");
        }
    } else {
        PC_PutString("ERR: unknown key. Use ref_max, ref_min, ref_period, mode, ref_status\r\n");
    }
}

/* Set de ref_* con validación y sección crítica */
static void set_ref_value(float *target, float val, const char *name)
{
    if (val < SAT_MIN || val > SAT_MAX) {
        char msg[64];
        long lo = my_lroundf(SAT_MIN*1000.0f), hi = my_lroundf(SAT_MAX*1000.0f);
        snprintf(msg, sizeof(msg), "ERR: %s out of range (%ld..%ld mV)\r\n", name, lo, hi);
        PC_PutString(msg);
        return;
    }

    uint8 intr = CyEnterCriticalSection();
    *target = val;
    /* Si está activa esa referencia, actualizá ref inmediatamente */
    ref = (ref_state_Read()) ? ref_max : ref_min;
    CyExitCriticalSection(intr);

    char msg[64];
    long v_mV = my_lroundf(val*1000.0f);
    snprintf(msg, sizeof(msg), "OK: %s=%ld mV\r\n", name, v_mV);
    PC_PutString(msg);
}

/* Cambia el periodo del timer de referencia de forma segura */
static void set_ref_period(uint16_t period)
{
    timer_ref_Stop();
    timer_ref_WritePeriod(period);
    timer_ref_WriteCounter(period);
    timer_ref_Start();

    char msg[64];
    snprintf(msg, sizeof(msg), "OK: ref_period=%u\r\n", (unsigned)period);
    PC_PutString(msg);
}

/* ==== main ==== */
int main(void)
{
    flag = '0';

    CyGlobalIntEnable;
    
    mat_zero_volatile(&X,2,1);
    /* HW init */
    Opa_dac_Start();
    Opa_stage1_Start();
    Opa_vdda_2_Start();
    Opa_stage2_Start();
    VADC_X1_Start();
    VADC_X2_Start();
    VDAC8_Start();
    VDAC8_ref_Start();

    /* UART PC */
    PC_Start();
    PC_PutString("\r\nReady. Commands:\r\n");
    PC_PutString("  ref_max:<float 0..4.08>\r\n");
    PC_PutString("  ref_min:<float 0..4.08>\r\n");
    PC_PutString("  ref_period:<uint16 0..65535>\r\n");
    PC_PutString("  mode:<open|closed|1|0>\r\n");
    PC_PutString("  ref_status\r\n");

    /* Timer / ISR de muestra */
    muestra_disponible_StartEx(adquirirMuestra);
    muestra_disponible_Enable();
    timer_ref_Start();

    
    for (;;)
    {
        /* Procesar UART sin bloquear */
        pc_process_rx();

        /* Procesar muestra si disponible */
        if (flag == '1') {
            flag = '0';
            actualizarEsfuerzo();           /* respeta el modo */
            muestra_disponible_Enable();    /* re-habilitar ISR */
        }
    }
}
