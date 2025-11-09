#include "project.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "config.h"
#include "mat.h"        /* tu librería */
#include "io.h"
#include "serial.h"
#include "control.h"

/* ===== Estado global ===== */
volatile char   flag = '0';
volatile double ref_max = (float)(A_AMPL/2);
volatile double ref_min = (float)(-A_AMPL/2);
volatile double ref     = 0.0;          /* referencia en AC */
volatile double y_last_volts = 0.0;     /* última muestra del ADC (tu hardware ya entrega AC) */

volatile uint8 mode = MODE_CLOSED;
static control_state_t ctrl_st;

/* ===== PEGAR EN main.c ===== */
static const control_model_t M = {
    .Ad = {.filas=2,.columnas=2,.d={{0.932581, 0.000000},
                                        {-1.145618, 0.455938}}},
    .Bd = {.filas=2,.columnas=1,.d={{-0.117824},
                                        {0.080093}}},
    .Cd = {.filas=1,.columnas=2,.d={{0.000000, 1.000000}}},
    .K  = {.filas=1,.columnas=2,.d={{2.907022, 1.636048}}},
    .K0 = {.filas=1,.columnas=1,.d={{0.569878}}},
    .L  = {.filas=2,.columnas=1,.d={{-0.503374},
                                        {0.988519}}}
};

/* poles(A):   0.455938  0.932581 */
/* poles(A-BK):   0.800000  0.800000 */
/* poles(A-LC):   0.200000  0.200000 */
/* Ts = 1.19483659e-03 s */
/* Podés cambiar a CL_CURRENT por UART ('law:curr') si luego querés */
static control_law_t g_law = CL_PREDICTOR;

/* ===== ISR de muestreo ===== */
CY_ISR(adquirirMuestra)
{
    /* NO limpiar pending aquí; el EOC siguiente genera un nuevo pulso */
    /* NO deshabilitar la interrupción aquí; la dejamos armada permanentemente */

    /* Conmutar referencia (en AC) por pin/switch */
    ref = (ref_state_Read()) ? ref_max : ref_min;

    /* Mostrar la referencia en el DAC auxiliar (para el osciloscopio) */
    io_ref_write((float)(ref + VDDA2));

    /* Leer ADC -> en tu hardware ya está en AC (centrado) */
    y_last_volts = io_adc_read_volts();

    flag = '1';
}

/* ===== Utilidades ===== */
static long my_lroundf(float x) { return (x >= 0.0f) ? (long)(x + 0.5f) : (long)(x - 0.5f); }
/* ===== helpers de parsing y print ===== */
static float parse_float_local(const char *s) {
    char buf[32]; size_t i=0;
    while (*s && i<sizeof(buf)-1) {
        char c = *s++;
        if (c==',') c='.';
        if (!((c>='0'&&c<='9') || c=='+' || c=='-' || c=='.' || c=='e' || c=='E')) break;
        buf[i++] = c;
    }
    buf[i]='\0';
    return (float)atof(buf);
}

static long parse_uint_local(const char *s) {
    while (*s==' '||*s=='\t') ++s;
    long v=0; while (*s>='0'&&*s<='9') { v = v*10 + (*s-'0'); ++s; }
    return v;
}

/* Set de amplitud 'a' => ref_max = +a/2, ref_min = -a/2, imprime en mV */
static void set_amp(float a) {
    if (a < 0.0f || a > 1.5f) { serial_puts("ERR: a out of range (0..1.5)\r\n"); return; }
    uint8 intr = CyEnterCriticalSection();
    ref_max = +a*0.5f; ref_min = -a*0.5f;
    ref = (ref_state_Read()) ? ref_max : ref_min;
    CyExitCriticalSection(intr);

    /* imprimir sin %f */
    int amp_mV = (int)(a*1000.0f + 0.5f);
    serial_puts("OK: a="); char msg[32]; snprintf(msg,sizeof(msg),"%d mVpp\r\n", amp_mV); serial_puts(msg);
}

/* STATUS reducido: m, a, p (sin %f) */
static void cmd_status(void) {
    float a = (float)(ref_max - ref_min);
    unsigned per = (unsigned)timer_ref_ReadPeriod();
    int a_mV = (int)(a*1000.0f + 0.5f);
    char msg[96];
    snprintf(msg,sizeof(msg),
        "STATUS:\r\n  m=%s\r\n  a=%d mVpp\r\n  p=%u\r\n",
        (mode==MODE_OPEN)?"open":"closed", a_mV, per);
    serial_puts(msg);
}
static void on_serial_line(const char *line_in)
{
    char line[RX_BUF_SZ];
    strncpy(line, line_in, sizeof(line));
    line[sizeof(line)-1]='\0';

    /* trim/basura */
    size_t n=strlen(line);
    while (n && (line[n-1]=='\r'||line[n-1]=='\n'||isspace((unsigned char)line[n-1]))) line[--n]='\0';
    char *p=line; while(*p && isspace((unsigned char)*p)) ++p;
    while(*p && !isalnum((unsigned char)*p)) ++p;
    for(char *q=p; *q; ++q) *q=(char)tolower((unsigned char)*q);

    if (!strcmp(p,"s") || !strcmp(p,"status")) { cmd_status(); return; }
    if (!strcmp(p,"help") || !strcmp(p,"?")) {
        serial_puts("a:<0..1.5>, p:<uint16>, m:<open|closed|1|0>, s\r\n"); return;
    }

    char *sep = strchr(p, ':');
    if (!sep) { sep=strchr(p,' '); if(sep) *sep=':'; }
    if (!sep) { serial_puts("ERR: expected a:<val> | p:<val> | m:<val> | s\r\n"); return; }
    *sep='\0'; const char *key=p, *val=sep+1; while(*val && isspace((unsigned char)*val)) ++val;

    if (!strcmp(key,"a")) { set_amp(parse_float_local(val)); return; }

    if (!strcmp(key,"p")) {
        long per = parse_uint_local(val);
        if (per<0 || per>65535) { serial_puts("ERR: p out of range (0..65535)\r\n"); return; }
        timer_ref_Stop(); timer_ref_WritePeriod((uint16)per); timer_ref_WriteCounter((uint16)per); timer_ref_Start();
        serial_puts("OK: p set\r\n"); return;
    }

    if (!strcmp(key,"m")) {
        if (!strcmp(val,"open")||!strcmp(val,"1")) { mode=MODE_OPEN; serial_puts("OK: m=open\r\n"); }
        else if (!strcmp(val,"closed")||!strcmp(val,"0")) { mode=MODE_CLOSED; serial_puts("OK: m=closed\r\n"); }
        else { serial_puts("ERR: m must be open|closed|1|0\r\n"); }
        return;
    }

    serial_puts("ERR: unknown key (use a, p, m, s)\r\n");
}




/* ===== main ===== */
int main(void)
{
    CyGlobalIntEnable;

    io_init();
    control_init(&ctrl_st);
    serial_init(on_serial_line);

    /* Timer/ISR de muestreo */
    muestra_disponible_StartEx(adquirirMuestra);
    muestra_disponible_ClearPending();   /* arranque limpio */
    muestra_disponible_Enable();
    timer_ref_Start();

    for(;;) {
        /* UART no bloqueante */
        serial_poll();

        /* procesar 1 muestra */
        if (flag=='1') {
            flag='0';

            float u_phys = control_step(&ctrl_st, &M,
                                        (float)ref,              /* AC */
                                        (float)y_last_volts,     /* AC en tu hardware */
                                        mode,
                                        g_law);

            io_dac_write(u_phys);

            /* ya NO re-armamos la IRQ: queda habilitada siempre */
        }
    }
}
