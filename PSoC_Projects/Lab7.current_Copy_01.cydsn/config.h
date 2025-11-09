#ifndef CONFIG_H
#define CONFIG_H

/* === Límites y referencias físicas === */
#define SAT_MIN        (0.0f)
#define SAT_MAX        (4.048f)
#define MIDDLE_VOLTAGE ((SAT_MAX - SAT_MIN)/2.0f)
#define VDDA2          SAT_MAX/2

/* UART */
#define RX_BUF_SZ 64

/* Amplitud para referencia cuadrada (renombrado: NO usar 'A') */
#define A_AMPL 1.5f

/* Modo de operación */
enum { MODE_CLOSED = 0, MODE_OPEN = 1 };

#endif
