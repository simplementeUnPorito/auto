#ifndef SERIAL_H
#define SERIAL_H
#include <stdint.h>
#include <stddef.h>

typedef void (*serial_line_cb_t)(const char *line);

/* Inicializa UART PC y registra callback por línea */
void serial_init(serial_line_cb_t on_line);

/* Poll no-bloqueante: drena RX, arma líneas y llama callback */
void serial_poll(void);

/* Envío utilitario */
void serial_puts(const char *s);

#endif
