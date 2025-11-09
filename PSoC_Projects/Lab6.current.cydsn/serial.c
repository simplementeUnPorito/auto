#include "project.h"
#include "config.h"
#include "serial.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

static serial_line_cb_t g_on_line = 0;
static char rx_buf[RX_BUF_SZ];
static uint8_t rx_len = 0;

void serial_init(serial_line_cb_t on_line)
{
    g_on_line = on_line;
    PC_Start();
    PC_PutString(
        "\r\nReady. Commands:\r\n"
        "  a:<float 0..1.5>     amplitud AC (admite 0,6 o 0.6)\r\n"
        "  p:<uint16>           periodo timer\r\n"
        "  m:<open|closed|1|0>  modo\r\n"
        "  s                    status\r\n"
    );
}



void serial_puts(const char *s) { PC_PutString(s); }

/* sane \r\n handling + overflow protection */
void serial_poll(void)
{
    int ch;
    while ((ch = PC_GetChar()) != 0) {
            PC_PutChar(ch); // eco inmediato
        if (ch == '\r' || ch == '\n') {
            if (rx_len > 0) {
                rx_buf[rx_len] = '\0';
                if (g_on_line) g_on_line(rx_buf);
                rx_len = 0;
            }
            /* colapsar múltiples \r\n consecutivos */
        } else {
            if (rx_len < (RX_BUF_SZ - 1)) {
                rx_buf[rx_len++] = (char)ch;
            } else {
                rx_len = 0;
                PC_PutString("ERR: line too long\r\n");
            }
        }
    }
}
