#ifndef IO_H
#define IO_H
#include <stdint.h>

/* Init de periféricos HW */
void io_init(void);

/* ADC -> voltios */
float io_adc_read_volts(void);

/* DAC principal: esfuerzo */
void io_dac_write(float volts);

/* DAC auxiliar: referencia visual al scope */
void io_ref_write(float volts);

/* Map 0..SAT_MAX -> 0..255 con clamps */
uint8_t io_volt_to_dac(float v);

#endif
