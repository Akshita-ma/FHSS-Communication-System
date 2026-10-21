#ifndef AIR_USART2_H
#define AIR_USART2_H

#include <stdint.h>
#include "air.h"

void air_usart2_init(void);
void air_trace_pins_init(void);
void air_set_locked_timestamp(uint32_t t_ms);
uint8_t air_is_channel_jammed(uint8_t ch);

/* Step 1 helpers for byte write, ring buffer inspection, and overrun/overflow stats */
void     air_usart2_write_byte(uint8_t b);
uint16_t air_usart2_rx_available(void);
int      air_usart2_rx_peek(uint16_t offset, uint8_t *b);
void     air_usart2_rx_advance(uint16_t count);
uint32_t air_usart2_get_ore_count(void);
uint32_t air_usart2_get_overflow_count(void);
void     air_usart2_clear_errors(void);

#endif /* AIR_USART2_H */
