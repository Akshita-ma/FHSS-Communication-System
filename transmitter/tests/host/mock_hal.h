#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include "hal.h"
#include <stddef.h>

/* Mock HAL state and inspection helpers for host unit tests */
void mock_hal_reset(void);
const char *mock_hal_get_uart_log(void);
size_t mock_hal_get_uart_log_len(void);
uint8_t mock_hal_get_nss_state(void);
uint8_t mock_hal_get_reset_pin_state(void);
void    mock_hal_set_reg(uint8_t addr, uint8_t val);
uint8_t mock_hal_get_reg(uint8_t addr);

#endif /* MOCK_HAL_H */
