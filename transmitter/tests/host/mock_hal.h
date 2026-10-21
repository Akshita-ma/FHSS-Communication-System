#ifndef FHSS_MOCK_HAL_H
#define FHSS_MOCK_HAL_H

#include "common/hal.h"
#include "common/status.h"

/* Mock HAL control helpers for unit testing */
void mock_hal_reset(void);
void mock_hal_set_millis(uint32_t ms);
void mock_hal_set_micros(uint32_t us);
int  mock_hal_get_nss_state(void);
int  mock_hal_get_reset_state(void);
const char *mock_hal_get_uart_buffer(void);
uint32_t    mock_hal_get_uart_bytes(void);
void        mock_hal_clear_uart(void);
void        mock_hal_set_reg(uint8_t addr, uint8_t val);
uint8_t     mock_hal_get_reg(uint8_t addr);

#endif /* FHSS_MOCK_HAL_H */
