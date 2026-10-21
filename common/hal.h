#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include "status.h"

/*
 * FROZEN Hardware Abstraction Interface (CONTEXT.md section 5)
 * Identical on both Node A and Node B.
 */
uint8_t SPI1_Transfer(uint8_t tx);
void    NSS_LOW(void);
void    NSS_HIGH(void);
int     TIM2_ScheduleHopMs(uint32_t ms, void (*cb)(void));   /* one-shot, replaces any pending one, 1..60000 */
void    UART1_SendString(const char *s);                     /* NULL-safe */

/*
 * Allowed extra helpers with required one-line reason (Hard Rule 3)
 */
void     Radio_Reset_Pin(uint8_t state);           /* Controls SX1278 active-low RESET pin (PA1): 0 = low/reset, 1 = high/run */
uint32_t millis(void);                             /* Returns millisecond counter since boot for timeouts and time tracking */
void     delay_ms(uint32_t ms);                    /* Busy-wait millisecond delay using master clock */
void     delay_us(uint32_t us);                    /* Busy-wait microsecond delay for radio reset pulse timing */
status_t EXTI0_RegisterCallback(void (*cb)(void)); /* Registers callback for PA0/EXTI0 DIO0 interrupt event */

#endif /* HAL_H */
