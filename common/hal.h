#ifndef FHSS_HAL_H
#define FHSS_HAL_H

#include <stdint.h>
#include "status.h"

/*
 * Frozen Hardware Abstraction Interface per CONTEXT.md section 5.
 * Shared byte-for-byte across all nodes.
 */
uint8_t SPI1_Transfer(uint8_t tx);
void    NSS_LOW(void);
void    NSS_HIGH(void);
int     TIM2_ScheduleHopMs(uint32_t ms, void (*cb)(void));   /* one-shot, replaces any pending one, 1..60000 */
void    UART1_SendString(const char *s);                     /* NULL-safe */

/*
 * Permitted extra helpers per CONTEXT.md section 5 (with a one-line reason):
 */
void     RADIO_RESET_LOW(void);                    /* Drives SX1278 active-low hardware reset pin */
void     RADIO_RESET_HIGH(void);                   /* Releases SX1278 hardware reset pin */
uint32_t millis(void);                             /* Monotonic millisecond counter for timeouts and scheduling */
void     delay_ms(uint32_t ms);                    /* Synchronous millisecond busy delay */
status_t EXTI0_RegisterCallback(void (*cb)(void)); /* Registers interrupt callback for DIO0 rising edge */
uint32_t micros(void);                             /* Microsecond timestamp function for airtime and guard timing */
void     TIM2_Poll(void);                          /* Cooperative event dispatcher for TIM2 hop timer callback */

#endif /* FHSS_HAL_H */
