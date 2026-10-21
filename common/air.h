#ifndef AIR_H
#define AIR_H

#include <stdint.h>
#include "status.h"
#include "config.h"

/*
 * Air Abstraction Interface
 * Real implementation: the existing sx1278 driver (used when DEMO_USART_LINK is not defined).
 * Mock implementation: USART2 wire link (used when DEMO_USART_LINK is defined).
 */
int      air_tx(uint8_t ch, const uint8_t *frame, uint8_t len);
int      air_rx_poll(uint8_t ch, uint8_t *buf, uint8_t *len);   /* non-blocking; ERR_TIMEOUT if nothing for channel ch */
uint8_t  air_rssi_raw(uint8_t ch);                              /* RSSI dBm = -raw/2 */
uint32_t air_airtime_us(uint8_t len);                           /* (5+2+1+len+2 bytes) * 160 us */

#endif /* AIR_H */
