#include "air.h"
#include "config.h"
#include "hal.h"
#include "status.h"
#include "sx1278_fsk.h"

#ifndef DEMO_USART_LINK

/*
 * Real Radio Air Abstraction (SX1278 FSK/GFSK)
 * Used when DEMO_USART_LINK is not defined.
 */

uint32_t air_airtime_us(uint8_t len) {
    return (10UL + (uint32_t)len) * 160UL;
}

int air_tx(uint8_t ch, const uint8_t *frame, uint8_t len) {
    if (frame == (const uint8_t *)0) return (int)ERR_NULL;
    if (len == 0U || len > MAX_FRAME_LEN) return (int)ERR_LEN;
    if (ch >= NUM_CHANNELS) return (int)ERR_RANGE;

    /* Blind transmit placeholder for hardware build when radio SPI is available */
    return (int)OK;
}

int air_rx_poll(uint8_t ch, uint8_t *buf, uint8_t *len) {
    if (buf == (uint8_t *)0 || len == (uint8_t *)0) return (int)ERR_NULL;
    if (ch >= NUM_CHANNELS) return (int)ERR_RANGE;

    return (int)ERR_TIMEOUT;
}

uint8_t air_rssi_raw(uint8_t ch) {
    if (ch >= NUM_CHANNELS) return 0xFFU;
    return 0xA0U; /* Default ~ -80 dBm */
}

#endif /* !DEMO_USART_LINK */
