#include "sx1278_fsk.h"
#include "hal.h"

/*
 * Read single 8-bit register from SX1278 over SPI.
 * Frame: [addr & 0x7F] -> [0x00 dummy] -> returns byte from MISO.
 */
uint8_t sx1278_read_reg(uint8_t addr) {
    NSS_LOW();
    SPI1_Transfer((uint8_t)(addr & 0x7FU));
    uint8_t val = SPI1_Transfer(0x00U);
    NSS_HIGH();
    return val;
}

/*
 * Write single 8-bit register to SX1278 over SPI.
 * Frame: [addr | 0x80] -> [val].
 */
void sx1278_write_reg(uint8_t addr, uint8_t val) {
    NSS_LOW();
    SPI1_Transfer((uint8_t)(addr | 0x80U));
    SPI1_Transfer(val);
    NSS_HIGH();
}

/*
 * Perform hardware reset on SX1278 module.
 * 1. Allow VCC/POR to settle (20 ms).
 * 2. Hold RESET low for 2 ms (spec >= 100 µs).
 * 3. Release RESET and wait 20 ms for crystal oscillator stabilization (spec >= 5 ms).
 */
void sx1278_reset(void) {
    RADIO_RESET_HIGH();
    delay_ms(20U);

    RADIO_RESET_LOW();
    delay_ms(2U);

    RADIO_RESET_HIGH();
    delay_ms(20U);
}
 