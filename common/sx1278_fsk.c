#include "sx1278_fsk.h"
#include "hal.h"
#include <stddef.h>

status_t sx1278_read_reg(uint8_t addr, uint8_t *val) {
    if (val == NULL) {
        return ERR_NULL;
    }
    if (addr > 0x7F) {
        return ERR_RANGE;
    }

    uint8_t cmd = (uint8_t)(addr & 0x7F); /* Bit 7 = 0 for read */

    NSS_LOW();
    delay_us(2);
    SPI1_Transfer(cmd);
    *val = SPI1_Transfer(0x00);
    delay_us(2);
    NSS_HIGH();
    delay_us(2);

    return OK;
}

status_t sx1278_write_reg(uint8_t addr, uint8_t val) {
    if (addr > 0x7F) {
        return ERR_RANGE;
    }

    uint8_t cmd = (uint8_t)(addr | 0x80); /* Bit 7 = 1 for write */

    NSS_LOW();
    delay_us(2);
    SPI1_Transfer(cmd);
    SPI1_Transfer(val);
    delay_us(2);
    NSS_HIGH();
    delay_us(2);

    return OK;
}

status_t sx1278_reset(void) {
    /* Pull RESET low for at least 100 us (using 5 ms for complete RC discharge) */
    Radio_Reset_Pin(0);
    delay_ms(5);

    /* Pull RESET high and wait at least 5 ms (using 15 ms for crystal osc startup) */
    Radio_Reset_Pin(1);
    delay_ms(15);

    return OK;
}
