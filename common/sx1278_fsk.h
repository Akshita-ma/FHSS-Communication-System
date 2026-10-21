#ifndef SX1278_FSK_H
#define SX1278_FSK_H

#include <stdint.h>
#include "status.h"

/*
 * SX1278 Register Addresses (Semtech SX1276/77/78/79 datasheet section 6.4)
 */
#define REG_PREAMBLE_LSB    0x26U
#define REG_VERSION         0x42U

/* Expected reset silicon version for SX1278 */
#define SX1278_EXPECTED_VERSION 0x12U

/*
 * Low-level register access functions (FROZEN interface across nodes).
 * Protocol: Write = addr | 0x80, Read = addr & 0x7F.
 */
uint8_t sx1278_read_reg(uint8_t addr);
void    sx1278_write_reg(uint8_t addr, uint8_t val);

/*
 * Radio hardware reset sequence:
 * Pull RESET low for >= 100 µs, then release and wait >= 5 ms.
 */
void    sx1278_reset(void);

#endif /* SX1278_FSK_H */
