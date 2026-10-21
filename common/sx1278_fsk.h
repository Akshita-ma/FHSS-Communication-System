#ifndef SX1278_FSK_H
#define SX1278_FSK_H

#include <stdint.h>
#include "status.h"

/*
 * SX1278 FSK/GFSK Modem Driver Interface
 * (Shared between Node A and Node B - no MCU specifics or NODE ifdefs)
 */

/* Register addresses from Semtech SX1276/77/78/79 datasheet */
#define REG_FIFO                   0x00
#define REG_OP_MODE                0x01
#define REG_BITRATE_MSB            0x02
#define REG_BITRATE_LSB            0x03
#define REG_FDEV_MSB               0x04
#define REG_FDEV_LSB               0x05
#define REG_FRF_MSB                0x06
#define REG_FRF_MID                0x07
#define REG_FRF_LSB                0x08
#define REG_PA_CONFIG              0x09
#define REG_PA_RAMP                0x0A
#define REG_RX_BW                  0x12
#define REG_AFC_BW                 0x13
#define REG_RSSI_VALUE             0x1B
#define REG_PREAMBLE_MSB           0x25
#define REG_PREAMBLE_LSB           0x26
#define REG_SYNC_CONFIG            0x27
#define REG_SYNC_VALUE1            0x28
#define REG_SYNC_VALUE2            0x29
#define REG_PACKET_CONFIG1         0x30
#define REG_PACKET_CONFIG2         0x31
#define REG_PAYLOAD_LENGTH         0x32
#define REG_IRQ_FLAGS1             0x3E
#define REG_IRQ_FLAGS2             0x3F
#define REG_DIO_MAPPING1           0x40
#define REG_DIO_MAPPING2           0x41
#define REG_VERSION                0x42

/* Register access and control functions */
status_t sx1278_read_reg(uint8_t addr, uint8_t *val);
status_t sx1278_write_reg(uint8_t addr, uint8_t val);
status_t sx1278_reset(void);

#endif /* SX1278_FSK_H */
