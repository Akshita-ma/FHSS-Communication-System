#ifndef STEP1_UART_TEST_H
#define STEP1_UART_TEST_H

#include <stdint.h>
#include "status.h"
#include "config.h"

/*
 * Step 1: Isolated Wired USART2 Communication Test Interface
 *
 * Wire format expected from Node A:
 *   [0x7E] [channel (1B)] [length (1B)] [frame_bytes (len B)] [crc8_smbus (1B)]
 *
 * Where:
 *   - 0x7E is the sync delimiter byte.
 *   - channel is an 8-bit channel index (0..7).
 *   - length is the byte count of frame_bytes (HEADER_LEN=6 to MAX_FRAME_LEN=22).
 *   - crc8_smbus is computed over [channel, length, frame_bytes] with poly 0x07, init 0x00.
 *
 * Response to Node A upon valid frame receipt:
 *   Single raw test-response byte: 0x06 (STEP1_TEST_ACK_BYTE).
 */

/* Step 1 test result codes */
typedef enum {
    STEP1_RX_IDLE = 0,           /* No frame ready / waiting for more bytes */
    STEP1_RX_PASS,               /* Valid test frame received and verified */
    STEP1_RX_ERR_CRC,            /* CRC mismatch */
    STEP1_RX_ERR_LEN,            /* Length outside [HEADER_LEN, MAX_FRAME_LEN] */
    STEP1_RX_ERR_CHANNEL,        /* Channel >= FHSS_NUM_CHANNELS */
    STEP1_RX_DISCARD_GARBAGE     /* Non-0x7E byte discarded before sync */
} step1_rx_result_t;

/* Step 1 test response marker byte returned to Node A upon valid frame */
#define STEP1_TEST_ACK_BYTE      0x06

/* Pure parsing and validation function for the ring buffer (host-testable) */
step1_rx_result_t step1_parse_ring_buffer(
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    uint8_t *ch_out,
    uint8_t *len_out,
    uint8_t *frame_buf_out,
    uint8_t *calc_crc_out,
    uint8_t *wire_crc_out
);

/* Main test loop when compiled under -DSTEP1_UART_TEST */
void step1_uart_test_run(void);

#endif /* STEP1_UART_TEST_H */
