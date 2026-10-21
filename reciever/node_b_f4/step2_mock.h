#ifndef STEP2_MOCK_H
#define STEP2_MOCK_H

#include <stdint.h>
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"

/*
 * Step 2: Frozen-Protocol Receive Test over USART2 Mock Transport
 *
 * Expected test envelope from Node A:
 *   [0x7E] [channel=0x00] [length=0x0B] [frame_bytes (11 B)] [wire_crc=0x18]
 *
 * Expected frame_bytes (Node A known-answer vector):
 *   A1 05 03 E8 6A 7D 48 45 4C 4C 4F
 *   - Type: DATA (0xA1)
 *   - SEQ: 5
 *   - MSG_ID: 3
 *   - FRAG: 0xE8 (TOTAL=8, IDX=2, LAST=0)
 *   - HOP: 0x6A (masked next_ch=5, dwell_code=17)
 *   - TAG: 0x7D (CRC-8/SMBUS)
 *   - PAYLOAD: "HELLO" (5 bytes)
 *
 * Receipt marker returned upon valid envelope AND valid protocol parsing:
 *   Single raw byte: 0x06 (STEP2_TEST_ACK_BYTE).
 */

/* Step 2 test result codes */
typedef enum {
    STEP2_RESULT_IDLE = 0,               /* No complete frame in buffer */
    STEP2_RESULT_PASS,                   /* Both envelope and protocol passed */
    STEP2_RESULT_ENV_ERR_CRC,            /* Envelope CRC-8 mismatch */
    STEP2_RESULT_ENV_ERR_LEN,            /* Envelope length out of bounds */
    STEP2_RESULT_ENV_ERR_CH,             /* Envelope channel out of bounds */
    STEP2_RESULT_ENV_DISCARD,            /* Discarded non-0x7E delimiter byte */
    STEP2_RESULT_PROTO_REJECT_CH,        /* Channel is not home channel 0 */
    STEP2_RESULT_PROTO_REJECT_TAG,       /* FHSS tag / version prefix mismatch */
    STEP2_RESULT_PROTO_REJECT_SEMANTIC   /* Unknown packet type or invalid field */
} step2_result_t;

/* Step 2 receipt marker byte */
#define STEP2_TEST_ACK_BYTE  0x06

/* Pure parsing and protocol validation function (host-testable) */
step2_result_t step2_process_incoming(
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *pkt_out,
    uint8_t *next_ch_out,
    uint8_t *dwell_code_out
);

/* Firmware entry point when running under -DSTEP2_MOCK */
void step2_mock_run(void);

#endif /* STEP2_MOCK_H */
