#ifndef STEP6_DATA_MOCK_H
#define STEP6_DATA_MOCK_H

#include <stdint.h>
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"

/*
 * Stage 6: DATA Frame Validation and Message Reassembly over USART2 (Node B)
 *
 * Architecture:
 *   - Fixed simulated-channel receiver on channel 0 (FHSS_HOME_CHANNEL).
 *   - Bypasses real SX1278 RF path; operates over wired USART2 mock transport.
 *   - Validates frames in strict documented order:
 *       1. Envelope framing, length and outer CRC (step1_parse_ring_buffer).
 *       2. Channel matches simulated channel 0.
 *       3. FHSS protocol prefix (0xA0) and CRC-8 integrity tag (fhss_parse_packet).
 *       4. Sequence / replay validation (1..127 delta rule).
 *       5. DATA fragment fields and semantic bounds (idx < total, total <= 8, payload <= 16).
 *   - Selective Bitmap ARQ reassembly buffer:
 *       - Message capacity: up to 128 bytes (ARQ_MAX_MSG_LEN), up to 8 fragments.
 *       - Fragment i carries bytes (16 * i ..).
 *       - Cumulative bitmap tracks received fragments.
 *       - Completion detected when rx_bitmap == (1 << total) - 1.
 *   - Transmits standard frozen-protocol ACKs (PKT_TYPE_ACK) with cumulative bitmap
 *     in payload[0] and RSSI in payload[1].
 *   - One-shot test session: halts in idle state after 2 messages are reassembled.
 */

#define STEP6_TARGET_MESSAGES 2

typedef enum {
    STEP6_RX_IDLE = 0,
    STEP6_RX_DISCARD_GARBAGE,
    STEP6_RX_ERR_ENV_CRC,
    STEP6_RX_ERR_ENV_LEN,
    STEP6_RX_ERR_ENV_CH,
    STEP6_RX_REJECT_WRONG_CH,      /* Envelope passed, but pkt_ch != expected_ch (0) */
    STEP6_RX_REJECT_BAD_TAG,       /* FHSS version prefix or CRC-8 integrity tag mismatch */
    STEP6_RX_REJECT_NON_DATA,      /* ver_type & 0x0F != PKT_TYPE_DATA */
    STEP6_RX_REJECT_REPLAY,        /* Replay rule violation: diff == 0 or diff > 127 */
    STEP6_RX_REJECT_METADATA,      /* idx >= total, total > 8, or total changed mid-message */
    STEP6_RX_DUPLICATE_FRAG,       /* Valid frame, but fragment idx already in bitmap (ACK sent with bitmap) */
    STEP6_RX_PASS_FRAG,            /* Valid fresh fragment accepted, message still in progress */
    STEP6_RX_PASS_MSG_COMPLETE     /* Valid fragment accepted, message reassembly complete */
} step6_rx_outcome_t;

typedef struct {
    uint8_t  active;                                /* 1 if a message reassembly is currently in progress */
    uint8_t  msg_id;                                /* Current message ID */
    uint8_t  total_frags;                           /* Expected total fragments (1..8) */
    uint8_t  rx_bitmap;                             /* Cumulative received fragment bitmap */
    uint8_t  frag_lens[ARQ_MAX_FRAGMENTS];          /* Length of each fragment (up to 16 bytes) */
    uint16_t total_len;                             /* Reassembled message length in bytes */
    uint8_t  data[ARQ_MAX_MSG_LEN + 1];             /* Message buffer (+1 for null terminator) */
} step6_reassembler_t;

typedef struct {
    uint8_t  expected_ch;           /* Fixed simulated channel: 0 */
    uint8_t  have_last;             /* 1 if last_seq is initialized */
    uint8_t  last_seq;              /* Last accepted packet sequence number */
    uint8_t  b_seq;                 /* Node B outgoing ACK sequence counter */
    uint8_t  target_messages;       /* Number of complete messages required for one-shot test (2) */
    uint8_t  completed_messages;    /* Count of successfully reassembled messages */
    uint8_t  cycle_done;            /* 1 if test cycle is complete */
    uint8_t  last_was_fault;        /* 1 if previous packet was rejected */
    uint8_t  last_rejected_ch;      /* Channel byte of last rejected wrong-channel envelope */
    uint8_t  last_rejected_seq;     /* Seq byte of last rejected replay frame */

    /* Active reassembler */
    step6_reassembler_t reasm;

    /* Last completed message copy for verification / logging */
    step6_reassembler_t last_completed;

    /* Diagnostic counters */
    uint32_t rx_total_count;        /* Total frames received (valid + rejected) */
    uint32_t rx_frag_count;         /* Fresh fragments accepted */
    uint32_t rx_dup_frag_count;     /* Duplicate fragments received with fresh sequence */
    uint32_t tx_ack_count;          /* Authentic FHSS ACKs transmitted */
    uint32_t env_crc_err_count;     /* Envelope CRC errors */
    uint32_t env_len_err_count;     /* Envelope framing/length errors */
    uint32_t env_ch_err_count;      /* Envelope channel range errors (>= 8) */
    uint32_t proto_wrong_ch_count;  /* Envelope channel != expected_ch */
    uint32_t proto_bad_tag_count;   /* Protocol tag integrity errors */
    uint32_t proto_non_data_count;  /* Non-DATA packet types */
    uint32_t proto_replay_count;    /* Replay window rejections */
    uint32_t proto_metadata_count;  /* Invalid fragment metadata */
} step6_harness_t;

/* Initialize Stage 6 test harness */
void step6_harness_init(step6_harness_t *harness);

/*
 * Pure frame processor and message reassembler (host-testable).
 * Inspects ring buffer using peek/advance functions.
 * Validates envelope framing, channel, FHSS tag, sequence/replay, DATA metadata.
 * Populates ack_out and sets *tx_ch_out when an ACK must be sent.
 */
step6_rx_outcome_t step6_harness_process_incoming(
    step6_harness_t *harness,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
);

/* Target firmware entry point */
void step6_data_mock_run(void);

#endif /* STEP6_DATA_MOCK_H */
