#ifndef CUSTOM_RX_H
#define CUSTOM_RX_H

#include <stdint.h>
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"

/*
 * Node B: Interactive Custom-Message Receiver with Selective-ACK ARQ
 *
 * Architecture:
 *   - Continuous receiver over USART2 mock transport on channel 0 (FHSS_HOME_CHANNEL).
 *   - Reassembles arbitrary one-fragment and multi-fragment application messages (up to 128 bytes).
 *   - Validates frames in strict order:
 *       1. Envelope framing, length (6..22), channel (0), and outer CRC-8/SMBUS.
 *       2. Simulated channel matches expected channel 0.
 *       3. FHSS protocol prefix (0xA0) and CRC-8 tag integrity.
 *       4. Packet type is DATA (0xA1).
 *       5. Sequence / replay validation (1..127 forward delta rule).
 *       6. DATA fragment metadata and semantic bounds (idx < total, total <= 8, payload <= 16,
 *          non-final frags == 16B, LAST flag consistency, buffer boundary <= 128B).
 *       7. Message ID management and duplicate detection:
 *          - Identical duplicate (matching stored payload & metadata): accepted, acknowledged
 *            with cumulative bitmap without re-appending or double-counting.
 *          - Conflicting duplicate (mismatched payload or metadata): strictly rejected without
 *            corrupting stored data or transmitting an ACK.
 *   - Selective Bitmap ARQ reassembly:
 *       - Cumulative bitmap tracks received fragments.
 *       - Completion detected when rx_bitmap == (1 << total) - 1.
 *       - Outputs plaintext message to UART1 upon successful reassembly.
 *       - Stays active for subsequent incoming messages indefinitely (non-blocking).
 */

typedef enum {
    CUSTOM_RX_IDLE = 0,
    CUSTOM_RX_DISCARD_GARBAGE,
    CUSTOM_RX_ERR_ENV_CRC,
    CUSTOM_RX_ERR_ENV_LEN,
    CUSTOM_RX_ERR_ENV_CH,
    CUSTOM_RX_REJECT_WRONG_CH,      /* Envelope passed, but pkt_ch != expected_ch (0) */
    CUSTOM_RX_REJECT_BAD_TAG,       /* FHSS version prefix or CRC-8 integrity tag mismatch */
    CUSTOM_RX_REJECT_NON_DATA,      /* ver_type & 0x0F != PKT_TYPE_DATA */
    CUSTOM_RX_REJECT_REPLAY,        /* Replay rule violation: diff == 0 or diff > 127 */
    CUSTOM_RX_REJECT_METADATA,      /* idx >= total, total > 8, non-final len < 16, or total changed mid-message */
    CUSTOM_RX_REJECT_CONFLICT,      /* Repeated logical fragment has conflicting payload/metadata */
    CUSTOM_RX_PASS_FRAG,            /* Valid fresh fragment accepted, message still in progress, ACK generated */
    CUSTOM_RX_DUPLICATE_FRAG,       /* Valid duplicate fragment recognized, cumulative ACK generated */
    CUSTOM_RX_PASS_MSG_COMPLETE     /* Valid fragment accepted, message reassembly complete, ACK generated */
} custom_rx_outcome_t;

typedef struct {
    uint8_t  active;                                /* 1 if a message reassembly is currently in progress */
    uint8_t  msg_id;                                /* Current message ID */
    uint8_t  total_frags;                           /* Expected total fragments (1..8) */
    uint8_t  rx_bitmap;                             /* Cumulative received fragment bitmap */
    uint8_t  frag_lens[ARQ_MAX_FRAGMENTS];          /* Length of each fragment (up to 16 bytes) */
    uint16_t total_len;                             /* Reassembled message length in bytes */
    uint8_t  data[ARQ_MAX_MSG_LEN + 1];             /* Message buffer (+1 for null terminator) */
} custom_reassembler_t;

typedef struct {
    uint8_t  expected_ch;           /* Fixed simulated channel: 0 */
    uint8_t  have_last;             /* 1 if last_seq is initialized */
    uint8_t  last_seq;              /* Last accepted packet sequence number */
    uint8_t  b_seq;                 /* Node B outgoing ACK sequence counter */
    uint8_t  last_was_fault;        /* 1 if previous packet was rejected */
    uint8_t  last_rejected_ch;      /* Channel byte of last rejected wrong-channel envelope */
    uint8_t  last_rejected_seq;     /* Seq byte of last rejected replay frame */
    uint8_t  has_completed_msg;     /* 1 if last_completed contains a valid completed message */

    /* Active reassembler */
    custom_reassembler_t reasm;

    /* Last completed message copy for verification / duplicate ACK recovery */
    custom_reassembler_t last_completed;

    /* Diagnostic counters */
    uint32_t rx_total_count;        /* Total frames received (valid + rejected) */
    uint32_t rx_frag_count;         /* Fresh fragments accepted */
    uint32_t rx_dup_frag_count;     /* Duplicate fragments received with fresh sequence */
    uint32_t tx_ack_count;          /* Authentic FHSS ACKs transmitted */
    uint32_t completed_messages;    /* Count of successfully reassembled messages */
    uint32_t env_crc_err_count;     /* Envelope CRC errors */
    uint32_t env_len_err_count;     /* Envelope framing/length errors */
    uint32_t env_ch_err_count;      /* Envelope channel range errors (>= 8) */
    uint32_t proto_wrong_ch_count;  /* Envelope channel != expected_ch */
    uint32_t proto_bad_tag_count;   /* Protocol tag integrity errors */
    uint32_t proto_non_data_count;  /* Non-DATA packet types */
    uint32_t proto_replay_count;    /* Replay window rejections */
    uint32_t proto_metadata_count;  /* Invalid fragment metadata */
    uint32_t proto_conflict_count;  /* Duplicate fragment payload/metadata mismatch */
} custom_rx_harness_t;

/* Initialize interactive custom receiver harness */
void custom_rx_harness_init(custom_rx_harness_t *harness);

/*
 * Pure frame processor and message reassembler for custom messages (host-testable).
 * Inspects ring buffer using peek/advance functions.
 * Validates envelope framing, channel, FHSS tag, sequence/replay, DATA metadata, payload matching.
 * Populates ack_out and sets *tx_ch_out when an ACK must be sent.
 */
custom_rx_outcome_t custom_rx_process_incoming(
    custom_rx_harness_t *harness,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
);

/* Interactive receiver target firmware entry point */
void custom_rx_run(void);

#endif /* CUSTOM_RX_H */
