#ifndef STEP7_ARQ_MOCK_H
#define STEP7_ARQ_MOCK_H

#include <stdint.h>
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"

/*
 * Stage 7: Selective-ACK Recovery for Duplicate DATA Fragments over USART2 (Node B)
 *
 * Architecture & Coordinated Test Schedule:
 *   - Fixed simulated-channel receiver on channel 0 (FHSS_HOME_CHANNEL).
 *   - Bypasses real SX1278 RF path; operates over wired USART2 mock transport.
 *   - Demonstrates Selective-ACK ARQ recovery when the first ACK for a valid
 *     DATA fragment is intentionally suppressed.
 *   - Schedule:
 *       1. DATA seq 0: Msg 1, frag 0/1 ("HELLO_NODE_B") -> Accept, reassemble, ACK bitmap 0x01
 *       2. DATA seq 1: Msg 2, frag 0/3 -> Accept, ACK bitmap 0x01
 *       3. DATA seq 2: Msg 2, frag 1/3 (first attempt) -> Validate & record, ACK intentionally suppressed
 *       4. DATA seq 3: Msg 2, frag 1/3 (retransmission) -> Fresh seq 3, match stored frag, no payload
 *                      duplication/corruption, return cumulative bitmap 0x03
 *       5. DATA seq 4: Msg 2, frag 2/3 -> Accept, ACK bitmap 0x07, complete 34-byte message
 *
 * Rules:
 *   - Validates frames in strict documented order (envelope framing, channel, FHSS tag, replay, DATA metadata).
 *   - Suppression rule is strictly one-shot and specific to msg_id=2, frag_idx=1.
 *   - Retransmission uses a fresh packet sequence (3), authenticated via standard FHSS tag.
 *   - Per-sender ACK sequence counter (b_seq) increments ONLY when an ACK is actually transmitted.
 *   - Duplicate fragment with conflicting payload or metadata is explicitly rejected without overwriting
 *     accepted data or generating an ACK.
 *   - Full replay protection preserved (packet-level replays rejected).
 */

#define STEP7_TARGET_MESSAGES 2

typedef enum {
    STEP7_RX_IDLE = 0,
    STEP7_RX_DISCARD_GARBAGE,
    STEP7_RX_ERR_ENV_CRC,
    STEP7_RX_ERR_ENV_LEN,
    STEP7_RX_ERR_ENV_CH,
    STEP7_RX_REJECT_WRONG_CH,      /* Envelope passed, but pkt_ch != expected_ch (0) */
    STEP7_RX_REJECT_BAD_TAG,       /* FHSS version prefix or CRC-8 integrity tag mismatch */
    STEP7_RX_REJECT_NON_DATA,      /* ver_type & 0x0F != PKT_TYPE_DATA */
    STEP7_RX_REJECT_REPLAY,        /* Replay rule violation: diff == 0 or diff > 127 */
    STEP7_RX_REJECT_METADATA,      /* idx >= total, total > 8, or total changed mid-message */
    STEP7_RX_REJECT_CONFLICT,      /* Repeated logical fragment has conflicting payload/metadata */
    STEP7_RX_PASS_FRAG,            /* Valid fresh fragment accepted, message still in progress, ACK generated */
    STEP7_RX_PASS_FRAG_ACK_SUPPRESSED, /* Valid fragment accepted, but ACK suppressed by Stage 7 rule */
    STEP7_RX_DUPLICATE_FRAG,       /* Valid duplicate fragment recognized, cumulative ACK generated */
    STEP7_RX_PASS_MSG_COMPLETE     /* Valid fragment accepted, message reassembly complete, ACK generated */
} step7_rx_outcome_t;

typedef struct {
    uint8_t  active;                                /* 1 if a message reassembly is currently in progress */
    uint8_t  msg_id;                                /* Current message ID */
    uint8_t  total_frags;                           /* Expected total fragments (1..8) */
    uint8_t  rx_bitmap;                             /* Cumulative received fragment bitmap */
    uint8_t  frag_lens[ARQ_MAX_FRAGMENTS];          /* Length of each fragment (up to 16 bytes) */
    uint16_t total_len;                             /* Reassembled message length in bytes */
    uint8_t  data[ARQ_MAX_MSG_LEN + 1];             /* Message buffer (+1 for null terminator) */
} step7_reassembler_t;

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
    uint8_t  ack_suppressed_once;   /* 1 once msg=2 frag=1 ACK has been intentionally suppressed */

    /* Active reassembler */
    step7_reassembler_t reasm;

    /* Last completed message copy for verification / logging */
    step7_reassembler_t last_completed;

    /* Diagnostic counters */
    uint32_t rx_total_count;        /* Total frames received (valid + rejected) */
    uint32_t rx_frag_count;         /* Fresh fragments accepted */
    uint32_t rx_dup_frag_count;     /* Duplicate fragments received with fresh sequence */
    uint32_t tx_ack_count;          /* Authentic FHSS ACKs transmitted */
    uint32_t ack_suppressed_count;  /* ACKs intentionally suppressed */
    uint32_t env_crc_err_count;     /* Envelope CRC errors */
    uint32_t env_len_err_count;     /* Envelope framing/length errors */
    uint32_t env_ch_err_count;      /* Envelope channel range errors (>= 8) */
    uint32_t proto_wrong_ch_count;  /* Envelope channel != expected_ch */
    uint32_t proto_bad_tag_count;   /* Protocol tag integrity errors */
    uint32_t proto_non_data_count;  /* Non-DATA packet types */
    uint32_t proto_replay_count;    /* Replay window rejections */
    uint32_t proto_metadata_count;  /* Invalid fragment metadata */
    uint32_t proto_conflict_count;  /* Duplicate fragment payload/metadata mismatch */
} step7_harness_t;

/* Initialize Stage 7 test harness */
void step7_harness_init(step7_harness_t *harness);

/*
 * Pure frame processor and message reassembler with selective-ACK ARQ recovery (host-testable).
 * Inspects ring buffer using peek/advance functions.
 * Validates envelope framing, channel, FHSS tag, sequence/replay, DATA metadata, payload matching.
 * Populates ack_out and sets *tx_ch_out when an ACK must be sent.
 */
step7_rx_outcome_t step7_harness_process_incoming(
    step7_harness_t *harness,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
);

/* Target firmware entry point */
void step7_arq_mock_run(void);

#endif /* STEP7_ARQ_MOCK_H */
