#ifndef STEP5_FAULT_TEST_H
#define STEP5_FAULT_TEST_H

#include <stdint.h>
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"

/*
 * Stage 5: Fault-Injection Receiver and Reliability Test Harness (Node B)
 *
 * Architecture:
 *   - Fixed simulated-channel receiver on simulated channel 0 (FHSS_HOME_CHANNEL).
 *   - Bypasses real SX1278 RF path; operates entirely over wired USART2 mock transport.
 *   - Reuses existing envelope parser (step1_parse_ring_buffer), frozen FHSS frame
 *     parser (fhss_parse_packet), CRC-8/SMBUS integrity tag checks, and replay window rules.
 *
 * Ordered Physical Fault-Test Contract with Node A:
 *   Case 1: Valid channel-0 packets with seq=254, 255, 0, 1 (wraparound accepted & ACKed).
 *   Case 2: Valid frame body with corrupted outer envelope CRC (rejected, no ACK).
 *   Case 3: Fresh packet with invalid FHSS integrity tag (rejected, no ACK).
 *   Case 4: Valid fresh packet after invalid tag (accepted & ACKed, post-fault recovery logged).
 *   Case 5: Duplicate packet sent twice with same seq (first copy accepted & ACKed;
 *           second copy rejected as replay, no ACK).
 *   Case 6: Valid fresh packet in envelope marked channel 1 (rejected, no ACK).
 *   Case 7: Valid fresh channel-0 packet after wrong-channel case (accepted & ACKed,
 *           post-fault recovery logged).
 *   Case 8: Designated valid fresh packet with intentionally suppressed ACK (frame
 *           validated and accepted; ACK transmission suppressed; intentional suppression logged).
 *   Case 9: Next valid fresh channel-0 packet (accepted & ACKed; normal ACK transmission resumed).
 */

#ifndef STEP5_DEFAULT_SUPPRESS_SEQ
#define STEP5_DEFAULT_SUPPRESS_SEQ 7
#endif

typedef enum {
    STEP5_RX_IDLE = 0,
    STEP5_RX_DISCARD_GARBAGE,
    STEP5_RX_ERR_ENV_CRC,
    STEP5_RX_ERR_ENV_LEN,
    STEP5_RX_ERR_ENV_CH,
    STEP5_RX_REJECT_WRONG_CH,      /* Envelope passed, but pkt_ch != expected_ch (0) */
    STEP5_RX_REJECT_BAD_TAG,       /* FHSS version prefix or CRC-8 integrity tag mismatch */
    STEP5_RX_REJECT_SEMANTIC,      /* Invalid packet type or frag bounds violation */
    STEP5_RX_REJECT_REPLAY,        /* Replay rule violation: diff == 0 or diff > 127 */
    STEP5_RX_PASS_ACK_SENT,        /* Valid packet accepted and standard FHSS ACK transmitted */
    STEP5_RX_PASS_ACK_SUPPRESSED   /* Valid packet accepted, but ACK intentionally suppressed */
} step5_rx_outcome_t;

typedef struct {
    uint8_t  expected_ch;           /* Fixed simulated channel: 0 */
    uint8_t  have_last;             /* 1 if last_seq is initialized */
    uint8_t  last_seq;              /* Last accepted packet sequence number */
    uint8_t  b_seq;                 /* Node B outgoing ACK sequence counter */
    uint8_t  suppress_ack_enabled;  /* 1 if intentional ACK suppression is armed */
    uint8_t  suppress_ack_seq;      /* Designated sequence number for intentional suppression */
    uint8_t  last_was_fault;        /* 1 if immediately preceding frame was rejected */
    uint8_t  last_rejected_ch;      /* Channel byte of last rejected wrong-channel envelope */
    uint8_t  last_rejected_seq;     /* Seq byte of last rejected replay frame */
    uint8_t  cycle_done;            /* 1 if one-shot test cycle has completed (seq 8 processed) */

    /* Diagnostic counters */
    uint32_t rx_total_count;        /* Total frames processed (valid + rejected) */
    uint32_t env_crc_err_count;     /* Outer envelope CRC errors */
    uint32_t env_len_err_count;     /* Envelope framing/length errors */
    uint32_t env_ch_err_count;      /* Envelope channel range errors (>= 8) */
    uint32_t proto_wrong_ch_count;  /* Envelope channel != expected_ch */
    uint32_t proto_bad_tag_count;   /* Protocol tag integrity errors */
    uint32_t proto_semantic_count;  /* Semantic / bounds errors */
    uint32_t proto_replay_count;    /* Replay window rejections */
    uint32_t rx_accept_count;       /* Total valid frames accepted */
    uint32_t tx_ack_count;          /* Authentic FHSS ACKs transmitted */
    uint32_t ack_suppressed_count;  /* Intentionally suppressed ACKs */
} step5_harness_t;

/* Initialize test harness to simulated channel 0 and armed ACK suppression */
void step5_harness_init(step5_harness_t *harness);

/* Configure designated sequence number for intentional ACK suppression */
void step5_harness_set_suppress_seq(step5_harness_t *harness, uint8_t seq);

/*
 * Pure frame processor and fault injector (host-testable).
 * Inspects ring buffer using peek/advance functions.
 * Validates envelope and protocol fields, checks replay rules.
 * If valid and not suppressed, populates standard FHSS ACK in ack_out and sets *tx_ch_out.
 */
step5_rx_outcome_t step5_harness_process_incoming(
    step5_harness_t *harness,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
);

/* Target firmware entry point */
void step5_fault_test_run(void);

#endif /* STEP5_FAULT_TEST_H */
