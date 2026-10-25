#ifndef STEP3_FHSS_MOCK_H
#define STEP3_FHSS_MOCK_H

#include <stdint.h>
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"

/*
 * Step 3.1: FHSS Channel and Timer Coordination Diagnostics
 *
 * Synchronization States:
 *   - SEARCH: Listens continuously on simulated channel 0 (FHSS_HOME_CHANNEL).
 *             Replay history cleared (have_last = 0).
 *             Awaits valid HOP or DATA packet with directive for Hop 1.
 *   - LOCKED: Follows hop schedule (home hop: HOME_DWELL_MS = 120 ms).
 *             On each hop k, uses latest accepted directive for hop k+1.
 *             Home hop is anchored by the sync ACK; NO re-anchoring in home hop.
 *             Subsequent hops re-anchor TIM2 ONCE on the first valid packet of the hop.
 *   - LOST:   3 consecutive missed hops without a valid packet.
 *             Resets to SEARCH on channel 0 with replay history cleared.
 *
 * Wire Envelope:
 *   [0x7E] [channel] [length] [frame_bytes] [CRC-8/SMBUS]
 *
 * Protocol Response:
 *   Upon receiving a valid packet, Node B returns a standard FHSS ACK packet
 *   (ver_type = 0xA2, PKT_TYPE_ACK) via air_tx on the current simulated channel.
 *   No Stage 2 raw 0x06 marker is used.
 */

typedef enum {
    STEP3_STATE_SEARCH = 0,
    STEP3_STATE_LOCKED,
    STEP3_STATE_LOST
} step3_sync_state_t;

typedef enum {
    STEP3_RX_IDLE = 0,
    STEP3_RX_DISCARD_GARBAGE,
    STEP3_RX_ERR_ENV_CRC,
    STEP3_RX_ERR_ENV_LEN,
    STEP3_RX_ERR_ENV_CH,
    STEP3_RX_REJECT_WRONG_CH,      /* Envelope passed, but pkt_ch != current_ch */
    STEP3_RX_REJECT_TAG,           /* Version prefix or tag integrity failure */
    STEP3_RX_REJECT_SEMANTIC,      /* Invalid packet type, frag bounds, or hop directive */
    STEP3_RX_REJECT_REPLAY,        /* Replay rule violation (diff == 0 or diff > 127) */
    STEP3_RX_PASS_LOCKED,          /* Valid packet in SEARCH -> transitioned to LOCKED */
    STEP3_RX_PASS_CONTINUED        /* Valid packet in LOCKED */
} step3_rx_outcome_t;

typedef enum {
    STEP3_TIMER_EVENT_NONE = 0,
    STEP3_TIMER_EVENT_HOP,         /* Hop timer expired with packet -> switched to next channel */
    STEP3_TIMER_EVENT_LOST         /* 3 consecutive missed hops -> returned to SEARCH */
} step3_timer_event_t;

typedef struct {
    step3_sync_state_t state;
    uint8_t  current_ch;
    uint32_t current_dwell_ms;
    uint8_t  next_ch;
    uint32_t next_dwell_ms;
    uint8_t  b_seq;
    uint8_t  have_last;
    uint8_t  last_seq;
    uint8_t  got_packet_this_hop;
    uint8_t  hop_reanchored;        /* 1 if timer has been re-anchored for the current hop */
    uint8_t  is_first_of_hop;       /* 1 if the packet just processed was the first of this hop */
    uint8_t  last_rejected_ch;      /* Offending rx envelope channel on wrong_channel rejection */
    uint8_t  last_rejected_seq;     /* Offending rx sequence number on replay rejection */
    uint8_t  consecutive_missed;
    uint32_t rx_pass_count;
    uint32_t tx_ack_count;
    uint32_t env_err_count;
    uint32_t proto_err_count;
    uint32_t replay_count;
    uint32_t hop_count;
} step3_fsm_t;

/* Initialize FSM to SEARCH state on channel 0 */
void step3_fsm_init(step3_fsm_t *fsm);

/*
 * Pure frame processor and state updater (host-testable).
 * Inspects ring buffer using peek/advance functions.
 * If a valid frame is accepted, constructs the ACK packet in ack_out and sets *tx_ch_out.
 */
step3_rx_outcome_t step3_fsm_process_incoming(
    step3_fsm_t *fsm,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
);

/*
 * Hop timer expiration handler (host-testable).
 * Advances channel or transitions to LOST if 3 consecutive hops missed.
 */
step3_timer_event_t step3_fsm_on_hop_timer(step3_fsm_t *fsm);

/* Target firmware entry point */
void step3_fhss_mock_run(void);

#endif /* STEP3_FHSS_MOCK_H */
