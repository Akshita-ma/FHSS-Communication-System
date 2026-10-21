#ifndef FHSS_PROTOCOL_H
#define FHSS_PROTOCOL_H

#include <stdint.h>
#include "status.h"
#include "config.h"

/*
 * Packet direction:
 * 0 = Node A to Node B (TX/Master to RX/Responder)
 * 1 = Node B to Node A (RX/Responder to TX/Master)
 */
#define DIR_A_TO_B  0U
#define DIR_B_TO_A  1U

/*
 * Decoded FHSS Frame Structure
 */
typedef struct {
    uint8_t ver_type;   /* 0xA0 | TYPE */
    uint8_t type;       /* 1 = DATA, 2 = ACK, 3 = HOP */
    uint8_t seq;        /* Sequence number */
    uint8_t msg_id;     /* Message ID */
    uint8_t total;      /* Total fragments (1..8) */
    uint8_t idx;        /* Fragment index (0..7) */
    uint8_t last;       /* 1 if last fragment in round, else 0 */
    uint8_t next_ch;    /* Next hop channel (0..7) */
    uint8_t dwell_code; /* Dwell code (0..31) */
    uint8_t tag;        /* Integrity tag */
    uint8_t payload[MAX_PAYLOAD];
    uint8_t payload_len;
} fhss_frame_t;

/*
 * Replay protection state per direction
 */
typedef struct {
    uint8_t last_seq;
    uint8_t have_last;
} fhss_replay_t;

/*
 * Core protocol functions
 */
uint8_t  fhss_crc8(const uint8_t *data, uint8_t len);
uint8_t  fhss_crc8_update(uint8_t crc, const uint8_t *data, uint8_t len);
uint8_t  fhss_calc_mask(uint8_t seq, uint8_t dir);
uint8_t  fhss_calc_tag(const uint8_t *header_b0_b4, const uint8_t *payload, uint8_t payload_len);

status_t fhss_build_data(uint8_t seq, uint8_t msg_id, uint8_t total, uint8_t idx, uint8_t last,
                         uint8_t next_ch, uint8_t dwell_code,
                         const uint8_t *payload, uint8_t payload_len,
                         uint8_t *out_frame, uint8_t *out_len);

status_t fhss_build_ack(uint8_t seq, uint8_t msg_id, uint8_t total,
                        uint8_t bitmap, uint8_t rssi_raw,
                        uint8_t next_ch, uint8_t dwell_code,
                        uint8_t *out_frame, uint8_t *out_len);

status_t fhss_build_hop(uint8_t seq, uint8_t next_ch, uint8_t dwell_code,
                        uint8_t *out_frame, uint8_t *out_len);

status_t fhss_parse_frame(const uint8_t *frame, uint8_t len, uint8_t dir,
                          fhss_replay_t *replay, fhss_frame_t *out);

/*
 * Self-test vectors verification (FROZEN vectors per CONTEXT.md section 10).
 * Returns OK (0) if all vectors pass, or ERR_CRC/ERR_TAG on mismatch.
 */
status_t fhss_verify_vectors(void);

#endif /* FHSS_PROTOCOL_H */
