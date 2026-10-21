#ifndef FHSS_PROTOCOL_H
#define FHSS_PROTOCOL_H

#include <stdint.h>
#include "status.h"
#include "config.h"

/*
 * FHSS Protocol framing, whitening, and CRC-8 integrity tags
 * (Shared between Node A and Node B - no MCU specifics or NODE ifdefs)
 */

typedef struct {
    uint8_t ver_type;
    uint8_t seq;
    uint8_t msg_id;
    uint8_t frag;
    uint8_t hop_masked;
    uint8_t tag;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t payload_len;
} fhss_packet_t;

uint8_t  crc8_smbus(const uint8_t *data, uint8_t len);
uint8_t  crc8_smbus_cont(const uint8_t *data, uint8_t len, uint8_t init);
uint8_t  fhss_generate_mask(uint8_t seq, uint8_t dir);
uint8_t  fhss_compute_tag(const uint8_t *wire_hdr5, const uint8_t *payload, uint8_t payload_len);
status_t fhss_build_packet(const fhss_packet_t *pkt, uint8_t *wire_out, uint8_t *wire_len);
status_t fhss_parse_packet(const uint8_t *wire_in, uint8_t wire_len, uint8_t expected_dir, fhss_packet_t *pkt_out);
uint8_t  fhss_self_test_vectors(void);

#endif /* FHSS_PROTOCOL_H */
