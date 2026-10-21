#include "fhss_protocol.h"
#include <stddef.h>
#include <string.h>

uint8_t crc8_smbus(const uint8_t *data, uint8_t len) {
    return crc8_smbus_cont(data, len, CRC8_SMBUS_INIT);
}

uint8_t crc8_smbus_cont(const uint8_t *data, uint8_t len, uint8_t init) {
    uint8_t crc = init;
    if (data == NULL) {
        return crc;
    }
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ CRC8_SMBUS_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t fhss_generate_mask(uint8_t seq, uint8_t dir) {
    uint32_t s = FHSS_KEY ^ ((uint32_t)seq * 0x9E3779B1UL) ^ (dir ? 0x5A5A5A5AUL : 0);
    if (s == 0) {
        s = 1;
    }
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return (uint8_t)(s >> 24);
}

uint8_t fhss_compute_tag(const uint8_t *wire_hdr5, const uint8_t *payload, uint8_t payload_len) {
    uint8_t tag_buf[9 + MAX_PAYLOAD_LEN];
    if (wire_hdr5 == NULL) {
        return 0;
    }
    for (int i = 0; i < 5; i++) {
        tag_buf[i] = wire_hdr5[i];
    }
    tag_buf[5] = 0xA5;
    tag_buf[6] = 0xC3;
    tag_buf[7] = 0x1F;
    tag_buf[8] = 0x7B;

    if (payload != NULL && payload_len > 0) {
        if (payload_len > MAX_PAYLOAD_LEN) {
            payload_len = MAX_PAYLOAD_LEN;
        }
        for (uint8_t i = 0; i < payload_len; i++) {
            tag_buf[9 + i] = payload[i];
        }
    } else {
        payload_len = 0;
    }

    return crc8_smbus(tag_buf, (uint8_t)(9 + payload_len));
}

status_t fhss_build_packet(const fhss_packet_t *pkt, uint8_t *wire_out, uint8_t *wire_len) {
    if (pkt == NULL || wire_out == NULL || wire_len == NULL) {
        return ERR_NULL;
    }
    if (pkt->payload_len > MAX_PAYLOAD_LEN) {
        return ERR_LEN;
    }

    wire_out[0] = pkt->ver_type;
    wire_out[1] = pkt->seq;
    wire_out[2] = pkt->msg_id;
    wire_out[3] = pkt->frag;
    wire_out[4] = pkt->hop_masked;
    wire_out[5] = fhss_compute_tag(wire_out, pkt->payload, pkt->payload_len);

    for (uint8_t i = 0; i < pkt->payload_len; i++) {
        wire_out[HEADER_LEN + i] = pkt->payload[i];
    }
    *wire_len = (uint8_t)(HEADER_LEN + pkt->payload_len);

    return OK;
}

status_t fhss_parse_packet(const uint8_t *wire_in, uint8_t wire_len, uint8_t expected_dir, fhss_packet_t *pkt_out) {
    (void)expected_dir;
    if (wire_in == NULL || pkt_out == NULL) {
        return ERR_NULL;
    }
    if (wire_len < HEADER_LEN || wire_len > MAX_FRAME_LEN) {
        return ERR_LEN;
    }
    if ((wire_in[0] & 0xF0) != PKT_VER_PREFIX) {
        return ERR_TAG;
    }

    uint8_t payload_len = (uint8_t)(wire_len - HEADER_LEN);
    uint8_t computed_tag = fhss_compute_tag(wire_in, &wire_in[HEADER_LEN], payload_len);
    if (computed_tag != wire_in[5]) {
        return ERR_TAG;
    }

    pkt_out->ver_type = wire_in[0];
    pkt_out->seq = wire_in[1];
    pkt_out->msg_id = wire_in[2];
    pkt_out->frag = wire_in[3];
    pkt_out->hop_masked = wire_in[4];
    pkt_out->tag = wire_in[5];
    pkt_out->payload_len = payload_len;
    for (uint8_t i = 0; i < payload_len; i++) {
        pkt_out->payload[i] = wire_in[HEADER_LEN + i];
    }

    return OK;
}

uint8_t fhss_self_test_vectors(void) {
    /* 1. Mask vectors */
    if (fhss_generate_mask(0, 0) != 0x03) return 0;
    if (fhss_generate_mask(1, 0) != 0x52) return 0;
    if (fhss_generate_mask(255, 0) != 0x29) return 0;
    if (fhss_generate_mask(0, 1) != 0x30) return 0;
    if (fhss_generate_mask(17, 1) != 0x35) return 0;

    /* 2. CRC-8 vector */
    const uint8_t check_str[9] = {'1','2','3','4','5','6','7','8','9'};
    if (crc8_smbus(check_str, 9) != 0xF4) return 0;

    /* 3. DATA vector */
    const uint8_t exp_data[11] = {
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x7D, 0x48, 0x45, 0x4C, 0x4C, 0x4F
    };
    fhss_packet_t p_data;
    p_data.ver_type = PKT_VER_PREFIX | PKT_TYPE_DATA;
    p_data.seq = 5;
    p_data.msg_id = 3;
    p_data.frag = (7 << 5) | (2 << 2) | (0 << 1); /* TOTAL=8, IDX=2, LAST=0 -> 0xE8 */
    p_data.hop_masked = (uint8_t)(((5 << 5) | 17) ^ fhss_generate_mask(5, 0)); /* next_ch=5, code=17 */
    p_data.payload[0] = 'H'; p_data.payload[1] = 'E'; p_data.payload[2] = 'L'; p_data.payload[3] = 'L'; p_data.payload[4] = 'O';
    p_data.payload_len = 5;

    uint8_t wire_data[16];
    uint8_t len_data = 0;
    if (fhss_build_packet(&p_data, wire_data, &len_data) != OK || len_data != 11) return 0;
    for (int i = 0; i < 11; i++) {
        if (wire_data[i] != exp_data[i]) return 0;
    }

    /* 4. ACK vector */
    const uint8_t exp_ack[8] = {
        0xA2, 0x09, 0x03, 0xE0, 0x35, 0xDA, 0xDB, 0x78
    };
    fhss_packet_t p_ack;
    p_ack.ver_type = PKT_VER_PREFIX | PKT_TYPE_ACK;
    p_ack.seq = 9;
    p_ack.msg_id = 3;
    p_ack.frag = (7 << 5); /* TOTAL=8 -> 0xE0 */
    p_ack.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(9, 1));
    p_ack.payload[0] = 0xDB;
    p_ack.payload[1] = 0x78;
    p_ack.payload_len = 2;

    uint8_t wire_ack[16];
    uint8_t len_ack = 0;
    if (fhss_build_packet(&p_ack, wire_ack, &len_ack) != OK || len_ack != 8) return 0;
    for (int i = 0; i < 8; i++) {
        if (wire_ack[i] != exp_ack[i]) return 0;
    }

    /* 5. HOP vector */
    const uint8_t exp_hop[6] = {
        0xA3, 0xC8, 0x00, 0x00, 0xA6, 0x82
    };
    fhss_packet_t p_hop;
    p_hop.ver_type = PKT_VER_PREFIX | PKT_TYPE_HOP;
    p_hop.seq = 200;
    p_hop.msg_id = 0;
    p_hop.frag = 0;
    p_hop.hop_masked = (uint8_t)(((7 << 5) | 31) ^ fhss_generate_mask(200, 0));
    p_hop.payload_len = 0;

    uint8_t wire_hop[16];
    uint8_t len_hop = 0;
    if (fhss_build_packet(&p_hop, wire_hop, &len_hop) != OK || len_hop != 6) return 0;
    for (int i = 0; i < 6; i++) {
        if (wire_hop[i] != exp_hop[i]) return 0;
    }

    return 1;
}
