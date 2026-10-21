#include "fhss_protocol.h"

/*
 * CRC-8 SMBus: poly 0x07 (x^8 + x^2 + x + 1), init 0x00, no reflection, no final XOR.
 * Check value: CRC-8("123456789") == 0xF4.
 */
uint8_t fhss_crc8_update(uint8_t crc, const uint8_t *data, uint8_t len) {
    if (data == (const uint8_t *)0) {
        return crc;
    }
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t fhss_crc8(const uint8_t *data, uint8_t len) {
    return fhss_crc8_update(0x00U, data, len);
}

/*
 * Mask calculation per CONTEXT.md section 10:
 * s = KEY ^ (seq * 0x9E3779B1) ^ (dir ? 0x5A5A5A5A : 0)  (32-bit arithmetic)
 * if (s == 0) s = 1
 * s ^= s << 13;  s ^= s >> 17;  s ^= s << 5
 * mask = s >> 24
 */
uint8_t fhss_calc_mask(uint8_t seq, uint8_t dir) {
    uint32_t s = FHSS_KEY ^ ((uint32_t)seq * 0x9E3779B1UL) ^ (dir != 0U ? 0x5A5A5A5AUL : 0UL);
    if (s == 0U) {
        s = 1U;
    }
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return (uint8_t)(s >> 24);
}

/*
 * Tag calculation per CONTEXT.md section 10:
 * CRC-8 SMBus over b0 b1 b2 b3 b4(masked, as on wire) 0xA5 0xC3 0x1F 0x7B followed by payload bytes.
 */
uint8_t fhss_calc_tag(const uint8_t *header_b0_b4, const uint8_t *payload, uint8_t payload_len) {
    static const uint8_t key_bytes[4] = {0xA5U, 0xC3U, 0x1FU, 0x7BU};
    uint8_t crc = fhss_crc8(header_b0_b4, 5U);
    crc = fhss_crc8_update(crc, key_bytes, 4U);
    if (payload != (const uint8_t *)0 && payload_len > 0U) {
        crc = fhss_crc8_update(crc, payload, payload_len);
    }
    return crc;
}

status_t fhss_build_data(uint8_t seq, uint8_t msg_id, uint8_t total, uint8_t idx, uint8_t last,
                         uint8_t next_ch, uint8_t dwell_code,
                         const uint8_t *payload, uint8_t payload_len,
                         uint8_t *out_frame, uint8_t *out_len) {
    if (out_frame == (uint8_t *)0 || out_len == (uint8_t *)0) {
        return ERR_NULL;
    }
    if (payload_len > MAX_PAYLOAD) {
        return ERR_LEN;
    }
    if (total == 0U || total > MAX_FRAGMENTS || idx >= total) {
        return ERR_RANGE;
    }
    if (next_ch >= NUM_CHANNELS || dwell_code > MAX_DWELL_CODE) {
        return ERR_RANGE;
    }

    uint8_t mask = fhss_calc_mask(seq, DIR_A_TO_B);
    uint8_t plain_hop = (uint8_t)((next_ch << 5) | (dwell_code & 0x1FU));

    out_frame[0] = (uint8_t)(PKT_VERSION_PREFIX | PKT_TYPE_DATA);
    out_frame[1] = seq;
    out_frame[2] = msg_id;
    out_frame[3] = (uint8_t)(((total - 1U) << 5) | (idx << 2) | ((last != 0U) ? 2U : 0U));
    out_frame[4] = (uint8_t)(plain_hop ^ mask);

    if (payload != (const uint8_t *)0 && payload_len > 0U) {
        for (uint8_t i = 0; i < payload_len; i++) {
            out_frame[HEADER_LEN + i] = payload[i];
        }
    }

    out_frame[5] = fhss_calc_tag(out_frame, payload, payload_len);
    *out_len = (uint8_t)(HEADER_LEN + payload_len);
    return OK;
}

status_t fhss_build_ack(uint8_t seq, uint8_t msg_id, uint8_t total,
                        uint8_t bitmap, uint8_t rssi_raw,
                        uint8_t next_ch, uint8_t dwell_code,
                        uint8_t *out_frame, uint8_t *out_len) {
    if (out_frame == (uint8_t *)0 || out_len == (uint8_t *)0) {
        return ERR_NULL;
    }
    if (total == 0U || total > MAX_FRAGMENTS) {
        return ERR_RANGE;
    }
    if (next_ch >= NUM_CHANNELS || dwell_code > MAX_DWELL_CODE) {
        return ERR_RANGE;
    }

    uint8_t mask = fhss_calc_mask(seq, DIR_B_TO_A);
    uint8_t plain_hop = (uint8_t)((next_ch << 5) | (dwell_code & 0x1FU));

    out_frame[0] = (uint8_t)(PKT_VERSION_PREFIX | PKT_TYPE_ACK);
    out_frame[1] = seq;
    out_frame[2] = msg_id;
    out_frame[3] = (uint8_t)((total - 1U) << 5);
    out_frame[4] = (uint8_t)(plain_hop ^ mask);

    uint8_t ack_payload[2];
    ack_payload[0] = bitmap;
    ack_payload[1] = rssi_raw;

    out_frame[HEADER_LEN + 0] = bitmap;
    out_frame[HEADER_LEN + 1] = rssi_raw;

    out_frame[5] = fhss_calc_tag(out_frame, ack_payload, 2U);
    *out_len = (uint8_t)(HEADER_LEN + 2U);
    return OK;
}

status_t fhss_build_hop(uint8_t seq, uint8_t next_ch, uint8_t dwell_code,
                        uint8_t *out_frame, uint8_t *out_len) {
    if (out_frame == (uint8_t *)0 || out_len == (uint8_t *)0) {
        return ERR_NULL;
    }
    if (next_ch >= NUM_CHANNELS || dwell_code > MAX_DWELL_CODE) {
        return ERR_RANGE;
    }

    uint8_t mask = fhss_calc_mask(seq, DIR_A_TO_B);
    uint8_t plain_hop = (uint8_t)((next_ch << 5) | (dwell_code & 0x1FU));

    out_frame[0] = (uint8_t)(PKT_VERSION_PREFIX | PKT_TYPE_HOP);
    out_frame[1] = seq;
    out_frame[2] = 0x00U;
    out_frame[3] = 0x00U;
    out_frame[4] = (uint8_t)(plain_hop ^ mask);
    out_frame[5] = fhss_calc_tag(out_frame, (const uint8_t *)0, 0U);

    *out_len = HEADER_LEN;
    return OK;
}

status_t fhss_parse_frame(const uint8_t *frame, uint8_t len, uint8_t dir,
                          fhss_replay_t *replay, fhss_frame_t *out) {
    if (frame == (const uint8_t *)0 || out == (fhss_frame_t *)0) {
        return ERR_NULL;
    }
    if (len < HEADER_LEN || len > MAX_FRAME_LEN) {
        return ERR_LEN;
    }

    /* Verify version prefix */
    if ((frame[0] & PKT_VER_MASK) != PKT_VERSION_PREFIX) {
        return ERR_STATE;
    }

    uint8_t type = frame[0] & PKT_TYPE_MASK;
    if (type < PKT_TYPE_DATA || type > PKT_TYPE_HOP) {
        return ERR_STATE;
    }

    /* Verify TAG */
    uint8_t payload_len = (uint8_t)(len - HEADER_LEN);
    const uint8_t *payload = (payload_len > 0U) ? &frame[HEADER_LEN] : (const uint8_t *)0;
    uint8_t expected_tag = fhss_calc_tag(frame, payload, payload_len);
    if (frame[5] != expected_tag) {
        return ERR_TAG;
    }

    uint8_t seq = frame[1];

    /* Replay rule */
    if (replay != (fhss_replay_t *)0) {
        if (replay->have_last != 0U) {
            uint8_t diff = (uint8_t)(seq - replay->last_seq);
            if (diff < 1U || diff > 127U) {
                return ERR_REPLAY;
            }
        }
        replay->last_seq = seq;
        replay->have_last = 1U;
    }

    /* Unmask hop directive */
    uint8_t mask = fhss_calc_mask(seq, dir);
    uint8_t plain_hop = (uint8_t)(frame[4] ^ mask);

    out->ver_type = frame[0];
    out->type = type;
    out->seq = seq;
    out->msg_id = frame[2];
    out->total = (uint8_t)(((frame[3] >> 5) & 0x07U) + 1U);
    out->idx = (uint8_t)((frame[3] >> 2) & 0x07U);
    out->last = (uint8_t)((frame[3] >> 1) & 0x01U);
    out->next_ch = (uint8_t)((plain_hop >> 5) & 0x07U);
    out->dwell_code = (uint8_t)(plain_hop & 0x1FU);
    out->tag = frame[5];
    out->payload_len = payload_len;

    for (uint8_t i = 0; i < payload_len; i++) {
        out->payload[i] = frame[HEADER_LEN + i];
    }

    return OK;
}

status_t fhss_verify_vectors(void) {
    /* 1. CRC-8("123456789") == 0xF4 */
    static const uint8_t s_test_crc_input[9] = {'1','2','3','4','5','6','7','8','9'};
    if (fhss_crc8(s_test_crc_input, 9U) != 0xF4U) {
        return ERR_CRC;
    }

    /* 2. Masks per table in CONTEXT.md section 10 */
    if (fhss_calc_mask(0U, DIR_A_TO_B) != 0x03U) return ERR_TAG;
    if (fhss_calc_mask(1U, DIR_A_TO_B) != 0x52U) return ERR_TAG;
    if (fhss_calc_mask(255U, DIR_A_TO_B) != 0x29U) return ERR_TAG;
    if (fhss_calc_mask(0U, DIR_B_TO_A) != 0x30U) return ERR_TAG;
    if (fhss_calc_mask(17U, DIR_B_TO_A) != 0x35U) return ERR_TAG;

    /* 3. DATA Vector: A to B, seq=5, msg_id=3, TOTAL=8, IDX=2, LAST=0, next_ch=5, code=17, "HELLO" */
    static const uint8_t s_exp_data[11] = {
        0xA1U, 0x05U, 0x03U, 0xE8U, 0x6AU, 0x7DU, 0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU
    };
    uint8_t f_buf[MAX_FRAME_LEN];
    uint8_t f_len = 0U;
    static const uint8_t s_hello[5] = {'H','E','L','L','O'};

    if (fhss_build_data(5U, 3U, 8U, 2U, 0U, 5U, 17U, s_hello, 5U, f_buf, &f_len) != OK) {
        return ERR_STATE;
    }
    if (f_len != 11U) return ERR_LEN;
    for (uint8_t i = 0; i < 11U; i++) {
        if (f_buf[i] != s_exp_data[i]) return ERR_TAG;
    }

    /* 4. ACK Vector: B to A, seq=9, msg_id=3, TOTAL=8, bitmap=0xDB, rssi_raw=0x78, next_ch=0, code=0 */
    static const uint8_t s_exp_ack[8] = {
        0xA2U, 0x09U, 0x03U, 0xE0U, 0x35U, 0xDAU, 0xDBU, 0x78U
    };
    if (fhss_build_ack(9U, 3U, 8U, 0xDBU, 0x78U, 0U, 0U, f_buf, &f_len) != OK) {
        return ERR_STATE;
    }
    if (f_len != 8U) return ERR_LEN;
    for (uint8_t i = 0; i < 8U; i++) {
        if (f_buf[i] != s_exp_ack[i]) return ERR_TAG;
    }

    /* 5. HOP Vector: A to B, seq=200, next_ch=7, code=31, no payload */
    static const uint8_t s_exp_hop[6] = {
        0xA3U, 0xC8U, 0x00U, 0x00U, 0xA6U, 0x82U
    };
    if (fhss_build_hop(200U, 7U, 31U, f_buf, &f_len) != OK) {
        return ERR_STATE;
    }
    if (f_len != 6U) return ERR_LEN;
    for (uint8_t i = 0; i < 6U; i++) {
        if (f_buf[i] != s_exp_hop[i]) return ERR_TAG;
    }

    /* 6. Verify parsing and replay */
    fhss_frame_t parsed;
    fhss_replay_t replay = {0, 0};

    if (fhss_parse_frame(s_exp_data, 11U, DIR_A_TO_B, &replay, &parsed) != OK) {
        return ERR_TAG;
    }
    if (parsed.seq != 5U || parsed.msg_id != 3U || parsed.total != 8U ||
        parsed.idx != 2U || parsed.last != 0U || parsed.next_ch != 5U ||
        parsed.dwell_code != 17U || parsed.payload_len != 5U) {
        return ERR_STATE;
    }

    /* Replay rejection: duplicate packet with same seq should fail */
    if (fhss_parse_frame(s_exp_data, 11U, DIR_A_TO_B, &replay, &parsed) != ERR_REPLAY) {
        return ERR_REPLAY;
    }

    return OK;
}
