#include "custom_rx.h"
#include "step1_uart_test.h"
#include "log.h"
#include "hal.h"
#include <stddef.h>
#include <string.h>

#ifndef HOST_TEST
#include "air_usart2.h"
#include "stm32f4_regs.h"
#include "stm32_baremetal.h"
#endif

void custom_rx_harness_init(custom_rx_harness_t *harness) {
    if (harness == NULL) return;
    memset(harness, 0, sizeof(*harness));
    harness->expected_ch = FHSS_HOME_CHANNEL; /* 0 */
    harness->have_last = 0;
    harness->last_seq = 0;
    harness->b_seq = 0;
    harness->last_was_fault = 0;
    harness->has_completed_msg = 0;
}

custom_rx_outcome_t custom_rx_process_incoming(
    custom_rx_harness_t *harness,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
) {
    if (harness == NULL) return CUSTOM_RX_IDLE;

    uint8_t ch = 0;
    uint8_t len = 0;
    uint8_t frame_buf[MAX_FRAME_LEN];
    uint8_t calc_crc = 0;
    uint8_t wire_crc = 0;

    /* 1. Validate envelope framing, length, and outer CRC */
    step1_rx_result_t env_res = step1_parse_ring_buffer(
        available,
        peek_fn,
        advance_fn,
        &ch,
        &len,
        frame_buf,
        &calc_crc,
        &wire_crc
    );

    if (env_res == STEP1_RX_IDLE) {
        return CUSTOM_RX_IDLE;
    }
    if (env_res == STEP1_RX_DISCARD_GARBAGE) {
        return CUSTOM_RX_DISCARD_GARBAGE;
    }
    if (env_res == STEP1_RX_ERR_CRC) {
        harness->rx_total_count++;
        harness->env_crc_err_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_ERR_ENV_CRC;
    }
    if (env_res == STEP1_RX_ERR_LEN) {
        harness->rx_total_count++;
        harness->env_len_err_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_ERR_ENV_LEN;
    }
    if (env_res == STEP1_RX_ERR_CHANNEL) {
        harness->rx_total_count++;
        harness->env_ch_err_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_ERR_ENV_CH;
    }

    /* 2. Channel check: Delivery requires simulated channel 0 */
    if (ch != harness->expected_ch) {
        harness->rx_total_count++;
        harness->proto_wrong_ch_count++;
        harness->last_rejected_ch = ch;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_WRONG_CH;
    }

    /* 3. FHSS protocol prefix (0xA0) and CRC-8 integrity tag check */
    fhss_packet_t parsed_pkt;
    status_t st = fhss_parse_packet(frame_buf, len, 0, &parsed_pkt);
    if (st != OK) {
        harness->rx_total_count++;
        harness->proto_bad_tag_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_BAD_TAG;
    }

    /* 4. Validate packet type is DATA (0xA1) */
    uint8_t type = (uint8_t)(parsed_pkt.ver_type & 0x0F);
    if (type != PKT_TYPE_DATA) {
        harness->rx_total_count++;
        harness->proto_non_data_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_NON_DATA;
    }

    /* 5. Sequence / Replay validation (1..127 forward delta) */
    if (harness->have_last) {
        uint8_t diff = (uint8_t)(parsed_pkt.seq - harness->last_seq);
        if (diff == 0 || diff > 127) {
            harness->rx_total_count++;
            harness->proto_replay_count++;
            harness->last_rejected_seq = parsed_pkt.seq;
            harness->last_was_fault = 1;
            return CUSTOM_RX_REJECT_REPLAY;
        }
    }

    /* Sequence is fresh: accept packet into sequence history */
    harness->rx_total_count++;
    harness->last_seq = parsed_pkt.seq;
    harness->have_last = 1;

    if (rx_pkt_out != NULL) {
        *rx_pkt_out = parsed_pkt;
    }

    /* 6. Validate DATA fragment fields and semantic bounds */
    uint8_t total = (uint8_t)(((parsed_pkt.frag >> 5) & 7) + 1);
    uint8_t idx   = (uint8_t)((parsed_pkt.frag >> 2) & 7);
    uint8_t last  = (uint8_t)((parsed_pkt.frag >> 1) & 1);
    uint8_t rsvd  = (uint8_t)(parsed_pkt.frag & 1);

    if (idx >= total || total > ARQ_MAX_FRAGMENTS || parsed_pkt.payload_len == 0 ||
        parsed_pkt.payload_len > MAX_PAYLOAD_LEN || rsvd != 0) {
        harness->proto_metadata_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_METADATA;
    }

    /* Single-fragment message must have LAST=1 */
    if (total == 1 && last != 1) {
        harness->proto_metadata_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_METADATA;
    }

    /* Non-final fragments must be full length (16 bytes) to preserve linear byte offsets */
    if (idx < (uint8_t)(total - 1) && parsed_pkt.payload_len != MAX_PAYLOAD_LEN) {
        harness->proto_metadata_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_METADATA;
    }

    uint16_t byte_offset = (uint16_t)(idx * MAX_PAYLOAD_LEN);
    if ((uint16_t)(byte_offset + parsed_pkt.payload_len) > ARQ_MAX_MSG_LEN) {
        harness->proto_metadata_count++;
        harness->last_was_fault = 1;
        return CUSTOM_RX_REJECT_METADATA;
    }

    /* 7. Check if this is a duplicate fragment of the already completed message */
    if (harness->has_completed_msg && !harness->reasm.active &&
        parsed_pkt.msg_id == harness->last_completed.msg_id) {
        if (total != harness->last_completed.total_frags ||
            parsed_pkt.payload_len != harness->last_completed.frag_lens[idx] ||
            memcmp(&harness->last_completed.data[byte_offset], parsed_pkt.payload, parsed_pkt.payload_len) != 0) {
            harness->proto_conflict_count++;
            harness->last_was_fault = 1;
            return CUSTOM_RX_REJECT_CONFLICT;
        }

        harness->rx_dup_frag_count++;
        if (ack_out != NULL) {
            ack_out->ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
            ack_out->seq = harness->b_seq++;
            ack_out->msg_id = parsed_pkt.msg_id;
            ack_out->frag = (uint8_t)(parsed_pkt.frag & 0xE0);
            ack_out->hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(ack_out->seq, 1));
            ack_out->payload[0] = harness->last_completed.rx_bitmap;
#ifndef HOST_TEST
            ack_out->payload[1] = air_rssi_raw(harness->expected_ch);
#else
            ack_out->payload[1] = 0xC8;
#endif
            ack_out->payload_len = 2;
            harness->tx_ack_count++;
        }
        if (tx_ch_out != NULL) *tx_ch_out = harness->expected_ch;
        return CUSTOM_RX_DUPLICATE_FRAG;
    }

    /* 8. Reassembly context management */
    if (harness->reasm.active) {
        if (parsed_pkt.msg_id == harness->reasm.msg_id) {
            if (total != harness->reasm.total_frags) {
                /* TOTAL changed mid-message: drop */
                harness->proto_metadata_count++;
                harness->last_was_fault = 1;
                return CUSTOM_RX_REJECT_METADATA;
            }
        } else {
            /* New message ID: reset reassembly buffer */
            memset(&harness->reasm, 0, sizeof(harness->reasm));
            harness->reasm.active = 1;
            harness->reasm.msg_id = parsed_pkt.msg_id;
            harness->reasm.total_frags = total;
        }
    } else {
        /* Start new message reassembly */
        memset(&harness->reasm, 0, sizeof(harness->reasm));
        harness->reasm.active = 1;
        harness->reasm.msg_id = parsed_pkt.msg_id;
        harness->reasm.total_frags = total;
    }

    /* 9. Check for duplicate fragment within active message */
    uint8_t bit_mask = (uint8_t)(1 << idx);
    uint8_t is_duplicate = (harness->reasm.rx_bitmap & bit_mask) != 0;

    if (is_duplicate) {
        /*
         * Retransmitted duplicate fragment validation:
         * Must have identical metadata and payload matching already-stored fragment.
         * If conflict exists, report explicit rejection and do not overwrite or ACK.
         */
        if (total != harness->reasm.total_frags ||
            parsed_pkt.payload_len != harness->reasm.frag_lens[idx] ||
            memcmp(&harness->reasm.data[byte_offset], parsed_pkt.payload, parsed_pkt.payload_len) != 0) {
            harness->proto_conflict_count++;
            harness->last_was_fault = 1;
            return CUSTOM_RX_REJECT_CONFLICT;
        }

        harness->rx_dup_frag_count++;
        /* Construct ACK with existing cumulative bitmap */
        if (ack_out != NULL) {
            ack_out->ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
            ack_out->seq = harness->b_seq++;
            ack_out->msg_id = parsed_pkt.msg_id;
            ack_out->frag = (uint8_t)(parsed_pkt.frag & 0xE0);
            ack_out->hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(ack_out->seq, 1));
            ack_out->payload[0] = harness->reasm.rx_bitmap;
#ifndef HOST_TEST
            ack_out->payload[1] = air_rssi_raw(harness->expected_ch);
#else
            ack_out->payload[1] = 0xC8;
#endif
            ack_out->payload_len = 2;
            harness->tx_ack_count++;
        }
        if (tx_ch_out != NULL) *tx_ch_out = harness->expected_ch;
        return CUSTOM_RX_DUPLICATE_FRAG;
    }

    /* 10. Store fresh fragment data */
    memcpy(&harness->reasm.data[byte_offset], parsed_pkt.payload, parsed_pkt.payload_len);
    harness->reasm.frag_lens[idx] = parsed_pkt.payload_len;
    harness->reasm.rx_bitmap |= bit_mask;
    harness->rx_frag_count++;

    /* 11. Check message completion */
    uint8_t expected_bitmap = (uint8_t)((1 << total) - 1);
    uint8_t is_complete = (harness->reasm.rx_bitmap == expected_bitmap);

    if (is_complete) {
        uint16_t total_len = 0;
        for (uint8_t f = 0; f < total; f++) {
            total_len = (uint16_t)(total_len + harness->reasm.frag_lens[f]);
        }
        harness->reasm.total_len = total_len;
        harness->reasm.data[total_len] = '\0'; /* Safe null termination */
        harness->last_completed = harness->reasm;
        harness->has_completed_msg = 1;
        harness->completed_messages++;
        harness->reasm.active = 0; /* Deactivate until next message ID */
    }

    /* 12. Construct standard frozen-protocol ACK */
    if (ack_out != NULL) {
        ack_out->ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
        ack_out->seq = harness->b_seq++;
        ack_out->msg_id = parsed_pkt.msg_id;
        ack_out->frag = (uint8_t)(parsed_pkt.frag & 0xE0);
        ack_out->hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(ack_out->seq, 1));
        ack_out->payload[0] = (is_complete) ? expected_bitmap : harness->reasm.rx_bitmap;
#ifndef HOST_TEST
        ack_out->payload[1] = air_rssi_raw(harness->expected_ch);
#else
        ack_out->payload[1] = 0xC8;
#endif
        ack_out->payload_len = 2;
        harness->tx_ack_count++;
    }

    if (tx_ch_out != NULL) {
        *tx_ch_out = harness->expected_ch;
    }

    return is_complete ? CUSTOM_RX_PASS_MSG_COMPLETE : CUSTOM_RX_PASS_FRAG;
}

#ifndef HOST_TEST

static void append_dec(char *buf, int *p, uint32_t val) {
    if (val == 0) {
        buf[(*p)++] = '0';
        return;
    }
    char rev[12];
    int r = 0;
    while (val > 0) {
        rev[r++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (r > 0) {
        buf[(*p)++] = rev[--r];
    }
}

static void append_hex2(char *buf, int *p, uint8_t val) {
    const char hex[] = "0123456789ABCDEF";
    buf[(*p)++] = hex[(val >> 4) & 0x0F];
    buf[(*p)++] = hex[val & 0x0F];
}

static void log_frame_rx(const fhss_packet_t *pkt, uint8_t is_dup) {
    char msg[64];
    int p = 0;
    if (is_dup) {
        const char pfx[] = "Duplicate fragment: msg=";
        for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    } else {
        const char pfx[] = "DATA fragment received: msg=";
        for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    }
    append_dec(msg, &p, pkt->msg_id);

    const char s_frag[] = " frag=";
    for (int i = 0; s_frag[i] != '\0'; i++) msg[p++] = s_frag[i];
    append_dec(msg, &p, (pkt->frag >> 2) & 7);
    msg[p++] = '/';
    append_dec(msg, &p, ((pkt->frag >> 5) & 7) + 1);

    const char s_seq[] = " seq=";
    for (int i = 0; s_seq[i] != '\0'; i++) msg[p++] = s_seq[i];
    append_dec(msg, &p, pkt->seq);

    msg[p] = '\0';
    Log_Msg("RX", msg);
}

static void log_ack_tx(uint8_t msg_id, uint8_t bitmap, uint8_t ack_seq) {
    char msg[64];
    int p = 0;
    const char pfx[] = "ACK transmitted: msg=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, msg_id);

    const char s_bm[] = " bitmap=0x";
    for (int i = 0; s_bm[i] != '\0'; i++) msg[p++] = s_bm[i];
    append_hex2(msg, &p, bitmap);

    const char s_seq[] = " seq=";
    for (int i = 0; s_seq[i] != '\0'; i++) msg[p++] = s_seq[i];
    append_dec(msg, &p, ack_seq);

    msg[p] = '\0';
    Log_Msg("ARQ", msg);
}

static void log_msg_complete(const custom_reassembler_t *r) {
    char msg[64];
    int p = 0;
    const char pfx[] = "Message reassembled successfully (id=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, r->msg_id);

    const char s_len[] = " len=";
    for (int i = 0; s_len[i] != '\0'; i++) msg[p++] = s_len[i];
    append_dec(msg, &p, r->total_len);

    const char s_bm[] = " bitmap=0x";
    for (int i = 0; s_bm[i] != '\0'; i++) msg[p++] = s_bm[i];
    append_hex2(msg, &p, r->rx_bitmap);
    msg[p++] = ')';
    msg[p] = '\0';
    Log_Msg("DATA", msg);

    /* Plaintext application output */
    UART1_SendString("Received message: ");
    UART1_SendString((const char *)r->data);
    UART1_SendString("\r\n");
}

static void log_stats_summary(const custom_rx_harness_t *h) {
    char msg[128];
    int p = 0;

    const char p1[] = "rx_frames=";
    for (int i = 0; p1[i] != '\0'; i++) msg[p++] = p1[i];
    append_dec(msg, &p, h->rx_total_count);

    const char p2[] = " frags=";
    for (int i = 0; p2[i] != '\0'; i++) msg[p++] = p2[i];
    append_dec(msg, &p, h->rx_frag_count);

    const char p3[] = " dup=";
    for (int i = 0; p3[i] != '\0'; i++) msg[p++] = p3[i];
    append_dec(msg, &p, h->rx_dup_frag_count);

    const char p4[] = " acks=";
    for (int i = 0; p4[i] != '\0'; i++) msg[p++] = p4[i];
    append_dec(msg, &p, h->tx_ack_count);

    const char p5[] = " msgs=";
    for (int i = 0; p5[i] != '\0'; i++) msg[p++] = p5[i];
    append_dec(msg, &p, h->completed_messages);

    const char p6[] = " replay=";
    for (int i = 0; p6[i] != '\0'; i++) msg[p++] = p6[i];
    append_dec(msg, &p, h->proto_replay_count);

    msg[p] = '\0';
    Log_Msg("STAT", msg);
}

void custom_rx_run(void) {
    /* 1. Initialize USART2 wire mock and trace pins */
    air_usart2_init();
    air_trace_pins_init();

    /* 2. Boot banner */
    UART1_SendString("\r\n====================================\r\n");
    UART1_SendString(" FHSS COMMUNICATION SYSTEM\r\n");
    UART1_SendString(" Node B — Receiver\r\n");
    UART1_SendString("====================================\r\n\r\n");

    Log_Msg("BOOT", "node=B mode=custom_rx link=USART2 baud=115200 ch=0 (fixed)");
    Log_Msg("RADIO", "Bypassed; USART2 wired transport active");
    Log_Msg("CRYPTO", "ChaCha20-Poly1305: NOT_IMPLEMENTED (uncoordinated with Node A)");
    Log_Msg("ARQ", "Selective bitmap ARQ ready; awaiting Node A messages...");

    custom_rx_harness_t harness;
    custom_rx_harness_init(&harness);

    uint32_t last_stat_ms = millis();
    fhss_packet_t rx_pkt;
    fhss_packet_t ack_pkt;
    uint8_t tx_ch = 0;
    uint8_t tx_buf[MAX_FRAME_LEN];
    uint8_t tx_len = 0;

    while (1) {
        /* Monitor and clear USART2 hardware errors safely */
        uint32_t ore = air_usart2_get_ore_count();
        uint32_t ovf = air_usart2_get_overflow_count();
        if (ore > 0 || ovf > 0) {
            air_usart2_clear_errors();
            if (ore > 0) Log_Msg("TEST", "UART2 hardware overrun detected");
            if (ovf > 0) Log_Msg("TEST", "UART2 RX buffer overflow detected");
        }

        uint16_t avail = air_usart2_rx_available();
        if (avail > 0) {
            custom_rx_outcome_t outcome = custom_rx_process_incoming(
                &harness,
                avail,
                air_usart2_rx_peek,
                air_usart2_rx_advance,
                &ack_pkt,
                &tx_ch,
                &rx_pkt
            );

            if (outcome == CUSTOM_RX_PASS_FRAG || outcome == CUSTOM_RX_PASS_MSG_COMPLETE ||
                outcome == CUSTOM_RX_DUPLICATE_FRAG) {
                /* Transmit standard FHSS ACK */
                tx_len = 0;
                if (fhss_build_packet(&ack_pkt, tx_buf, &tx_len) == OK) {
                    air_tx(tx_ch, tx_buf, tx_len);
                }

                /* Pulse trace PB12 */
                GPIOB->BSRR = (1UL << 12);
                delay_us(50);
                GPIOB->BSRR = (1UL << (12 + 16));

                /* Log fragment receipt and ACK transmission */
                uint8_t is_dup = (outcome == CUSTOM_RX_DUPLICATE_FRAG) ? 1 : 0;
                log_frame_rx(&rx_pkt, is_dup);
                log_ack_tx(ack_pkt.msg_id, ack_pkt.payload[0], ack_pkt.seq);

                if (outcome == CUSTOM_RX_PASS_MSG_COMPLETE) {
                    log_msg_complete(&harness.last_completed);
                }

            } else if (outcome == CUSTOM_RX_REJECT_CONFLICT) {
                char msg[64];
                int p = 0;
                const char pfx[] = "REJECT duplicate_conflict msg=";
                for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
                append_dec(msg, &p, rx_pkt.msg_id);
                const char s_frag[] = " frag=";
                for (int i = 0; s_frag[i] != '\0'; i++) msg[p++] = s_frag[i];
                append_dec(msg, &p, (rx_pkt.frag >> 2) & 7);
                msg[p] = '\0';
                Log_Msg("ARQ", msg);

            } else if (outcome == CUSTOM_RX_ERR_ENV_CRC) {
                Log_Msg("FAULT", "REJECT env_crc");
            } else if (outcome == CUSTOM_RX_ERR_ENV_LEN) {
                Log_Msg("FAULT", "REJECT env_len");
            } else if (outcome == CUSTOM_RX_ERR_ENV_CH) {
                Log_Msg("FAULT", "REJECT env_ch_range");
            } else if (outcome == CUSTOM_RX_REJECT_WRONG_CH) {
                char msg[32];
                int p = 0;
                const char pfx[] = "REJECT wrong_ch rx=";
                for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
                append_dec(msg, &p, harness.last_rejected_ch);
                const char exp[] = " exp=0";
                for (int i = 0; exp[i] != '\0'; i++) msg[p++] = exp[i];
                msg[p] = '\0';
                Log_Msg("FAULT", msg);
            } else if (outcome == CUSTOM_RX_REJECT_BAD_TAG) {
                Log_Msg("FAULT", "REJECT bad_tag");
            } else if (outcome == CUSTOM_RX_REJECT_NON_DATA) {
                Log_Msg("FAULT", "REJECT non_data");
            } else if (outcome == CUSTOM_RX_REJECT_REPLAY) {
                char msg[48];
                int p = 0;
                const char pfx[] = "REJECT replay rx_seq=";
                for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
                append_dec(msg, &p, harness.last_rejected_seq);
                const char s_last[] = " last_seq=";
                for (int i = 0; s_last[i] != '\0'; i++) msg[p++] = s_last[i];
                append_dec(msg, &p, harness.last_seq);
                msg[p] = '\0';
                Log_Msg("FAULT", msg);
            } else if (outcome == CUSTOM_RX_REJECT_METADATA) {
                Log_Msg("FAULT", "REJECT bad_metadata");
            }
        }

        /* Periodic stats report every 5 seconds */
        uint32_t now = millis();
        if ((uint32_t)(now - last_stat_ms) >= 5000UL) {
            last_stat_ms = now;
            log_stats_summary(&harness);
        }
    }
}

#endif
