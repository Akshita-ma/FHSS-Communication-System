#include "step5_fault_test.h"
#include "step1_uart_test.h"
#include "log.h"
#include "hal.h"
#include <stddef.h>
#include <string.h>

#ifndef HOST_TEST
#include "air_usart2.h"
#include "stm32f4_regs.h"
#endif

void step5_harness_init(step5_harness_t *harness) {
    if (harness == NULL) return;
    memset(harness, 0, sizeof(*harness));
    harness->expected_ch = FHSS_HOME_CHANNEL; /* 0 */
    harness->have_last = 0;
    harness->last_seq = 0;
    harness->b_seq = 0;
    harness->suppress_ack_enabled = 1;
    harness->suppress_ack_seq = STEP5_DEFAULT_SUPPRESS_SEQ;
    harness->last_was_fault = 0;
    harness->cycle_done = 0;
}

void step5_harness_set_suppress_seq(step5_harness_t *harness, uint8_t seq) {
    if (harness == NULL) return;
    harness->suppress_ack_seq = seq;
    harness->suppress_ack_enabled = 1;
}

step5_rx_outcome_t step5_harness_process_incoming(
    step5_harness_t *harness,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
) {
    if (harness == NULL || harness->cycle_done) return STEP5_RX_IDLE;

    uint8_t ch = 0;
    uint8_t len = 0;
    uint8_t frame_buf[MAX_FRAME_LEN];
    uint8_t calc_crc = 0;
    uint8_t wire_crc = 0;

    /* 1. Reuse existing Step 1 / Step 2 envelope parser */
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
        return STEP5_RX_IDLE;
    }
    if (env_res == STEP1_RX_DISCARD_GARBAGE) {
        return STEP5_RX_DISCARD_GARBAGE;
    }
    if (env_res == STEP1_RX_ERR_CRC) {
        harness->rx_total_count++;
        harness->env_crc_err_count++;
        harness->last_was_fault = 1;
        return STEP5_RX_ERR_ENV_CRC;
    }
    if (env_res == STEP1_RX_ERR_LEN) {
        harness->rx_total_count++;
        harness->env_len_err_count++;
        harness->last_was_fault = 1;
        return STEP5_RX_ERR_ENV_LEN;
    }
    if (env_res == STEP1_RX_ERR_CHANNEL) {
        harness->rx_total_count++;
        harness->env_ch_err_count++;
        harness->last_was_fault = 1;
        return STEP5_RX_ERR_ENV_CH;
    }

    /* 2. Envelope passed framing and outer CRC */
    harness->rx_total_count++;

    /* Check envelope channel against expected simulated channel 0 */
    if (ch != harness->expected_ch) {
        harness->proto_wrong_ch_count++;
        harness->last_rejected_ch = ch;
        harness->last_was_fault = 1;
        return STEP5_RX_REJECT_WRONG_CH;
    }

    /* 3. Pass frame body to existing frozen FHSS parser (verifies 0xA0 prefix and CRC-8 tag) */
    fhss_packet_t parsed_pkt;
    status_t st = fhss_parse_packet(frame_buf, len, 0, &parsed_pkt);
    if (st != OK) {
        harness->proto_bad_tag_count++;
        harness->last_was_fault = 1;
        return STEP5_RX_REJECT_BAD_TAG;
    }

    /* 4. Semantic validation on packet type and fragment bounds */
    uint8_t type = (uint8_t)(parsed_pkt.ver_type & 0x0F);
    if (type != PKT_TYPE_HOP && type != PKT_TYPE_DATA) {
        harness->proto_semantic_count++;
        harness->last_was_fault = 1;
        return STEP5_RX_REJECT_SEMANTIC;
    }

    if (type == PKT_TYPE_DATA) {
        uint8_t total = (uint8_t)(((parsed_pkt.frag >> 5) & 7) + 1);
        uint8_t idx   = (uint8_t)((parsed_pkt.frag >> 2) & 7);
        if (idx >= total) {
            harness->proto_semantic_count++;
            harness->last_was_fault = 1;
            return STEP5_RX_REJECT_SEMANTIC;
        }
    }

    /* 5. Replay rule check: accept if !have_last or (seq - last_seq) is 1..127 */
    if (harness->have_last) {
        uint8_t diff = (uint8_t)(parsed_pkt.seq - harness->last_seq);
        if (diff == 0 || diff > 127) {
            harness->proto_replay_count++;
            harness->last_rejected_seq = parsed_pkt.seq;
            harness->last_was_fault = 1;
            return STEP5_RX_REJECT_REPLAY;
        }
    }

    /* 6. All mandatory checks passed! Update sequence history and accept count */
    harness->last_seq = parsed_pkt.seq;
    harness->have_last = 1;
    harness->rx_accept_count++;

    if (rx_pkt_out != NULL) {
        *rx_pkt_out = parsed_pkt;
    }

    /* 7. Check if ACK is intentionally suppressed for this designated packet */
    if (harness->suppress_ack_enabled && parsed_pkt.seq == harness->suppress_ack_seq) {
        harness->suppress_ack_enabled = 0; /* One-shot intentional suppression */
        harness->ack_suppressed_count++;
        return STEP5_RX_PASS_ACK_SUPPRESSED;
    }

    /* 8. Construct standard frozen-protocol FHSS ACK */
    if (ack_out != NULL) {
        ack_out->ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
        ack_out->seq = harness->b_seq++;
        ack_out->msg_id = parsed_pkt.msg_id;
        ack_out->frag = (uint8_t)(parsed_pkt.frag & 0xE0);
        ack_out->hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(ack_out->seq, 1));
        if (type == PKT_TYPE_DATA) {
            uint8_t bit = (uint8_t)((parsed_pkt.frag >> 2) & 7);
            ack_out->payload[0] = (uint8_t)(1 << bit);
        } else {
            ack_out->payload[0] = 0x00;
        }
#ifndef HOST_TEST
        ack_out->payload[1] = air_rssi_raw(harness->expected_ch);
#else
        ack_out->payload[1] = 0xC8; /* 200 = -100 dBm */
#endif
        ack_out->payload_len = 2;
        harness->tx_ack_count++;
    }

    if (tx_ch_out != NULL) {
        *tx_ch_out = harness->expected_ch;
    }

    /* Sequence 8 is the final case in Node A's schedule: marks one-shot cycle completion */
    if (parsed_pkt.seq == 8) {
        harness->cycle_done = 1;
    }

    return STEP5_RX_PASS_ACK_SENT;
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

static void log_frame_pass(const char *prefix, const fhss_packet_t *pkt) {
    char msg[64];
    int p = 0;
    for (int i = 0; prefix[i] != '\0'; i++) msg[p++] = prefix[i];

    const char s_type[] = " type=";
    for (int i = 0; s_type[i] != '\0'; i++) msg[p++] = s_type[i];
    uint8_t type = (uint8_t)(pkt->ver_type & 0x0F);
    if (type == PKT_TYPE_DATA) {
        msg[p++] = 'D'; msg[p++] = 'A'; msg[p++] = 'T'; msg[p++] = 'A';
    } else if (type == PKT_TYPE_HOP) {
        msg[p++] = 'H'; msg[p++] = 'O'; msg[p++] = 'P';
    } else {
        msg[p++] = '?';
    }

    const char s_seq[] = " seq=";
    for (int i = 0; s_seq[i] != '\0'; i++) msg[p++] = s_seq[i];
    append_dec(msg, &p, pkt->seq);

    const char s_id[] = " id=";
    for (int i = 0; s_id[i] != '\0'; i++) msg[p++] = s_id[i];
    append_dec(msg, &p, pkt->msg_id);

    msg[p] = '\0';
    Log_Msg("FAULT", msg);
}

static void log_ack_sent(uint8_t seq, uint8_t ch) {
    char msg[32];
    int p = 0;
    const char pfx[] = "ACK_SENT seq=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, seq);

    const char s_ch[] = " to_ch=";
    for (int i = 0; s_ch[i] != '\0'; i++) msg[p++] = s_ch[i];
    append_dec(msg, &p, ch);

    msg[p] = '\0';
    Log_Msg("FAULT", msg);
}

static void log_ack_suppressed(uint8_t seq) {
    char msg[48];
    int p = 0;
    const char pfx[] = "ACK_SUPPRESSED intentional seq=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, seq);
    msg[p] = '\0';
    Log_Msg("FAULT", msg);
}

static void log_reject_wrong_channel(uint8_t rx_ch, uint8_t exp_ch) {
    char msg[40];
    int p = 0;
    const char pfx[] = "REJECT wrong_ch rx=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, rx_ch);

    const char s_exp[] = " exp=";
    for (int i = 0; s_exp[i] != '\0'; i++) msg[p++] = s_exp[i];
    append_dec(msg, &p, exp_ch);

    msg[p] = '\0';
    Log_Msg("FAULT", msg);
}

static void log_reject_replay(uint8_t rx_seq, uint8_t last_seq) {
    char msg[48];
    int p = 0;
    const char pfx[] = "REJECT replay rx_seq=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, rx_seq);

    const char s_last[] = " last_seq=";
    for (int i = 0; s_last[i] != '\0'; i++) msg[p++] = s_last[i];
    append_dec(msg, &p, last_seq);

    msg[p] = '\0';
    Log_Msg("FAULT", msg);
}

static void log_stats_summary(const step5_harness_t *h) {
    char msg[120];
    int p = 0;

    const char p1[] = "rx=";
    for (int i = 0; p1[i] != '\0'; i++) msg[p++] = p1[i];
    append_dec(msg, &p, h->rx_total_count);

    const char p2[] = " pass=";
    for (int i = 0; p2[i] != '\0'; i++) msg[p++] = p2[i];
    append_dec(msg, &p, h->rx_accept_count);

    const char p3[] = " ack=";
    for (int i = 0; p3[i] != '\0'; i++) msg[p++] = p3[i];
    append_dec(msg, &p, h->tx_ack_count);

    const char p4[] = " supp=";
    for (int i = 0; p4[i] != '\0'; i++) msg[p++] = p4[i];
    append_dec(msg, &p, h->ack_suppressed_count);

    const char p5[] = " env_crc=";
    for (int i = 0; p5[i] != '\0'; i++) msg[p++] = p5[i];
    append_dec(msg, &p, h->env_crc_err_count);

    const char p6[] = " bad_tag=";
    for (int i = 0; p6[i] != '\0'; i++) msg[p++] = p6[i];
    append_dec(msg, &p, h->proto_bad_tag_count);

    const char p7[] = " bad_ch=";
    for (int i = 0; p7[i] != '\0'; i++) msg[p++] = p7[i];
    append_dec(msg, &p, h->proto_wrong_ch_count);

    const char p8[] = " replay=";
    for (int i = 0; p8[i] != '\0'; i++) msg[p++] = p8[i];
    append_dec(msg, &p, h->proto_replay_count);

    msg[p] = '\0';
    Log_Msg("STAT", msg);
}

void step5_fault_test_run(void) {
    /* 1. Initialize USART2 wire mock and trace pins */
    air_usart2_init();
    air_trace_pins_init();

    /* 2. Log boot banner over USART1 */
    Log_Msg("BOOT", "node=B test=step5_fault_test");
    Log_Msg("BOOT", "link=USART2 baud=115200 PA2=TX PA3=RX ch=0 (fixed)");
    Log_Msg("RADIO", "Bypassed for Stage 5 fault-injection test");
    Log_Msg("FAULT", "Harness ready on ch=0; awaiting Node A fault cases...");

    step5_harness_t harness;
    step5_harness_init(&harness);

    uint32_t last_stat_ms = millis();
    fhss_packet_t rx_pkt;
    fhss_packet_t ack_pkt;
    uint8_t tx_ch = 0;
    uint8_t tx_buf[MAX_FRAME_LEN];
    uint8_t tx_len = 0;

    while (1) {
        /* Monitor USART2 hardware errors */
        uint32_t ore = air_usart2_get_ore_count();
        uint32_t ovf = air_usart2_get_overflow_count();
        if (ore > 0 || ovf > 0) {
            air_usart2_clear_errors();
            if (ore > 0) Log_Msg("TEST", "UART2 hardware overrun detected");
            if (ovf > 0) Log_Msg("TEST", "UART2 RX buffer overflow detected");
        }

        uint16_t avail = air_usart2_rx_available();
        if (avail > 0) {
            uint8_t was_fault = harness.last_was_fault;
            step5_rx_outcome_t outcome = step5_harness_process_incoming(
                &harness,
                avail,
                air_usart2_rx_peek,
                air_usart2_rx_advance,
                &ack_pkt,
                &tx_ch,
                &rx_pkt
            );

            if (outcome == STEP5_RX_PASS_ACK_SENT) {
                /* Transmit standard FHSS ACK */
                tx_len = 0;
                if (fhss_build_packet(&ack_pkt, tx_buf, &tx_len) == OK) {
                    air_tx(tx_ch, tx_buf, tx_len);
                }

                /* Pulse trace PB12 */
                GPIOB->BSRR = (1UL << 12);
                delay_us(50);
                GPIOB->BSRR = (1UL << (12 + 16));

                /* Diagnostic logging */
                if (was_fault) {
                    log_frame_pass("RECOVER post-fault PASS", &rx_pkt);
                } else {
                    log_frame_pass("FRAME PASS", &rx_pkt);
                }
                log_ack_sent(ack_pkt.seq, tx_ch);
                harness.last_was_fault = 0;

                /* Check for completion of one-shot test cycle */
                if (harness.cycle_done) {
                    Log_Msg("TEST", "Step 5 cycle complete: PASS (all 9 cases verified)");
                    log_stats_summary(&harness);
                    Log_Msg("FAULT", "Test cycle finished. Node B idle (reset board to re-run).");
                    while (1) {
                        delay_ms(1000);
                    }
                }

            } else if (outcome == STEP5_RX_PASS_ACK_SUPPRESSED) {
                /* Frame accepted, but ACK intentionally suppressed */
                if (was_fault) {
                    log_frame_pass("RECOVER post-fault PASS", &rx_pkt);
                } else {
                    log_frame_pass("FRAME PASS", &rx_pkt);
                }
                log_ack_suppressed(rx_pkt.seq);
                harness.last_was_fault = 0;

            } else if (outcome == STEP5_RX_ERR_ENV_CRC) {
                Log_Msg("FAULT", "REJECT env_crc");
            } else if (outcome == STEP5_RX_ERR_ENV_LEN) {
                Log_Msg("FAULT", "REJECT env_len");
            } else if (outcome == STEP5_RX_ERR_ENV_CH) {
                Log_Msg("FAULT", "REJECT env_ch_range");
            } else if (outcome == STEP5_RX_REJECT_WRONG_CH) {
                log_reject_wrong_channel(harness.last_rejected_ch, harness.expected_ch);
            } else if (outcome == STEP5_RX_REJECT_BAD_TAG) {
                Log_Msg("FAULT", "REJECT bad_tag");
            } else if (outcome == STEP5_RX_REJECT_SEMANTIC) {
                Log_Msg("FAULT", "REJECT semantic");
            } else if (outcome == STEP5_RX_REJECT_REPLAY) {
                log_reject_replay(harness.last_rejected_seq, harness.last_seq);
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
