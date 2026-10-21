#include "step3_fhss_mock.h"
#include "step1_uart_test.h"
#include "fhss_protocol.h"
#include "log.h"
#include "hal.h"
#include <stddef.h>

#ifndef HOST_TEST
#include "air_usart2.h"
#include "stm32f4_regs.h"
#endif

void step3_fsm_init(step3_fsm_t *fsm) {
    if (fsm == NULL) return;
    fsm->state = STEP3_STATE_SEARCH;
    fsm->current_ch = FHSS_HOME_CHANNEL;
    fsm->current_dwell_ms = HOME_DWELL_MS;
    fsm->next_ch = FHSS_HOME_CHANNEL;
    fsm->next_dwell_ms = HOME_DWELL_MS;
    fsm->b_seq = 0;
    fsm->have_last = 0;
    fsm->last_seq = 0;
    fsm->got_packet_this_hop = 0;
    fsm->hop_reanchored = 1; /* Home hop is anchored by ACK; no re-anchoring in home hop */
    fsm->is_first_of_hop = 0;
    fsm->last_rejected_ch = 0;
    fsm->last_rejected_seq = 0;
    fsm->consecutive_missed = 0;
    fsm->rx_pass_count = 0;
    fsm->tx_ack_count = 0;
    fsm->env_err_count = 0;
    fsm->proto_err_count = 0;
    fsm->replay_count = 0;
    fsm->hop_count = 0;
}

step3_rx_outcome_t step3_fsm_process_incoming(
    step3_fsm_t *fsm,
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *ack_out,
    uint8_t *tx_ch_out,
    fhss_packet_t *rx_pkt_out
) {
    if (fsm == NULL) return STEP3_RX_IDLE;

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
        return STEP3_RX_IDLE;
    }
    if (env_res == STEP1_RX_DISCARD_GARBAGE) {
        return STEP3_RX_DISCARD_GARBAGE;
    }
    if (env_res == STEP1_RX_ERR_CRC) {
        fsm->env_err_count++;
        return STEP3_RX_ERR_ENV_CRC;
    }
    if (env_res == STEP1_RX_ERR_LEN) {
        fsm->env_err_count++;
        return STEP3_RX_ERR_ENV_LEN;
    }
    if (env_res == STEP1_RX_ERR_CHANNEL) {
        fsm->env_err_count++;
        return STEP3_RX_ERR_ENV_CH;
    }

    /* 2. Channel check: Delivery rule requires frame channel match current listening channel */
    if (ch != fsm->current_ch) {
        fsm->proto_err_count++;
        fsm->last_rejected_ch = ch;
        return STEP3_RX_REJECT_WRONG_CH;
    }

    /* 3. Pass frame body to existing frozen FHSS parser (verifies 0xA0 prefix and CRC-8 integrity tag) */
    fhss_packet_t parsed_pkt;
    status_t st = fhss_parse_packet(frame_buf, len, 0, &parsed_pkt);
    if (st != OK) {
        fsm->proto_err_count++;
        return STEP3_RX_REJECT_TAG;
    }

    /* 4. Semantic check on packet type (Node A transmits HOP=3 or DATA=1) */
    uint8_t type = (uint8_t)(parsed_pkt.ver_type & 0x0F);
    if (type != PKT_TYPE_HOP && type != PKT_TYPE_DATA) {
        fsm->proto_err_count++;
        return STEP3_RX_REJECT_SEMANTIC;
    }

    /* For DATA packets, validate fragment bounds: idx < total */
    if (type == PKT_TYPE_DATA) {
        uint8_t total = (uint8_t)(((parsed_pkt.frag >> 5) & 7) + 1);
        uint8_t idx   = (uint8_t)((parsed_pkt.frag >> 2) & 7);
        if (idx >= total) {
            fsm->proto_err_count++;
            return STEP3_RX_REJECT_SEMANTIC;
        }
    }

    /* 5. Unmask hop directive from byte 4 using Node A mask (dir=0) */
    uint8_t mask = fhss_generate_mask(parsed_pkt.seq, 0);
    uint8_t plaintext = (uint8_t)(parsed_pkt.hop_masked ^ mask);
    uint8_t next_ch = (uint8_t)(plaintext >> 5);
    uint8_t dwell_code = (uint8_t)(plaintext & DWELL_CODE_MAX);

    if (next_ch >= FHSS_NUM_CHANNELS || dwell_code > DWELL_CODE_MAX) {
        fsm->proto_err_count++;
        return STEP3_RX_REJECT_SEMANTIC;
    }

    /* 6. Replay rule check: accept if !have_last or (seq - last_seq) is 1..127 */
    if (fsm->have_last) {
        uint8_t diff = (uint8_t)(parsed_pkt.seq - fsm->last_seq);
        if (diff == 0 || diff > 127) {
            fsm->replay_count++;
            fsm->last_rejected_seq = parsed_pkt.seq;
            return STEP3_RX_REJECT_REPLAY;
        }
    }
    fsm->last_seq = parsed_pkt.seq;
    fsm->have_last = 1;

    /* 7. Valid packet accepted! Update state and hop directive */
    fsm->rx_pass_count++;
    fsm->is_first_of_hop = (!fsm->got_packet_this_hop) ? 1 : 0;
    fsm->got_packet_this_hop = 1;
    fsm->consecutive_missed = 0;
    fsm->next_ch = next_ch;
    fsm->next_dwell_ms = DWELL_MIN_MS + (uint32_t)dwell_code * DWELL_STEP_MS;

    if (rx_pkt_out != NULL) {
        *rx_pkt_out = parsed_pkt;
    }

    /* 8. Construct standard FHSS ACK packet response */
    if (ack_out != NULL) {
        ack_out->ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
        ack_out->seq = fsm->b_seq++;
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
        ack_out->payload[1] = air_rssi_raw(fsm->current_ch);
#else
        ack_out->payload[1] = 0xC8; /* 200 = -100 dBm */
#endif
        ack_out->payload_len = 2;
        fsm->tx_ack_count++;
    }

    if (tx_ch_out != NULL) {
        *tx_ch_out = fsm->current_ch;
    }

    /* 9. Handle state transition */
    if (fsm->state == STEP3_STATE_SEARCH) {
        fsm->state = STEP3_STATE_LOCKED;
        fsm->current_ch = FHSS_HOME_CHANNEL;
        fsm->current_dwell_ms = HOME_DWELL_MS;
        fsm->hop_reanchored = 1; /* Home hop is anchored by ACK; do not re-anchor */
        return STEP3_RX_PASS_LOCKED;
    }

    return STEP3_RX_PASS_CONTINUED;
}

step3_timer_event_t step3_fsm_on_hop_timer(step3_fsm_t *fsm) {
    if (fsm == NULL) return STEP3_TIMER_EVENT_NONE;

    if (!fsm->got_packet_this_hop) {
        fsm->consecutive_missed++;
        if (fsm->consecutive_missed >= SYNC_LOST_HOPS_THRESHOLD) {
            fsm->state = STEP3_STATE_SEARCH;
            fsm->current_ch = FHSS_HOME_CHANNEL;
            fsm->current_dwell_ms = HOME_DWELL_MS;
            fsm->have_last = 0;
            fsm->got_packet_this_hop = 0;
            fsm->hop_reanchored = 1;
            fsm->consecutive_missed = 0;
            return STEP3_TIMER_EVENT_LOST;
        }
    } else {
        fsm->consecutive_missed = 0;
    }

    /* Advance to scheduled next channel and dwell */
    fsm->current_ch = fsm->next_ch;
    fsm->current_dwell_ms = fsm->next_dwell_ms;
    fsm->got_packet_this_hop = 0;
    fsm->hop_reanchored = 0; /* New hop ready to be re-anchored on first valid packet */
    fsm->is_first_of_hop = 0;
    fsm->hop_count++;

    return STEP3_TIMER_EVENT_HOP;
}

#ifndef HOST_TEST

static volatile uint8_t g_hop_switch_flag = 0;

static void on_hop_timer_cb(void) {
    /* Toggle PB1 logic analyzer trace pin inside TIM2 interrupt */
    GPIOB->ODR ^= (1UL << 1);
    g_hop_switch_flag = 1;
}

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

static void log_compact_rx_pass(const fhss_packet_t *pkt, uint8_t next_ch, uint32_t dwell_ms) {
    char msg[64];
    int p = 0;
    const char pfx[] = "RX type=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];

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

    const char s_next[] = " next=";
    for (int i = 0; s_next[i] != '\0'; i++) msg[p++] = s_next[i];
    msg[p++] = (char)('0' + next_ch);

    const char s_dwell[] = " dwell=";
    for (int i = 0; s_dwell[i] != '\0'; i++) msg[p++] = s_dwell[i];
    append_dec(msg, &p, dwell_ms);

    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_compact_ack_sent(uint8_t seq, uint8_t ch) {
    char msg[32];
    int p = 0;
    const char pfx[] = "ACK_TX seq=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, seq);

    const char s_ch[] = " ch=";
    for (int i = 0; s_ch[i] != '\0'; i++) msg[p++] = s_ch[i];
    msg[p++] = (char)('0' + ch);

    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_compact_reject_channel(uint8_t rx_ch, uint8_t cur_ch) {
    char msg[36];
    int p = 0;
    const char pfx[] = "REJECT wrong_ch rx=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    msg[p++] = (char)('0' + rx_ch);

    const char s_cur[] = " cur=";
    for (int i = 0; s_cur[i] != '\0'; i++) msg[p++] = s_cur[i];
    msg[p++] = (char)('0' + cur_ch);

    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_compact_reject_replay(uint8_t seq, uint8_t last_seq) {
    char msg[36];
    int p = 0;
    const char pfx[] = "REJECT replay seq=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    append_dec(msg, &p, seq);

    const char s_last[] = " last=";
    for (int i = 0; s_last[i] != '\0'; i++) msg[p++] = s_last[i];
    append_dec(msg, &p, last_seq);

    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_locked_event(uint8_t ch, uint32_t delta) {
    char msg[32];
    int p = 0;
    const char pfx[] = "LOCKED ch=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    msg[p++] = (char)('0' + ch);
    const char d_str[] = " delta=";
    for (int i = 0; d_str[i] != '\0'; i++) msg[p++] = d_str[i];
    append_dec(msg, &p, delta);
    msg[p] = '\0';
    Log_Msg("SYNC", msg);
}

static void log_hop_event(uint8_t ch, uint32_t delta) {
    char msg[32];
    int p = 0;
    const char ch_str[] = "ch=";
    for (int i = 0; ch_str[i] != '\0'; i++) msg[p++] = ch_str[i];
    msg[p++] = (char)('0' + ch);
    const char d_str[] = " delta=";
    for (int i = 0; d_str[i] != '\0'; i++) msg[p++] = d_str[i];
    append_dec(msg, &p, delta);
    msg[p] = '\0';
    Log_Msg("HOP", msg);
}

static void log_stats_summary(const step3_fsm_t *fsm) {
    char buf[80];
    int p = 0;
    const char p1[] = "rx_pass=";
    for (int i = 0; p1[i] != '\0'; i++) buf[p++] = p1[i];
    append_dec(buf, &p, fsm->rx_pass_count);

    const char p2[] = " tx_ack=";
    for (int i = 0; p2[i] != '\0'; i++) buf[p++] = p2[i];
    append_dec(buf, &p, fsm->tx_ack_count);

    const char p3[] = " hops=";
    for (int i = 0; p3[i] != '\0'; i++) buf[p++] = p3[i];
    append_dec(buf, &p, fsm->hop_count);

    const char p4[] = " ch=";
    for (int i = 0; p4[i] != '\0'; i++) buf[p++] = p4[i];
    buf[p++] = (char)('0' + fsm->current_ch);

    buf[p] = '\0';
    Log_Msg("STAT", buf);
}

void step3_fhss_mock_run(void) {
    /* 1. Initialize USART2 wire mock and trace pins */
    air_usart2_init();
    air_trace_pins_init();

    /* 2. Print boot banners */
    Log_Msg("BOOT", "node=B test=step3_fhss_mock");
    Log_Msg("BOOT", "link=USART2 baud=115200 PA2=TX PA3=RX");
    Log_Msg("RADIO", "Bypassed for Step 3 mock test");

    /* 3. Run known-answer vector self-test */
    if (fhss_self_test_vectors()) {
        Log_Msg("TEST", "vectors PASS");
    } else {
        Log_Msg("TEST", "vectors FAIL");
        while (1) {}
    }

    /* 4. Initialize FSM */
    step3_fsm_t fsm;
    step3_fsm_init(&fsm);

    Log_Msg("SYNC", "SEARCH ch=0");

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

        /* 5. Check TIM2 hop timer flag */
        if (g_hop_switch_flag) {
            g_hop_switch_flag = 0;
            step3_timer_event_t evt = step3_fsm_on_hop_timer(&fsm);
            if (evt == STEP3_TIMER_EVENT_LOST) {
                /* Immediately reset trace pins */
                GPIOB->BSRR = (1UL << (5 + 16)); /* PB5 LOW in SEARCH */
                GPIOB->BSRR = (7UL << (13 + 16)); /* PB13..PB15 = 0 */
                Log_Msg("SYNC", "LOST");
                Log_Msg("SYNC", "SEARCH ch=0");
            } else if (evt == STEP3_TIMER_EVENT_HOP) {
                /* 1. Schedule next hop timer IMMEDIATELY with hardware TIM2 */
                TIM2_ScheduleHopMs(fsm.current_dwell_ms, on_hop_timer_cb);

                /* 2. Update PB13..PB15 trace with new channel immediately */
                GPIOB->BSRR = (7UL << (13 + 16)) | (((uint32_t)(fsm.current_ch & 7)) << 13);

                /* 3. Diagnostic logging AFTER time-critical timer schedule */
                log_hop_event(fsm.current_ch, fsm.current_dwell_ms);
            }
        }

        /* 6. Process incoming frames from USART2 ring buffer */
        uint16_t avail = air_usart2_rx_available();
        if (avail > 0) {
            step3_rx_outcome_t outcome = step3_fsm_process_incoming(
                &fsm,
                avail,
                air_usart2_rx_peek,
                air_usart2_rx_advance,
                &ack_pkt,
                &tx_ch,
                &rx_pkt
            );

            if (outcome == STEP3_RX_PASS_LOCKED) {
                /* 1. Transmit authentic FHSS ACK immediately */
                tx_len = 0;
                if (fhss_build_packet(&ack_pkt, tx_buf, &tx_len) == OK) {
                    air_tx(tx_ch, tx_buf, tx_len);
                }

                /* 2. Schedule home hop timer immediately upon PacketSent */
                TIM2_ScheduleHopMs(fsm.current_dwell_ms, on_hop_timer_cb);

                /* 3. Set trace pin PB5 HIGH immediately */
                GPIOB->BSRR = (1UL << 5);

                /* 4. Post-transmission diagnostic logging */
                log_compact_rx_pass(&rx_pkt, fsm.next_ch, fsm.next_dwell_ms);
                log_compact_ack_sent(ack_pkt.seq, tx_ch);
                log_locked_event(fsm.current_ch, fsm.current_dwell_ms);

            } else if (outcome == STEP3_RX_PASS_CONTINUED) {
                /* 1. If first packet of this hop, re-anchor TIM2 once */
                if (fsm.is_first_of_hop && !fsm.hop_reanchored) {
                    uint32_t now = millis();
                    uint32_t airtime_ms = (air_airtime_us((uint8_t)(HEADER_LEN + rx_pkt.payload_len)) + 999) / 1000;
                    uint32_t hop_start = now - airtime_ms - HOP_GUARD_MS;
                    uint32_t elapsed = now - hop_start;
                    if (elapsed < fsm.current_dwell_ms) {
                        uint32_t remaining = fsm.current_dwell_ms - elapsed;
                        if (remaining >= 1) {
                            TIM2_ScheduleHopMs(remaining, on_hop_timer_cb);
                        }
                    }
                    fsm.hop_reanchored = 1;
                }

                /* 2. Transmit authentic FHSS ACK immediately */
                tx_len = 0;
                if (fhss_build_packet(&ack_pkt, tx_buf, &tx_len) == OK) {
                    air_tx(tx_ch, tx_buf, tx_len);
                }

                /* 3. Post-transmission diagnostic logging */
                log_compact_rx_pass(&rx_pkt, fsm.next_ch, fsm.next_dwell_ms);
                log_compact_ack_sent(ack_pkt.seq, tx_ch);

            } else if (outcome == STEP3_RX_REJECT_WRONG_CH) {
                log_compact_reject_channel(fsm.last_rejected_ch, fsm.current_ch);
            } else if (outcome == STEP3_RX_REJECT_REPLAY) {
                log_compact_reject_replay(fsm.last_rejected_seq, fsm.last_seq);
            } else if (outcome == STEP3_RX_ERR_ENV_CRC) {
                Log_Msg("TEST", "REJECT env_crc");
            } else if (outcome == STEP3_RX_ERR_ENV_LEN) {
                Log_Msg("TEST", "REJECT env_len");
            } else if (outcome == STEP3_RX_ERR_ENV_CH) {
                Log_Msg("TEST", "REJECT env_ch_range");
            } else if (outcome == STEP3_RX_REJECT_TAG) {
                Log_Msg("TEST", "REJECT tag_crc");
            } else if (outcome == STEP3_RX_REJECT_SEMANTIC) {
                Log_Msg("TEST", "REJECT semantic");
            }
        }

        /* 7. Periodic stats report every 5 seconds */
        uint32_t now = millis();
        if ((uint32_t)(now - last_stat_ms) >= STATS_REPORT_INTERVAL_MS) {
            last_stat_ms = now;
            log_stats_summary(&fsm);
        }
    }
}

#endif
