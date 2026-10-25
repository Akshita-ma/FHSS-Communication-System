#include "demo_node_b.h"
#include "air_usart2.h"
#include "stm32f4_regs.h"
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"
#include "hal.h"
#include "log.h"
#include <stddef.h>

#if defined(DEMO_RF_KEYING)
#include "sx1278_fsk.h"
#endif

typedef enum {
    DEMO_STATE_SEARCH = 0,
    DEMO_STATE_LOCKED,
    DEMO_STATE_LOST
} demo_sync_state_t;

static volatile uint8_t  g_hop_switch_flag = 0;
static demo_sync_state_t g_sync_state = DEMO_STATE_SEARCH;

static uint8_t  g_current_ch = FHSS_HOME_CHANNEL;
static uint32_t g_current_dwell_ms = HOME_DWELL_MS;

static uint8_t  g_next_ch = FHSS_HOME_CHANNEL;
static uint32_t g_next_dwell_ms = HOME_DWELL_MS;

static uint8_t  g_b_seq = 0;
static uint8_t  g_have_last = 0;
static uint8_t  g_last_seq = 0;

static uint8_t  g_got_packet_this_hop = 0;
static uint8_t  g_consecutive_missed = 0;

static void on_hop_timer_cb(void) {
    /* Toggle PB1 inside TIM2 hop callback (Logic Analyzer trace rule) */
    GPIOB->ODR ^= (1UL << 1);
    g_hop_switch_flag = 1;
}

static void log_locked_event(uint8_t ch, uint32_t delta) {
    char msg[32];
    int p = 0;
    const char pfx[] = "LOCKED ch=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    msg[p++] = (char)('0' + ch);
    const char d_str[] = " delta=";
    for (int i = 0; d_str[i] != '\0'; i++) msg[p++] = d_str[i];

    char num[12];
    int k = 0;
    uint32_t temp = delta;
    char rev[12];
    int r = 0;
    while (temp > 0) {
        rev[r++] = (char)('0' + (temp % 10));
        temp /= 10;
    }
    while (r > 0) {
        num[k++] = rev[--r];
    }
    for (int m = 0; m < k; m++) {
        msg[p++] = num[m];
    }
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

    char num[12];
    int k = 0;
    uint32_t temp = delta;
    char rev[12];
    int r = 0;
    while (temp > 0) {
        rev[r++] = (char)('0' + (temp % 10));
        temp /= 10;
    }
    while (r > 0) {
        num[k++] = rev[--r];
    }
    for (int m = 0; m < k; m++) {
        msg[p++] = num[m];
    }
    msg[p] = '\0';
    Log_Msg("HOP", msg);
}

void demo_node_b_run(void) {
    /* 1. Initialize USART2 wire mock and trace pins */
    air_usart2_init();
    air_trace_pins_init();

#if defined(DEMO_RF_KEYING)
    /* RF Keying initialization (writes only) */
    Radio_Reset_Pin(0);
    delay_ms(1);
    Radio_Reset_Pin(1);
    delay_ms(5);
    sx1278_write_reg(0x01, 0x01); /* FSK Standby */
    sx1278_write_reg(0x09, 0x8F); /* PA_BOOST */
    sx1278_write_reg(0x30, 0x90); /* Variable length, CRC ON */
    sx1278_write_reg(0x31, 0x40); /* Packet mode */
#endif

    /* 2. Print boot banners */
    Log_Msg("BOOT", "node=B link=USART2");
    Log_Msg("BOOT", "trace PB1=hop PB10=tx PB12=ack PB13..15=ch PA8=jam PB5=locked");

    /* 3. Execute known-answer vector self-test */
    if (fhss_self_test_vectors()) {
        Log_Msg("TEST", "vectors PASS");
    } else {
        Log_Msg("TEST", "vectors FAIL");
        while (1) {}
    }

    /* 4. Enter SEARCH state */
    g_sync_state = DEMO_STATE_SEARCH;
    g_current_ch = FHSS_HOME_CHANNEL;
    g_have_last = 0;
    g_last_seq = 0;
    GPIOB->BSRR = (1UL << (5 + 16)); /* PB5 LOW in SEARCH */
    GPIOB->BSRR = (7UL << (13 + 16)); /* PB13..PB15 = 0 */
    GPIOA->BSRR = (1UL << (8 + 16));  /* PA8 LOW */
    Log_Msg("SYNC", "SEARCH ch=0");

    uint8_t rx_buf[MAX_FRAME_LEN];
    uint8_t rx_len = 0;

    while (1) {
        if (g_sync_state == DEMO_STATE_SEARCH) {
            /* Listen continuously on channel 0 */
            if (air_rx_poll(FHSS_HOME_CHANNEL, rx_buf, &rx_len) == OK) {
                fhss_packet_t pkt;
                if (fhss_parse_packet(rx_buf, rx_len, 0, &pkt) == OK) {
                    /* Check replay rule */
                    if (g_have_last) {
                        uint8_t diff = (uint8_t)(pkt.seq - g_last_seq);
                        if (diff == 0 || diff > 127) {
                            continue; /* Ignore duplicate or replay */
                        }
                    }
                    g_last_seq = pkt.seq;
                    g_have_last = 1;

                    /* Unmask hop directive for Hop 1 */
                    uint8_t mask = fhss_generate_mask(pkt.seq, 0);
                    uint8_t plaintext = pkt.hop_masked ^ mask;
                    g_next_ch = (uint8_t)(plaintext >> 5);
                    uint8_t dwell_code = (uint8_t)(plaintext & DWELL_CODE_MAX);
                    g_next_dwell_ms = DWELL_MIN_MS + (uint32_t)dwell_code * DWELL_STEP_MS;

                    /* Reply with ACK on channel 0 */
                    fhss_packet_t ack;
                    ack.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
                    ack.seq = g_b_seq++;
                    ack.msg_id = pkt.msg_id;
                    ack.frag = (uint8_t)(pkt.frag & 0xE0);
                    ack.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(ack.seq, 1));
                    ack.payload[0] = 0x00; /* Bitmap */
                    ack.payload[1] = air_rssi_raw(FHSS_HOME_CHANNEL);
                    ack.payload_len = 2;

                    uint8_t tx_buf[MAX_FRAME_LEN];
                    uint8_t tx_len = 0;
                    fhss_build_packet(&ack, tx_buf, &tx_len);

                    /* High pulse of ~200 us on PB12 when sending ACK */
                    GPIOB->BSRR = (1UL << 12);
                    delay_us(200);
                    GPIOB->BSRR = (1UL << (12 + 16));

                    air_tx(FHSS_HOME_CHANNEL, tx_buf, tx_len);

                    /* Transition to LOCKED */
                    air_set_locked_timestamp(millis());
                    g_sync_state = DEMO_STATE_LOCKED;
                    GPIOB->BSRR = (1UL << 5); /* PB5 HIGH when LOCKED */

                    log_locked_event(FHSS_HOME_CHANNEL, HOME_DWELL_MS);

                    g_current_ch = FHSS_HOME_CHANNEL;
                    g_current_dwell_ms = HOME_DWELL_MS;
                    g_consecutive_missed = 0;
                    g_got_packet_this_hop = 0;

                    TIM2_ScheduleHopMs(HOME_DWELL_MS, on_hop_timer_cb);
                }
            }
        } else if (g_sync_state == DEMO_STATE_LOCKED) {
            /* Check if hop timer fired */
            if (g_hop_switch_flag) {
                g_hop_switch_flag = 0;

                if (!g_got_packet_this_hop) {
                    g_consecutive_missed++;
                    if (g_consecutive_missed >= SYNC_LOST_HOPS_THRESHOLD) {
                        /* 3 consecutive missed hops -> LOST -> SEARCH */
                        char lost_msg[16];
                        lost_msg[0] = 'L'; lost_msg[1] = 'O'; lost_msg[2] = 'S'; lost_msg[3] = 'T';
                        lost_msg[4] = ' '; lost_msg[5] = 'm'; lost_msg[6] = 'i'; lost_msg[7] = 's';
                        lost_msg[8] = 's'; lost_msg[9] = 'e'; lost_msg[10] = 'd'; lost_msg[11] = '=';
                        lost_msg[12] = (char)('0' + g_consecutive_missed);
                        lost_msg[13] = '\0';
                        Log_Msg("SYNC", lost_msg);

                        g_sync_state = DEMO_STATE_SEARCH;
                        g_current_ch = FHSS_HOME_CHANNEL;
                        g_have_last = 0;
                        GPIOB->BSRR = (1UL << (5 + 16)); /* PB5 LOW */
                        GPIOB->BSRR = (7UL << (13 + 16)); /* PB13..PB15 = 0 */
                        GPIOA->BSRR = (1UL << (8 + 16)); /* PA8 LOW */
                        Log_Msg("SYNC", "SEARCH ch=0");
                        continue;
                    }
                } else {
                    g_consecutive_missed = 0;
                }

                /* Switch to next channel */
                g_current_ch = g_next_ch;
                g_current_dwell_ms = g_next_dwell_ms;
                g_got_packet_this_hop = 0;

                /* Update PB13..PB15 trace with new channel */
                GPIOB->BSRR = (7UL << (13 + 16)) | (((uint32_t)(g_current_ch & 7)) << 13);

                /* Update PA8 trace (jam status) */
                if (air_is_channel_jammed(g_current_ch)) {
                    GPIOA->BSRR = (1UL << 8);
                } else {
                    GPIOA->BSRR = (1UL << (8 + 16));
                }

                log_hop_event(g_current_ch, g_current_dwell_ms);

                /* Schedule next hop */
                TIM2_ScheduleHopMs(g_current_dwell_ms, on_hop_timer_cb);

#if defined(DEMO_RF_KEYING)
                uint8_t msb = g_channel_table[g_current_ch].frf_msb;
                uint8_t mid = g_channel_table[g_current_ch].frf_mid;
                uint8_t lsb = g_channel_table[g_current_ch].frf_lsb;
                sx1278_write_reg(0x06, msb);
                sx1278_write_reg(0x07, mid);
                sx1278_write_reg(0x08, lsb);
#endif
            }

            /* Listen for incoming packets on current channel */
            if (air_rx_poll(g_current_ch, rx_buf, &rx_len) == OK) {
                fhss_packet_t pkt;
                if (fhss_parse_packet(rx_buf, rx_len, 0, &pkt) == OK) {
                    /* Check replay rule */
                    if (g_have_last) {
                        uint8_t diff = (uint8_t)(pkt.seq - g_last_seq);
                        if (diff == 0 || diff > 127) {
                            continue;
                        }
                    }
                    g_last_seq = pkt.seq;
                    g_have_last = 1;

                    /* Re-anchor hop timer on A's first valid packet each hop */
                    if (!g_got_packet_this_hop) {
                        g_got_packet_this_hop = 1;
                        g_consecutive_missed = 0;

                        uint32_t now = millis();
                        uint32_t airtime_ms = (air_airtime_us(rx_len) + 999) / 1000;
                        uint32_t hop_start = now - airtime_ms - HOP_GUARD_MS;
                        uint32_t elapsed_in_hop = now - hop_start;
                        if (elapsed_in_hop < g_current_dwell_ms) {
                            uint32_t remaining = g_current_dwell_ms - elapsed_in_hop;
                            if (remaining >= 1) {
                                TIM2_ScheduleHopMs(remaining, on_hop_timer_cb);
                            }
                        }
                    }

                    /* Extract next hop directive from b4 */
                    uint8_t mask = fhss_generate_mask(pkt.seq, 0);
                    uint8_t plaintext = pkt.hop_masked ^ mask;
                    g_next_ch = (uint8_t)(plaintext >> 5);
                    uint8_t dwell_code = (uint8_t)(plaintext & DWELL_CODE_MAX);
                    g_next_dwell_ms = DWELL_MIN_MS + (uint32_t)dwell_code * DWELL_STEP_MS;

                    /* Reply with ACK */
                    fhss_packet_t ack;
                    ack.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK);
                    ack.seq = g_b_seq++;
                    ack.msg_id = pkt.msg_id;
                    ack.frag = (uint8_t)(pkt.frag & 0xE0);
                    ack.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(ack.seq, 1));
                    ack.payload[0] = 0x00; /* Bitmap */
                    ack.payload[1] = air_rssi_raw(g_current_ch);
                    ack.payload_len = 2;

                    uint8_t tx_buf[MAX_FRAME_LEN];
                    uint8_t tx_len = 0;
                    fhss_build_packet(&ack, tx_buf, &tx_len);

                    GPIOB->BSRR = (1UL << 12);
                    delay_us(200);
                    GPIOB->BSRR = (1UL << (12 + 16));

                    air_tx(g_current_ch, tx_buf, tx_len);
                }
            }
        }
    }
}
