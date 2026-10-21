#include "common/status.h"
#include "common/config.h"
#include "common/hal.h"
#include "common/log.h"
#include "common/sx1278_fsk.h"
#include "stm32f1_regs.h"
#include "aead_chacha20_poly1305.h"
#include "nonce_mgr.h"
#include <string.h>

/* Baseline hardware initialization in stm32_baremetal.c */
void baremetal_init(void);

#if !defined(SELFTEST_SPI_LOOPBACK) && !defined(SELFTEST_PINS) && !defined(STEP1_UART_TEST) && !defined(STEP2_MOCK_TEST)
/*
 * Convert unsigned 32-bit integer to decimal ASCII string.
 */
static void uint32_to_ascii(uint32_t val, char *buf) {
    char tmp[12];
    uint32_t i = 0;

    if (val == 0U) {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }

    while (val > 0U) {
        tmp[i++] = (char)('0' + (val % 10U));
        val /= 10U;
    }

    uint32_t j = 0;
    while (i > 0U) {
        buf[j++] = tmp[--i];
    }
    buf[j] = '\0';
}

#if !defined(DEMO_USART_LINK) && !defined(STEP1_UART_TEST) && !defined(STEP2_MOCK_TEST)
/*
 * Convert 8-bit value to 2-digit hex string "0x.."
 */
static void uint8_to_hex(uint8_t val, char *buf) {
    static const char hex_digits[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    buf[2] = hex_digits[(val >> 4) & 0x0FU];
    buf[3] = hex_digits[val & 0x0FU];
    buf[4] = '\0';
}

#if !defined(STEP4_RADIO_SPI_DIAG)
/*
 * Print "[BOOT] node=A hello n\r\n"
 */
static void print_boot_hello(uint32_t n) {
    char num_buf[12];
    uint32_to_ascii(n, num_buf);

    UART1_SendString("[");
    UART1_SendString(TAG_BOOT);
    UART1_SendString("] node=A hello ");
    UART1_SendString(num_buf);
    UART1_SendString("\r\n");
}
#endif
#endif
#endif

#if defined(SELFTEST_SPI_LOOPBACK) || defined(STEP4_RADIO_SPI_DIAG)
/*
 * Convert 16-bit value to 6-digit hex string "0x...."
 */
static void uint16_to_hex(uint16_t val, char *buf) {
    static const char hex_digits[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    buf[2] = hex_digits[(val >> 12) & 0x0FU];
    buf[3] = hex_digits[(val >> 8) & 0x0FU];
    buf[4] = hex_digits[(val >> 4) & 0x0FU];
    buf[5] = hex_digits[val & 0x0FU];
    buf[6] = '\0';
}
#endif

#ifdef SELFTEST_SPI_LOOPBACK
/*
 * SELFTEST_SPI_LOOPBACK:
 * With the radio disconnected and PA7 (MOSI) jumpered to PA6 (MISO),
 * transfer 0xA5, 0x5A, 0x00, 0xFF and print "[TEST] SPI loopback PASS"
 * only if each byte is echoed. Also print raw SPI1 CR1 and SR register values.
 * Does not run the radio.
 */
static void selftest_spi_loopback_run(void) {
    static const uint8_t pattern[4] = {0xA5U, 0x5AU, 0x00U, 0xFFU};

    while (1) {
        uint16_t cr1 = (uint16_t)SPI1->CR1;
        uint16_t sr  = (uint16_t)SPI1->SR;

        char hex_cr1[7];
        char hex_sr[7];
        uint16_to_hex(cr1, hex_cr1);
        uint16_to_hex(sr, hex_sr);

        char reg_buf[48];
        char *p = reg_buf;
        const char *s_cr1 = "SPI1 CR1=";
        while (*s_cr1 != '\0') { *p++ = *s_cr1++; }
        for (int i = 0; i < 6; i++) { *p++ = hex_cr1[i]; }
        const char *s_sr = " SR=";
        while (*s_sr != '\0') { *p++ = *s_sr++; }
        for (int i = 0; i < 6; i++) { *p++ = hex_sr[i]; }
        *p = '\0';

        log_line(TAG_TEST, reg_buf);

        int pass = 1;
        for (int i = 0; i < 4; i++) {
            uint8_t rx = SPI1_Transfer(pattern[i]);
            if (rx != pattern[i]) {
                pass = 0;
            }
        }

        if (pass) {
            log_line(TAG_TEST, "SPI loopback PASS");
        } else {
            log_line(TAG_TEST, "SPI loopback FAIL");
        }

        delay_ms(2000);
    }
}
#endif

#ifdef SELFTEST_PINS
/*
 * SELFTEST_PINS:
 * Configure NSS (PB0), RESET (PA1), SCK (PA5) and MOSI (PA7) as plain GPIO outputs,
 * and for each pin in turn drive it high for 3 s then low for 3 s, printing
 * "[TEST] PINS driving <name> HIGH/LOW" before each change.
 * Configure MISO (PA6) as an input with pull-up and print its level every second.
 * Does not run the radio.
 */
static void selftest_pins_hold_3s(void) {
    for (int sec = 0; sec < 3; sec++) {
        delay_ms(1000);
        uint8_t miso = (uint8_t)((GPIOA->IDR & (1UL << 6)) ? 1U : 0U);
        if (miso != 0U) {
            log_line(TAG_TEST, "PINS MISO=1");
        } else {
            log_line(TAG_TEST, "PINS MISO=0");
        }
    }
}

static void selftest_pin_toggle(const char *name, volatile uint32_t *bsrr_reg, uint32_t pin_idx) {
    char msg[48];

    /* 1. Drive HIGH: print "[TEST] PINS driving <name> HIGH" before change */
    char *p = msg;
    const char *prefix = "PINS driving ";
    while (*prefix != '\0') { *p++ = *prefix++; }
    const char *n = name;
    while (*n != '\0') { *p++ = *n++; }
    const char *s_high = " HIGH";
    while (*s_high != '\0') { *p++ = *s_high++; }
    *p = '\0';
    log_line(TAG_TEST, msg);

    *bsrr_reg = (1UL << pin_idx);
    selftest_pins_hold_3s();

    /* 2. Drive LOW: print "[TEST] PINS driving <name> LOW" before change */
    p = msg;
    prefix = "PINS driving ";
    while (*prefix != '\0') { *p++ = *prefix++; }
    n = name;
    while (*n != '\0') { *p++ = *n++; }
    const char *s_low = " LOW";
    while (*s_low != '\0') { *p++ = *s_low++; }
    *p = '\0';
    log_line(TAG_TEST, msg);

    *bsrr_reg = (1UL << (pin_idx + 16U));
    selftest_pins_hold_3s();
}

static void selftest_pins_run(void) {
    /* Disable SPI1 peripheral so it does not override PA5/PA6/PA7 */
    SPI1->CR1 &= ~SPI_CR1_SPE;

    /* Reconfigure pins as plain GPIOs:
     * PB0: NSS   - GP Output Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     * PA1: RESET - GP Output Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     * PA5: SCK   - GP Output Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     * PA7: MOSI  - GP Output Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     * PA6: MISO  - Input with pull-up (MODE=00, CNF=10 -> 0x8, ODR=1)
     */
    GPIOB->CRL &= ~(0xFUL << 0);
    GPIOB->CRL |=  (0x3UL << 0);

    GPIOA->CRL &= ~(0xFUL << 4);
    GPIOA->CRL |=  (0x3UL << 4);

    GPIOA->CRL &= ~(0xFUL << 20);
    GPIOA->CRL |=  (0x3UL << 20);

    GPIOA->CRL &= ~(0xFUL << 28);
    GPIOA->CRL |=  (0x3UL << 28);

    GPIOA->CRL &= ~(0xFUL << 24);
    GPIOA->CRL |=  (0x8UL << 24);
    GPIOA->BSRR = (1UL << 6); /* Pull-up */

    /* Initialize all 4 test pins LOW */
    GPIOB->BSRR = (1UL << 16); /* PB0 LOW */
    GPIOA->BSRR = (1UL << 17) | (1UL << 21) | (1UL << 23); /* PA1, PA5, PA7 LOW */

    while (1) {
        selftest_pin_toggle("NSS", &(GPIOB->BSRR), 0U);
        selftest_pin_toggle("RESET", &(GPIOA->BSRR), 1U);
        selftest_pin_toggle("SCK", &(GPIOA->BSRR), 5U);
        selftest_pin_toggle("MOSI", &(GPIOA->BSRR), 7U);
    }
}
#endif

#if defined(DEMO_USART_LINK) || defined(STEP2_MOCK_TEST) || defined(STEP3_FHSS_MOCK) || defined(STEP5_FAULT_TEST) || defined(STEP6_DATA_MOCK) || defined(STEP7_ARQ_MOCK) || defined(INTERACTIVE_MODE)
#include "common/air.h"
#include "common/fhss_protocol.h"
#include "air_mock.h"
#endif

#if (defined(DEMO_USART_LINK) || defined(STEP3_FHSS_MOCK)) && !defined(STEP2_MOCK_TEST) && !defined(STEP5_FAULT_TEST) && !defined(STEP6_DATA_MOCK) && !defined(STEP7_ARQ_MOCK) && !defined(INTERACTIVE_MODE)

static void log_sync_locked(uint8_t ch, uint32_t delta) {
    char buf[48];
    char num_buf[12];
    char *p = buf;

    const char *s_ch = "LOCKED ch=";
    while (*s_ch != '\0') *p++ = *s_ch++;
    uint32_to_ascii((uint32_t)ch, num_buf);
    const char *s_num = num_buf;
    while (*s_num != '\0') *p++ = *s_num++;

    const char *s_delta = " delta=";
    while (*s_delta != '\0') *p++ = *s_delta++;
    uint32_to_ascii(delta, num_buf);
    s_num = num_buf;
    while (*s_num != '\0') *p++ = *s_num++;

    *p = '\0';
    log_line(TAG_SYNC, buf);
}

static void log_hop(uint32_t k, uint8_t ch, uint32_t delta) {
    char buf[48];
    char num_buf[12];
    char *p = buf;

    const char *s_k = "k=";
    while (*s_k != '\0') *p++ = *s_k++;
    uint32_to_ascii(k, num_buf);
    const char *s_num = num_buf;
    while (*s_num != '\0') *p++ = *s_num++;

    const char *s_ch = " ch=";
    while (*s_ch != '\0') *p++ = *s_ch++;
    uint32_to_ascii((uint32_t)ch, num_buf);
    s_num = num_buf;
    while (*s_num != '\0') *p++ = *s_num++;

    const char *s_delta = " delta=";
    while (*s_delta != '\0') *p++ = *s_delta++;
    uint32_to_ascii(delta, num_buf);
    s_num = num_buf;
    while (*s_num != '\0') *p++ = *s_num++;

    *p = '\0';
    log_line(TAG_HOP, buf);
}

static void log_rx_ack(uint8_t ch, uint8_t seq, uint8_t rssi_raw) {
    char buf[48];
    char num[12];
    char *p = buf;

    const char *s = "ACK ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " seq="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)seq, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " rssi="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)rssi_raw, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

static void log_rx_reject_channel(uint8_t wire_ch, uint8_t expected_ch, uint8_t missed) {
    char buf[64];
    char num[12];
    char *p = buf;

    const char *s = "REJECT wire_ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)wire_ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " expected="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)expected_ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " missed="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)missed, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

static void log_rx_reject_crc(uint8_t ch, uint8_t missed) {
    char buf[48];
    char num[12];
    char *p = buf;

    const char *s = "REJECT CRC ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " missed="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)missed, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

static void log_rx_reject_replay(uint8_t seq, uint8_t last_seq, uint8_t ch, uint8_t missed) {
    char buf[64];
    char num[12];
    char *p = buf;

    const char *s = "REJECT REPLAY seq="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)seq, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " last="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)last_seq, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " missed="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)missed, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

static void log_rx_reject_tag(uint8_t ch, uint8_t missed) {
    char buf[48];
    char num[12];
    char *p = buf;

    const char *s = "REJECT TAG ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " missed="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)missed, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

static void log_rx_reject_type(uint8_t type, uint8_t ch, uint8_t missed) {
    char buf[64];
    char num[12];
    char *p = buf;

    const char *s = "REJECT type="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)type, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " expected=ACK ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " missed="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)missed, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

static void log_rx_timeout(uint8_t ch, uint8_t missed) {
    char buf[48];
    char num[12];
    char *p = buf;

    const char *s = "TIMEOUT ch="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)ch, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " missed="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)missed, num); s = num; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_RX, buf);
}

/* Local PRNG for channel and dwell selection */
static uint32_t s_node_a_prng = 0x83A14B2CUL;

static uint32_t prng_next(void) {
    uint32_t x = s_node_a_prng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_node_a_prng = x;
    return x;
}

static uint8_t pick_next_channel(void) {
    return (uint8_t)(prng_next() % NUM_CHANNELS);
}

static volatile uint8_t s_hop_event = 0;

static void hop_callback(void) {
#ifdef DEMO_TRACE
    /* PB1 toggles at every hop switch */
    GPIOB->ODR ^= (1UL << 1);
#endif
    s_hop_event = 1U;
}

static void node_a_demo_run(void) {
#ifdef STEP3_FHSS_MOCK
    log_line(TAG_BOOT, "node=A test=STAGE3_FHSS_MOCK link=USART2 mock_rf=simulated");
    log_line(TAG_BOOT, "RF bypassed: SX1278 uninitialized; UART wire does not physically hop RF channels");
#else
    log_line(TAG_BOOT, "node=A link=USART2");
#endif
#ifdef DEMO_TRACE
    log_line(TAG_BOOT, "trace PB1=hop PB10=tx PB12=ack PB13..15=ch PA8=jam PB5=locked");
#endif

    air_mock_flush();

    if (fhss_verify_vectors() == OK) {
        log_line(TAG_TEST, "vectors PASS");
    } else {
        log_line(TAG_TEST, "vectors FAIL");
        while (1) { __asm__ volatile ("nop"); }
    }

    uint8_t s_current_ch = 0U;
    uint8_t s_next_ch = 5U;
    uint8_t s_current_dwell_code = 13U;
    uint8_t s_next_dwell_code = 13U;
    uint32_t s_current_dwell_ms = HOME_DWELL_MS;
    uint32_t s_next_dwell_ms = 80U + 5U * 13U; /* 145 ms */
    uint32_t s_hop_k = 0U;
    uint8_t s_tx_seq = 0U;
    fhss_replay_t s_replay = {0, 0};
    uint8_t s_missed_hops = 0U;

    enum { STATE_SEARCH = 0, STATE_LOCKED } state = STATE_SEARCH;

    while (1) {
        if (state == STATE_SEARCH) {
#ifdef DEMO_TRACE
            /* PB5 LOW in SEARCH */
            GPIOB->BRR = (1UL << 5);
            /* PB13..PB15 set to channel 0 */
            GPIOB->BSRR = (1UL << (13 + 16)) | (1UL << (14 + 16)) | (1UL << (15 + 16));
#endif
            air_mock_flush();
            s_current_ch = 0U;
            s_replay.have_last = 0U;
            log_line(TAG_SYNC, "SEARCH ch=0");

            /* Directive for hop 1: ch=5, delta=145 (code=13) */
            s_next_ch = 5U;
            s_next_dwell_code = 13U;
            s_next_dwell_ms = 80U + 5U * 13U;

            uint32_t last_beacon = 0U;

            while (state == STATE_SEARCH) {
                TIM2_Poll();

                uint32_t now = millis();
                if ((now - last_beacon) >= SYNC_BEACON_INTERVAL_MS) {
                    last_beacon = now;

                    /* Transmit HOP frame carrying directive for hop 1 */
                    uint8_t tx_buf[MAX_FRAME_LEN];
                    uint8_t tx_len = 0U;
                    fhss_build_hop(s_tx_seq++, s_next_ch, s_next_dwell_code, tx_buf, &tx_len);
                    air_tx(0U, tx_buf, tx_len);
                }

                /* Poll for ACK */
                uint8_t rx_buf[MAX_FRAME_LEN];
                uint8_t rx_len = 0U;
                if (air_rx_poll(0U, rx_buf, &rx_len) == OK) {
                    fhss_frame_t parsed;
                    if (fhss_parse_frame(rx_buf, rx_len, DIR_B_TO_A, &s_replay, &parsed) == OK) {
                        if (parsed.type == PKT_TYPE_ACK) {
                            mock_air_notify_locked();
#ifdef DEMO_TRACE
                            /* PB12 high pulse of ~200 us when valid ACK received */
                            GPIOB->BSRR = (1UL << 12);
                            uint32_t tp = micros();
                            while ((micros() - tp) < 200U) { __asm__ volatile ("nop"); }
                            GPIOB->BRR = (1UL << 12);

                            /* PB5 HIGH in LOCKED */
                            GPIOB->BSRR = (1UL << 5);
#endif
                            state = STATE_LOCKED;
                            s_hop_k = 0U;
                            s_missed_hops = 0U;
                            s_hop_event = 0U;
                            uint8_t ack_rssi = (parsed.payload_len >= 2U) ? parsed.payload[1] : 0U;
                            log_rx_ack(0U, parsed.seq, ack_rssi);
                            log_sync_locked(0U, HOME_DWELL_MS);

                            /* First ACK anchors the home hop */
                            TIM2_ScheduleHopMs(HOME_DWELL_MS, hop_callback);
                            break;
                        }
                    }
                }
            }
        }

        if (state == STATE_LOCKED) {
            while (state == STATE_LOCKED) {
                TIM2_Poll();

                if (s_hop_event != 0U) {
                    s_hop_event = 0U;
                    s_hop_k++;
                    s_current_ch = s_next_ch;
                    s_current_dwell_ms = s_next_dwell_ms;
                    s_current_dwell_code = s_next_dwell_code;

#ifdef DEMO_TRACE
                    /* Update PB13..PB15 with current channel */
                    uint32_t bsrr = 0U;
                    if ((s_current_ch & 1U) != 0U) bsrr |= (1UL << 13); else bsrr |= (1UL << (13 + 16));
                    if ((s_current_ch & 2U) != 0U) bsrr |= (1UL << 14); else bsrr |= (1UL << (14 + 16));
                    if ((s_current_ch & 4U) != 0U) bsrr |= (1UL << 15); else bsrr |= (1UL << (15 + 16));
                    GPIOB->BSRR = bsrr;
#endif
                    log_hop(s_hop_k, s_current_ch, s_current_dwell_ms);

                    /* Pick directive for hop k + 1 */
                    s_next_ch = pick_next_channel();
                    uint8_t next_code;
                    do {
                        next_code = (uint8_t)(prng_next() % 32U);
                    } while (next_code == s_current_dwell_code);
                    s_next_dwell_code = next_code;
                    s_next_dwell_ms = 80U + 5U * (uint32_t)next_code;

                    /* Schedule TIM2 for this hop dwell */
                    TIM2_ScheduleHopMs(s_current_dwell_ms, hop_callback);

                    /* Hop guard: wait HOP_GUARD_MS (8 ms) while taking listen-before-talk RSSI */
                    uint32_t t_guard = millis();
                    while ((millis() - t_guard) < HOP_GUARD_MS) {
                        TIM2_Poll();
                        __asm__ volatile ("nop");
                    }
                    uint8_t rssi_lbt = air_rssi_raw(s_current_ch);
                    (void)rssi_lbt;

                    /* Transmit hop directive for k + 1 */
                    uint8_t tx_buf[MAX_FRAME_LEN];
                    uint8_t tx_len = 0U;
                    fhss_build_hop(s_tx_seq++, s_next_ch, s_next_dwell_code, tx_buf, &tx_len);
                    air_tx(s_current_ch, tx_buf, tx_len);

                    /* Clear diagnostic counters before waiting for ACK */
                    air_mock_clear_diag();

                    /* Wait up to ACK_TIMEOUT_MS for ACK */
                    uint32_t t_ack_start = millis();
                    int ack_rcvd = 0;
                    status_t last_parse_err = OK;
                    uint8_t last_bad_type = 0U;
                    uint8_t last_rej_seq = 0U;
                    uint8_t ack_seq = 0U;
                    uint8_t ack_rssi = 0U;

                    while ((millis() - t_ack_start) < ACK_TIMEOUT_MS) {
                        TIM2_Poll();
                        uint8_t rx_buf[MAX_FRAME_LEN];
                        uint8_t rx_len = 0U;
                        if (air_rx_poll(s_current_ch, rx_buf, &rx_len) == OK) {
                            fhss_frame_t parsed;
                            status_t parse_res = fhss_parse_frame(rx_buf, rx_len, DIR_B_TO_A, &s_replay, &parsed);
                            if (parse_res == OK) {
                                if (parsed.type == PKT_TYPE_ACK) {
                                    ack_rcvd = 1;
                                    ack_seq = parsed.seq;
                                    ack_rssi = (parsed.payload_len >= 2U) ? parsed.payload[1] : 0U;
#ifdef DEMO_TRACE
                                    GPIOB->BSRR = (1UL << 12);
                                    uint32_t tp = micros();
                                    while ((micros() - tp) < 200U) { __asm__ volatile ("nop"); }
                                    GPIOB->BRR = (1UL << 12);
#endif
                                    break;
                                } else {
                                    last_bad_type = parsed.type;
                                }
                            } else {
                                last_parse_err = parse_res;
                                if (rx_len >= 2U) {
                                    last_rej_seq = rx_buf[1];
                                }
                            }
                        }
                    }

                    if (ack_rcvd != 0) {
                        s_missed_hops = 0U;
                        log_rx_ack(s_current_ch, ack_seq, ack_rssi);
                    } else {
                        s_missed_hops++;
                        air_mock_diag_t diag;
                        air_mock_get_diag(&diag);

                        if (last_parse_err == ERR_REPLAY) {
                            log_rx_reject_replay(last_rej_seq, s_replay.last_seq, s_current_ch, s_missed_hops);
                        } else if (last_parse_err == ERR_TAG) {
                            log_rx_reject_tag(s_current_ch, s_missed_hops);
                        } else if (last_bad_type != 0U) {
                            log_rx_reject_type(last_bad_type, s_current_ch, s_missed_hops);
                        } else if (diag.channel_mismatch_count > 0U) {
                            log_rx_reject_channel(diag.last_mismatch_wire_ch, s_current_ch, s_missed_hops);
                        } else if (diag.crc_error_count > 0U) {
                            log_rx_reject_crc(s_current_ch, s_missed_hops);
                        } else {
                            log_rx_timeout(s_current_ch, s_missed_hops);
                        }

                        if (s_missed_hops >= SYNC_MAX_MISSED_HOPS) {
                            log_line(TAG_SYNC, "LOST");
                            state = STATE_SEARCH;
                            break;
                        }
                    }
                }
            }
        }
    }
}
#endif /* DEMO_USART_LINK && !STEP2_MOCK_TEST */

#ifdef STEP1_UART_TEST
void air_mock_flush(void);
int  air_mock_read_byte(uint8_t *byte);
int  air_mock_send_raw(const uint8_t *data, uint8_t len);

/*
 * Step 1 Isolated USART2 Ping-Pong Test for Node A (STM32F103)
 * Transmits the 15-byte UART envelope every 500 ms:
 *   7E 00 0B A1 05 03 E8 6A 7D 48 45 4C 4C 4F 18
 * Listens on USART2 for raw response byte 0x06 with a bounded 50 ms timeout.
 * Logs [TEST] Step 1 Ping-Pong PASS or [TEST] Step 1 Ping-Pong TIMEOUT to USART1.
 */
static void step1_uart_test_run(void) {
    static const uint8_t s_ping_frame[15] = {
        0x7EU, 0x00U, 0x0BU,
        0xA1U, 0x05U, 0x03U, 0xE8U, 0x6AU, 0x7DU, 0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU,
        0x18U
    };

    log_line(TAG_BOOT, "node=A test=STEP1_PINGPONG usart=USART2");

    while (1) {
        uint32_t t_cycle_start = millis();

        /* Flush any stale receive bytes */
        air_mock_flush();

        /* Transmit exact 15-byte envelope on USART2 */
        air_mock_send_raw(s_ping_frame, 15U);

        /* Listen on USART2 for raw response byte 0x06 with bounded 50 ms timeout */
        int got_ack = 0;
        uint32_t t_listen = millis();
        while ((millis() - t_listen) < 50U) {
            uint8_t rx_byte = 0U;
            if (air_mock_read_byte(&rx_byte) == OK) {
                if (rx_byte == 0x06U) {
                    got_ack = 1;
                    break;
                }
            }
            __asm__ volatile ("nop");
        }

        if (got_ack != 0) {
            log_line(TAG_TEST, "Step 1 Ping-Pong PASS");
        } else {
            log_line(TAG_TEST, "Step 1 Ping-Pong TIMEOUT");
        }

        /* Wait for remainder of 500 ms period */
        uint32_t elapsed = millis() - t_cycle_start;
        if (elapsed < 500U) {
            delay_ms(500U - elapsed);
        }
    }
}
#endif

#ifdef STEP2_MOCK_TEST
void air_mock_flush(void);
int  air_mock_read_byte(uint8_t *byte);

/*
 * Stage 2 Dedicated Mock Transport Test for Node A (STM32F103)
 * Transmits the frozen known-answer frame via production air_tx() on channel 0 every 500 ms:
 *   Frame body: A1 05 03 E8 6A 7D 48 45 4C 4C 4F (11 bytes)
 * air_tx() serializes the wire envelope:
 *   7E 00 0B A1 05 03 E8 6A 7D 48 45 4C 4C 4F 18 (15 bytes)
 * Listens on USART2 for Node B test-harness receipt marker 0x06 with a bounded 50 ms timeout.
 * Logs TX event, receipt PASS, or TIMEOUT to USART1 debug console.
 */
static void step2_mock_run(void) {
    static const uint8_t s_frozen_frame[11] = {
        0xA1U, 0x05U, 0x03U, 0xE8U, 0x6AU, 0x7DU,
        0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU
    };

    log_line(TAG_BOOT, "node=A test=STAGE2_MOCK usart=USART2 ch=0");

    while (1) {
        uint32_t t_cycle_start = millis();

        /* Flush any stale receive bytes */
        air_mock_flush();

        /* Log local transmission event */
        log_line(TAG_TX, "Stage 2 air_tx ch=0 len=11");

        /* Transmit via production air_tx path (serializes 0x7E, ch, len, frame, CRC-8) */
        int tx_status = air_tx(0U, s_frozen_frame, (uint8_t)sizeof(s_frozen_frame));
        if (tx_status != (int)OK) {
            log_line(TAG_ERR, "Stage 2 air_tx failed");
        } else {
            /* Poll for Node B receipt marker 0x06 with bounded 50 ms timeout */
            int got_marker = 0;
            uint32_t t_listen = millis();
            while ((millis() - t_listen) < 50U) {
                uint8_t rx_byte = 0U;
                if (air_mock_read_byte(&rx_byte) == OK) {
                    if (rx_byte == 0x06U) {
                        got_marker = 1;
                        break;
                    }
                }
                __asm__ volatile ("nop");
            }

            if (got_marker != 0) {
                log_line(TAG_TEST, "Stage 2 Mock PASS");
            } else {
                log_line(TAG_TEST, "Stage 2 Mock TIMEOUT");
            }
        }

        /* Wait for remainder of 500 ms period */
        uint32_t elapsed = millis() - t_cycle_start;
        if (elapsed < 500U) {
            delay_ms(500U - elapsed);
        }
    }
}
#endif

#ifdef STEP3_FHSS_MOCK
static void step3_fhss_mock_run(void) {
    node_a_demo_run();
}
#endif

#ifdef STEP4_RADIO_SPI_DIAG
/*
 * Stage 4 SX1278 SPI Diagnostic for Node A (STM32F103)
 * Read-only register diagnosis:
 * - Logs established SPI1 configuration and pin mapping
 * - Performs hardware reset sequence via PA1
 * - Samples MISO (PA6) under idle and active NSS states
 * - Performs 32 consecutive reads of RegVersion (0x42)
 * - Reports individual reads, valid/fail counts, first unexpected value, and diagnostic inference
 * - NEVER enters TX/RX mode, transmits packets, or writes arbitrary registers
 */
static void step4_radio_spi_diag_run(void) {
    log_line(TAG_BOOT, "node=A test=STEP4_RADIO_SPI_DIAG mode=read-only");

    /* 1. Log SPI settings and GPIO assumptions established from source and registers */
    uint16_t cr1 = (uint16_t)SPI1->CR1;
    uint16_t sr  = (uint16_t)SPI1->SR;
    char hex_cr1[7], hex_sr[7];
    uint16_to_hex(cr1, hex_cr1);
    uint16_to_hex(sr, hex_sr);

    char reg_buf[64];
    char *p = reg_buf;
    const char *s_cr1 = "SPI1 CR1="; while (*s_cr1 != '\0') *p++ = *s_cr1++;
    for (int i = 0; i < 6; i++) *p++ = hex_cr1[i];
    const char *s_sr = " SR="; while (*s_sr != '\0') *p++ = *s_sr++;
    for (int i = 0; i < 6; i++) *p++ = hex_sr[i];
    *p = '\0';
    log_line(TAG_RADIO, reg_buf);

    log_line(TAG_RADIO, "Config: Mode=0 (CPOL=0, CPHA=0), MSB-first, 8-bit, Master, SSM=1, SSI=1");
    log_line(TAG_RADIO, "Clock: APB2=8MHz, Prescaler=/8 -> f_sck=1.0MHz (SX1278 max is 10MHz)");
    log_line(TAG_RADIO, "Pins: PA5=SCK(AF_PP) PA7=MOSI(AF_PP) PA6=MISO(IN_PULLUP) PB0=NSS(GP_PP) PA1=RESET(GP_PP)");

    /* Sample MISO line idle level with NSS HIGH (pull-up should read 1) */
    uint8_t miso_idle = (uint8_t)((GPIOA->IDR & (1UL << 6)) ? 1U : 0U);
    if (miso_idle != 0U) {
        log_line(TAG_RADIO, "Observed: MISO idle (NSS=HIGH) is HIGH (internal pull-up OK)");
    } else {
        log_line(TAG_RADIO, "Observed: MISO idle (NSS=HIGH) is LOW (WARNING: MISO held low externally)");
    }

    /* 2. Execute SX1278 hardware reset procedure (PA1 LOW 2ms, wait 20ms) */
    log_line(TAG_RADIO, "Executing SX1278 hardware reset sequence (PA1)...");
    sx1278_reset();
    log_line(TAG_RADIO, "Reset sequence complete (PA1 LOW 2ms, settled 20ms)");

    /* Sample MISO level with NSS asserted LOW prior to clocking */
    NSS_LOW();
    uint8_t miso_active = (uint8_t)((GPIOA->IDR & (1UL << 6)) ? 1U : 0U);
    NSS_HIGH();
    if (miso_active != 0U) {
        log_line(TAG_RADIO, "Observed: MISO selected (NSS=LOW, no clock) is HIGH");
    } else {
        log_line(TAG_RADIO, "Observed: MISO selected (NSS=LOW, no clock) is LOW");
    }

    /* 3. Diagnostic loop: perform 32 consecutive reads of RegVersion (0x42) */
    while (1) {
        uint8_t reads[32];
        uint32_t valid_count = 0U;
        uint32_t fail_count = 0U;
        uint8_t first_unexpected = 0U;
        int have_unexpected = 0;

        for (uint32_t i = 0; i < 32U; i++) {
            reads[i] = sx1278_read_reg(REG_VERSION);
            if (reads[i] == SX1278_EXPECTED_VERSION) {
                valid_count++;
            } else {
                fail_count++;
                if (have_unexpected == 0) {
                    first_unexpected = reads[i];
                    have_unexpected = 1;
                }
            }
            delay_ms(5U);
        }

        /* Report individual read values in 4 lines of 8 */
        for (uint32_t block = 0; block < 4U; block++) {
            char line_buf[64];
            char *lp = line_buf;
            const char *blk_pfx = "reads ["; while (*blk_pfx != '\0') *lp++ = *blk_pfx++;
            char nbuf[12];
            uint32_to_ascii(block * 8U, nbuf);
            if (block * 8U < 10U) *lp++ = '0';
            const char *s = nbuf; while (*s != '\0') *lp++ = *s++;
            const char *mid = ".."; while (*mid != '\0') *lp++ = *mid++;
            uint32_to_ascii(block * 8U + 7U, nbuf);
            if (block * 8U + 7U < 10U) *lp++ = '0';
            s = nbuf; while (*s != '\0') *lp++ = *s++;
            const char *blk_sfx = "]: "; while (*blk_sfx != '\0') *lp++ = *blk_sfx++;

            for (uint32_t j = 0; j < 8U; j++) {
                char hbuf[5];
                uint8_to_hex(reads[block * 8U + j], hbuf);
                for (int k = 0; k < 4; k++) *lp++ = hbuf[k];
                if (j < 7U) *lp++ = ' ';
            }
            *lp = '\0';
            log_line(TAG_RADIO, line_buf);
        }

        /* Report summary */
        char summary_buf[96];
        char *sp = summary_buf;
        const char *s1 = "RegVersion summary: total=32 valid="; while (*s1 != '\0') *sp++ = *s1++;
        char num[12];
        uint32_to_ascii(valid_count, num); const char *sn = num; while (*sn != '\0') *sp++ = *sn++;
        const char *s2 = " fail="; while (*s2 != '\0') *sp++ = *s2++;
        uint32_to_ascii(fail_count, num); sn = num; while (*sn != '\0') *sp++ = *sn++;
        const char *s3 = " first_unexp="; while (*s3 != '\0') *sp++ = *s3++;
        if (have_unexpected != 0) {
            char hbuf[5];
            uint8_to_hex(first_unexpected, hbuf);
            for (int k = 0; k < 4; k++) *sp++ = hbuf[k];
        } else {
            const char *none_s = "none"; while (*none_s != '\0') *sp++ = *none_s++;
        }
        const char *s4 = " expected=0x12"; while (*s4 != '\0') *sp++ = *s4++;
        *sp = '\0';
        log_line(TAG_TEST, summary_buf);

        /* Diagnostic classification */
        if (valid_count == 32U) {
            log_line(TAG_TEST, "Classification: PASS (SX1278 silicon ID 0x12 stable across 32 reads)");
        } else if (valid_count == 0U && first_unexpected == 0x00U) {
            log_line(TAG_TEST, "Classification: FAIL_MISO_STUCK_LOW (all reads 0x00; check MISO wiring, 3.3V VCC, or GND)");
        } else if (valid_count == 0U && first_unexpected == 0xFFU) {
            log_line(TAG_TEST, "Classification: FAIL_MISO_STUCK_HIGH (all reads 0xFF; radio unselected/unpowered/disconnected)");
        } else if (valid_count > 0U && fail_count > 0U) {
            log_line(TAG_TEST, "Classification: FAIL_INTERMITTENT (intermittent reads; check signal integrity, contact, or clock noise)");
        } else {
            log_line(TAG_TEST, "Classification: FAIL_UNEXPECTED_VALUE (unexpected silicon version; verify module chip type)");
        }

        delay_ms(2000U);
    }
}
#endif

#ifdef STEP5_FAULT_TEST
/*
 * Stage 5 Fault-Injection and Reliability Test Harness for Node A (STM32F103)
 *
 * Runs 8 deterministic test cases on channel 0 over the wired USART2 mock transport:
 * 1. Sequence wraparound: seq = 254, 255, 0, 1 -> expect valid ACKs
 * 2. Bad envelope CRC: raw send of valid frame with corrupted wire CRC -> expect TIMEOUT
 * 3. Bad protocol tag: fresh frame with corrupted TAG byte -> expect TIMEOUT
 * 4. Recovery: valid fresh frame with correct TAG -> expect valid ACK
 * 5. Duplicate replay: send identical seq 4 frame twice -> 1st ACK, 2nd TIMEOUT
 * 6. Wrong channel: send on envelope channel 1 -> expect TIMEOUT
 * 7. Recovery: send on channel 0 -> expect valid ACK
 * 8. ACK suppression & recovery: seq 7 suppressed -> TIMEOUT; seq 8 -> valid ACK
 */

#define STEP5_ACK_TIMEOUT_MS 100U

typedef enum {
    ACK_STAT_OK = 0,
    ACK_STAT_TIMEOUT,
    ACK_STAT_ENV_CRC,
    ACK_STAT_ENV_CH,
    ACK_STAT_BAD_TAG,
    ACK_STAT_REPLAY,
    ACK_STAT_BAD_TYPE,
    ACK_STAT_BAD_LEN,
    ACK_STAT_BAD_VER
} step5_ack_status_t;

typedef struct {
    step5_ack_status_t status;
    uint8_t ack_seq;        /* Node B's ACK sequence number */
    uint8_t ack_type;       /* Received packet type */
    uint8_t rx_wire_ch;     /* Wire channel if mismatch */
    uint8_t replay_last;    /* last_seq in replay state when replay rejected */
} step5_ack_detail_t;

static step5_ack_status_t step5_wait_ack(uint8_t tuned_ch, uint32_t timeout_ms,
                                         fhss_replay_t *replay,
                                         fhss_frame_t *out_ack,
                                         step5_ack_detail_t *detail) {
    uint32_t t_start = millis();
    step5_ack_status_t last_status = ACK_STAT_TIMEOUT;
    air_mock_clear_diag();

    if (detail != (step5_ack_detail_t *)0) {
        detail->status = ACK_STAT_TIMEOUT;
        detail->ack_seq = 0U;
        detail->ack_type = 0U;
        detail->rx_wire_ch = 0U;
        detail->replay_last = (replay != (fhss_replay_t *)0) ? replay->last_seq : 0U;
    }

    while ((millis() - t_start) < timeout_ms) {
        uint8_t rx_buf[MAX_FRAME_LEN];
        uint8_t rx_len = 0U;
        int poll_res = air_rx_poll(tuned_ch, rx_buf, &rx_len);

        if (poll_res == (int)OK) {
            fhss_frame_t parsed;
            status_t parse_res = fhss_parse_frame(rx_buf, rx_len, DIR_B_TO_A, replay, &parsed);

            if (parse_res == OK) {
                if (parsed.type == PKT_TYPE_ACK) {
                    if (out_ack != (fhss_frame_t *)0) {
                        *out_ack = parsed;
                    }
                    if (detail != (step5_ack_detail_t *)0) {
                        detail->status = ACK_STAT_OK;
                        detail->ack_seq = parsed.seq;
                        detail->ack_type = parsed.type;
                    }
                    return ACK_STAT_OK;
                } else {
                    last_status = ACK_STAT_BAD_TYPE;
                    if (detail != (step5_ack_detail_t *)0) {
                        detail->status = ACK_STAT_BAD_TYPE;
                        detail->ack_type = parsed.type;
                        detail->ack_seq = parsed.seq;
                    }
                }
            } else if (parse_res == ERR_TAG) {
                last_status = ACK_STAT_BAD_TAG;
                if (detail != (step5_ack_detail_t *)0) {
                    detail->status = ACK_STAT_BAD_TAG;
                    if (rx_len >= 2U) { detail->ack_seq = rx_buf[1]; }
                }
            } else if (parse_res == ERR_REPLAY) {
                last_status = ACK_STAT_REPLAY;
                if (detail != (step5_ack_detail_t *)0) {
                    detail->status = ACK_STAT_REPLAY;
                    if (rx_len >= 2U) { detail->ack_seq = rx_buf[1]; }
                    detail->replay_last = (replay != (fhss_replay_t *)0) ? replay->last_seq : 0U;
                }
            } else if (parse_res == ERR_LEN) {
                last_status = ACK_STAT_BAD_LEN;
                if (detail != (step5_ack_detail_t *)0) {
                    detail->status = ACK_STAT_BAD_LEN;
                }
            } else {
                last_status = ACK_STAT_BAD_VER;
                if (detail != (step5_ack_detail_t *)0) {
                    detail->status = ACK_STAT_BAD_VER;
                }
            }
        } else {
            /* Check mock transport diagnostics for wire CRC or channel mismatch */
            air_mock_diag_t diag;
            air_mock_get_diag(&diag);
            if (diag.crc_error_count > 0U) {
                last_status = ACK_STAT_ENV_CRC;
                if (detail != (step5_ack_detail_t *)0) {
                    detail->status = ACK_STAT_ENV_CRC;
                }
            } else if (diag.channel_mismatch_count > 0U) {
                last_status = ACK_STAT_ENV_CH;
                if (detail != (step5_ack_detail_t *)0) {
                    detail->status = ACK_STAT_ENV_CH;
                    detail->rx_wire_ch = diag.last_mismatch_wire_ch;
                }
            }
        }
        __asm__ volatile ("nop");
    }

    if (detail != (step5_ack_detail_t *)0) {
        detail->status = last_status;
    }
    return last_status;
}

static void log_step5_case(const char *case_id, uint8_t seq, const char *tx_res,
                           const char *exp, const char *obs, int pass) {
    char buf[112];
    char num[12];
    char *p = buf;

    const char *s = case_id; while (*s != '\0') *p++ = *s++;
    s = " seq="; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii((uint32_t)seq, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " tx="; while (*s != '\0') *p++ = *s++;
    s = tx_res; while (*s != '\0') *p++ = *s++;
    s = " exp="; while (*s != '\0') *p++ = *s++;
    s = exp; while (*s != '\0') *p++ = *s++;
    s = " obs="; while (*s != '\0') *p++ = *s++;
    s = obs; while (*s != '\0') *p++ = *s++;
    s = pass ? " -> PASS" : " -> FAIL"; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_TEST, buf);
}

static void format_obs(const step5_ack_detail_t *detail, char *out_buf) {
    char num[12];
    char *p = out_buf;

    switch (detail->status) {
        case ACK_STAT_OK: {
            const char *s = "ACK(b_seq="; while (*s != '\0') *p++ = *s++;
            uint32_to_ascii((uint32_t)detail->ack_seq, num);
            s = num; while (*s != '\0') *p++ = *s++;
            *p++ = ')';
            *p = '\0';
            break;
        }
        case ACK_STAT_TIMEOUT: {
            const char *s = "TIMEOUT(no_data)"; while (*s != '\0') *p++ = *s++;
            *p = '\0';
            break;
        }
        case ACK_STAT_ENV_CRC: {
            const char *s = "REJECT_ENV_CRC"; while (*s != '\0') *p++ = *s++;
            *p = '\0';
            break;
        }
        case ACK_STAT_ENV_CH: {
            const char *s = "REJECT_ENV_CH(wire="; while (*s != '\0') *p++ = *s++;
            uint32_to_ascii((uint32_t)detail->rx_wire_ch, num);
            s = num; while (*s != '\0') *p++ = *s++;
            *p++ = ')';
            *p = '\0';
            break;
        }
        case ACK_STAT_BAD_TAG: {
            const char *s = "REJECT_TAG(seq="; while (*s != '\0') *p++ = *s++;
            uint32_to_ascii((uint32_t)detail->ack_seq, num);
            s = num; while (*s != '\0') *p++ = *s++;
            *p++ = ')';
            *p = '\0';
            break;
        }
        case ACK_STAT_REPLAY: {
            const char *s = "REJECT_REPLAY(seq="; while (*s != '\0') *p++ = *s++;
            uint32_to_ascii((uint32_t)detail->ack_seq, num);
            s = num; while (*s != '\0') *p++ = *s++;
            s = ",last="; while (*s != '\0') *p++ = *s++;
            uint32_to_ascii((uint32_t)detail->replay_last, num);
            s = num; while (*s != '\0') *p++ = *s++;
            *p++ = ')';
            *p = '\0';
            break;
        }
        case ACK_STAT_BAD_TYPE: {
            const char *s = "REJECT_TYPE("; while (*s != '\0') *p++ = *s++;
            uint32_to_ascii((uint32_t)detail->ack_type, num);
            s = num; while (*s != '\0') *p++ = *s++;
            *p++ = ')';
            *p = '\0';
            break;
        }
        case ACK_STAT_BAD_LEN: {
            const char *s = "REJECT_LEN"; while (*s != '\0') *p++ = *s++;
            *p = '\0';
            break;
        }
        case ACK_STAT_BAD_VER:
        default: {
            const char *s = "REJECT_VER"; while (*s != '\0') *p++ = *s++;
            *p = '\0';
            break;
        }
    }
}

static void step5_fault_test_run(void) {
    log_line(TAG_BOOT, "node=A test=STAGE5_FAULT_TEST link=USART2 mock_rf=simulated");
    log_line(TAG_BOOT, "RF bypassed: SX1278 uninitialized; protocol fault testing over USART2 (one-shot)");

    delay_ms(100U);
    air_mock_flush();
    air_mock_clear_diag();

    if (fhss_verify_vectors() == OK) {
        log_line(TAG_TEST, "vectors PASS");
    } else {
        log_line(TAG_TEST, "vectors FAIL");
        while (1) { __asm__ volatile ("nop"); }
    }

    fhss_replay_t s_rx_replay = {0, 0};
    uint32_t total_cases = 8U;
    uint32_t passed_cases = 0U;

    log_line(TAG_TEST, "=== Stage 5 Fault Test (One-Shot) Start ===");

    /* -----------------------------------------------------------------
     * Case 1: Sequence Wraparound (seq = 254, 255, 0, 1)
     * ----------------------------------------------------------------- */
    {
        static const uint8_t wrap_seqs[4] = {254U, 255U, 0U, 1U};
        int c1_pass = 1;

        for (uint8_t i = 0; i < 4U; i++) {
            uint8_t seq = wrap_seqs[i];
            uint8_t tx_buf[MAX_FRAME_LEN];
            uint8_t tx_len = 0U;
            fhss_frame_t ack;
            step5_ack_detail_t detail;
            char obs[40];

            delay_ms(50U);
            air_mock_flush();
            fhss_build_hop(seq, 0U, 0U, tx_buf, &tx_len);
            int tx_res = air_tx(0U, tx_buf, tx_len);
            const char *tx_str = (tx_res == (int)OK) ? "OK" : "FAIL";

            step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
            format_obs(&detail, obs);
            int step_pass = (detail.status == ACK_STAT_OK) && (tx_res == (int)OK);
            if (!step_pass) {
                c1_pass = 0;
            }
            log_step5_case("Case 1 (Wraparound)", seq, tx_str, "ACK", obs, step_pass);
        }

        if (c1_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 1 Wraparound: PASS");
        } else {
            log_line(TAG_TEST, "Case 1 Wraparound: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 2: Bad Envelope CRC (seq = 2, raw corrupted CRC)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];

        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(2U, 0U, 0U, tx_buf, &tx_len);

        uint8_t raw_env[MAX_FRAME_LEN + 4U];
        raw_env[0] = 0x7EU;
        raw_env[1] = 0x00U; /* envelope ch 0 */
        raw_env[2] = tx_len;
        for (uint8_t i = 0; i < tx_len; i++) {
            raw_env[3U + i] = tx_buf[i];
        }
        uint8_t crc = fhss_crc8(&raw_env[1], (uint8_t)(tx_len + 2U));
        raw_env[3U + tx_len] = crc ^ 0xA5U; /* Deliberately corrupted wire CRC */
        uint8_t raw_len = (uint8_t)(tx_len + 4U);

        int tx_res = air_mock_send_raw(raw_env, raw_len);
        const char *tx_str = (tx_res == (int)OK) ? "OK(raw)" : "FAIL";

        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int step_pass = (detail.status == ACK_STAT_TIMEOUT) && (tx_res == (int)OK);
        log_step5_case("Case 2 (Bad Envelope CRC)", 2U, tx_str, "TIMEOUT", obs, step_pass);

        if (step_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 2 Bad Envelope CRC: PASS");
        } else {
            log_line(TAG_TEST, "Case 2 Bad Envelope CRC: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 3: Bad Protocol Tag (seq = 3, corrupt tag, valid envelope CRC)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];

        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(3U, 0U, 0U, tx_buf, &tx_len);
        tx_buf[5] ^= 0xFFU; /* Invert integrity tag */

        int tx_res = air_tx(0U, tx_buf, tx_len);
        const char *tx_str = (tx_res == (int)OK) ? "OK" : "FAIL";

        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int step_pass = (detail.status == ACK_STAT_TIMEOUT) && (tx_res == (int)OK);
        log_step5_case("Case 3 (Bad Protocol Tag)", 3U, tx_str, "TIMEOUT", obs, step_pass);

        if (step_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 3 Bad Protocol Tag: PASS");
        } else {
            log_line(TAG_TEST, "Case 3 Bad Protocol Tag: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 4: Recovery (seq = 3, correct tag)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];

        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(3U, 0U, 0U, tx_buf, &tx_len);

        int tx_res = air_tx(0U, tx_buf, tx_len);
        const char *tx_str = (tx_res == (int)OK) ? "OK" : "FAIL";

        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int step_pass = (detail.status == ACK_STAT_OK) && (tx_res == (int)OK);
        log_step5_case("Case 4 (Recovery)", 3U, tx_str, "ACK", obs, step_pass);

        if (step_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 4 Recovery: PASS");
        } else {
            log_line(TAG_TEST, "Case 4 Recovery: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 5: Duplicate Replay (seq = 4, sent twice)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];
        int c5_pass = 1;

        fhss_build_hop(4U, 0U, 0U, tx_buf, &tx_len);

        /* 1st copy: expect valid ACK */
        delay_ms(50U);
        air_mock_flush();
        int tx_res1 = air_tx(0U, tx_buf, tx_len);
        const char *tx_str1 = (tx_res1 == (int)OK) ? "OK" : "FAIL";
        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int pass1 = (detail.status == ACK_STAT_OK) && (tx_res1 == (int)OK);
        if (!pass1) { c5_pass = 0; }
        log_step5_case("Case 5 (Duplicate 1st)", 4U, tx_str1, "ACK", obs, pass1);

        /* 2nd copy (exact duplicate): expect TIMEOUT */
        delay_ms(50U);
        air_mock_flush();
        int tx_res2 = air_tx(0U, tx_buf, tx_len);
        const char *tx_str2 = (tx_res2 == (int)OK) ? "OK" : "FAIL";
        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int pass2 = (detail.status == ACK_STAT_TIMEOUT) && (tx_res2 == (int)OK);
        if (!pass2) { c5_pass = 0; }
        log_step5_case("Case 5 (Duplicate 2nd)", 4U, tx_str2, "TIMEOUT", obs, pass2);

        if (c5_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 5 Duplicate Replay: PASS");
        } else {
            log_line(TAG_TEST, "Case 5 Duplicate Replay: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 6: Wrong Envelope Channel (seq = 5 on ch 1, receiver on ch 0)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];

        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(5U, 0U, 0U, tx_buf, &tx_len);

        int tx_res = air_tx(1U, tx_buf, tx_len); /* envelope channel 1 */
        const char *tx_str = (tx_res == (int)OK) ? "OK" : "FAIL";

        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int step_pass = (detail.status == ACK_STAT_TIMEOUT) && (tx_res == (int)OK);
        log_step5_case("Case 6 (Wrong Channel)", 5U, tx_str, "TIMEOUT", obs, step_pass);

        if (step_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 6 Wrong Channel: PASS");
        } else {
            log_line(TAG_TEST, "Case 6 Wrong Channel: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 7: Recovery after Wrong Channel (seq = 6 on ch 0)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];

        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(6U, 0U, 0U, tx_buf, &tx_len);

        int tx_res = air_tx(0U, tx_buf, tx_len);
        const char *tx_str = (tx_res == (int)OK) ? "OK" : "FAIL";

        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int step_pass = (detail.status == ACK_STAT_OK) && (tx_res == (int)OK);
        log_step5_case("Case 7 (Recovery)", 6U, tx_str, "ACK", obs, step_pass);

        if (step_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 7 Recovery: PASS");
        } else {
            log_line(TAG_TEST, "Case 7 Recovery: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Case 8: ACK Suppression and Recovery (seq = 7 suppressed, seq = 8 OK)
     * ----------------------------------------------------------------- */
    {
        uint8_t tx_buf[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        fhss_frame_t ack;
        step5_ack_detail_t detail;
        char obs[40];
        int c8_pass = 1;

        /* 1st copy (seq 7, designated suppression frame): expect TIMEOUT */
        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(7U, 0U, 0U, tx_buf, &tx_len);
        int tx_res1 = air_tx(0U, tx_buf, tx_len);
        const char *tx_str1 = (tx_res1 == (int)OK) ? "OK" : "FAIL";
        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int pass1 = (detail.status == ACK_STAT_TIMEOUT) && (tx_res1 == (int)OK);
        if (!pass1) { c8_pass = 0; }
        log_step5_case("Case 8 (ACK Suppress)", 7U, tx_str1, "TIMEOUT", obs, pass1);

        /* 2nd copy (seq 8, recovery frame): expect valid ACK */
        delay_ms(50U);
        air_mock_flush();
        fhss_build_hop(8U, 0U, 0U, tx_buf, &tx_len);
        int tx_res2 = air_tx(0U, tx_buf, tx_len);
        const char *tx_str2 = (tx_res2 == (int)OK) ? "OK" : "FAIL";
        step5_wait_ack(0U, STEP5_ACK_TIMEOUT_MS, &s_rx_replay, &ack, &detail);
        format_obs(&detail, obs);
        int pass2 = (detail.status == ACK_STAT_OK) && (tx_res2 == (int)OK);
        if (!pass2) { c8_pass = 0; }
        log_step5_case("Case 8 (Recovery)", 8U, tx_str2, "ACK", obs, pass2);

        if (c8_pass) {
            passed_cases++;
            log_line(TAG_TEST, "Case 8 ACK Suppression & Recovery: PASS");
        } else {
            log_line(TAG_TEST, "Case 8 ACK Suppression & Recovery: FAIL");
        }
    }

    /* -----------------------------------------------------------------
     * Final One-Shot Summary and Idle State
     * ----------------------------------------------------------------- */
    char sum_buf[64];
    char num[12];
    char *p = sum_buf;
    const char *s = "Stage 5 Summary: "; while (*s != '\0') *p++ = *s++;
    uint32_to_ascii(passed_cases, num); s = num; while (*s != '\0') *p++ = *s++;
    *p++ = '/';
    uint32_to_ascii(total_cases, num); s = num; while (*s != '\0') *p++ = *s++;
    s = " cases PASS"; while (*s != '\0') *p++ = *s++;
    *p = '\0';
    log_line(TAG_TEST, sum_buf);

    if (passed_cases == total_cases) {
        log_line(TAG_TEST, "Stage 5 Fault Test Complete: ALL PASS (one-shot)");
    } else {
        log_line(TAG_TEST, "Stage 5 Fault Test Complete: FAIL (one-shot)");
    }

    /* Idle forever; do NOT restart at sequence 254 */
    log_line(TAG_BOOT, "Stage 5 idle; reset board to rerun");
    uint32_t idle_sec = 0;
    while (1) {
        delay_ms(5000U);
        idle_sec += 5U;
        char idle_buf[48];
        char num_idle[12];
        char *ip = idle_buf;
        const char *m = "idle ("; while (*m != '\0') *ip++ = *m++;
        uint32_to_ascii(idle_sec, num_idle); const char *sn = num_idle; while (*sn != '\0') *ip++ = *sn++;
        m = "s); press RESET to rerun"; while (*m != '\0') *ip++ = *m++;
        *ip = '\0';
        log_line(TAG_BOOT, idle_buf);
    }
}
#endif

#ifdef STEP6_DATA_MOCK
/*
 * Stage 6: End-to-End DATA Messaging over USART2 Mock Transport for Node A (STM32F103)
 *
 * Exercises application DATA messaging using the frozen FHSS protocol over the wired
 * USART2 mock transport (SX1278 radios bypassed).
 * Transmits two deterministic messages:
 *   1. Short single-fragment message: "HELLO_NODE_B" (12 bytes, msg_id=1, total=1)
 *   2. Multi-fragment message: "FHSS_DATA_TRANSPORT_TEST_PACKET_#2" (35 bytes, msg_id=2, total=3)
 *
 * Validates each fragment's standard FHSS ACK according to frozen protocol rules.
 * Reports local transmission, ACK validation, peer reconstruction confirmation status,
 * and payload match verification status separately.
 */

#define STEP6_ACK_TIMEOUT_MS   150U
#define STEP6_STARTUP_DELAY_MS 500U

typedef enum {
    STEP6_ACK_OK = 0,
    STEP6_ACK_TIMEOUT,
    STEP6_ACK_BAD_TAG,
    STEP6_ACK_REPLAY,
    STEP6_ACK_BAD_TYPE,
    STEP6_ACK_BAD_MSG_ID,
    STEP6_ACK_BAD_TOTAL,
    STEP6_ACK_FRAG_NOT_ACKED,
    STEP6_ACK_BAD_LEN
} step6_ack_status_t;

typedef struct {
    step6_ack_status_t status;
    uint8_t ack_seq;        /* Node B's independent ACK sequence number */
    uint8_t ack_msg_id;     /* Echoed msg_id */
    uint8_t ack_total;      /* Total fragments indicated in ACK */
    uint8_t ack_bitmap;     /* Fragment received bitmap */
    uint8_t ack_rssi;       /* Raw RSSI */
} step6_ack_detail_t;

typedef struct {
    uint8_t idx;
    uint8_t seq;
    uint8_t last;
    uint8_t chunk_len;
    uint8_t tx_ok;
    uint8_t ack_valid;
    step6_ack_detail_t ack_detail;
} step6_frag_report_t;

typedef struct {
    const char *text;
    uint8_t msg_id;
    uint8_t msg_len;
    uint8_t total_frags;
    uint8_t frags_tx_ok;
    uint8_t frags_ack_valid;
    uint8_t all_acked;
    step6_frag_report_t frags[MAX_FRAGMENTS];
} step6_msg_report_t;

static void step6_uint8_to_hex(uint8_t val, char *buf) {
    static const char hex_digits[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    buf[2] = hex_digits[(val >> 4) & 0x0FU];
    buf[3] = hex_digits[val & 0x0FU];
    buf[4] = '\0';
}

static void step6_append_str(char *buf, uint32_t max_len, uint32_t *pos, const char *s) {
    if (s == (const char *)0) return;
    while (*s != '\0' && (*pos + 1U) < max_len) {
        buf[*pos] = *s++;
        (*pos)++;
    }
    buf[*pos] = '\0';
}

static void step6_append_u32(char *buf, uint32_t max_len, uint32_t *pos, uint32_t val) {
    char num[12];
    uint32_to_ascii(val, num);
    step6_append_str(buf, max_len, pos, num);
}

static void step6_append_hex8(char *buf, uint32_t max_len, uint32_t *pos, uint8_t val) {
    char hex[5];
    step6_uint8_to_hex(val, hex);
    step6_append_str(buf, max_len, pos, hex);
}

static step6_ack_status_t step6_wait_ack(uint8_t tuned_ch, uint8_t exp_msg_id,
                                         uint8_t exp_total, uint8_t exp_idx,
                                         uint32_t timeout_ms,
                                         fhss_replay_t *replay,
                                         step6_ack_detail_t *detail) {
    uint32_t t_start = millis();
    step6_ack_status_t last_status = STEP6_ACK_TIMEOUT;
    air_mock_clear_diag();

    if (detail != (step6_ack_detail_t *)0) {
        detail->status = STEP6_ACK_TIMEOUT;
        detail->ack_seq = 0U;
        detail->ack_msg_id = 0U;
        detail->ack_total = 0U;
        detail->ack_bitmap = 0U;
        detail->ack_rssi = 0U;
    }

    while ((millis() - t_start) < timeout_ms) {
        uint8_t rx_buf[MAX_FRAME_LEN];
        uint8_t rx_len = 0U;
        int poll_res = air_rx_poll(tuned_ch, rx_buf, &rx_len);

        if (poll_res == (int)OK) {
            fhss_frame_t parsed;
            status_t parse_res = fhss_parse_frame(rx_buf, rx_len, DIR_B_TO_A, replay, &parsed);

            if (parse_res == OK) {
                if (detail != (step6_ack_detail_t *)0) {
                    detail->ack_seq = parsed.seq;
                    detail->ack_msg_id = parsed.msg_id;
                    detail->ack_total = parsed.total;
                    detail->ack_bitmap = (parsed.payload_len >= 1U) ? parsed.payload[0] : 0U;
                    detail->ack_rssi = (parsed.payload_len >= 2U) ? parsed.payload[1] : 0U;
                }

                if (parsed.type != PKT_TYPE_ACK) {
                    last_status = STEP6_ACK_BAD_TYPE;
                } else if (parsed.msg_id != exp_msg_id) {
                    last_status = STEP6_ACK_BAD_MSG_ID;
                } else if (parsed.total != exp_total) {
                    last_status = STEP6_ACK_BAD_TOTAL;
                } else if ((parsed.payload[0] & (1U << exp_idx)) == 0U) {
                    last_status = STEP6_ACK_FRAG_NOT_ACKED;
                } else {
                    if (detail != (step6_ack_detail_t *)0) {
                        detail->status = STEP6_ACK_OK;
                    }
                    return STEP6_ACK_OK;
                }
            } else if (parse_res == ERR_TAG) {
                last_status = STEP6_ACK_BAD_TAG;
            } else if (parse_res == ERR_REPLAY) {
                last_status = STEP6_ACK_REPLAY;
            } else if (parse_res == ERR_LEN) {
                last_status = STEP6_ACK_BAD_LEN;
            } else {
                last_status = STEP6_ACK_BAD_TYPE;
            }
        }
        __asm__ volatile ("nop");
    }

    if (detail != (step6_ack_detail_t *)0) {
        detail->status = last_status;
    }
    return last_status;
}

static void step6_log_ack_fail(uint8_t msg_id, const step6_ack_detail_t *detail) {
    char buf[80];
    uint32_t pos = 0;
    step6_append_str(buf, sizeof(buf), &pos, "ACK msg_id=");
    step6_append_u32(buf, sizeof(buf), &pos, (uint32_t)msg_id);
    step6_append_str(buf, sizeof(buf), &pos, " -> FAIL (");

    switch (detail->status) {
        case STEP6_ACK_TIMEOUT:
            step6_append_str(buf, sizeof(buf), &pos, "TIMEOUT");
            break;
        case STEP6_ACK_BAD_TAG:
            step6_append_str(buf, sizeof(buf), &pos, "BAD_TAG");
            break;
        case STEP6_ACK_REPLAY:
            step6_append_str(buf, sizeof(buf), &pos, "REPLAY");
            break;
        case STEP6_ACK_BAD_TYPE:
            step6_append_str(buf, sizeof(buf), &pos, "BAD_TYPE");
            break;
        case STEP6_ACK_BAD_MSG_ID:
            step6_append_str(buf, sizeof(buf), &pos, "WRONG_MSG_ID=");
            step6_append_u32(buf, sizeof(buf), &pos, (uint32_t)detail->ack_msg_id);
            break;
        case STEP6_ACK_BAD_TOTAL:
            step6_append_str(buf, sizeof(buf), &pos, "WRONG_TOTAL=");
            step6_append_u32(buf, sizeof(buf), &pos, (uint32_t)detail->ack_total);
            break;
        case STEP6_ACK_FRAG_NOT_ACKED:
            step6_append_str(buf, sizeof(buf), &pos, "BITMAP_BIT_CLEAR bm=");
            step6_append_hex8(buf, sizeof(buf), &pos, detail->ack_bitmap);
            break;
        case STEP6_ACK_BAD_LEN:
            step6_append_str(buf, sizeof(buf), &pos, "BAD_LEN");
            break;
        default:
            step6_append_str(buf, sizeof(buf), &pos, "ERROR");
            break;
    }
    step6_append_str(buf, sizeof(buf), &pos, ")");
    log_line(TAG_RX, buf);
}

static int step6_send_message(const char *text, uint8_t msg_id,
                              uint8_t *inout_seq,
                              fhss_replay_t *rx_replay,
                              step6_msg_report_t *report) {
    if (text == (const char *)0 || inout_seq == (uint8_t *)0 ||
        rx_replay == (fhss_replay_t *)0 || report == (step6_msg_report_t *)0) {
        return 0;
    }

    uint8_t msg_len = 0U;
    while (text[msg_len] != '\0') {
        msg_len++;
    }

    uint8_t total_frags = (uint8_t)((msg_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
    if (total_frags == 0U) {
        total_frags = 1U;
    }

    report->text = text;
    report->msg_id = msg_id;
    report->msg_len = msg_len;
    report->total_frags = total_frags;
    report->frags_tx_ok = 0U;
    report->frags_ack_valid = 0U;
    report->all_acked = 0U;

    for (uint8_t idx = 0U; idx < total_frags; idx++) {
        uint8_t offset = (uint8_t)(idx * MAX_PAYLOAD);
        uint8_t rem = (uint8_t)(msg_len - offset);
        uint8_t chunk_len = (rem > MAX_PAYLOAD) ? MAX_PAYLOAD : rem;
        uint8_t last = (idx == (total_frags - 1U)) ? 1U : 0U;
        uint8_t seq = *inout_seq;

        step6_frag_report_t *frag_rep = &report->frags[idx];
        frag_rep->idx = idx;
        frag_rep->seq = seq;
        frag_rep->last = last;
        frag_rep->chunk_len = chunk_len;
        frag_rep->tx_ok = 0U;
        frag_rep->ack_valid = 0U;

        uint8_t tx_frame[MAX_FRAME_LEN];
        uint8_t tx_len = 0U;
        const uint8_t *payload_ptr = (const uint8_t *)&text[offset];

        status_t build_st = fhss_build_data(seq, msg_id, total_frags, idx, last,
                                            0U, 0U, payload_ptr, chunk_len,
                                            tx_frame, &tx_len);
        if (build_st != OK) {
            log_line(TAG_ERR, "fhss_build_data failed");
            return 0;
        }

        /* Flush stale bytes before sending */
        air_mock_flush();

        int tx_res = air_tx(0U, tx_frame, tx_len);

        /* Format and log TX line */
        char tx_log[96];
        uint32_t t_pos = 0;
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, "DATA msg_id=");
        step6_append_u32(tx_log, sizeof(tx_log), &t_pos, (uint32_t)msg_id);
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, " frag=");
        step6_append_u32(tx_log, sizeof(tx_log), &t_pos, (uint32_t)idx);
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, "/");
        step6_append_u32(tx_log, sizeof(tx_log), &t_pos, (uint32_t)total_frags);
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, " seq=");
        step6_append_u32(tx_log, sizeof(tx_log), &t_pos, (uint32_t)seq);
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, " last=");
        step6_append_u32(tx_log, sizeof(tx_log), &t_pos, (uint32_t)last);
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, " len=");
        step6_append_u32(tx_log, sizeof(tx_log), &t_pos, (uint32_t)tx_len);
        step6_append_str(tx_log, sizeof(tx_log), &t_pos, (tx_res == (int)OK) ? " tx=OK" : " tx=FAIL");
        log_line(TAG_TX, tx_log);

        if (tx_res != (int)OK) {
            frag_rep->tx_ok = 0U;
            char err_buf[64];
            uint32_t e_pos = 0;
            step6_append_str(err_buf, sizeof(err_buf), &e_pos, "Msg ");
            step6_append_u32(err_buf, sizeof(err_buf), &e_pos, (uint32_t)msg_id);
            step6_append_str(err_buf, sizeof(err_buf), &e_pos, " Frag ");
            step6_append_u32(err_buf, sizeof(err_buf), &e_pos, (uint32_t)idx);
            step6_append_str(err_buf, sizeof(err_buf), &e_pos, " air_tx failed");
            log_line(TAG_TEST, err_buf);
            return 0; /* Abort sequential delivery */
        }

        frag_rep->tx_ok = 1U;
        report->frags_tx_ok++;
        (*inout_seq)++;

        /* Wait for standard FHSS ACK */
        step6_ack_status_t ack_res = step6_wait_ack(0U, msg_id, total_frags, idx,
                                                    STEP6_ACK_TIMEOUT_MS,
                                                    rx_replay,
                                                    &frag_rep->ack_detail);

        if (ack_res == STEP6_ACK_OK) {
            frag_rep->ack_valid = 1U;
            report->frags_ack_valid++;

            char rx_log[96];
            uint32_t r_pos = 0;
            step6_append_str(rx_log, sizeof(rx_log), &r_pos, "ACK msg_id=");
            step6_append_u32(rx_log, sizeof(rx_log), &r_pos, (uint32_t)msg_id);
            step6_append_str(rx_log, sizeof(rx_log), &r_pos, " b_seq=");
            step6_append_u32(rx_log, sizeof(rx_log), &r_pos, (uint32_t)frag_rep->ack_detail.ack_seq);
            step6_append_str(rx_log, sizeof(rx_log), &r_pos, " bitmap=");
            step6_append_hex8(rx_log, sizeof(rx_log), &r_pos, frag_rep->ack_detail.ack_bitmap);
            step6_append_str(rx_log, sizeof(rx_log), &r_pos, " rssi=");
            step6_append_u32(rx_log, sizeof(rx_log), &r_pos, (uint32_t)frag_rep->ack_detail.ack_rssi);
            step6_append_str(rx_log, sizeof(rx_log), &r_pos, " -> VALID");
            log_line(TAG_RX, rx_log);

            char pass_log[64];
            uint32_t p_pos = 0;
            step6_append_str(pass_log, sizeof(pass_log), &p_pos, "Msg ");
            step6_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)msg_id);
            step6_append_str(pass_log, sizeof(pass_log), &p_pos, " Frag ");
            step6_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)idx);
            step6_append_str(pass_log, sizeof(pass_log), &p_pos, "/");
            step6_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)total_frags);
            step6_append_str(pass_log, sizeof(pass_log), &p_pos, ": TX=OK ACK=VALID");
            log_line(TAG_TEST, pass_log);
        } else {
            frag_rep->ack_valid = 0U;
            step6_log_ack_fail(msg_id, &frag_rep->ack_detail);

            char fail_log[64];
            uint32_t f_pos = 0;
            step6_append_str(fail_log, sizeof(fail_log), &f_pos, "Msg ");
            step6_append_u32(fail_log, sizeof(fail_log), &f_pos, (uint32_t)msg_id);
            step6_append_str(fail_log, sizeof(fail_log), &f_pos, " Frag ");
            step6_append_u32(fail_log, sizeof(fail_log), &f_pos, (uint32_t)idx);
            step6_append_str(fail_log, sizeof(fail_log), &f_pos, "/");
            step6_append_u32(fail_log, sizeof(fail_log), &f_pos, (uint32_t)total_frags);
            step6_append_str(fail_log, sizeof(fail_log), &f_pos, ": TX=OK ACK=FAIL");
            log_line(TAG_TEST, fail_log);

            /* Hard rule: do not send later fragment if earlier fragment was not acknowledged */
            return 0;
        }

        delay_ms(25U);
    }

    if (report->frags_ack_valid == total_frags) {
        report->all_acked = 1U;
        return 1;
    }
    return 0;
}

static void step6_data_mock_run(void) {
    log_line(TAG_BOOT, "node=A test=STAGE6_DATA_MOCK link=USART2 mock_rf=simulated");
    log_line(TAG_BOOT, "RF bypassed: SX1278 uninitialized; testing DATA messaging over USART2 (one-shot)");

    /* Test-only startup delay to allow Node B to complete initialization */
    delay_ms(STEP6_STARTUP_DELAY_MS);
    air_mock_flush();
    air_mock_clear_diag();

    if (fhss_verify_vectors() == OK) {
        log_line(TAG_TEST, "vectors PASS");
    } else {
        log_line(TAG_TEST, "vectors FAIL");
        while (1) { __asm__ volatile ("nop"); }
    }

    log_line(TAG_TEST, "=== Stage 6 DATA Mock Messaging Test (One-Shot) Start ===");

    uint8_t s_tx_seq = 0U;
    fhss_replay_t s_rx_replay = {0, 0};

    step6_msg_report_t rep1;
    step6_msg_report_t rep2;

    /* -----------------------------------------------------------------
     * Message 1: Short, single-fragment ASCII message (12 bytes)
     * ----------------------------------------------------------------- */
    static const char s_msg1[] = "HELLO_NODE_B";
    log_line(TAG_TEST, "Msg 1 (single-frag): len=12 msg_id=1 total=1 \"HELLO_NODE_B\"");

    int m1_ok = step6_send_message(s_msg1, 1U, &s_tx_seq, &s_rx_replay, &rep1);

    log_line(TAG_TEST, "Msg 1 Peer Reconstruction Confirmation: NOT_SUPPORTED (protocol lacks peer completion confirmation)");
    log_line(TAG_TEST, "Msg 1 Payload Match Verification: UNVERIFIABLE_OVER_AIR (no reverse data path)");
    if (m1_ok) {
        log_line(TAG_TEST, "Msg 1 Result: PASS (all fragments ACKed)");
    } else {
        log_line(TAG_TEST, "Msg 1 Result: FAIL");
    }

    delay_ms(100U);

    /* -----------------------------------------------------------------
     * Message 2: Multi-fragment ASCII message (34 bytes, 3 fragments)
     * ----------------------------------------------------------------- */
    static const char s_msg2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
    log_line(TAG_TEST, "Msg 2 (multi-frag): len=34 msg_id=2 total=3 \"FHSS_DATA_TRANSPORT_TEST_PACKET_#2\"");

    int m2_ok = step6_send_message(s_msg2, 2U, &s_tx_seq, &s_rx_replay, &rep2);

    log_line(TAG_TEST, "Msg 2 Peer Reconstruction Confirmation: NOT_SUPPORTED (protocol lacks peer completion confirmation)");
    log_line(TAG_TEST, "Msg 2 Payload Match Verification: UNVERIFIABLE_OVER_AIR (no reverse data path)");
    if (m2_ok) {
        log_line(TAG_TEST, "Msg 2 Result: PASS (all 3 fragments ACKed)");
    } else {
        log_line(TAG_TEST, "Msg 2 Result: FAIL");
    }

    /* -----------------------------------------------------------------
     * Final Summary Reporting
     * ----------------------------------------------------------------- */
    log_line(TAG_TEST, "=== Stage 6 DATA Mock Messaging Summary ===");

    /* Summary for Message 1 */
    char s1_buf[96];
    uint32_t sp = 0;
    step6_append_str(s1_buf, sizeof(s1_buf), &sp, "Message 1 (single-frag, len=");
    step6_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.msg_len);
    step6_append_str(s1_buf, sizeof(s1_buf), &sp, ", msg_id=1): TX=");
    step6_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.frags_tx_ok);
    step6_append_str(s1_buf, sizeof(s1_buf), &sp, "/");
    step6_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.total_frags);
    step6_append_str(s1_buf, sizeof(s1_buf), &sp, " ACK=");
    step6_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.frags_ack_valid);
    step6_append_str(s1_buf, sizeof(s1_buf), &sp, "/");
    step6_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.total_frags);
    log_line(TAG_TEST, s1_buf);
    log_line(TAG_TEST, "  Peer reconstruction confirmed: NO (protocol limitation: no peer completion confirmation)");
    log_line(TAG_TEST, "  Peer payload match verified: NO (protocol limitation: no reverse payload verification)");

    /* Summary for Message 2 */
    char s2_buf[96];
    sp = 0;
    step6_append_str(s2_buf, sizeof(s2_buf), &sp, "Message 2 (multi-frag, len=");
    step6_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.msg_len);
    step6_append_str(s2_buf, sizeof(s2_buf), &sp, ", msg_id=2): TX=");
    step6_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.frags_tx_ok);
    step6_append_str(s2_buf, sizeof(s2_buf), &sp, "/");
    step6_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.total_frags);
    step6_append_str(s2_buf, sizeof(s2_buf), &sp, " ACK=");
    step6_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.frags_ack_valid);
    step6_append_str(s2_buf, sizeof(s2_buf), &sp, "/");
    step6_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.total_frags);
    log_line(TAG_TEST, s2_buf);
    log_line(TAG_TEST, "  Peer reconstruction confirmed: NO (protocol limitation: no peer completion confirmation)");
    log_line(TAG_TEST, "  Peer payload match verified: NO (protocol limitation: no reverse payload verification)");

    if (m1_ok && m2_ok) {
        log_line(TAG_TEST, "Stage 6 DATA Messaging Test Complete: ALL FRAGMENTS TRANSMITTED AND ACKED (one-shot)");
    } else {
        log_line(TAG_TEST, "Stage 6 DATA Messaging Test Complete: FAIL (one-shot)");
    }

    /* Idle forever; do not restart sequence numbers */
    log_line(TAG_BOOT, "Stage 6 idle; reset board to rerun");
    uint32_t idle_sec = 0U;
    while (1) {
        delay_ms(5000U);
        idle_sec += 5U;
        char idle_buf[48];
        char num_idle[12];
        char *ip = idle_buf;
        const char *m = "idle ("; while (*m != '\0') *ip++ = *m++;
        uint32_to_ascii(idle_sec, num_idle); const char *sn = num_idle; while (*sn != '\0') *ip++ = *sn++;
        m = "s); press RESET to rerun"; while (*m != '\0') *ip++ = *m++;
        *ip = '\0';
        log_line(TAG_BOOT, idle_buf);
    }
}
#endif

#if defined(STEP7_ARQ_MOCK) || defined(INTERACTIVE_MODE)
/*
 * Stage 7: Node A DATA Retransmission and ACK-Loss Recovery over USART2 Mock Transport
 *
 * Exercises ARQ retransmission recovery using the frozen FHSS protocol over the wired
 * USART2 mock transport (SX1278 radios bypassed).
 * Schedule:
 *   Step 1: Msg 1, ID 1, frag 0/1, seq 0, attempt 1 -> ACK bitmap 0x01
 *   Step 2: Msg 2, ID 2, frag 0/3, seq 1, attempt 1 -> ACK bitmap 0x01
 *   Step 3: Msg 2, ID 2, frag 1/3, seq 2, attempt 1 -> ACK suppressed by Node B (timeout)
 *   Step 4: Msg 2, ID 2, frag 1/3, seq 3, attempt 2 (RETRY) -> fresh seq 3, recomputed tag -> ACK bitmap 0x03
 *   Step 5: Msg 2, ID 2, frag 2/3, seq 4, attempt 1 -> ACK bitmap 0x07 (message complete)
 *
 * Rules:
 *   - Sequence increments on every transmission attempt (including retries).
 *   - Packet rebuilt with fresh sequence number, masked hop, and recomputed tag.
 *   - Never resend stale frame bytes.
 *   - Do not flush queued ACKs before retries.
 *   - Maximum 3 attempts per fragment.
 *   - Abort remaining fragments if retry limit exceeded.
 */

#define STEP7_ACK_TIMEOUT_MS   150U
#define STEP7_STARTUP_DELAY_MS 500U
#define STEP7_MAX_ATTEMPTS     3U

typedef enum {
    STEP7_ACK_OK = 0,
    STEP7_ACK_TIMEOUT,
    STEP7_ACK_BAD_TAG,
    STEP7_ACK_REPLAY,
    STEP7_ACK_BAD_TYPE,
    STEP7_ACK_BAD_MSG_ID,
    STEP7_ACK_BAD_TOTAL,
    STEP7_ACK_FRAG_NOT_ACKED,
    STEP7_ACK_BAD_LEN
} step7_ack_status_t;

typedef struct {
    step7_ack_status_t status;
    uint8_t ack_seq;        /* Node B's independent ACK sequence number */
    uint8_t ack_msg_id;     /* Echoed msg_id */
    uint8_t ack_total;      /* Total fragments indicated in ACK */
    uint8_t ack_bitmap;     /* Fragment received bitmap */
    uint8_t ack_rssi;       /* Raw RSSI */
} step7_ack_detail_t;

typedef struct {
    uint8_t idx;
    uint8_t last_seq;       /* Sequence number used in final attempt */
    uint8_t last;
    uint8_t chunk_len;
    uint8_t attempts;       /* Number of attempts made */
    uint8_t ack_valid;
    step7_ack_detail_t ack_detail;
} step7_frag_report_t;

typedef struct {
    const char *text;
    uint8_t msg_id;
    uint8_t msg_len;
    uint8_t total_frags;
    uint8_t frags_tx_ok;
    uint8_t frags_ack_valid;
    uint8_t total_retries;
    uint8_t all_acked;
    step7_frag_report_t frags[MAX_FRAGMENTS];
} step7_msg_report_t;

static void step7_uint8_to_hex(uint8_t val, char *buf) {
    static const char hex_digits[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    buf[2] = hex_digits[(val >> 4) & 0x0FU];
    buf[3] = hex_digits[val & 0x0FU];
    buf[4] = '\0';
}

static void step7_append_str(char *buf, uint32_t max_len, uint32_t *pos, const char *s) {
    if (s == (const char *)0) return;
    while (*s != '\0' && (*pos + 1U) < max_len) {
        buf[*pos] = *s++;
        (*pos)++;
    }
    buf[*pos] = '\0';
}

static void step7_append_u32(char *buf, uint32_t max_len, uint32_t *pos, uint32_t val) {
    char num[12];
    uint32_to_ascii(val, num);
    step7_append_str(buf, max_len, pos, num);
}

static void step7_append_hex8(char *buf, uint32_t max_len, uint32_t *pos, uint8_t val) {
    char hex[5];
    step7_uint8_to_hex(val, hex);
    step7_append_str(buf, max_len, pos, hex);
}

static void step7_log_arq_tx(uint8_t msg_id, uint8_t frag_idx, uint8_t seq, uint8_t attempt) {
    char buf[64];
    uint32_t pos = 0;
    step7_append_str(buf, sizeof(buf), &pos, "TX msg=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)msg_id);
    step7_append_str(buf, sizeof(buf), &pos, " frag=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)frag_idx);
    step7_append_str(buf, sizeof(buf), &pos, " seq=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)seq);
    step7_append_str(buf, sizeof(buf), &pos, " attempt=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)attempt);
    log_line(TAG_ARQ, buf);
}

static void step7_log_ack_timeout(uint8_t msg_id, uint8_t frag_idx, uint8_t seq) {
    char buf[64];
    uint32_t pos = 0;
    step7_append_str(buf, sizeof(buf), &pos, "ACK_TIMEOUT msg=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)msg_id);
    step7_append_str(buf, sizeof(buf), &pos, " frag=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)frag_idx);
    step7_append_str(buf, sizeof(buf), &pos, " seq=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)seq);
    log_line(TAG_ARQ, buf);
}

static void step7_log_arq_retry(uint8_t msg_id, uint8_t frag_idx, uint8_t seq, uint8_t attempt) {
    char buf[64];
    uint32_t pos = 0;
    step7_append_str(buf, sizeof(buf), &pos, "RETRY msg=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)msg_id);
    step7_append_str(buf, sizeof(buf), &pos, " frag=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)frag_idx);
    step7_append_str(buf, sizeof(buf), &pos, " seq=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)seq);
    step7_append_str(buf, sizeof(buf), &pos, " attempt=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)attempt);
    log_line(TAG_ARQ, buf);
}

static void step7_log_ack_valid(uint8_t msg_id, uint8_t bitmap) {
    char buf[64];
    uint32_t pos = 0;
    step7_append_str(buf, sizeof(buf), &pos, "ACK msg=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)msg_id);
    step7_append_str(buf, sizeof(buf), &pos, " bitmap=");
    step7_append_hex8(buf, sizeof(buf), &pos, bitmap);
    step7_append_str(buf, sizeof(buf), &pos, " -> VALID");
    log_line(TAG_RX, buf);
}

static void step7_log_ack_fail(uint8_t msg_id, const step7_ack_detail_t *detail) {
    char buf[80];
    uint32_t pos = 0;
    step7_append_str(buf, sizeof(buf), &pos, "ACK msg_id=");
    step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)msg_id);
    step7_append_str(buf, sizeof(buf), &pos, " -> FAIL (");

    switch (detail->status) {
        case STEP7_ACK_TIMEOUT:
            step7_append_str(buf, sizeof(buf), &pos, "TIMEOUT");
            break;
        case STEP7_ACK_BAD_TAG:
            step7_append_str(buf, sizeof(buf), &pos, "BAD_TAG");
            break;
        case STEP7_ACK_REPLAY:
            step7_append_str(buf, sizeof(buf), &pos, "REPLAY");
            break;
        case STEP7_ACK_BAD_TYPE:
            step7_append_str(buf, sizeof(buf), &pos, "BAD_TYPE");
            break;
        case STEP7_ACK_BAD_MSG_ID:
            step7_append_str(buf, sizeof(buf), &pos, "WRONG_MSG_ID=");
            step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)detail->ack_msg_id);
            break;
        case STEP7_ACK_BAD_TOTAL:
            step7_append_str(buf, sizeof(buf), &pos, "WRONG_TOTAL=");
            step7_append_u32(buf, sizeof(buf), &pos, (uint32_t)detail->ack_total);
            break;
        case STEP7_ACK_FRAG_NOT_ACKED:
            step7_append_str(buf, sizeof(buf), &pos, "BITMAP_BIT_CLEAR bm=");
            step7_append_hex8(buf, sizeof(buf), &pos, detail->ack_bitmap);
            break;
        case STEP7_ACK_BAD_LEN:
            step7_append_str(buf, sizeof(buf), &pos, "BAD_LEN");
            break;
        default:
            step7_append_str(buf, sizeof(buf), &pos, "ERROR");
            break;
    }
    step7_append_str(buf, sizeof(buf), &pos, ")");
    log_line(TAG_RX, buf);
}

static step7_ack_status_t step7_wait_ack(uint8_t tuned_ch, uint8_t exp_msg_id,
                                         uint8_t exp_total, uint8_t exp_idx,
                                         uint32_t timeout_ms,
                                         fhss_replay_t *replay,
                                         step7_ack_detail_t *detail) {
    uint32_t t_start = millis();
    step7_ack_status_t last_status = STEP7_ACK_TIMEOUT;

    if (detail != (step7_ack_detail_t *)0) {
        detail->status = STEP7_ACK_TIMEOUT;
        detail->ack_seq = 0U;
        detail->ack_msg_id = 0U;
        detail->ack_total = 0U;
        detail->ack_bitmap = 0U;
        detail->ack_rssi = 0U;
    }

    while ((millis() - t_start) < timeout_ms) {
        uint8_t rx_buf[MAX_FRAME_LEN];
        uint8_t rx_len = 0U;
        int poll_res = air_rx_poll(tuned_ch, rx_buf, &rx_len);

        if (poll_res == (int)OK) {
            fhss_frame_t parsed;
            status_t parse_res = fhss_parse_frame(rx_buf, rx_len, DIR_B_TO_A, replay, &parsed);

            if (parse_res == OK) {
                if (detail != (step7_ack_detail_t *)0) {
                    detail->ack_seq = parsed.seq;
                    detail->ack_msg_id = parsed.msg_id;
                    detail->ack_total = parsed.total;
                    detail->ack_bitmap = (parsed.payload_len >= 1U) ? parsed.payload[0] : 0U;
                    detail->ack_rssi = (parsed.payload_len >= 2U) ? parsed.payload[1] : 0U;
                }

                if (parsed.type != PKT_TYPE_ACK) {
                    last_status = STEP7_ACK_BAD_TYPE;
                } else if (parsed.msg_id != exp_msg_id) {
                    last_status = STEP7_ACK_BAD_MSG_ID;
                } else if (parsed.total != exp_total) {
                    last_status = STEP7_ACK_BAD_TOTAL;
                } else if ((parsed.payload[0] & (1U << exp_idx)) == 0U) {
                    last_status = STEP7_ACK_FRAG_NOT_ACKED;
                } else {
                    if (detail != (step7_ack_detail_t *)0) {
                        detail->status = STEP7_ACK_OK;
                    }
                    return STEP7_ACK_OK;
                }
            } else if (parse_res == ERR_TAG) {
                last_status = STEP7_ACK_BAD_TAG;
            } else if (parse_res == ERR_REPLAY) {
                last_status = STEP7_ACK_REPLAY;
            } else if (parse_res == ERR_LEN) {
                last_status = STEP7_ACK_BAD_LEN;
            } else {
                last_status = STEP7_ACK_BAD_TYPE;
            }
        }
        __asm__ volatile ("nop");
    }

    if (detail != (step7_ack_detail_t *)0) {
        detail->status = last_status;
    }
    return last_status;
}

static int step7_send_blob_arq(const uint8_t *blob, uint8_t blob_len, uint8_t msg_id,
                               uint8_t *inout_seq,
                               fhss_replay_t *rx_replay,
                               step7_msg_report_t *report) {
    if (blob == (const uint8_t *)0 || inout_seq == (uint8_t *)0 ||
        rx_replay == (fhss_replay_t *)0 || report == (step7_msg_report_t *)0) {
        return 0;
    }

    uint8_t total_frags = (uint8_t)((blob_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
    if (total_frags == 0U) {
        total_frags = 1U;
    }

    report->text = (const char *)blob;
    report->msg_id = msg_id;
    report->msg_len = blob_len;
    report->total_frags = total_frags;
    report->frags_tx_ok = 0U;
    report->frags_ack_valid = 0U;
    report->total_retries = 0U;
    report->all_acked = 0U;

    for (uint8_t idx = 0U; idx < total_frags; idx++) {
        uint8_t offset = (uint8_t)(idx * MAX_PAYLOAD);
        uint8_t rem = (uint8_t)(blob_len - offset);
        uint8_t chunk_len = (rem > MAX_PAYLOAD) ? MAX_PAYLOAD : rem;
        uint8_t last = (idx == (total_frags - 1U)) ? 1U : 0U;

        step7_frag_report_t *frag_rep = &report->frags[idx];
        frag_rep->idx = idx;
        frag_rep->last = last;
        frag_rep->chunk_len = chunk_len;
        frag_rep->attempts = 0U;
        frag_rep->ack_valid = 0U;

        int frag_success = 0;

        for (uint8_t attempt = 1U; attempt <= STEP7_MAX_ATTEMPTS; attempt++) {
            uint8_t seq = *inout_seq;
            frag_rep->last_seq = seq;
            frag_rep->attempts = attempt;

            uint8_t tx_frame[MAX_FRAME_LEN];
            uint8_t tx_len = 0U;
            const uint8_t *payload_ptr = &blob[offset];

            status_t build_st = fhss_build_data(seq, msg_id, total_frags, idx, last,
                                                0U, 0U, payload_ptr, chunk_len,
                                                tx_frame, &tx_len);
            if (build_st != OK) {
                log_line(TAG_ERR, "fhss_build_data failed");
                return 0;
            }

            /* Log attempt per requirement:
             * attempt 1: [ARQ] TX msg=X frag=Y seq=Z attempt=1
             * attempt > 1: [ARQ] RETRY msg=X frag=Y seq=Z attempt=N
             */
            if (attempt == 1U) {
                step7_log_arq_tx(msg_id, idx, seq, attempt);
            } else {
                step7_log_arq_retry(msg_id, idx, seq, attempt);
                report->total_retries++;
            }

            /* Rule 8: Do NOT flush potentially valid queued ACKs indiscriminately before retries */
            int tx_res = air_tx(0U, tx_frame, tx_len);
            if (tx_res != (int)OK) {
                log_line(TAG_ERR, "air_tx failed");
                return 0;
            }

            /* Sequence number increments for EVERY transmitted DATA packet, including retransmission */
            (*inout_seq)++;

            /* Wait for standard FHSS ACK */
            step7_ack_status_t ack_res = step7_wait_ack(0U, msg_id, total_frags, idx,
                                                        STEP7_ACK_TIMEOUT_MS,
                                                        rx_replay,
                                                        &frag_rep->ack_detail);

            if (ack_res == STEP7_ACK_OK) {
                frag_rep->ack_valid = 1U;
                step7_log_ack_valid(msg_id, frag_rep->ack_detail.ack_bitmap);

                char pass_log[64];
                uint32_t p_pos = 0;
                step7_append_str(pass_log, sizeof(pass_log), &p_pos, "Msg ");
                step7_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)msg_id);
                step7_append_str(pass_log, sizeof(pass_log), &p_pos, " Frag ");
                step7_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)idx);
                step7_append_str(pass_log, sizeof(pass_log), &p_pos, "/");
                step7_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)total_frags);
                step7_append_str(pass_log, sizeof(pass_log), &p_pos, ": TX=OK ACK=VALID (attempts=");
                step7_append_u32(pass_log, sizeof(pass_log), &p_pos, (uint32_t)attempt);
                step7_append_str(pass_log, sizeof(pass_log), &p_pos, ")");
                log_line(TAG_TEST, pass_log);

                frag_success = 1;
                break;
            } else if (ack_res == STEP7_ACK_TIMEOUT) {
                step7_log_ack_timeout(msg_id, idx, seq);
                /* If attempts remain, loop continues to retry with fresh seq */
            } else {
                /* Invalid or corrupted ACK received */
                step7_log_ack_fail(msg_id, &frag_rep->ack_detail);
            }

            delay_ms(25U);
        }

        if (!frag_success) {
            /* Abort remaining fragments if required ACK cannot be obtained after retry limit */
            char fail_log[80];
            uint32_t f_pos = 0;
            step7_append_str(fail_log, sizeof(fail_log), &f_pos, "Msg ");
            step7_append_u32(fail_log, sizeof(fail_log), &f_pos, (uint32_t)msg_id);
            step7_append_str(fail_log, sizeof(fail_log), &f_pos, " Frag ");
            step7_append_u32(fail_log, sizeof(fail_log), &f_pos, (uint32_t)idx);
            step7_append_str(fail_log, sizeof(fail_log), &f_pos, " failed after max retries; aborting message");
            log_line(TAG_ARQ, fail_log);
            return 0;
        }

        report->frags_tx_ok++;
        report->frags_ack_valid++;
        delay_ms(25U);
    }

    if (report->frags_ack_valid == total_frags) {
        report->all_acked = 1U;
        return 1;
    }
    return 0;
}

#if defined(STEP7_ARQ_MOCK)
static int step7_send_message_arq(const char *text, uint8_t msg_id,
                                  uint8_t *inout_seq,
                                  fhss_replay_t *rx_replay,
                                  step7_msg_report_t *report) {
    if (text == (const char *)0) {
        return 0;
    }
    uint8_t len = 0U;
    while (text[len] != '\0') {
        len++;
    }
    return step7_send_blob_arq((const uint8_t *)text, len, msg_id, inout_seq, rx_replay, report);
}

static void step7_arq_mock_run(void) {
    log_line(TAG_BOOT, "node=A test=STAGE7_ARQ_MOCK link=USART2 mock_rf=simulated");
    log_line(TAG_BOOT, "RF bypassed: SX1278 uninitialized; testing ARQ retransmission over USART2 (one-shot)");

    /* Test-only startup delay to allow Node B to complete initialization */
    delay_ms(STEP7_STARTUP_DELAY_MS);
    air_mock_flush();
    air_mock_clear_diag();

    if (fhss_verify_vectors() == OK) {
        log_line(TAG_TEST, "vectors PASS");
    } else {
        log_line(TAG_TEST, "vectors FAIL");
        while (1) { __asm__ volatile ("nop"); }
    }

    log_line(TAG_TEST, "=== Stage 7 ARQ Mock Messaging Test (One-Shot) Start ===");

    uint8_t s_tx_seq = 0U;
    fhss_replay_t s_rx_replay = {0, 0};

    step7_msg_report_t rep1;
    step7_msg_report_t rep2;

    /* -----------------------------------------------------------------
     * Message 1: Short, single-fragment ASCII message (12 bytes)
     * ----------------------------------------------------------------- */
    static const char s_msg1[] = "HELLO_NODE_B";
    log_line(TAG_TEST, "Msg 1 (single-frag): len=12 msg_id=1 total=1 \"HELLO_NODE_B\"");

    int m1_ok = step7_send_message_arq(s_msg1, 1U, &s_tx_seq, &s_rx_replay, &rep1);

    log_line(TAG_TEST, "Msg 1 Peer Reconstruction Confirmation: NOT_SUPPORTED (protocol lacks peer completion confirmation)");
    log_line(TAG_TEST, "Msg 1 Payload Match Verification: UNVERIFIABLE_OVER_AIR (no reverse data path)");
    if (m1_ok) {
        log_line(TAG_TEST, "Msg 1 Result: PASS (all fragments ACKed)");
    } else {
        log_line(TAG_TEST, "Msg 1 Result: FAIL");
    }

    delay_ms(100U);

    /* -----------------------------------------------------------------
     * Message 2: Multi-fragment ASCII message (34 bytes, 3 fragments)
     * ----------------------------------------------------------------- */
    static const char s_msg2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
    log_line(TAG_TEST, "Msg 2 (multi-frag): len=34 msg_id=2 total=3 \"FHSS_DATA_TRANSPORT_TEST_PACKET_#2\"");

    int m2_ok = step7_send_message_arq(s_msg2, 2U, &s_tx_seq, &s_rx_replay, &rep2);

    log_line(TAG_TEST, "Msg 2 Peer Reconstruction Confirmation: NOT_SUPPORTED (protocol lacks peer completion confirmation)");
    log_line(TAG_TEST, "Msg 2 Payload Match Verification: UNVERIFIABLE_OVER_AIR (no reverse data path)");
    if (m2_ok) {
        log_line(TAG_TEST, "Msg 2 Result: PASS (all 3 fragments ACKed, 1 retry recovered)");
    } else {
        log_line(TAG_TEST, "Msg 2 Result: FAIL");
    }

    /* -----------------------------------------------------------------
     * Final Summary Reporting
     * ----------------------------------------------------------------- */
    log_line(TAG_TEST, "=== Stage 7 ARQ Mock Test Summary ===");

    /* Summary for Message 1 */
    char s1_buf[96];
    uint32_t sp = 0;
    step7_append_str(s1_buf, sizeof(s1_buf), &sp, "Message 1 (single-frag, len=");
    step7_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.msg_len);
    step7_append_str(s1_buf, sizeof(s1_buf), &sp, ", msg_id=1): TX=");
    step7_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.frags_tx_ok);
    step7_append_str(s1_buf, sizeof(s1_buf), &sp, "/");
    step7_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.total_frags);
    step7_append_str(s1_buf, sizeof(s1_buf), &sp, " ACK=");
    step7_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.frags_ack_valid);
    step7_append_str(s1_buf, sizeof(s1_buf), &sp, "/");
    step7_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.total_frags);
    step7_append_str(s1_buf, sizeof(s1_buf), &sp, " retries=");
    step7_append_u32(s1_buf, sizeof(s1_buf), &sp, (uint32_t)rep1.total_retries);
    log_line(TAG_TEST, s1_buf);
    log_line(TAG_TEST, "  Peer reconstruction confirmed: NO (protocol limitation: no peer completion confirmation)");
    log_line(TAG_TEST, "  Peer payload match verified: NO (protocol limitation: no reverse payload verification)");

    /* Summary for Message 2 */
    char s2_buf[96];
    sp = 0;
    step7_append_str(s2_buf, sizeof(s2_buf), &sp, "Message 2 (multi-frag, len=");
    step7_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.msg_len);
    step7_append_str(s2_buf, sizeof(s2_buf), &sp, ", msg_id=2): TX=");
    step7_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.frags_tx_ok);
    step7_append_str(s2_buf, sizeof(s2_buf), &sp, "/");
    step7_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.total_frags);
    step7_append_str(s2_buf, sizeof(s2_buf), &sp, " ACK=");
    step7_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.frags_ack_valid);
    step7_append_str(s2_buf, sizeof(s2_buf), &sp, "/");
    step7_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.total_frags);
    step7_append_str(s2_buf, sizeof(s2_buf), &sp, " retries=");
    step7_append_u32(s2_buf, sizeof(s2_buf), &sp, (uint32_t)rep2.total_retries);
    log_line(TAG_TEST, s2_buf);
    log_line(TAG_TEST, "  Peer reconstruction confirmed: NO (protocol limitation: no peer completion confirmation)");
    log_line(TAG_TEST, "  Peer payload match verified: NO (protocol limitation: no reverse payload verification)");

    if (m1_ok && m2_ok && rep2.total_retries == 1U) {
        log_line(TAG_TEST, "Stage 7 ARQ Test Complete: ALL FRAGMENTS ACKED WITH 1 RETRY RECOVERY (one-shot)");
    } else if (m1_ok && m2_ok) {
        log_line(TAG_TEST, "Stage 7 ARQ Test Complete: ALL FRAGMENTS TRANSMITTED AND ACKED (one-shot)");
    } else {
        log_line(TAG_TEST, "Stage 7 ARQ Test Complete: FAIL (one-shot)");
    }

    /* Idle forever; do not restart sequence numbers */
    log_line(TAG_BOOT, "Stage 7 idle; reset board to rerun");
    uint32_t idle_sec = 0U;
    while (1) {
        delay_ms(5000U);
        idle_sec += 5U;
        char idle_buf[48];
        char num_idle[12];
        char *ip = idle_buf;
        const char *m = "idle ("; while (*m != '\0') *ip++ = *m++;
        uint32_to_ascii(idle_sec, num_idle); const char *sn = num_idle; while (*sn != '\0') *ip++ = *sn++;
        m = "s); press RESET to rerun"; while (*m != '\0') *ip++ = *m++;
        *ip = '\0';
        log_line(TAG_BOOT, idle_buf);
    }
}
#endif /* STEP7_ARQ_MOCK */

#if defined(INTERACTIVE_MODE)
#define MAX_ENC_PLAINTEXT_LEN 99U
#define ENC_OVERHEAD_LEN     (1U + CHACHA20_NONCE_LEN + POLY1305_TAG_LEN) /* 29 bytes */
#define MAX_ENC_PACKET_LEN   (MAX_ENC_PLAINTEXT_LEN + ENC_OVERHEAD_LEN)   /* 128 bytes */

static const uint8_t s_demo_key[CHACHA20_KEY_LEN] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};

static inline void UART1_SendChar(char c) {
    char s[2];
    s[0] = c;
    s[1] = '\0';
    UART1_SendString(s);
}

static void UART1_SendHexBytes(const uint8_t *data, size_t len) {
    static const char hex_chars[] = "0123456789abcdef";
    for (size_t i = 0; i < len; i++) {
        char h[3];
        h[0] = hex_chars[(data[i] >> 4) & 0x0FU];
        h[1] = hex_chars[data[i] & 0x0FU];
        h[2] = '\0';
        UART1_SendString(h);
        if (i + 1 < len) {
            UART1_SendChar(' ');
        }
    }
}

static int UART1_GetChar(uint8_t *ch) {
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t dr = (uint8_t)(USART1->DR & 0xFFU);
        if (sr & USART_SR_RXNE) {
            if (ch != (uint8_t *)0) {
                *ch = dr;
            }
            return 1;
        }
    }
    return 0;
}

static void interactive_mode_run(void) {
    UART1_SendString("\r\n====================================\r\n");
    UART1_SendString(" FHSS COMMUNICATION SYSTEM\r\n");
    UART1_SendString(" Node A — Transmitter (AEAD Mode)\r\n");
    UART1_SendString("====================================\r\n\r\n");

    /* Initialize persistent monotonic nonce manager */
    nonce_mgr_init();

    /* Startup delay to allow Node B to complete initialization */
    delay_ms(STEP7_STARTUP_DELAY_MS);
    air_mock_flush();
    air_mock_clear_diag();

    if (fhss_verify_vectors() == OK) {
        log_line(TAG_TEST, "vectors PASS");
    } else {
        log_line(TAG_TEST, "vectors FAIL");
        while (1) { __asm__ volatile ("nop"); }
    }

    uint8_t s_tx_seq = 0U;
    fhss_replay_t s_rx_replay = {0, 0};
    uint8_t s_msg_id = 1U;

    char input_buf[MAX_ENC_PLAINTEXT_LEN + 4U];
    uint32_t input_len = 0U;
    uint8_t oversized = 0U;
    uint8_t last_was_cr = 0U;

    UART1_SendString("Enter message: ");

    while (1) {
        uint8_t c = 0U;
        if (UART1_GetChar(&c)) {
            /* Handle CRLF: If previous was CR and current is LF, ignore this LF */
            if (last_was_cr != 0U && c == '\n') {
                last_was_cr = 0U;
                continue;
            }
            last_was_cr = (c == '\r') ? 1U : 0U;

            if (c == '\r' || c == '\n') {
                UART1_SendString("\r\n");

                if (oversized != 0U || input_len > MAX_ENC_PLAINTEXT_LEN) {
                    log_line("APP", "Message rejected: length exceeds encrypted maximum (99 bytes)");
                    oversized = 0U;
                    input_len = 0U;
                    UART1_SendString("Enter message: ");
                    continue;
                }

                if (input_len == 0U) {
                    log_line("APP", "Empty message rejected");
                    UART1_SendString("Enter message: ");
                    continue;
                }

                input_buf[input_len] = '\0';
                log_line("APP", "Input accepted");

                /* Print original plaintext per specification */
                UART1_SendString("[ENC] Original plaintext: ");
                UART1_SendString(input_buf);
                UART1_SendString("\r\n");

                /* Obtain next strictly unique 12-byte nonce */
                uint8_t nonce[CHACHA20_NONCE_LEN];
                nonce_mgr_get_next(nonce);

                UART1_SendString("[ENC] Nonce (12 bytes): ");
                UART1_SendHexBytes(nonce, CHACHA20_NONCE_LEN);
                UART1_SendString("\r\n");

                uint8_t cur_msg_id = s_msg_id++;
                if (s_msg_id == 0U) {
                    s_msg_id = 1U;
                }

                /* Construct Associated Data: exact string "FHSS-APP-ENC-v1" + 1-byte DATA msg_id */
                uint8_t aad[16];
                memcpy(aad, "FHSS-APP-ENC-v1", 15);
                aad[15] = cur_msg_id;

                /* Encrypt plaintext using standard ChaCha20-Poly1305 */
                uint8_t ciphertext[MAX_ENC_PLAINTEXT_LEN];
                uint8_t tag[POLY1305_TAG_LEN];
                aead_chacha20_poly1305_encrypt(
                    s_demo_key,
                    nonce,
                    aad, sizeof(aad),
                    (const uint8_t *)input_buf, input_len,
                    ciphertext,
                    tag
                );

                /* Print actual ciphertext bytes in hexadecimal */
                UART1_SendString("[ENC] Ciphertext: ");
                UART1_SendHexBytes(ciphertext, input_len);
                UART1_SendString("\r\n");

                /* Print 16-byte Poly1305 authentication tag in hexadecimal */
                UART1_SendString("[ENC] Poly1305 Tag: ");
                UART1_SendHexBytes(tag, POLY1305_TAG_LEN);
                UART1_SendString("\r\n");

                /* Assemble encrypted application message format:
                 * - byte 0: version 0xE1
                 * - bytes 1..12: nonce
                 * - following bytes: ciphertext
                 * - final 16 bytes: Poly1305 authentication tag
                 */
                uint8_t enc_packet[MAX_ENC_PACKET_LEN];
                enc_packet[0] = ENC_VERSION_BYTE;
                memcpy(&enc_packet[1], nonce, CHACHA20_NONCE_LEN);
                memcpy(&enc_packet[1 + CHACHA20_NONCE_LEN], ciphertext, input_len);
                memcpy(&enc_packet[1 + CHACHA20_NONCE_LEN + input_len], tag, POLY1305_TAG_LEN);
                uint8_t enc_packet_len = (uint8_t)(1U + CHACHA20_NONCE_LEN + input_len + POLY1305_TAG_LEN);

                char num_buf[48];
                uint32_t np = 0;
                step7_append_str(num_buf, sizeof(num_buf), &np, "Encrypted packet size: ");
                step7_append_u32(num_buf, sizeof(num_buf), &np, (uint32_t)enc_packet_len);
                step7_append_str(num_buf, sizeof(num_buf), &np, " bytes");
                log_line("ENC", num_buf);

                uint8_t total_frags = (uint8_t)((enc_packet_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
                if (total_frags == 0U) {
                    total_frags = 1U;
                }

                np = 0;
                step7_append_str(num_buf, sizeof(num_buf), &np, "Fragments required: ");
                step7_append_u32(num_buf, sizeof(num_buf), &np, (uint32_t)total_frags);
                log_line("DATA", num_buf);

                log_line(TAG_ARQ, "Sending message...");

                step7_msg_report_t rep;
                int res = step7_send_blob_arq(enc_packet, enc_packet_len, cur_msg_id, &s_tx_seq, &s_rx_replay, &rep);

                if (res != 0) {
                    log_line(TAG_ARQ, "Message transmission result: SUCCESS");
                } else {
                    log_line(TAG_ARQ, "Message transmission result: FAILED");
                }

                input_len = 0U;
                oversized = 0U;
                UART1_SendString("Enter message: ");
            } else if (c == 0x08 || c == 0x7F) {
                /* Backspace handling */
                if (input_len > 0U) {
                    input_len--;
                    UART1_SendString("\b \b");
                }
                if (input_len <= MAX_ENC_PLAINTEXT_LEN) {
                    oversized = 0U;
                }
            } else if (c >= 0x20 && c <= 0x7E) {
                if (input_len < MAX_ENC_PLAINTEXT_LEN) {
                    input_buf[input_len++] = (char)c;
                    UART1_SendChar((char)c);
                } else {
                    oversized = 1U;
                    UART1_SendChar((char)c);
                }
            }
        }
        __asm__ volatile ("nop");
    }
}
#endif /* INTERACTIVE_MODE */

#endif /* STEP7_ARQ_MOCK || INTERACTIVE_MODE */

/*
 * Node A (Transmitter and Hop Master)
 * Step A2: SPI1 Master Mode 0, RegVersion check, retry logic, and safe halt.
 */
int main(void) {
    baremetal_init();

#if defined(INTERACTIVE_MODE)
    interactive_mode_run();
#elif defined(STEP7_ARQ_MOCK)
    step7_arq_mock_run();
#elif defined(STEP6_DATA_MOCK)
    step6_data_mock_run();
#elif defined(STEP5_FAULT_TEST)
    step5_fault_test_run();
#elif defined(STEP4_RADIO_SPI_DIAG)
    step4_radio_spi_diag_run();
#elif defined(STEP3_FHSS_MOCK)
    step3_fhss_mock_run();
#elif defined(STEP2_MOCK_TEST)
    step2_mock_run();
#elif defined(STEP1_UART_TEST)
    step1_uart_test_run();
#elif defined(SELFTEST_PINS)
    selftest_pins_run();
#elif defined(SELFTEST_SPI_LOOPBACK)
    selftest_spi_loopback_run();
#elif defined(DEMO_USART_LINK)
    node_a_demo_run();
#else

#ifdef SELFTEST_A1
    /* A1-N2: NULL and empty string safe */
    UART1_SendString((const char *)0);
    UART1_SendString("");
    log_line(TAG_TEST, "A1-N2 PASS");

    /* A1-P2: Verify BRR == 0x0045 */
    if (USART1->BRR == 0x0045UL) {
        log_line(TAG_TEST, "A1-P2 PASS");
    } else {
        log_line(TAG_TEST, "A1-P2 FAIL");
    }

    /* A1-P1: Verify millis() advances with delay_ms(100) */
    uint32_t t_start = millis();
    delay_ms(100);
    uint32_t t_elapsed = millis() - t_start;
    if (t_elapsed >= 98U && t_elapsed <= 102U) {
        log_line(TAG_TEST, "A1-P1 PASS");
    } else {
        log_line(TAG_TEST, "A1-P1 FAIL");
    }
#endif

    /* Radio reset: low >= 100 us, then wait >= 5 ms */
    sx1278_reset();

    /* Read RegVersion (0x42), expect 0x12; retry 3 times on mismatch */
    uint8_t ver = 0U;
    int radio_ok = 0;
    for (int attempt = 0; attempt < 4; attempt++) {
        ver = sx1278_read_reg(REG_VERSION);
        if (ver == SX1278_EXPECTED_VERSION) {
            radio_ok = 1;
            break;
        }
        delay_ms(25U);
    }

    if (radio_ok) {
        log_line(TAG_RADIO, "RegVersion=0x12 OK");
    } else {
        /* Probe pin orientations using bit-bang SPI */
        SPI1->CR1 &= ~SPI_CR1_SPE; /* Disable SPI1 */
        GPIOA->CRL &= ~((0xFUL << 20) | (0xFUL << 28));
        GPIOA->CRL |=  ((0x3UL << 20) | (0x3UL << 28)); /* PA5, PA7 GP Output PP */
        GPIOA->BRR = (1UL << 5) | (1UL << 7);

        /* Test 1: Standard (PA5=SCK, PA7=MOSI) */
        uint8_t v_std = 0;
        /* NSS LOW */
        GPIOB->BRR = (1UL << 0); GPIOA->BRR = (1UL << 4);
        delay_ms(1);
        for (int i = 7; i >= 0; i--) {
            if (0x42U & (1U << i)) { GPIOA->BSRR = (1UL << 7); } else { GPIOA->BRR = (1UL << 7); }
            delay_ms(1); GPIOA->BSRR = (1UL << 5); delay_ms(1); GPIOA->BRR = (1UL << 5);
        }
        for (int i = 7; i >= 0; i--) {
            GPIOA->BRR = (1UL << 7); delay_ms(1); GPIOA->BSRR = (1UL << 5); delay_ms(1);
            if (GPIOA->IDR & (1UL << 6)) { v_std |= (uint8_t)(1U << i); }
            GPIOA->BRR = (1UL << 5);
        }
        delay_ms(1); GPIOB->BSRR = (1UL << 0); GPIOA->BSRR = (1UL << 4); delay_ms(5);

        /* Test 2: Swapped (PA7=SCK, PA5=MOSI) */
        uint8_t v_swp = 0;
        GPIOB->BRR = (1UL << 0); GPIOA->BRR = (1UL << 4);
        delay_ms(1);
        for (int i = 7; i >= 0; i--) {
            if (0x42U & (1U << i)) { GPIOA->BSRR = (1UL << 5); } else { GPIOA->BRR = (1UL << 5); }
            delay_ms(1); GPIOA->BSRR = (1UL << 7); delay_ms(1); GPIOA->BRR = (1UL << 7);
        }
        for (int i = 7; i >= 0; i--) {
            GPIOA->BRR = (1UL << 5); delay_ms(1); GPIOA->BSRR = (1UL << 7); delay_ms(1);
            if (GPIOA->IDR & (1UL << 6)) { v_swp |= (uint8_t)(1U << i); }
            GPIOA->BRR = (1UL << 7);
        }
        delay_ms(1); GPIOB->BSRR = (1UL << 0); GPIOA->BSRR = (1UL << 4); delay_ms(5);

        char h_std[5], h_swp[5], diag_msg[48];
        uint8_to_hex(v_std, h_std);
        uint8_to_hex(v_swp, h_swp);

        char *dp = diag_msg;
        const char *m1 = "PROBE std="; while (*m1) *dp++ = *m1++;
        for (int i = 0; i < 4; i++) *dp++ = h_std[i];
        const char *m2 = " swp="; while (*m2) *dp++ = *m2++;
        for (int i = 0; i < 4; i++) *dp++ = h_swp[i];
        *dp = '\0';
        log_line(TAG_TEST, diag_msg);

        if (v_swp == 0x12U) {
            log_line(TAG_TEST, "Radio detected with PA5/PA7 swapped!");
        } else if (v_std == 0x12U) {
            log_line(TAG_TEST, "Radio detected with slow clock!");
        }

        /* Continuous diagnostic loop: read and write registers every 1s */
        while (1) {
            sx1278_write_reg(0x26U, 0x5AU);
            uint8_t r26 = sx1278_read_reg(0x26U);
            uint8_t r42 = sx1278_read_reg(0x42U);
            uint8_t r01 = sx1278_read_reg(0x01U);

            char h26[5], h42[5], h01[5], buf[64];
            uint8_to_hex(r26, h26);
            uint8_to_hex(r42, h42);
            uint8_to_hex(r01, h01);

            char *p = buf;
            const char *s1 = "RegVer="; while (*s1) *p++ = *s1++;
            for (int i = 0; i < 4; i++) *p++ = h42[i];
            const char *s2 = " OpMode="; while (*s2) *p++ = *s2++;
            for (int i = 0; i < 4; i++) *p++ = h01[i];
            const char *s3 = " TestR26="; while (*s3) *p++ = *s3++;
            for (int i = 0; i < 4; i++) *p++ = h26[i];
            *p = '\0';
            log_line(TAG_RADIO, buf);

            delay_ms(1000);
        }
    }

#ifdef SELFTEST_A2
    /* A2-P3: Write 0x5A to RegPreambleLsb (0x26), read back, restore 0x03 */
    sx1278_write_reg(REG_PREAMBLE_LSB, 0x5AU);
    uint8_t test_val = sx1278_read_reg(REG_PREAMBLE_LSB);
    sx1278_write_reg(REG_PREAMBLE_LSB, 0x03U);
    uint8_t restored_val = sx1278_read_reg(REG_PREAMBLE_LSB);

    if (test_val == 0x5AU && restored_val == 0x03U) {
        log_line(TAG_TEST, "A2-P3 PASS");
    } else {
        log_line(TAG_TEST, "A2-P3 FAIL");
    }
#endif

    uint32_t n = 0;
    while (1) {
        print_boot_hello(n++);
        delay_ms(1000);
    }
#endif

    return 0;
}
