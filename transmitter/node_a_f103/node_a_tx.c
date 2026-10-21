#include "common/status.h"
#include "common/config.h"
#include "common/hal.h"
#include "common/log.h"
#include "common/sx1278_fsk.h"
#include "stm32f1_regs.h"

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

#ifdef SELFTEST_SPI_LOOPBACK
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

#if defined(DEMO_USART_LINK) || defined(STEP2_MOCK_TEST)
#include "common/air.h"
#include "common/fhss_protocol.h"
#endif

#if defined(DEMO_USART_LINK) && !defined(STEP2_MOCK_TEST)

void mock_air_notify_locked(void);

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
    log_line(TAG_BOOT, "node=A link=USART2");
#ifdef DEMO_TRACE
    log_line(TAG_BOOT, "trace PB1=hop PB10=tx PB12=ack PB13..15=ch PA8=jam PB5=locked");
#endif

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

                    /* Wait up to ACK_TIMEOUT_MS for ACK */
                    uint32_t t_ack_start = millis();
                    int ack_rcvd = 0;
                    while ((millis() - t_ack_start) < ACK_TIMEOUT_MS) {
                        TIM2_Poll();
                        uint8_t rx_buf[MAX_FRAME_LEN];
                        uint8_t rx_len = 0U;
                        if (air_rx_poll(s_current_ch, rx_buf, &rx_len) == OK) {
                            fhss_frame_t parsed;
                            if (fhss_parse_frame(rx_buf, rx_len, DIR_B_TO_A, &s_replay, &parsed) == OK) {
                                if (parsed.type == PKT_TYPE_ACK) {
                                    ack_rcvd = 1;
#ifdef DEMO_TRACE
                                    GPIOB->BSRR = (1UL << 12);
                                    uint32_t tp = micros();
                                    while ((micros() - tp) < 200U) { __asm__ volatile ("nop"); }
                                    GPIOB->BRR = (1UL << 12);
#endif
                                    break;
                                }
                            }
                        }
                    }

                    if (ack_rcvd != 0) {
                        s_missed_hops = 0U;
                    } else {
                        s_missed_hops++;
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

/*
 * Node A (Transmitter and Hop Master)
 * Step A2: SPI1 Master Mode 0, RegVersion check, retry logic, and safe halt.
 */
int main(void) {
    baremetal_init();

#if defined(STEP2_MOCK_TEST)
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
