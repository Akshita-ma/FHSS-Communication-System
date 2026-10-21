#include "common/status.h"
#include "common/config.h"
#include "common/hal.h"
#include "common/log.h"
#include "common/sx1278_fsk.h"
#include "stm32f4_regs.h"

/* Baseline hardware initialization in stm32_baremetal.c */
void baremetal_init(void);

#if !defined(SELFTEST_SPI_LOOPBACK) && !defined(SELFTEST_PINS)
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

#if !defined(DEMO_USART_LINK)
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
 * Print "[BOOT] node=B hello n\r\n"
 */
static void print_boot_hello(uint32_t n) {
    char num_buf[12];
    uint32_to_ascii(n, num_buf);

    UART1_SendString("[");
    UART1_SendString(TAG_BOOT);
    UART1_SendString("] node=B hello ");
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
     * PB0: NSS   - GP Output Push-Pull (MODER=01, OTYPER=0)
     * PA1: RESET - GP Output Push-Pull (MODER=01, OTYPER=0)
     * PA5: SCK   - GP Output Push-Pull (MODER=01, OTYPER=0)
     * PA7: MOSI  - GP Output Push-Pull (MODER=01, OTYPER=0)
     * PA6: MISO  - Input with pull-up (MODER=00, PUPDR=01)
     */
    GPIOB->MODER &= ~(3UL << 0);
    GPIOB->MODER |=  (1UL << 0);
    GPIOB->OTYPER &= ~(1UL << 0);

    GPIOA->MODER &= ~((3UL << 2) | (3UL << 10) | (3UL << 14));
    GPIOA->MODER |=  ((1UL << 2) | (1UL << 10) | (1UL << 14));
    GPIOA->OTYPER &= ~((1UL << 1) | (1UL << 5) | (1UL << 7));

    GPIOA->MODER &= ~(3UL << 12); /* Input mode */
    GPIOA->PUPDR &= ~(3UL << 12);
    GPIOA->PUPDR |=  (1UL << 12); /* Pull-Up */

    /* Initialize all 4 test pins LOW */
    GPIOB->BSRR = (1UL << (0 + 16)); /* PB0 LOW */
    GPIOA->BSRR = (1UL << (1 + 16)) | (1UL << (5 + 16)) | (1UL << (7 + 16)); /* PA1, PA5, PA7 LOW */

    while (1) {
        selftest_pin_toggle("NSS", &(GPIOB->BSRR), 0U);
        selftest_pin_toggle("RESET", &(GPIOA->BSRR), 1U);
        selftest_pin_toggle("SCK", &(GPIOA->BSRR), 5U);
        selftest_pin_toggle("MOSI", &(GPIOA->BSRR), 7U);
    }
}
#endif

/*
 * Node B (Receiver and Responder)
 * Step B2: SPI1 Master Mode 0, RegVersion check, retry logic, and safe halt.
 */
int main(void) {
    baremetal_init();

#if defined(SELFTEST_PINS)
    selftest_pins_run();
#elif defined(SELFTEST_SPI_LOOPBACK)
    selftest_spi_loopback_run();
#else

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
        /* On mismatch after 3 retries, report diagnostic and safe halt */
        if (ver == 0x00U) {
            log_line(TAG_ERR, "RegVersion=0x00 MISO stuck low");
        } else if (ver == 0xFFU) {
            log_line(TAG_ERR, "RegVersion=0xFF MISO stuck high");
        } else {
            char hex_buf[5];
            char err_buf[32];
            uint8_to_hex(ver, hex_buf);
            /* Construct "[ERR] RegVersion=0x.. mismatch" */
            char *p = err_buf;
            const char *prefix = "RegVersion=";
            while (*prefix != '\0') { *p++ = *prefix++; }
            p[0] = hex_buf[0]; p[1] = hex_buf[1]; p[2] = hex_buf[2]; p[3] = hex_buf[3];
            p += 4;
            const char *suffix = " mismatch";
            while (*suffix != '\0') { *p++ = *suffix++; }
            *p = '\0';
            log_line(TAG_ERR, err_buf);
        }
        NSS_HIGH();
        while (1) {
            __asm__ volatile ("nop");
        }
    }

    uint32_t n = 0;
    while (1) {
        print_boot_hello(n++);
        delay_ms(1000);
    }
#endif

    return 0;
}
