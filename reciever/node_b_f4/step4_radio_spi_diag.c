#include "step4_radio_spi_diag.h"
#include "sx1278_fsk.h"
#include "log.h"
#include "hal.h"
#include <stddef.h>

#ifndef HOST_TEST
#include "stm32f4_regs.h"
#endif

void step4_run_diagnostic(
    step4_diag_result_t *res,
    status_t (*read_fn)(uint8_t addr, uint8_t *val)
) {
    if (res == NULL || read_fn == NULL) return;

    res->total_reads = STEP4_READ_COUNT;
    res->pass_count = 0;
    res->fail_count = 0;
    res->first_unexpected = 0;
    res->first_unexpected_valid = 0;
    res->all_same = 1;

    for (uint8_t i = 0; i < STEP4_READ_COUNT; i++) {
        uint8_t val = 0;
        status_t st = read_fn(REG_VERSION, &val);
        res->reads[i] = val;
        res->statuses[i] = st;

        if (st == OK && val == SX1278_EXPECTED_REG_VERSION) {
            res->pass_count++;
        } else {
            res->fail_count++;
            if (!res->first_unexpected_valid) {
                res->first_unexpected = (st == OK) ? val : 0xEE;
                res->first_unexpected_valid = 1;
            }
        }

        if (i > 0 && (res->reads[i] != res->reads[0] || res->statuses[i] != res->statuses[0])) {
            res->all_same = 0;
        }
    }
}

#ifndef HOST_TEST

static void append_hex2(char *buf, int *p, uint8_t val) {
    const char hex[] = "0123456789ABCDEF";
    buf[(*p)++] = hex[(val >> 4) & 0x0F];
    buf[(*p)++] = hex[val & 0x0F];
}

static void append_dec_u8(char *buf, int *p, uint8_t val) {
    if (val == 0) {
        buf[(*p)++] = '0';
        return;
    }
    char rev[4];
    int r = 0;
    while (val > 0) {
        rev[r++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (r > 0) {
        buf[(*p)++] = rev[--r];
    }
}

static void log_raw_row(const char *label, const uint8_t *vals, uint8_t count) {
    char msg[48];
    int p = 0;
    for (int i = 0; label[i] != '\0'; i++) msg[p++] = label[i];
    for (uint8_t i = 0; i < count; i++) {
        append_hex2(msg, &p, vals[i]);
        if (i < count - 1) msg[p++] = ' ';
    }
    msg[p] = '\0';
    Log_Msg("RADIO", msg);
}

static void log_diagnostic_summary(const step4_diag_result_t *res) {
    char msg[64];
    int p = 0;
    const char p1[] = "RegVersion reads=";
    for (int i = 0; p1[i] != '\0'; i++) msg[p++] = p1[i];
    append_dec_u8(msg, &p, res->total_reads);

    const char p2[] = " pass=";
    for (int i = 0; p2[i] != '\0'; i++) msg[p++] = p2[i];
    append_dec_u8(msg, &p, res->pass_count);

    const char p3[] = " fail=";
    for (int i = 0; p3[i] != '\0'; i++) msg[p++] = p3[i];
    append_dec_u8(msg, &p, res->fail_count);

    if (res->first_unexpected_valid) {
        const char p4[] = " first_bad=0x";
        for (int i = 0; p4[i] != '\0'; i++) msg[p++] = p4[i];
        append_hex2(msg, &p, res->first_unexpected);
    }
    msg[p] = '\0';
    Log_Msg("TEST", msg);

    if (res->pass_count == res->total_reads) {
        Log_Msg("TEST", "SPI_READ PASS (all 0x12)");
    } else if (res->fail_count == res->total_reads && res->first_unexpected == 0x00) {
        Log_Msg("TEST", "SPI_READ FAIL: 32/32 reads returned 0x00 (MISO low / unpowered / reset)");
    } else if (res->fail_count == res->total_reads && res->first_unexpected == 0xFF) {
        Log_Msg("TEST", "SPI_READ FAIL: 32/32 reads returned 0xFF (MISO floating / pulled-up)");
    } else {
        Log_Msg("TEST", "SPI_READ FAIL: inconsistent reads");
    }
}

static void test_slower_spi_rate(void) {
    /* Temporarily reconfigure SPI1 prescaler to /128 (125 kHz @ 16 MHz HSI) for diagnostic comparison */
    Log_Msg("SPI", "Testing diagnostic rate: 125 kHz (DIV128)...");
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~(7UL << 3)) | (6UL << 3) | SPI_CR1_SPE; /* BR[2:0]=110 -> /128 */

    uint8_t slow_vals[8];
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t v = 0;
        sx1278_read_reg(REG_VERSION, &v);
        slow_vals[i] = v;
    }
    log_raw_row("125kHz R01..R08: ", slow_vals, 8);

    /* Restore production prescaler /16 (1.0 MHz @ 16 MHz HSI) */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~(7UL << 3)) | (3UL << 3) | SPI_CR1_SPE; /* BR[2:0]=011 -> /16 */
    Log_Msg("SPI", "Restored production rate: 1.0 MHz (DIV16)");
}

void step4_radio_spi_diag_run(void) {
    /* 1. Hardware baseline is initialized by baremetal_init() in main():
     *    - USART1: PA9 TX 115200 8N1
     *    - PA4: Output HIGH (on-board SPI flash deselected)
     *    - PB0: Output HIGH (SX1278 NSS inactive)
     *    - PA1: Output HIGH (SX1278 RESET inactive)
     *    - PA5/PA6/PA7: AF5 SPI1 (SCK, MISO with pull-up, MOSI)
     *    - SPI1: Master, CPOL=0 CPHA=0, 1.0 MHz (DIV16), MSB first, 8-bit
     */

    /* 2. Boot banner and configuration report */
    Log_Msg("BOOT", "node=B test=step4_radio_spi_diag");
    Log_Msg("SPI", "pins SCK=PA5 MISO=PA6 MOSI=PA7 NSS=PB0 RESET=PA1 PA4=FLASH_CS");
    Log_Msg("SPI", "cfg Mode=0 (CPOL=0 CPHA=0) 1.0MHz (DIV16) MSB-first 8-bit");
    Log_Msg("RADIO", "Target register: RegVersion (0x42), expected: 0x12");

    uint32_t loop_count = 0;

    while (1) {
        loop_count++;

        /* 3. Execute documented reset procedure */
        Log_Msg("RADIO", "Resetting SX1278 (PA1 low 5ms, high 15ms)...");
        sx1278_reset();

        /* 4. Execute 32 consecutive read-only queries of RegVersion */
        step4_diag_result_t res;
        step4_run_diagnostic(&res, sx1278_read_reg);

        /* 5. Report raw read observations (4 rows of 8 bytes) */
        log_raw_row("R01..R08: ", &res.reads[0], 8);
        log_raw_row("R09..R16: ", &res.reads[8], 8);
        log_raw_row("R17..R24: ", &res.reads[16], 8);
        log_raw_row("R25..R32: ", &res.reads[24], 8);

        /* 6. Report summary outcome */
        log_diagnostic_summary(&res);

        /* 7. If standard 1.0 MHz failed, run a test pass at 125 kHz */
        if (res.pass_count < res.total_reads) {
            test_slower_spi_rate();
        }

        /* 8. Log pin status observation */
        uint32_t pa_idr = GPIOA->IDR;
        char pin_msg[48];
        int p = 0;
        const char pfx[] = "PIN IDR PA6(MISO)=";
        for (int i = 0; pfx[i] != '\0'; i++) pin_msg[p++] = pfx[i];
        pin_msg[p++] = (pa_idr & (1UL << 6)) ? '1' : '0';
        const char p_sck[] = " PA5=";
        for (int i = 0; p_sck[i] != '\0'; i++) pin_msg[p++] = p_sck[i];
        pin_msg[p++] = (pa_idr & (1UL << 5)) ? '1' : '0';
        const char p_mosi[] = " PA7=";
        for (int i = 0; p_mosi[i] != '\0'; i++) pin_msg[p++] = p_mosi[i];
        pin_msg[p++] = (pa_idr & (1UL << 7)) ? '1' : '0';
        pin_msg[p] = '\0';
        Log_Msg("RADIO", pin_msg);

        Log_Msg("TEST", "Waiting 5s before next diagnostic sweep (probe signals on bench)...");
        for (int s = 0; s < 5; s++) {
            delay_ms(1000);
        }
    }
}

#endif
