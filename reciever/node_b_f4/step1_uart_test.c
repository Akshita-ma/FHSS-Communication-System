#include "step1_uart_test.h"
#include "fhss_protocol.h"
#include "log.h"
#include "hal.h"
#include <stddef.h>

#ifndef HOST_TEST
#include "air_usart2.h"
#endif

step1_rx_result_t step1_parse_ring_buffer(
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    uint8_t *ch_out,
    uint8_t *len_out,
    uint8_t *frame_buf_out,
    uint8_t *calc_crc_out,
    uint8_t *wire_crc_out
) {
    if (peek_fn == NULL || advance_fn == NULL) {
        return STEP1_RX_IDLE;
    }
    if (available == 0) {
        return STEP1_RX_IDLE;
    }

    /* 1. Search for sync byte 0x7E at offset 0 */
    uint8_t sync = 0;
    if (peek_fn(0, &sync) != (int)OK) {
        return STEP1_RX_IDLE;
    }

    if (sync != 0x7E) {
        /* Discard non-sync byte to find next potential delimiter */
        advance_fn(1);
        return STEP1_RX_DISCARD_GARBAGE;
    }

    /* Need at least 4 bytes: 0x7E, ch, len, crc */
    if (available < 4) {
        return STEP1_RX_IDLE;
    }

    uint8_t pkt_ch = 0, pkt_len = 0;
    peek_fn(1, &pkt_ch);
    peek_fn(2, &pkt_len);

    if (ch_out != NULL) *ch_out = pkt_ch;
    if (len_out != NULL) *len_out = pkt_len;

    /* 2. Validate channel */
    if (pkt_ch >= FHSS_NUM_CHANNELS) {
        advance_fn(1); /* Discard false delimiter */
        return STEP1_RX_ERR_CHANNEL;
    }

    /* 3. Validate frame length */
    if (pkt_len < HEADER_LEN || pkt_len > MAX_FRAME_LEN) {
        advance_fn(1); /* Discard false delimiter */
        return STEP1_RX_ERR_LEN;
    }

    /* 4. Check if full frame has arrived */
    uint16_t total_wire_len = (uint16_t)(1 + 1 + 1 + pkt_len + 1);
    if (available < total_wire_len) {
        return STEP1_RX_IDLE;
    }

    /* 5. Compute CRC over [pkt_ch, pkt_len, frame_bytes] */
    uint8_t calc_crc = crc8_smbus_cont(&pkt_ch, 1, CRC8_SMBUS_INIT);
    calc_crc = crc8_smbus_cont(&pkt_len, 1, calc_crc);

    for (uint8_t i = 0; i < pkt_len; i++) {
        uint8_t b = 0;
        peek_fn((uint16_t)(3 + i), &b);
        if (frame_buf_out != NULL) {
            frame_buf_out[i] = b;
        }
        calc_crc = crc8_smbus_cont(&b, 1, calc_crc);
    }

    uint8_t wire_crc = 0;
    peek_fn((uint16_t)(3 + pkt_len), &wire_crc);

    if (calc_crc_out != NULL) *calc_crc_out = calc_crc;
    if (wire_crc_out != NULL) *wire_crc_out = wire_crc;

    /* Consume full frame from buffer */
    advance_fn(total_wire_len);

    /* 6. Verify CRC */
    if (calc_crc != wire_crc) {
        return STEP1_RX_ERR_CRC;
    }

    return STEP1_RX_PASS;
}

#ifndef HOST_TEST

static void log_rx_pass(uint8_t len, uint8_t ch) {
    char msg[40];
    int p = 0;
    const char prefix[] = "UART2 RX len=";
    for (int i = 0; prefix[i] != '\0'; i++) msg[p++] = prefix[i];
    if (len >= 10) {
        msg[p++] = (char)('0' + (len / 10));
        msg[p++] = (char)('0' + (len % 10));
    } else {
        msg[p++] = (char)('0' + len);
    }
    const char ch_str[] = " ch=";
    for (int i = 0; ch_str[i] != '\0'; i++) msg[p++] = ch_str[i];
    msg[p++] = (char)('0' + ch);
    const char pass_str[] = " PASS";
    for (int i = 0; pass_str[i] != '\0'; i++) msg[p++] = pass_str[i];
    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_stats_summary(uint32_t pass_cnt, uint32_t crc_cnt, uint32_t len_cnt, char *buf) {
    int p = 0;
    const char p1[] = "step1 pass=";
    for (int i = 0; p1[i] != '\0'; i++) buf[p++] = p1[i];

    char num[12];
    int k = 0;
    uint32_t temp = pass_cnt;
    if (temp == 0) {
        num[k++] = '0';
    } else {
        char rev[12];
        int r = 0;
        while (temp > 0) { rev[r++] = (char)('0' + (temp % 10)); temp /= 10; }
        while (r > 0) { num[k++] = rev[--r]; }
    }
    for (int i = 0; i < k; i++) buf[p++] = num[i];

    const char p2[] = " crc_err=";
    for (int i = 0; p2[i] != '\0'; i++) buf[p++] = p2[i];
    k = 0;
    temp = crc_cnt;
    if (temp == 0) {
        num[k++] = '0';
    } else {
        char rev[12];
        int r = 0;
        while (temp > 0) { rev[r++] = (char)('0' + (temp % 10)); temp /= 10; }
        while (r > 0) { num[k++] = rev[--r]; }
    }
    for (int i = 0; i < k; i++) buf[p++] = num[i];

    const char p3[] = " len_err=";
    for (int i = 0; p3[i] != '\0'; i++) buf[p++] = p3[i];
    k = 0;
    temp = len_cnt;
    if (temp == 0) {
        num[k++] = '0';
    } else {
        char rev[12];
        int r = 0;
        while (temp > 0) { rev[r++] = (char)('0' + (temp % 10)); temp /= 10; }
        while (r > 0) { num[k++] = rev[--r]; }
    }
    for (int i = 0; i < k; i++) buf[p++] = num[i];

    buf[p] = '\0';
}

void step1_uart_test_run(void) {
    /* 1. Initialize USART2 wire mock peripheral and pins */
    air_usart2_init();

    /* 2. Log boot banner over USART1 */
    Log_Msg("BOOT", "node=B test=step1_uart_test");
    Log_Msg("BOOT", "link=USART2 baud=115200 PA2=TX PA3=RX");
    Log_Msg("RADIO", "Bypassed for Step 1 wired test");

    /* 3. Run known-answer vector self-test */
    if (fhss_self_test_vectors()) {
        Log_Msg("TEST", "vectors PASS");
    } else {
        Log_Msg("TEST", "vectors FAIL");
    }

    Log_Msg("TEST", "Step 1 receiver waiting for Node A frames...");

    uint8_t frame_buf[MAX_FRAME_LEN];
    uint8_t pkt_ch = 0;
    uint8_t pkt_len = 0;
    uint8_t calc_crc = 0;
    uint8_t wire_crc = 0;

    uint32_t last_stat_ms = millis();
    uint32_t rx_pass_count = 0;
    uint32_t rx_crc_err_count = 0;
    uint32_t rx_len_err_count = 0;

    while (1) {
        /* Check hardware overrun and buffer overflow flags */
        uint32_t ore = air_usart2_get_ore_count();
        uint32_t ovf = air_usart2_get_overflow_count();
        if (ore > 0 || ovf > 0) {
            air_usart2_clear_errors();
            if (ore > 0) {
                Log_Msg("TEST", "UART2 hardware overrun detected");
            }
            if (ovf > 0) {
                Log_Msg("TEST", "UART2 RX buffer overflow detected");
            }
        }

        uint16_t avail = air_usart2_rx_available();
        if (avail > 0) {
            step1_rx_result_t res = step1_parse_ring_buffer(
                avail,
                air_usart2_rx_peek,
                air_usart2_rx_advance,
                &pkt_ch,
                &pkt_len,
                frame_buf,
                &calc_crc,
                &wire_crc
            );

            if (res == STEP1_RX_PASS) {
                rx_pass_count++;
                log_rx_pass(pkt_len, pkt_ch);

                /* Transmit single raw test-response byte 0x06 on USART2 */
                air_usart2_write_byte(STEP1_TEST_ACK_BYTE);
                Log_Msg("TEST", "ACK_SENT value=0x06");
            } else if (res == STEP1_RX_ERR_CRC) {
                rx_crc_err_count++;
                Log_Msg("TEST", "UART2 RX CRC mismatch");
            } else if (res == STEP1_RX_ERR_LEN) {
                rx_len_err_count++;
                Log_Msg("TEST", "UART2 RX length error");
            } else if (res == STEP1_RX_ERR_CHANNEL) {
                Log_Msg("TEST", "UART2 RX invalid channel");
            }
        }

        /* Periodic stat report every 10 seconds */
        uint32_t now = millis();
        if ((uint32_t)(now - last_stat_ms) >= 10000UL) {
            last_stat_ms = now;
            char stat_buf[64];
            log_stats_summary(rx_pass_count, rx_crc_err_count, rx_len_err_count, stat_buf);
            Log_Msg("STAT", stat_buf);
        }
    }
}
#endif
