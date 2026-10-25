#include "step2_mock.h"
#include "step1_uart_test.h"
#include "fhss_protocol.h"
#include "log.h"
#include "hal.h"
#include <stddef.h>

#ifndef HOST_TEST
#include "air_usart2.h"
#endif

step2_result_t step2_process_incoming(
    uint16_t available,
    int (*peek_fn)(uint16_t offset, uint8_t *b),
    void (*advance_fn)(uint16_t count),
    fhss_packet_t *pkt_out,
    uint8_t *next_ch_out,
    uint8_t *dwell_code_out
) {
    uint8_t ch = 0;
    uint8_t len = 0;
    uint8_t frame_buf[MAX_FRAME_LEN];
    uint8_t calc_crc = 0;
    uint8_t wire_crc = 0;

    /* 1. Reuse existing envelope parser */
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
        return STEP2_RESULT_IDLE;
    }
    if (env_res == STEP1_RX_DISCARD_GARBAGE) {
        return STEP2_RESULT_ENV_DISCARD;
    }
    if (env_res == STEP1_RX_ERR_CRC) {
        return STEP2_RESULT_ENV_ERR_CRC;
    }
    if (env_res == STEP1_RX_ERR_LEN) {
        return STEP2_RESULT_ENV_ERR_LEN;
    }
    if (env_res == STEP1_RX_ERR_CHANNEL) {
        return STEP2_RESULT_ENV_ERR_CH;
    }

    /* 2. Envelope passed. Check home channel 0 requirement for Stage 2 */
    if (ch != FHSS_HOME_CHANNEL) {
        return STEP2_RESULT_PROTO_REJECT_CH;
    }

    /* 3. Pass frame body to existing frozen FHSS frame parser */
    fhss_packet_t parsed_pkt;
    status_t st = fhss_parse_packet(frame_buf, len, 0, &parsed_pkt);
    if (st != OK) {
        return STEP2_RESULT_PROTO_REJECT_TAG;
    }

    /* 4. Semantic validation of packet type */
    uint8_t type = (uint8_t)(parsed_pkt.ver_type & 0x0F);
    if (type != PKT_TYPE_DATA && type != PKT_TYPE_ACK && type != PKT_TYPE_HOP) {
        return STEP2_RESULT_PROTO_REJECT_SEMANTIC;
    }

    /* For DATA packets, validate fragment bounds: idx must be < total */
    if (type == PKT_TYPE_DATA) {
        uint8_t total = (uint8_t)(((parsed_pkt.frag >> 5) & 7) + 1);
        uint8_t idx   = (uint8_t)((parsed_pkt.frag >> 2) & 7);
        if (idx >= total) {
            return STEP2_RESULT_PROTO_REJECT_SEMANTIC;
        }
    }

    /* 5. Unmask hop directive using PRNG mask for Node A (dir=0) */
    uint8_t mask = fhss_generate_mask(parsed_pkt.seq, 0);
    uint8_t plaintext = (uint8_t)(parsed_pkt.hop_masked ^ mask);
    uint8_t next_ch = (uint8_t)(plaintext >> 5);
    uint8_t dwell_code = (uint8_t)(plaintext & DWELL_CODE_MAX);

    if (next_ch >= FHSS_NUM_CHANNELS || dwell_code > DWELL_CODE_MAX) {
        return STEP2_RESULT_PROTO_REJECT_SEMANTIC;
    }

    if (next_ch_out != NULL) *next_ch_out = next_ch;
    if (dwell_code_out != NULL) *dwell_code_out = dwell_code;
    if (pkt_out != NULL) *pkt_out = parsed_pkt;

    return STEP2_RESULT_PASS;
}

#ifndef HOST_TEST

static void log_envelope_pass(uint8_t len, uint8_t ch) {
    char msg[40];
    int p = 0;
    const char pfx[] = "UART2 RX envelope PASS len=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];
    if (len >= 10) {
        msg[p++] = (char)('0' + (len / 10));
        msg[p++] = (char)('0' + (len % 10));
    } else {
        msg[p++] = (char)('0' + len);
    }
    const char c_s[] = " ch=";
    for (int i = 0; c_s[i] != '\0'; i++) msg[p++] = c_s[i];
    msg[p++] = (char)('0' + ch);
    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_protocol_pass(const fhss_packet_t *pkt, uint8_t next_ch, uint8_t code) {
    char msg[72];
    int p = 0;
    const char pfx[] = "PROTOCOL PASS type=";
    for (int i = 0; pfx[i] != '\0'; i++) msg[p++] = pfx[i];

    uint8_t type = (uint8_t)(pkt->ver_type & 0x0F);
    if (type == PKT_TYPE_DATA) {
        msg[p++] = 'D'; msg[p++] = 'A'; msg[p++] = 'T'; msg[p++] = 'A';
    } else if (type == PKT_TYPE_ACK) {
        msg[p++] = 'A'; msg[p++] = 'C'; msg[p++] = 'K';
    } else if (type == PKT_TYPE_HOP) {
        msg[p++] = 'H'; msg[p++] = 'O'; msg[p++] = 'P';
    } else {
        msg[p++] = '?';
    }

    const char s_seq[] = " seq=";
    for (int i = 0; s_seq[i] != '\0'; i++) msg[p++] = s_seq[i];
    if (pkt->seq >= 100) {
        msg[p++] = (char)('0' + (pkt->seq / 100));
        msg[p++] = (char)('0' + ((pkt->seq / 10) % 10));
        msg[p++] = (char)('0' + (pkt->seq % 10));
    } else if (pkt->seq >= 10) {
        msg[p++] = (char)('0' + (pkt->seq / 10));
        msg[p++] = (char)('0' + (pkt->seq % 10));
    } else {
        msg[p++] = (char)('0' + pkt->seq);
    }

    const char s_id[] = " id=";
    for (int i = 0; s_id[i] != '\0'; i++) msg[p++] = s_id[i];
    if (pkt->msg_id >= 10) {
        msg[p++] = (char)('0' + (pkt->msg_id / 10));
        msg[p++] = (char)('0' + (pkt->msg_id % 10));
    } else {
        msg[p++] = (char)('0' + pkt->msg_id);
    }

    const char s_frag[] = " frag=";
    for (int i = 0; s_frag[i] != '\0'; i++) msg[p++] = s_frag[i];
    uint8_t tot = (uint8_t)(((pkt->frag >> 5) & 7) + 1);
    uint8_t idx = (uint8_t)((pkt->frag >> 2) & 7);
    msg[p++] = (char)('0' + idx);
    msg[p++] = '/';
    msg[p++] = (char)('0' + tot);

    const char s_ch[] = " next_ch=";
    for (int i = 0; s_ch[i] != '\0'; i++) msg[p++] = s_ch[i];
    msg[p++] = (char)('0' + next_ch);

    const char s_code[] = " code=";
    for (int i = 0; s_code[i] != '\0'; i++) msg[p++] = s_code[i];
    if (code >= 10) {
        msg[p++] = (char)('0' + (code / 10));
        msg[p++] = (char)('0' + (code % 10));
    } else {
        msg[p++] = (char)('0' + code);
    }

    msg[p] = '\0';
    Log_Msg("TEST", msg);
}

static void log_stats_summary(uint32_t pass_cnt, uint32_t env_err, uint32_t proto_err, char *buf) {
    int p = 0;
    const char p1[] = "step2 pass=";
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

    const char p2[] = " env_err=";
    for (int i = 0; p2[i] != '\0'; i++) buf[p++] = p2[i];
    k = 0;
    temp = env_err;
    if (temp == 0) {
        num[k++] = '0';
    } else {
        char rev[12];
        int r = 0;
        while (temp > 0) { rev[r++] = (char)('0' + (temp % 10)); temp /= 10; }
        while (r > 0) { num[k++] = rev[--r]; }
    }
    for (int i = 0; i < k; i++) buf[p++] = num[i];

    const char p3[] = " proto_err=";
    for (int i = 0; p3[i] != '\0'; i++) buf[p++] = p3[i];
    k = 0;
    temp = proto_err;
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

void step2_mock_run(void) {
    /* 1. Initialize USART2 wire mock */
    air_usart2_init();

    /* 2. Log boot banner over USART1 */
    Log_Msg("BOOT", "node=B test=step2_mock");
    Log_Msg("BOOT", "link=USART2 baud=115200 PA2=TX PA3=RX ch=0");
    Log_Msg("RADIO", "Bypassed for Step 2 mock test");

    /* 3. Run known-answer vector self-test */
    if (fhss_self_test_vectors()) {
        Log_Msg("TEST", "vectors PASS");
    } else {
        Log_Msg("TEST", "vectors FAIL");
    }

    Log_Msg("TEST", "Step 2 receiver waiting for Node A FHSS frames on ch=0...");

    fhss_packet_t pkt;
    uint8_t next_ch = 0;
    uint8_t dwell_code = 0;

    uint32_t last_stat_ms = millis();
    uint32_t pass_count = 0;
    uint32_t env_err_count = 0;
    uint32_t proto_err_count = 0;

    while (1) {
        /* Check hardware overrun and buffer overflow flags */
        uint32_t ore = air_usart2_get_ore_count();
        uint32_t ovf = air_usart2_get_overflow_count();
        if (ore > 0 || ovf > 0) {
            air_usart2_clear_errors();
            if (ore > 0) Log_Msg("TEST", "UART2 hardware overrun detected");
            if (ovf > 0) Log_Msg("TEST", "UART2 RX buffer overflow detected");
        }

        uint16_t avail = air_usart2_rx_available();
        if (avail > 0) {
            step2_result_t res = step2_process_incoming(
                avail,
                air_usart2_rx_peek,
                air_usart2_rx_advance,
                &pkt,
                &next_ch,
                &dwell_code
            );

            if (res == STEP2_RESULT_PASS) {
                pass_count++;
                log_envelope_pass((uint8_t)(HEADER_LEN + pkt.payload_len), FHSS_HOME_CHANNEL);
                log_protocol_pass(&pkt, next_ch, dwell_code);

                /* Transmit Stage 2 test-harness receipt marker 0x06 */
                air_usart2_write_byte(STEP2_TEST_ACK_BYTE);
                Log_Msg("TEST", "ACK_SENT value=0x06");
            } else if (res == STEP2_RESULT_ENV_ERR_CRC) {
                env_err_count++;
                Log_Msg("TEST", "UART2 RX envelope CRC FAIL");
            } else if (res == STEP2_RESULT_ENV_ERR_LEN) {
                env_err_count++;
                Log_Msg("TEST", "UART2 RX envelope len FAIL");
            } else if (res == STEP2_RESULT_ENV_ERR_CH) {
                env_err_count++;
                Log_Msg("TEST", "UART2 RX envelope ch FAIL");
            } else if (res == STEP2_RESULT_PROTO_REJECT_CH) {
                proto_err_count++;
                Log_Msg("TEST", "PROTOCOL REJECT ch!=0");
            } else if (res == STEP2_RESULT_PROTO_REJECT_TAG) {
                proto_err_count++;
                Log_Msg("TEST", "PROTOCOL REJECT tag FAIL");
            } else if (res == STEP2_RESULT_PROTO_REJECT_SEMANTIC) {
                proto_err_count++;
                Log_Msg("TEST", "PROTOCOL REJECT semantic FAIL");
            }
        }

        /* Periodic stat report every 10 seconds */
        uint32_t now = millis();
        if ((uint32_t)(now - last_stat_ms) >= 10000UL) {
            last_stat_ms = now;
            char stat_buf[64];
            log_stats_summary(pass_count, env_err_count, proto_err_count, stat_buf);
            Log_Msg("STAT", stat_buf);
        }
    }
}
#endif
