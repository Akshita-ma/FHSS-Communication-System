#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "status.h"
#include "config.h"
#include "log.h"
#include "mock_hal.h"

static void test_status_strings(void) {
    assert(strcmp(status_to_string(OK), "OK") == 0);
    assert(strcmp(status_to_string(ERR_NULL), "ERR_NULL") == 0);
    assert(strcmp(status_to_string(ERR_LEN), "ERR_LEN") == 0);
    assert(strcmp(status_to_string(ERR_RANGE), "ERR_RANGE") == 0);
    assert(strcmp(status_to_string(ERR_TIMEOUT), "ERR_TIMEOUT") == 0);
    assert(strcmp(status_to_string(ERR_CRC), "ERR_CRC") == 0);
    assert(strcmp(status_to_string(ERR_TAG), "ERR_TAG") == 0);
    assert(strcmp(status_to_string(ERR_REPLAY), "ERR_REPLAY") == 0);
    assert(strcmp(status_to_string(ERR_NOACK), "ERR_NOACK") == 0);
    assert(strcmp(status_to_string(ERR_STATE), "ERR_STATE") == 0);
    assert(strcmp(status_to_string(ERR_HW), "ERR_HW") == 0);
    printf("[HOST_TEST] test_status_strings: PASS\n");
}

static void test_channel_plan(void) {
    uint32_t frf = 0;
    assert(config_get_channel_frf(0, &frf) == OK);
    assert(frf == 0x6C4666);

    assert(config_get_channel_frf(7, &frf) == OK);
    assert(frf == 0x6CA000);

    /* Out of range channel */
    assert(config_get_channel_frf(8, &frf) == ERR_RANGE);

    /* NULL pointer validation */
    assert(config_get_channel_frf(0, NULL) == ERR_NULL);

    uint8_t msb = 0, mid = 0, lsb = 0;
    assert(config_get_channel_bytes(3, &msb, &mid, &lsb) == OK);
    assert(msb == 0x6C && mid == 0x6C && lsb == 0xCD);

    printf("[HOST_TEST] test_channel_plan: PASS\n");
}

static void test_log_formatter(void) {
    mock_hal_reset();

    Log_Msg("BOOT", "Test boot message");
    const char *log = mock_hal_get_uart_log();
    assert(strcmp(log, "[BOOT] Test boot message\r\n") == 0);

    mock_hal_reset();
    Log_Test("B0-01", 1);
    log = mock_hal_get_uart_log();
    assert(strcmp(log, "[TEST] B0-01 PASS\r\n") == 0);

    mock_hal_reset();
    Log_Test("B0-02", 0);
    log = mock_hal_get_uart_log();
    assert(strcmp(log, "[TEST] B0-02 FAIL\r\n") == 0);

    printf("[HOST_TEST] test_log_formatter: PASS\n");
}

static void test_hal_null_safety(void) {
    mock_hal_reset();
    /* Passing NULL to UART1_SendString must be safe */
    UART1_SendString(NULL);
    assert(mock_hal_get_uart_log_len() == 0);

    /* TIM2_ScheduleHopMs input validation */
    assert(TIM2_ScheduleHopMs(0, NULL) == (int)ERR_RANGE);
    printf("[HOST_TEST] test_hal_null_safety: PASS\n");
}

#include "sx1278_fsk.h"

static void test_sx1278_spi(void) {
    mock_hal_reset();
    uint8_t ver = 0;

    /* NULL check */
    assert(sx1278_read_reg(REG_VERSION, NULL) == ERR_NULL);

    /* Out of range address check */
    assert(sx1278_read_reg(0x80, &ver) == ERR_RANGE);
    assert(sx1278_write_reg(0x80, 0x12) == ERR_RANGE);

    /* Read default RegVersion (0x42 == 0x12) */
    assert(sx1278_read_reg(REG_VERSION, &ver) == OK);
    assert(ver == 0x12);
    assert(mock_hal_get_nss_state() == 1); /* NSS should return HIGH after transfer */

    /* Write register and read back */
    assert(sx1278_write_reg(REG_OP_MODE, 0x01) == OK);
    uint8_t op_mode = 0;
    assert(sx1278_read_reg(REG_OP_MODE, &op_mode) == OK);
    assert(op_mode == 0x01);

    /* Reset sequence verification */
    assert(sx1278_reset() == OK);
    assert(mock_hal_get_reset_pin_state() == 1); /* Should end up HIGH */

    /* Verify mismatch handling */
    mock_hal_set_reg(REG_VERSION, 0x00);
    assert(sx1278_read_reg(REG_VERSION, &ver) == OK);
    assert(ver == 0x00);

    mock_hal_set_reg(REG_VERSION, 0xFF);
    assert(sx1278_read_reg(REG_VERSION, &ver) == OK);
    assert(ver == 0xFF);

    printf("[HOST_TEST] test_sx1278_spi: PASS\n");
}

#include "step1_uart_test.h"

static uint8_t s_mock_rx_ring[256];
static uint16_t s_mock_rx_head = 0;
static uint16_t s_mock_rx_tail = 0;

static void mock_rx_reset(void) {
    s_mock_rx_head = 0;
    s_mock_rx_tail = 0;
}

static void mock_rx_push(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        s_mock_rx_ring[s_mock_rx_head] = data[i];
        s_mock_rx_head = (uint16_t)((s_mock_rx_head + 1) & 255);
    }
}

static int mock_rx_peek(uint16_t offset, uint8_t *b) {
    uint16_t avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    if (offset >= avail) return (int)ERR_RANGE;
    *b = s_mock_rx_ring[(s_mock_rx_tail + offset) & 255];
    return (int)OK;
}

static void mock_rx_advance(uint16_t count) {
    uint16_t avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    if (count > avail) count = avail;
    s_mock_rx_tail = (uint16_t)((s_mock_rx_tail + count) & 255);
}

static void test_step1_framing(void) {
    mock_rx_reset();

    /* 1. Test Valid Frame:
     * 7E 00 0B A1 05 03 E8 6A 7D 48 45 4C 4C 4F 18
     */
    const uint8_t valid_frame[15] = {
        0x7E, 0x00, 0x0B,
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x7D, 0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0x18
    };
    mock_rx_push(valid_frame, 15);

    uint8_t ch = 0xFF, len = 0;
    uint8_t frame_buf[32];
    uint8_t calc_crc = 0, wire_crc = 0;

    uint16_t avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    step1_rx_result_t res = step1_parse_ring_buffer(
        avail, mock_rx_peek, mock_rx_advance,
        &ch, &len, frame_buf, &calc_crc, &wire_crc
    );

    assert(res == STEP1_RX_PASS);
    assert(ch == 0x00);
    assert(len == 0x0B);
    assert(calc_crc == 0x18);
    assert(wire_crc == 0x18);
    assert(memcmp(frame_buf, &valid_frame[3], 11) == 0);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    /* 2. Test CRC Mismatch */
    mock_rx_reset();
    uint8_t bad_crc_frame[15];
    memcpy(bad_crc_frame, valid_frame, 15);
    bad_crc_frame[14] = 0x99;
    mock_rx_push(bad_crc_frame, 15);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step1_parse_ring_buffer(avail, mock_rx_peek, mock_rx_advance, &ch, &len, frame_buf, &calc_crc, &wire_crc);
    assert(res == STEP1_RX_ERR_CRC);
    assert(calc_crc == 0x18);
    assert(wire_crc == 0x99);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    /* 3. Test Length Error (< 6) */
    mock_rx_reset();
    uint8_t bad_len_frame[5] = {0x7E, 0x00, 0x03, 0x01, 0x18};
    mock_rx_push(bad_len_frame, 5);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step1_parse_ring_buffer(avail, mock_rx_peek, mock_rx_advance, &ch, &len, frame_buf, &calc_crc, &wire_crc);
    assert(res == STEP1_RX_ERR_LEN);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 4);

    /* 4. Test Garbage Discard before valid frame */
    mock_rx_reset();
    uint8_t garbage[3] = {0xAA, 0x55, 0x00};
    mock_rx_push(garbage, 3);
    mock_rx_push(valid_frame, 15);

    for (int g = 0; g < 3; g++) {
        avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
        res = step1_parse_ring_buffer(avail, mock_rx_peek, mock_rx_advance, &ch, &len, frame_buf, &calc_crc, &wire_crc);
        assert(res == STEP1_RX_DISCARD_GARBAGE);
    }
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step1_parse_ring_buffer(avail, mock_rx_peek, mock_rx_advance, &ch, &len, frame_buf, &calc_crc, &wire_crc);
    assert(res == STEP1_RX_PASS);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    /* 5. Test Incomplete Frame */
    mock_rx_reset();
    mock_rx_push(valid_frame, 8);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step1_parse_ring_buffer(avail, mock_rx_peek, mock_rx_advance, &ch, &len, frame_buf, &calc_crc, &wire_crc);
    assert(res == STEP1_RX_IDLE);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 8);

    printf("[HOST_TEST] test_step1_framing: PASS\n");
}

#include "step2_mock.h"

static void test_step2_protocol(void) {
    mock_rx_reset();

    /* 1. Test Valid Stage 2 Known-Answer Frame:
     * 7E 00 0B A1 05 03 E8 6A 7D 48 45 4C 4C 4F 18
     */
    const uint8_t valid_vector[15] = {
        0x7E, 0x00, 0x0B,
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x7D, 0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0x18
    };
    mock_rx_push(valid_vector, 15);

    fhss_packet_t pkt;
    uint8_t next_ch = 0xFF;
    uint8_t dwell_code = 0xFF;

    uint16_t avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    step2_result_t res = step2_process_incoming(
        avail, mock_rx_peek, mock_rx_advance,
        &pkt, &next_ch, &dwell_code
    );

    assert(res == STEP2_RESULT_PASS);
    assert((pkt.ver_type & 0xF0) == 0xA0);
    assert((pkt.ver_type & 0x0F) == PKT_TYPE_DATA);
    assert(pkt.seq == 5);
    assert(pkt.msg_id == 3);
    assert(pkt.frag == 0xE8);
    assert((((pkt.frag >> 5) & 7) + 1) == 8); /* TOTAL = 8 */
    assert(((pkt.frag >> 2) & 7) == 2);        /* IDX = 2 */
    assert(((pkt.frag >> 1) & 1) == 0);        /* LAST = 0 */
    assert(pkt.hop_masked == 0x6A);
    assert(next_ch == 5);                      /* Unmasked next_ch = 5 */
    assert(dwell_code == 17);                  /* Unmasked code = 17 */
    assert(pkt.tag == 0x7D);
    assert(pkt.payload_len == 5);
    assert(memcmp(pkt.payload, "HELLO", 5) == 0);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0); /* Consumed */

    /* 2. Test Envelope CRC Failure */
    mock_rx_reset();
    uint8_t bad_env_crc[15];
    memcpy(bad_env_crc, valid_vector, 15);
    bad_env_crc[14] = 0x99;
    mock_rx_push(bad_env_crc, 15);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step2_process_incoming(avail, mock_rx_peek, mock_rx_advance, &pkt, &next_ch, &dwell_code);
    assert(res == STEP2_RESULT_ENV_ERR_CRC);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    /* 3. Test Channel != 0 Rejection (ch=1, env_crc=0x8C) */
    mock_rx_reset();
    uint8_t ch1_frame[15];
    memcpy(ch1_frame, valid_vector, 15);
    ch1_frame[1] = 0x01; /* ch = 1 */
    ch1_frame[14] = 0x8C; /* Valid envelope CRC for ch=1 */
    mock_rx_push(ch1_frame, 15);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step2_process_incoming(avail, mock_rx_peek, mock_rx_advance, &pkt, &next_ch, &dwell_code);
    assert(res == STEP2_RESULT_PROTO_REJECT_CH);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    /* 4. Test Protocol TAG Rejection (corrupted TAG byte 5 = 0x00, env_crc=0x09) */
    mock_rx_reset();
    uint8_t bad_tag_frame[15];
    memcpy(bad_tag_frame, valid_vector, 15);
    bad_tag_frame[8] = 0x00; /* tag = 0x00 (frame_bytes[5]) */
    bad_tag_frame[14] = 0x09; /* Valid envelope CRC for this modified frame */
    mock_rx_push(bad_tag_frame, 15);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step2_process_incoming(avail, mock_rx_peek, mock_rx_advance, &pkt, &next_ch, &dwell_code);
    assert(res == STEP2_RESULT_PROTO_REJECT_TAG);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    /* 5. Test Semantic Rejection: Fragment idx >= total (idx=3, total=2 -> frag=0x2C) */
    mock_rx_reset();
    uint8_t bad_frag_wire[16];
    fhss_packet_t bad_frag_pkt;
    bad_frag_pkt.ver_type = PKT_VER_PREFIX | PKT_TYPE_DATA;
    bad_frag_pkt.seq = 5;
    bad_frag_pkt.msg_id = 3;
    bad_frag_pkt.frag = (1 << 5) | (3 << 2); /* TOTAL=2, IDX=3 -> idx >= total! */
    bad_frag_pkt.hop_masked = 0x6A;
    bad_frag_pkt.payload[0] = 'H'; bad_frag_pkt.payload[1] = 'E';
    bad_frag_pkt.payload_len = 2;

    uint8_t bf_wire_len = 0;
    fhss_build_packet(&bad_frag_pkt, bad_frag_wire, &bf_wire_len);

    uint8_t env_bf[32];
    env_bf[0] = 0x7E;
    env_bf[1] = 0x00;
    env_bf[2] = bf_wire_len;
    memcpy(&env_bf[3], bad_frag_wire, bf_wire_len);
    env_bf[3 + bf_wire_len] = crc8_smbus_cont(&env_bf[1], (uint8_t)(2 + bf_wire_len), CRC8_SMBUS_INIT);
    mock_rx_push(env_bf, (uint16_t)(4 + bf_wire_len));

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    res = step2_process_incoming(avail, mock_rx_peek, mock_rx_advance, &pkt, &next_ch, &dwell_code);
    assert(res == STEP2_RESULT_PROTO_REJECT_SEMANTIC);
    assert(((s_mock_rx_head - s_mock_rx_tail) & 255) == 0);

    printf("[HOST_TEST] test_step2_protocol: PASS\n");
}

int main(void) {
    printf("--- Running Node B Host Tests (Step B0, B2, Step 1 & Stage 2) ---\n");
    test_status_strings();
    test_channel_plan();
    test_log_formatter();
    test_hal_null_safety();
    test_sx1278_spi();
    test_step1_framing();
    test_step2_protocol();
    printf("--- All Host Tests Passed ---\n");
    return 0;
}
