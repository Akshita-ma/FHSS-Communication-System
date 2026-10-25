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

#include "step3_fhss_mock.h"

static void test_step3_state_machine(void) {
    step3_fsm_t fsm;
    step3_fsm_init(&fsm);

    /* 1. Initial State Verification */
    assert(fsm.state == STEP3_STATE_SEARCH);
    assert(fsm.current_ch == 0);
    assert(fsm.current_dwell_ms == 120);
    assert(fsm.have_last == 0);
    assert(fsm.consecutive_missed == 0);
    assert(fsm.rx_pass_count == 0);

    /* 2. Envelope CRC Rejection in SEARCH */
    mock_rx_reset();
    const uint8_t bad_crc_env[15] = {
        0x7E, 0x00, 0x0B,
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x7D, 0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0xFF /* Corrupted envelope CRC */
    };
    mock_rx_push(bad_crc_env, 15);
    fhss_packet_t ack;
    uint8_t tx_ch = 0xFF;
    fhss_packet_t rx_pkt;
    uint16_t avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    step3_rx_outcome_t outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_ERR_ENV_CRC);
    assert(fsm.state == STEP3_STATE_SEARCH);
    assert(fsm.env_err_count == 1);
    assert(fsm.tx_ack_count == 0);

    /* 3. Wrong Channel Rejection in SEARCH (envelope has ch=1 instead of home ch=0) */
    mock_rx_reset();
    uint8_t ch1_env[15] = {
        0x7E, 0x01, 0x0B,
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x7D, 0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0x00
    };
    ch1_env[14] = crc8_smbus_cont(&ch1_env[1], 13, CRC8_SMBUS_INIT);
    mock_rx_push(ch1_env, 15);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_REJECT_WRONG_CH);
    assert(fsm.state == STEP3_STATE_SEARCH);
    assert(fsm.tx_ack_count == 0);

    /* 4. Protocol Tag Rejection in SEARCH */
    mock_rx_reset();
    uint8_t bad_tag_env[15] = {
        0x7E, 0x00, 0x0B,
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x00 /* Bad tag */, 0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0x00
    };
    bad_tag_env[14] = crc8_smbus_cont(&bad_tag_env[1], 13, CRC8_SMBUS_INIT);
    mock_rx_push(bad_tag_env, 15);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_REJECT_TAG);
    assert(fsm.state == STEP3_STATE_SEARCH);
    assert(fsm.tx_ack_count == 0);

    /* 5. Valid HOP Packet in SEARCH -> Transitions to LOCKED */
    mock_rx_reset();
    fhss_packet_t hop_pkt;
    hop_pkt.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_HOP);
    hop_pkt.seq = 200;
    hop_pkt.msg_id = 0;
    hop_pkt.frag = 0;
    uint8_t hop_mask = fhss_generate_mask(200, 0);
    hop_pkt.hop_masked = (uint8_t)(((7 << 5) | 31) ^ hop_mask); /* next_ch=7, code=31 */
    hop_pkt.payload_len = 0;

    uint8_t hop_wire[16];
    uint8_t hop_wire_len = 0;
    assert(fhss_build_packet(&hop_pkt, hop_wire, &hop_wire_len) == OK);
    assert(hop_wire_len == 6);
    assert(hop_wire[4] == 0xA6);
    assert(hop_wire[5] == 0x82); /* Matches frozen vector from CONTEXT.md line 129 */

    uint8_t hop_env[16];
    hop_env[0] = 0x7E;
    hop_env[1] = 0x00;
    hop_env[2] = hop_wire_len;
    memcpy(&hop_env[3], hop_wire, hop_wire_len);
    hop_env[3 + hop_wire_len] = crc8_smbus_cont(&hop_env[1], (uint8_t)(2 + hop_wire_len), CRC8_SMBUS_INIT);
    mock_rx_push(hop_env, (uint16_t)(4 + hop_wire_len));

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_PASS_LOCKED);
    assert(fsm.state == STEP3_STATE_LOCKED);
    assert(fsm.current_ch == 0);
    assert(fsm.current_dwell_ms == 120);
    assert(fsm.next_ch == 7);
    assert(fsm.next_dwell_ms == 235);
    assert(fsm.tx_ack_count == 1);
    assert(ack.ver_type == (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK));
    assert(ack.msg_id == 0);
    assert(tx_ch == 0);

    /* 6. Replay Rule Check in LOCKED (diff == 0) */
    mock_rx_push(hop_env, (uint16_t)(4 + hop_wire_len));
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_REJECT_REPLAY);
    assert(fsm.state == STEP3_STATE_LOCKED);
    assert(fsm.replay_count == 1);

    /* 7. Hop Timer Fires -> Advances to Channel 7 */
    step3_timer_event_t evt = step3_fsm_on_hop_timer(&fsm);
    assert(evt == STEP3_TIMER_EVENT_HOP);
    assert(fsm.current_ch == 7);
    assert(fsm.current_dwell_ms == 235);
    assert(fsm.got_packet_this_hop == 0);

    /* 8. Three Missed Hops on Channel 7 -> Transitions to LOST -> SEARCH */
    evt = step3_fsm_on_hop_timer(&fsm);
    assert(evt == STEP3_TIMER_EVENT_HOP);
    assert(fsm.consecutive_missed == 1);

    evt = step3_fsm_on_hop_timer(&fsm);
    assert(evt == STEP3_TIMER_EVENT_HOP);
    assert(fsm.consecutive_missed == 2);

    evt = step3_fsm_on_hop_timer(&fsm);
    assert(evt == STEP3_TIMER_EVENT_LOST);
    assert(fsm.state == STEP3_STATE_SEARCH);
    assert(fsm.current_ch == 0);
    assert(fsm.current_dwell_ms == 120);
    assert(fsm.have_last == 0);

    /* 9. Valid Stage 2 DATA Vector in SEARCH -> Transitions to LOCKED */
    mock_rx_reset();
    const uint8_t valid_stage2_env[15] = {
        0x7E, 0x00, 0x0B,
        0xA1, 0x05, 0x03, 0xE8, 0x6A, 0x7D, 0x48, 0x45, 0x4C, 0x4C, 0x4F,
        0x18
    };
    mock_rx_push(valid_stage2_env, 15);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_PASS_LOCKED);
    assert(fsm.state == STEP3_STATE_LOCKED);
    assert(fsm.next_ch == 5);
    assert(fsm.next_dwell_ms == 165);
    assert(ack.msg_id == 3);
    assert(ack.frag == 0xE0);
    assert(ack.payload[0] == 0x04); /* Bit 2 set for fragment idx 2 */

    /* 10. Advance Hop to Channel 5 */
    evt = step3_fsm_on_hop_timer(&fsm);
    assert(evt == STEP3_TIMER_EVENT_HOP);
    assert(fsm.current_ch == 5);
    assert(fsm.current_dwell_ms == 165);

    /* 11. Receive First DATA Frame on Channel 5 */
    mock_rx_reset();
    fhss_packet_t p2;
    p2.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_DATA);
    p2.seq = 6;
    p2.msg_id = 3;
    p2.frag = (7 << 5) | (3 << 2); /* TOTAL=8, IDX=3 */
    p2.hop_masked = (uint8_t)(((2 << 5) | 10) ^ fhss_generate_mask(6, 0)); /* next_ch=2, code=10 */
    p2.payload[0] = 'W'; p2.payload[1] = 'O'; p2.payload[2] = 'R'; p2.payload[3] = 'L'; p2.payload[4] = 'D';
    p2.payload_len = 5;

    uint8_t wire2[16];
    uint8_t len2 = 0;
    assert(fhss_build_packet(&p2, wire2, &len2) == OK);

    uint8_t env2[32];
    env2[0] = 0x7E;
    env2[1] = 5; /* Channel 5 */
    env2[2] = len2;
    memcpy(&env2[3], wire2, len2);
    env2[3 + len2] = crc8_smbus_cont(&env2[1], (uint8_t)(2 + len2), CRC8_SMBUS_INIT);
    mock_rx_push(env2, (uint16_t)(4 + len2));

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_PASS_CONTINUED);
    assert(fsm.state == STEP3_STATE_LOCKED);
    assert(fsm.is_first_of_hop == 1); /* First packet of this hop */
    assert(fsm.current_ch == 5);
    assert(fsm.next_ch == 2);
    assert(fsm.next_dwell_ms == (80 + 10 * 5)); /* 130 ms */
    assert(tx_ch == 5);
    assert(ack.msg_id == 3);
    assert(ack.payload[0] == 0x08); /* Bit 3 set for fragment idx 3 */

    /* 12. Receive Second DATA Frame on Channel 5 in the Same Hop */
    mock_rx_reset();
    fhss_packet_t p3;
    p3.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_DATA);
    p3.seq = 7;
    p3.msg_id = 3;
    p3.frag = (7 << 5) | (4 << 2); /* TOTAL=8, IDX=4 */
    p3.hop_masked = (uint8_t)(((2 << 5) | 10) ^ fhss_generate_mask(7, 0));
    p3.payload[0] = 'X';
    p3.payload_len = 1;

    uint8_t wire3[16];
    uint8_t len3 = 0;
    assert(fhss_build_packet(&p3, wire3, &len3) == OK);

    uint8_t env3[32];
    env3[0] = 0x7E;
    env3[1] = 5; /* Channel 5 */
    env3[2] = len3;
    memcpy(&env3[3], wire3, len3);
    env3[3 + len3] = crc8_smbus_cont(&env3[1], (uint8_t)(2 + len3), CRC8_SMBUS_INIT);
    mock_rx_push(env3, (uint16_t)(4 + len3));

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step3_fsm_process_incoming(
        &fsm, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP3_RX_PASS_CONTINUED);
    assert(fsm.state == STEP3_STATE_LOCKED);
    assert(fsm.is_first_of_hop == 0); /* Second packet must NOT be marked as first of hop! */
    assert(fsm.current_ch == 5);
    assert(tx_ch == 5);
    assert(ack.payload[0] == 0x10); /* Bit 4 set */

    printf("[HOST_TEST] test_step3_state_machine: PASS\n");
}

#include "step4_radio_spi_diag.h"

static uint8_t s_mock_reg_ver_val = 0x12;
static status_t s_mock_reg_ver_status = OK;
static uint8_t s_mock_reg_ver_call_count = 0;
static uint8_t s_mock_reg_ver_glitch_at = 0xFF;
static uint8_t s_mock_reg_ver_glitch_val = 0x00;

static status_t mock_read_reg_custom(uint8_t addr, uint8_t *val) {
    (void)addr;
    if (val == NULL) return ERR_NULL;
    if (s_mock_reg_ver_call_count == s_mock_reg_ver_glitch_at) {
        *val = s_mock_reg_ver_glitch_val;
    } else {
        *val = s_mock_reg_ver_val;
    }
    s_mock_reg_ver_call_count++;
    return s_mock_reg_ver_status;
}

static void test_step4_spi_diag(void) {
    step4_diag_result_t res;

    /* 1. NULL safety */
    step4_run_diagnostic(NULL, mock_read_reg_custom);
    step4_run_diagnostic(&res, NULL);

    /* 2. All 32 reads passing (0x12) */
    s_mock_reg_ver_val = 0x12;
    s_mock_reg_ver_status = OK;
    s_mock_reg_ver_call_count = 0;
    s_mock_reg_ver_glitch_at = 0xFF;
    step4_run_diagnostic(&res, mock_read_reg_custom);
    assert(res.total_reads == STEP4_READ_COUNT);
    assert(res.pass_count == STEP4_READ_COUNT);
    assert(res.fail_count == 0);
    assert(res.first_unexpected_valid == 0);
    assert(res.all_same == 1);
    for (int i = 0; i < STEP4_READ_COUNT; i++) {
        assert(res.reads[i] == 0x12);
        assert(res.statuses[i] == OK);
    }

    /* 3. All 32 reads returning 0x00 (e.g. MISO low / reset / unpowered) */
    s_mock_reg_ver_val = 0x00;
    s_mock_reg_ver_status = OK;
    s_mock_reg_ver_call_count = 0;
    s_mock_reg_ver_glitch_at = 0xFF;
    step4_run_diagnostic(&res, mock_read_reg_custom);
    assert(res.total_reads == STEP4_READ_COUNT);
    assert(res.pass_count == 0);
    assert(res.fail_count == STEP4_READ_COUNT);
    assert(res.first_unexpected == 0x00);
    assert(res.first_unexpected_valid == 1);
    assert(res.all_same == 1);

    /* 4. All 32 reads returning 0xFF (e.g. floating / pull-up) */
    s_mock_reg_ver_val = 0xFF;
    s_mock_reg_ver_status = OK;
    s_mock_reg_ver_call_count = 0;
    s_mock_reg_ver_glitch_at = 0xFF;
    step4_run_diagnostic(&res, mock_read_reg_custom);
    assert(res.total_reads == STEP4_READ_COUNT);
    assert(res.pass_count == 0);
    assert(res.fail_count == STEP4_READ_COUNT);
    assert(res.first_unexpected == 0xFF);
    assert(res.first_unexpected_valid == 1);
    assert(res.all_same == 1);

    /* 5. Inconsistent reads (glitch at read 5) */
    s_mock_reg_ver_val = 0x12;
    s_mock_reg_ver_status = OK;
    s_mock_reg_ver_call_count = 0;
    s_mock_reg_ver_glitch_at = 5;
    s_mock_reg_ver_glitch_val = 0x00;
    step4_run_diagnostic(&res, mock_read_reg_custom);
    assert(res.total_reads == STEP4_READ_COUNT);
    assert(res.pass_count == STEP4_READ_COUNT - 1);
    assert(res.fail_count == 1);
    assert(res.first_unexpected == 0x00);
    assert(res.first_unexpected_valid == 1);
    assert(res.all_same == 0);
    assert(res.reads[5] == 0x00);
    assert(res.reads[0] == 0x12);

    /* 6. SPI timeout failure */
    s_mock_reg_ver_val = 0x00;
    s_mock_reg_ver_status = ERR_TIMEOUT;
    s_mock_reg_ver_call_count = 0;
    s_mock_reg_ver_glitch_at = 0xFF;
    step4_run_diagnostic(&res, mock_read_reg_custom);
    assert(res.total_reads == STEP4_READ_COUNT);
    assert(res.pass_count == 0);
    assert(res.fail_count == STEP4_READ_COUNT);
    assert(res.first_unexpected == 0xEE);
    assert(res.first_unexpected_valid == 1);
    assert(res.all_same == 1);

    printf("[HOST_TEST] test_step4_spi_diag: PASS\n");
}

#include "step5_fault_test.h"

static void make_test_envelope(
    uint8_t ch,
    const fhss_packet_t *pkt,
    uint8_t corrupt_env_crc,
    uint8_t corrupt_tag,
    uint8_t *env_out,
    uint16_t *env_len_out
) {
    uint8_t wire[32];
    uint8_t wire_len = 0;
    assert(fhss_build_packet(pkt, wire, &wire_len) == OK);
    if (corrupt_tag) {
        wire[5] ^= 0xA5; /* Corrupt byte 5 (TAG) */
    }
    env_out[0] = 0x7E;
    env_out[1] = ch;
    env_out[2] = wire_len;
    memcpy(&env_out[3], wire, wire_len);
    uint8_t crc = crc8_smbus_cont(&env_out[1], (uint8_t)(2 + wire_len), CRC8_SMBUS_INIT);
    if (corrupt_env_crc) {
        crc ^= 0x55;
    }
    env_out[3 + wire_len] = crc;
    *env_len_out = (uint16_t)(4 + wire_len);
}

static void test_step5_fault_test(void) {
    step5_harness_t harness;
    step5_harness_init(&harness);
    assert(harness.suppress_ack_seq == 7); /* Node A schedule: seq 7 has ACK suppressed */
    assert(harness.cycle_done == 0);

    fhss_packet_t ack;
    uint8_t tx_ch = 0;
    fhss_packet_t rx_pkt;
    uint8_t env[48];
    uint16_t env_len = 0;
    step5_rx_outcome_t outcome;
    uint16_t avail = 0;

    /* Base test packet template (DATA) */
    fhss_packet_t p;
    p.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_DATA);
    p.msg_id = 1;
    p.frag = 0; /* TOTAL=1, IDX=0, LAST=0 */
    p.payload[0] = 'T';
    p.payload_len = 1;

    /* =========================================================================
     * Case 1: Valid channel-0 packets with sequence values 254, 255, 0, 1.
     * Must be accepted under replay-window rules including wraparound.
     * ========================================================================= */
    const uint8_t c1_seqs[4] = {254, 255, 0, 1};
    for (int i = 0; i < 4; i++) {
        mock_rx_reset();
        p.seq = c1_seqs[i];
        p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
        make_test_envelope(0, &p, 0, 0, env, &env_len);
        mock_rx_push(env, env_len);

        avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
        outcome = step5_harness_process_incoming(
            &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
        );
        assert(outcome == STEP5_RX_PASS_ACK_SENT);
        assert(harness.have_last == 1);
        assert(harness.last_seq == c1_seqs[i]);
        assert(tx_ch == 0);
        assert(ack.ver_type == (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK));
        assert(ack.msg_id == 1);
    }
    assert(harness.rx_accept_count == 4);
    assert(harness.tx_ack_count == 4);

    /* =========================================================================
     * Case 2: Sequence 2 with deliberately corrupted outer envelope CRC.
     * Reject it, do NOT transmit ACK, do NOT advance last_seq.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 2;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 1 /* corrupt env crc */, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_ERR_ENV_CRC);
    assert(harness.last_seq == 1); /* Retained previous sequence */
    assert(harness.last_was_fault == 1);
    assert(harness.env_crc_err_count == 1);
    assert(harness.tx_ack_count == 4); /* No ACK sent */

    /* =========================================================================
     * Case 3: Sequence 3 with invalid protocol TAG with valid envelope CRC.
     * Reject it, do NOT transmit ACK, do NOT advance last_seq.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 3;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 0, 1 /* corrupt tag */, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_REJECT_BAD_TAG);
    assert(harness.last_seq == 1);
    assert(harness.last_was_fault == 1);
    assert(harness.proto_bad_tag_count == 1);
    assert(harness.tx_ack_count == 4);

    /* =========================================================================
     * Case 4: Sequence 3 again: corrected valid frame.
     * Accept and ACK it, verify post-fault recovery.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 3;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_PASS_ACK_SENT);
    assert(harness.last_seq == 3);
    assert(harness.rx_accept_count == 5);
    assert(harness.tx_ack_count == 5);

    /* =========================================================================
     * Case 5: Sequence 4 twice: first accepted and ACKed; duplicate rejected without ACK.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 4;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    /* First copy */
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_PASS_ACK_SENT);
    assert(harness.last_seq == 4);
    assert(harness.rx_accept_count == 6);
    assert(harness.tx_ack_count == 6);

    /* Duplicate copy (same seq=4) */
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_REJECT_REPLAY);
    assert(harness.last_seq == 4);
    assert(harness.last_rejected_seq == 4);
    assert(harness.proto_replay_count == 1);
    assert(harness.tx_ack_count == 6); /* No new ACK */

    /* =========================================================================
     * Case 6: Sequence 5 on envelope channel 1: reject without ACK.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 5;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(1 /* channel 1 */, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_REJECT_WRONG_CH);
    assert(harness.last_seq == 4); /* Seq 5 rejected, so last_seq remains 4 */
    assert(harness.last_rejected_ch == 1);
    assert(harness.proto_wrong_ch_count == 1);
    assert(harness.tx_ack_count == 6); /* No ACK */

    /* =========================================================================
     * Case 7: Sequence 6 on envelope channel 0: accept and ACK.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 6;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0 /* channel 0 */, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_PASS_ACK_SENT);
    assert(harness.last_seq == 6);
    assert(harness.rx_accept_count == 7);
    assert(harness.tx_ack_count == 7);

    /* =========================================================================
     * Case 8: Sequence 7 on envelope channel 0: validate and accept, but deliberately suppress ACK.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 7;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_PASS_ACK_SUPPRESSED);
    assert(harness.last_seq == 7); /* History MUST be updated */
    assert(harness.rx_accept_count == 8);
    assert(harness.ack_suppressed_count == 1);
    assert(harness.tx_ack_count == 7); /* Suppressed: tx_ack_count stays 7 */
    assert(harness.suppress_ack_enabled == 0); /* One-shot disarmed */
    assert(harness.cycle_done == 0);

    /* =========================================================================
     * Case 9: Sequence 8 on envelope channel 0: accept and ACK normally.
     * Marks one-shot test cycle completion!
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 8;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_PASS_ACK_SENT);
    assert(harness.last_seq == 8);
    assert(harness.rx_accept_count == 9);
    assert(harness.tx_ack_count == 8); /* Normal transmission resumed */
    assert(harness.rx_total_count == 13);
    assert(harness.cycle_done == 1); /* One-shot cycle complete! */

    /* =========================================================================
     * Verify One-Shot Idle Behavior:
     * When cycle is complete, incoming packets (e.g. Node A restarting at seq 254)
     * are safely ignored without stale replay rejections or ACKs.
     * ========================================================================= */
    mock_rx_reset();
    p.seq = 254;
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(p.seq, 0));
    make_test_envelope(0, &p, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step5_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP5_RX_IDLE);
    assert(harness.last_seq == 8); /* Stays 8 */
    assert(harness.tx_ack_count == 8); /* No ACK sent */
    assert(harness.proto_replay_count == 1); /* No false replay error incremented! */

    printf("[HOST_TEST] test_step5_fault_test: PASS\n");
}

#include "step6_data_mock.h"
#include "step7_arq_mock.h"

static void make_test_data_envelope(
    uint8_t ch,
    uint8_t seq,
    uint8_t msg_id,
    uint8_t total,
    uint8_t idx,
    uint8_t last,
    const uint8_t *payload,
    uint8_t payload_len,
    uint8_t corrupt_env_crc,
    uint8_t corrupt_tag,
    uint8_t *env_out,
    uint16_t *env_len_out
) {
    fhss_packet_t p;
    p.ver_type = (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_DATA);
    p.seq = seq;
    p.msg_id = msg_id;
    p.frag = (uint8_t)(((total - 1) << 5) | (idx << 2) | (last << 1));
    p.hop_masked = (uint8_t)(0x00 ^ fhss_generate_mask(seq, 0));
    p.payload_len = payload_len;
    if (payload != NULL && payload_len > 0) {
        memcpy(p.payload, payload, payload_len);
    }
    uint8_t wire[32];
    uint8_t wire_len = 0;
    assert(fhss_build_packet(&p, wire, &wire_len) == OK);
    if (corrupt_tag) {
        wire[5] ^= 0xA5;
    }
    env_out[0] = 0x7E;
    env_out[1] = ch;
    env_out[2] = wire_len;
    memcpy(&env_out[3], wire, wire_len);
    uint8_t crc = crc8_smbus_cont(&env_out[1], (uint8_t)(2 + wire_len), CRC8_SMBUS_INIT);
    if (corrupt_env_crc) {
        crc ^= 0x55;
    }
    env_out[3 + wire_len] = crc;
    *env_len_out = (uint16_t)(4 + wire_len);
}

static void test_step6_data_mock(void) {
    step6_harness_t harness;
    step6_harness_init(&harness);
    assert(harness.target_messages == 2);
    assert(harness.completed_messages == 0);
    assert(harness.cycle_done == 0);

    fhss_packet_t ack;
    uint8_t tx_ch = 0;
    fhss_packet_t rx_pkt;
    uint8_t env[48];
    uint16_t env_len = 0;
    step6_rx_outcome_t outcome;
    uint16_t avail = 0;

    /* =========================================================================
     * 1. Message 1: Single-fragment ASCII message (TOTAL=1, IDX=0, LAST=1)
     * ========================================================================= */
    const char msg1_text[] = "PING_STAGE6";
    uint8_t msg1_len = (uint8_t)strlen(msg1_text);
    mock_rx_reset();
    make_test_data_envelope(
        0, 1 /* seq */, 1 /* msg_id */, 1 /* total */, 0 /* idx */, 1 /* last */,
        (const uint8_t *)msg1_text, msg1_len, 0, 0, env, &env_len
    );
    mock_rx_push(env, env_len);

    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(harness.completed_messages == 1);
    assert(harness.last_completed.msg_id == 1);
    assert(harness.last_completed.total_len == msg1_len);
    assert(strcmp((char *)harness.last_completed.data, msg1_text) == 0);
    assert(ack.ver_type == (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK));
    assert(ack.msg_id == 1);
    assert(ack.frag == 0x00); /* (1 - 1) << 5 = 0 */
    assert(ack.payload[0] == 0x01); /* Bitmap: bit 0 */
    assert(harness.cycle_done == 0);

    /* =========================================================================
     * 2. Message 2: Multi-fragment ASCII message (TOTAL=3, 40 bytes)
     * Frag 0: "0123456789ABCDEF" (16B, idx=0, last=0)
     * Frag 1: "GHIJKLMNOPQRSTUV" (16B, idx=1, last=0)
     * Frag 2: "WXYZ1234"         (8B,  idx=2, last=1)
     * Reconstructed: "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ1234" (40 bytes)
     * ========================================================================= */
    const char f0_data[] = "0123456789ABCDEF";
    const char f1_data[] = "GHIJKLMNOPQRSTUV";
    const char f2_data[] = "WXYZ1234";

    /* Frag 0 */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 2, 3, 0, 0, (const uint8_t *)f0_data, 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x01); /* Bitmap: bit 0 */
    assert(harness.completed_messages == 1);

    /* Frag 1 */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 2, 3, 1, 0, (const uint8_t *)f1_data, 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x03); /* Bitmap: bits 0, 1 */
    assert(harness.completed_messages == 1);

    /* Frag 2 (final fragment) */
    mock_rx_reset();
    make_test_data_envelope(0, 4, 2, 3, 2, 1, (const uint8_t *)f2_data, 8, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(ack.payload[0] == 0x07); /* Bitmap: bits 0, 1, 2 */
    assert(harness.completed_messages == 2);
    assert(harness.last_completed.msg_id == 2);
    assert(harness.last_completed.total_len == 40);
    assert(memcmp(harness.last_completed.data, "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ1234", 40) == 0);
    assert(harness.last_completed.data[40] == '\0');
    assert(harness.cycle_done == 1); /* Both messages complete! */

    /* =========================================================================
     * 3. One-Shot Idle Behavior: After 2 messages complete, subsequent frames are ignored
     * ========================================================================= */
    mock_rx_reset();
    make_test_data_envelope(0, 5, 3, 1, 0, 1, (const uint8_t *)"EXTRA", 5, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(
        &harness, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt
    );
    assert(outcome == STEP6_RX_IDLE);
    assert(harness.completed_messages == 2);

    /* =========================================================================
     * 4. Fragment Ordering and LAST Semantics (Out-of-order reassembly)
     * Frag 1 arrives first, then Frag 2, then Frag 0 last.
     * ========================================================================= */
    step6_harness_t h_ooo;
    step6_harness_init(&h_ooo);

    /* Frag 1 arrives first */
    mock_rx_reset();
    make_test_data_envelope(0, 10, 5, 3, 1, 0, (const uint8_t *)f1_data, 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_ooo, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x02); /* Bit 1 */

    /* Frag 2 arrives next */
    mock_rx_reset();
    make_test_data_envelope(0, 11, 5, 3, 2, 1, (const uint8_t *)f2_data, 8, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_ooo, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x06); /* Bits 1, 2 */

    /* Frag 0 arrives last */
    mock_rx_reset();
    make_test_data_envelope(0, 12, 5, 3, 0, 0, (const uint8_t *)f0_data, 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_ooo, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(ack.payload[0] == 0x07); /* Bits 0, 1, 2 */
    assert(h_ooo.last_completed.total_len == 40);
    assert(memcmp(h_ooo.last_completed.data, "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ1234", 40) == 0);

    /* =========================================================================
     * 5. Duplicate Fragment Handling (Fresh Sequence Number on ARQ Retry)
     * ========================================================================= */
    step6_harness_t h_dup;
    step6_harness_init(&h_dup);

    /* Send Frag 0 */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 7, 2, 0, 0, (const uint8_t *)"FIRST_HALF______", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_dup, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x01);

    /* Retransmit Frag 0 with fresh sequence 2 (e.g. ACK was lost on previous attempt) */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 7, 2, 0, 0, (const uint8_t *)"FIRST_HALF______", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_dup, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_DUPLICATE_FRAG);
    assert(ack.payload[0] == 0x01); /* Responds with cumulative bitmap */
    assert(h_dup.rx_dup_frag_count == 1);

    /* Send Frag 1 */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 7, 2, 1, 1, (const uint8_t *)"SECOND_HALF_____", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_dup, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(ack.payload[0] == 0x03);

    /* =========================================================================
     * 6. Replay Rejection (Same sequence number sent twice)
     * ========================================================================= */
    mock_rx_reset();
    make_test_data_envelope(0, 3 /* same seq as above */, 7, 2, 1, 1, (const uint8_t *)"SECOND_HALF_____", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_dup, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_REJECT_REPLAY);
    assert(h_dup.proto_replay_count == 1);

    /* =========================================================================
     * 7. Invalid Metadata: idx >= total, total changed mid-message
     * ========================================================================= */
    step6_harness_t h_meta;
    step6_harness_init(&h_meta);

    /* idx=2, total=2 (idx must be < total) */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 9, 2, 2, 1, (const uint8_t *)"BAD", 3, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_meta, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_REJECT_METADATA);
    assert(h_meta.proto_metadata_count == 1);

    /* Send valid frag 0 with total=2 */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 9, 2, 0, 0, (const uint8_t *)"VALID_PART_ONE__", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_meta, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_FRAG);

    /* Send frag 1 with total=3 (changed total mid-message) */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 9, 3, 1, 0, (const uint8_t *)"CHANGED_TOTAL___", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_meta, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_REJECT_METADATA);

    /* Recovery: send valid frag 1 with total=2 -> completes message */
    mock_rx_reset();
    make_test_data_envelope(0, 4, 9, 2, 1, 1, (const uint8_t *)"VALID_PART_TWO__", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_meta, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(h_meta.completed_messages == 1);

    /* =========================================================================
     * 8. Exact Physical Node A Test Vectors:
     * Msg 1: "HELLO_NODE_B" (12B, seq=0, id=1, total=1, idx=0, last=1)
     * Msg 2: "FHSS_DATA_TRANSPORT_TEST_PACKET_#2" (34B, id=2, total=3)
     *   Frag 0: "FHSS_DATA_TRANSP" (16B, seq=1, idx=0, last=0)
     *   Frag 1: "ORT_TEST_PACKET_" (16B, seq=2, idx=1, last=0)
     *   Frag 2: "#2"               (2B,  seq=3, idx=2, last=1)
     * ========================================================================= */
    step6_harness_t h_phys;
    step6_harness_init(&h_phys);

    /* Node A Msg 1 */
    mock_rx_reset();
    make_test_data_envelope(0, 0, 1, 1, 0, 1, (const uint8_t *)"HELLO_NODE_B", 12, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_phys, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(h_phys.completed_messages == 1);
    assert(h_phys.last_completed.total_len == 12);
    assert(strcmp((char *)h_phys.last_completed.data, "HELLO_NODE_B") == 0);
    assert(ack.payload[0] == 0x01);

    /* Node A Msg 2 Frag 0 */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 2, 3, 0, 0, (const uint8_t *)"FHSS_DATA_TRANSP", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_phys, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x01);

    /* Node A Msg 2 Frag 1 */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 2, 3, 1, 0, (const uint8_t *)"ORT_TEST_PACKET_", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_phys, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_FRAG);
    assert(ack.payload[0] == 0x03);

    /* Node A Msg 2 Frag 2 */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 2, 3, 2, 1, (const uint8_t *)"#2", 2, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step6_harness_process_incoming(&h_phys, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP6_RX_PASS_MSG_COMPLETE);
    assert(h_phys.completed_messages == 2);
    assert(h_phys.last_completed.total_len == 34);
    assert(strcmp((char *)h_phys.last_completed.data, "FHSS_DATA_TRANSPORT_TEST_PACKET_#2") == 0);
    assert(ack.payload[0] == 0x07);
    assert(h_phys.cycle_done == 1);

    printf("[HOST_TEST] test_step6_data_mock: PASS\n");
}

static void test_step7_arq_mock(void) {
    /* =========================================================================
     * 1. Exact Coordinated Test Schedule from Section 3:
     * - Clean boot
     * - DATA seq 0: Msg 1, frag 0/1 ("HELLO_NODE_B", 12B) -> Accept, reassemble, ACK bitmap 0x01
     * - DATA seq 1: Msg 2, frag 0/3 ("FHSS_DATA_TRANSP", 16B) -> Accept, ACK bitmap 0x01
     * - DATA seq 2: Msg 2, frag 1/3 ("ORT_TEST_PACKET_", 16B), 1st try -> Accept, ACK suppressed
     * - DATA seq 3: Msg 2, frag 1/3 ("ORT_TEST_PACKET_", 16B), retry -> Duplicate, ACK bitmap 0x03
     * - DATA seq 4: Msg 2, frag 2/3 ("#2", 2B) -> Accept, ACK bitmap 0x07, complete 34-byte message
     * ========================================================================= */
    step7_harness_t h_sched;
    step7_harness_init(&h_sched);
    assert(h_sched.target_messages == 2);
    assert(h_sched.completed_messages == 0);
    assert(h_sched.cycle_done == 0);
    assert(h_sched.b_seq == 0);

    fhss_packet_t ack;
    uint8_t tx_ch = 0;
    fhss_packet_t rx_pkt;
    uint8_t env[48];
    uint16_t env_len = 0;
    step7_rx_outcome_t outcome;
    uint16_t avail = 0;

    /* Packet 0: DATA seq 0, Msg 1, frag 0/1 ("HELLO_NODE_B", 12 bytes) */
    mock_rx_reset();
    make_test_data_envelope(0, 0, 1, 1, 0, 1, (const uint8_t *)"HELLO_NODE_B", 12, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_sched, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_MSG_COMPLETE);
    assert(h_sched.completed_messages == 1);
    assert(h_sched.last_completed.total_len == 12);
    assert(strcmp((char *)h_sched.last_completed.data, "HELLO_NODE_B") == 0);
    assert(ack.ver_type == (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK));
    assert(ack.seq == 0);
    assert(ack.msg_id == 1);
    assert(ack.payload[0] == 0x01); /* Bitmap: bit 0 */
    assert(h_sched.b_seq == 1);     /* Sequence incremented for sent ACK */
    assert(h_sched.tx_ack_count == 1);
    assert(h_sched.ack_suppressed_count == 0);

    /* Packet 1: DATA seq 1, Msg 2, frag 0/3 ("FHSS_DATA_TRANSP", 16 bytes) */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 2, 3, 0, 0, (const uint8_t *)"FHSS_DATA_TRANSP", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_sched, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG);
    assert(ack.seq == 1);
    assert(ack.msg_id == 2);
    assert(ack.payload[0] == 0x01); /* Bitmap: bit 0 */
    assert(h_sched.b_seq == 2);
    assert(h_sched.tx_ack_count == 2);
    assert(h_sched.ack_suppressed_count == 0);

    /* Packet 2: DATA seq 2, Msg 2, frag 1/3 ("ORT_TEST_PACKET_", 16 bytes), first attempt */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 2, 3, 1, 0, (const uint8_t *)"ORT_TEST_PACKET_", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_sched, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG_ACK_SUPPRESSED);
    assert(h_sched.reasm.rx_bitmap == 0x03); /* Bits 0 and 1 recorded */
    assert(h_sched.ack_suppressed_count == 1);
    assert(h_sched.ack_suppressed_once == 1);
    assert(h_sched.b_seq == 2);        /* Node B sequence NOT incremented for suppressed ACK */
    assert(h_sched.tx_ack_count == 2); /* No ACK transmitted */

    /* Packet 3: DATA seq 3, Msg 2, frag 1/3 ("ORT_TEST_PACKET_", 16 bytes), retransmission */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 2, 3, 1, 0, (const uint8_t *)"ORT_TEST_PACKET_", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_sched, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_DUPLICATE_FRAG);
    assert(h_sched.rx_dup_frag_count == 1);
    /* Retransmission recognized with fresh sequence, payload not corrupted */
    assert(memcmp(&h_sched.reasm.data[16], "ORT_TEST_PACKET_", 16) == 0);
    assert(ack.seq == 2);           /* Contiguous outgoing ACK sequence (2 follows 1) */
    assert(ack.msg_id == 2);
    assert(ack.payload[0] == 0x03); /* Cumulative bitmap */
    assert(h_sched.b_seq == 3);
    assert(h_sched.tx_ack_count == 3);

    /* Packet 4: DATA seq 4, Msg 2, frag 2/3 ("#2", 2 bytes), final fragment */
    mock_rx_reset();
    make_test_data_envelope(0, 4, 2, 3, 2, 1, (const uint8_t *)"#2", 2, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_sched, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_MSG_COMPLETE);
    assert(h_sched.completed_messages == 2);
    assert(h_sched.cycle_done == 1);
    assert(h_sched.last_completed.total_len == 34);
    assert(strcmp((char *)h_sched.last_completed.data, "FHSS_DATA_TRANSPORT_TEST_PACKET_#2") == 0);
    assert(ack.seq == 3);           /* Contiguous outgoing ACK sequence (3 follows 2) */
    assert(ack.msg_id == 2);
    assert(ack.payload[0] == 0x07); /* Cumulative bitmap bits 0, 1, 2 */
    assert(h_sched.b_seq == 4);
    assert(h_sched.tx_ack_count == 4);

    /* Verify all counters for the successful session */
    assert(h_sched.rx_total_count == 5);
    assert(h_sched.rx_frag_count == 4);
    assert(h_sched.rx_dup_frag_count == 1);
    assert(h_sched.tx_ack_count == 4);
    assert(h_sched.ack_suppressed_count == 1);
    assert(h_sched.completed_messages == 2);
    assert(h_sched.proto_replay_count == 0);
    assert(h_sched.proto_bad_tag_count == 0);
    assert(h_sched.env_crc_err_count == 0);
    assert(h_sched.env_len_err_count == 0);
    assert(h_sched.env_ch_err_count == 0);
    assert(h_sched.proto_wrong_ch_count == 0);
    assert(h_sched.proto_non_data_count == 0);
    assert(h_sched.proto_metadata_count == 0);
    assert(h_sched.proto_conflict_count == 0);

    /* Verify idle behavior once session is complete */
    mock_rx_reset();
    make_test_data_envelope(0, 5, 3, 1, 0, 1, (const uint8_t *)"EXTRA", 5, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_sched, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_IDLE);

    /* =========================================================================
     * 2. Conflicting Duplicate Fragment Rejection (Section 4, Rule 8):
     * Repeated logical fragment with conflicting payload or metadata must NOT
     * overwrite already accepted data or be ACKed as an identical duplicate.
     * ========================================================================= */
    step7_harness_t h_conflict;
    step7_harness_init(&h_conflict);

    /* Valid Frag 0 of Msg 5 */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 5, 2, 0, 0, (const uint8_t *)"VALID_PAYLOAD_00", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_conflict, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG);
    assert(h_conflict.reasm.rx_bitmap == 0x01);
    assert(h_conflict.tx_ack_count == 1);
    assert(h_conflict.b_seq == 1);

    /* Duplicate Frag 0 with fresh sequence 2, but conflicting payload bytes */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 5, 2, 0, 0, (const uint8_t *)"CORRUPT_PAYLOAD!", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_conflict, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_REJECT_CONFLICT);
    assert(h_conflict.proto_conflict_count == 1);
    /* Accepted data must NOT be overwritten */
    assert(memcmp(&h_conflict.reasm.data[0], "VALID_PAYLOAD_00", 16) == 0);
    assert(h_conflict.tx_ack_count == 1); /* No ACK sent */
    assert(h_conflict.b_seq == 1);        /* b_seq unchanged */

    /* Duplicate Frag 0 with fresh sequence 3, but conflicting payload length */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 5, 2, 0, 0, (const uint8_t *)"SHORT", 5, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_conflict, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_REJECT_CONFLICT);
    assert(h_conflict.proto_conflict_count == 2);
    assert(memcmp(&h_conflict.reasm.data[0], "VALID_PAYLOAD_00", 16) == 0);
    assert(h_conflict.tx_ack_count == 1);

    /* Duplicate Frag 0 with fresh sequence 4 and exact matching payload */
    mock_rx_reset();
    make_test_data_envelope(0, 4, 5, 2, 0, 0, (const uint8_t *)"VALID_PAYLOAD_00", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_conflict, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_DUPLICATE_FRAG);
    assert(h_conflict.rx_dup_frag_count == 1);
    assert(ack.seq == 1);
    assert(ack.payload[0] == 0x01);
    assert(h_conflict.tx_ack_count == 2);
    assert(h_conflict.b_seq == 2);

    /* Complete message with valid Frag 1 */
    mock_rx_reset();
    make_test_data_envelope(0, 5, 5, 2, 1, 1, (const uint8_t *)"VALID_PART_TWO__", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_conflict, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_MSG_COMPLETE);
    assert(h_conflict.completed_messages == 1);
    assert(h_conflict.last_completed.total_len == 32);
    assert(memcmp(h_conflict.last_completed.data, "VALID_PAYLOAD_00VALID_PART_TWO__", 32) == 0);

    /* =========================================================================
     * 3. Replay Protection Preservation (Section 4, Rule 9):
     * Sequence 3 is recognized as a fresh retry, while actual packet-level replays
     * (diff == 0 or diff > 127) are strictly rejected.
     * ========================================================================= */
    step7_harness_t h_rep;
    step7_harness_init(&h_rep);

    /* Send Frag 0 with seq 10 */
    mock_rx_reset();
    make_test_data_envelope(0, 10, 8, 2, 0, 0, (const uint8_t *)"SAMPLE_DATA_AAAA", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_rep, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG);
    assert(h_rep.last_seq == 10);

    /* Packet-level replay: same sequence 10 sent again -> ERR_REPLAY */
    mock_rx_reset();
    make_test_data_envelope(0, 10, 8, 2, 0, 0, (const uint8_t *)"SAMPLE_DATA_AAAA", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_rep, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_REJECT_REPLAY);
    assert(h_rep.proto_replay_count == 1);

    /* Older sequence 9 -> ERR_REPLAY */
    mock_rx_reset();
    make_test_data_envelope(0, 9, 8, 2, 0, 0, (const uint8_t *)"SAMPLE_DATA_AAAA", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_rep, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_REJECT_REPLAY);
    assert(h_rep.proto_replay_count == 2);

    /* Fresh sequence 11 with same logical fragment -> DUPLICATE_FRAG (NOT replay) */
    mock_rx_reset();
    make_test_data_envelope(0, 11, 8, 2, 0, 0, (const uint8_t *)"SAMPLE_DATA_AAAA", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_rep, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_DUPLICATE_FRAG);
    assert(h_rep.last_seq == 11);
    assert(h_rep.proto_replay_count == 2); /* No replay error counted for fresh sequence! */

    /* Replaying sequence 10 now -> ERR_REPLAY */
    mock_rx_reset();
    make_test_data_envelope(0, 10, 8, 2, 0, 0, (const uint8_t *)"SAMPLE_DATA_AAAA", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_rep, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_REJECT_REPLAY);
    assert(h_rep.proto_replay_count == 3);

    /* =========================================================================
     * 4. One-Shot Suppression Specificity (Section 4, Rule 6):
     * Suppression rule must ONLY trigger on Msg 2 Frag 1, exactly once.
     * ========================================================================= */
    step7_harness_t h_spec;
    step7_harness_init(&h_spec);

    /* Msg 3, Frag 1/3: Not Msg ID 2 -> ACK NOT suppressed */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 3, 3, 1, 0, (const uint8_t *)"PART_ONE________", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_spec, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG);
    assert(h_spec.ack_suppressed_count == 0);
    assert(h_spec.tx_ack_count == 1);

    /* Msg 2, Frag 0/3: Frag index 0 -> ACK NOT suppressed */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 2, 3, 0, 0, (const uint8_t *)"PART_ZERO_______", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_spec, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG);
    assert(h_spec.ack_suppressed_count == 0);
    assert(h_spec.tx_ack_count == 2);

    /* Msg 2, Frag 1/3: Frag index 1 -> ACK SUPPRESSED! */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 2, 3, 1, 0, (const uint8_t *)"PART_ONE________", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_spec, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_PASS_FRAG_ACK_SUPPRESSED);
    assert(h_spec.ack_suppressed_count == 1);
    assert(h_spec.ack_suppressed_once == 1);
    assert(h_spec.tx_ack_count == 2);

    /* Msg 2, Frag 1/3 retry: Duplicate recognized -> ACK NOT suppressed! */
    mock_rx_reset();
    make_test_data_envelope(0, 4, 2, 3, 1, 0, (const uint8_t *)"PART_ONE________", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = step7_harness_process_incoming(&h_spec, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == STEP7_RX_DUPLICATE_FRAG);
    assert(h_spec.ack_suppressed_count == 1); /* Still 1 */
    assert(h_spec.tx_ack_count == 3);

    printf("[HOST_TEST] test_step7_arq_mock: PASS\n");
}

#include "custom_rx.h"

static void test_custom_rx(void) {
    custom_rx_harness_t h;
    fhss_packet_t ack;
    uint8_t tx_ch = 0;
    fhss_packet_t rx_pkt;
    uint8_t env[48];
    uint16_t env_len = 0;
    custom_rx_outcome_t outcome;
    uint16_t avail = 0;

    /* 1. Single-fragment message reception */
    custom_rx_harness_init(&h);
    mock_rx_reset();
    make_test_data_envelope(0, 0, 1, 1, 0, 1, (const uint8_t *)"Hello Node B!", 13, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_MSG_COMPLETE);
    assert(h.completed_messages == 1);
    assert(h.has_completed_msg == 1);
    assert(h.last_completed.total_len == 13);
    assert(strcmp((char *)h.last_completed.data, "Hello Node B!") == 0);
    assert(ack.ver_type == (uint8_t)(PKT_VER_PREFIX | PKT_TYPE_ACK));
    assert(ack.seq == 0);
    assert(ack.msg_id == 1);
    assert(ack.payload[0] == 0x01);
    assert(h.b_seq == 1);
    assert(h.tx_ack_count == 1);

    /* 2. Multi-fragment message reconstruction (3 fragments) */
    /* Frag 0: seq=1, msg_id=2, total=3, idx=0, last=0 */
    mock_rx_reset();
    make_test_data_envelope(0, 1, 2, 3, 0, 0, (const uint8_t *)"FirstFragment16B", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_FRAG);
    assert(ack.seq == 1);
    assert(ack.msg_id == 2);
    assert(ack.payload[0] == 0x01);
    assert(h.b_seq == 2);

    /* Frag 1: seq=2, msg_id=2, total=3, idx=1, last=0 */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 2, 3, 1, 0, (const uint8_t *)"SecondFragmt16B!", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_FRAG);
    assert(ack.seq == 2);
    assert(ack.msg_id == 2);
    assert(ack.payload[0] == 0x03);
    assert(h.b_seq == 3);

    /* Frag 2: seq=3, msg_id=2, total=3, idx=2, last=1 */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 2, 3, 2, 1, (const uint8_t *)"Done!", 5, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_MSG_COMPLETE);
    assert(h.completed_messages == 2);
    assert(h.last_completed.total_len == 37);
    assert(strcmp((char *)h.last_completed.data, "FirstFragment16BSecondFragmt16B!Done!") == 0);
    assert(ack.seq == 3);
    assert(ack.msg_id == 2);
    assert(ack.payload[0] == 0x07);
    assert(h.b_seq == 4);

    /* 3. Maximum supported message length (exact 128 bytes, 8 fragments) */
    uint8_t exp_128[129];
    for (int i = 0; i < 128; i++) {
        exp_128[i] = (uint8_t)('A' + (i % 26));
    }
    exp_128[128] = '\0';

    for (uint8_t f = 0; f < 8; f++) {
        mock_rx_reset();
        uint8_t is_last_f = (f == 7) ? 1 : 0;
        make_test_data_envelope(0, (uint8_t)(4 + f), 3, 8, f, is_last_f, &exp_128[f * 16], 16, 0, 0, env, &env_len);
        mock_rx_push(env, env_len);
        avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
        outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
        if (f < 7) {
            assert(outcome == CUSTOM_RX_PASS_FRAG);
            assert(ack.payload[0] == (uint8_t)((1 << (f + 1)) - 1));
        } else {
            assert(outcome == CUSTOM_RX_PASS_MSG_COMPLETE);
            assert(ack.payload[0] == 0xFF);
        }
    }
    assert(h.completed_messages == 3);
    assert(h.last_completed.total_len == 128);
    assert(memcmp(h.last_completed.data, exp_128, 128) == 0);

    /* 4. Identical duplicate fragment handling (during active reassembly) */
    /* Start Msg 4, frag 0 */
    mock_rx_reset();
    make_test_data_envelope(0, 15, 4, 2, 0, 0, (const uint8_t *)"0123456789ABCDEF", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_FRAG);
    assert(h.reasm.rx_bitmap == 0x01);
    uint32_t acks_before = h.tx_ack_count;

    /* Duplicate of frag 0 with fresh sequence 16 */
    mock_rx_reset();
    make_test_data_envelope(0, 16, 4, 2, 0, 0, (const uint8_t *)"0123456789ABCDEF", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_DUPLICATE_FRAG);
    assert(h.rx_dup_frag_count == 1);
    assert(h.reasm.rx_bitmap == 0x01); /* bitmap unchanged */
    assert(h.tx_ack_count == acks_before + 1);
    assert(ack.payload[0] == 0x01); /* cumulative ACK returned */

    /* 5. Conflicting duplicate rejection */
    /* Repeated Frag 0 with fresh sequence 17, but conflicting payload */
    mock_rx_reset();
    make_test_data_envelope(0, 17, 4, 2, 0, 0, (const uint8_t *)"CONFLICT_PAYLOAD", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_CONFLICT);
    assert(h.proto_conflict_count == 1);
    /* Stored data must NOT be corrupted */
    assert(memcmp(&h.reasm.data[0], "0123456789ABCDEF", 16) == 0);

    /* Complete Msg 4 with valid frag 1 */
    mock_rx_reset();
    make_test_data_envelope(0, 18, 4, 2, 1, 1, (const uint8_t *)"FINAL_PART_TEST!", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_MSG_COMPLETE);
    assert(h.completed_messages == 4);
    assert(strcmp((char *)h.last_completed.data, "0123456789ABCDEFFINAL_PART_TEST!") == 0);

    /* 6. Duplicate of already completed message */
    /* Node A lost final ACK of Msg 4, retransmits frag 1 with fresh seq 19 */
    mock_rx_reset();
    make_test_data_envelope(0, 19, 4, 2, 1, 1, (const uint8_t *)"FINAL_PART_TEST!", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_DUPLICATE_FRAG);
    assert(ack.payload[0] == 0x03); /* full completed bitmap */
    assert(h.completed_messages == 4); /* count not incremented */

    /* 7. Replay rejection */
    /* Same sequence 19 replayed */
    mock_rx_reset();
    make_test_data_envelope(0, 19, 5, 1, 0, 1, (const uint8_t *)"REPLAY", 6, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_REPLAY);
    assert(h.proto_replay_count == 1);

    /* Older sequence 10 */
    mock_rx_reset();
    make_test_data_envelope(0, 10, 5, 1, 0, 1, (const uint8_t *)"OLD_SEQ", 7, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_REPLAY);
    assert(h.proto_replay_count == 2);

    /* 8. Sequence wraparound (e.g. from seq 255 to 0) */
    h.last_seq = 255;
    mock_rx_reset();
    make_test_data_envelope(0, 0, 6, 1, 0, 1, (const uint8_t *)"WRAP", 4, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_MSG_COMPLETE);
    assert(h.last_seq == 0);

    /* 9. Wrong-channel rejection */
    mock_rx_reset();
    make_test_data_envelope(2 /* wrong channel */, 1, 7, 1, 0, 1, (const uint8_t *)"WRONG_CH", 8, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_WRONG_CH);
    assert(h.proto_wrong_ch_count == 1);

    /* 10. Corrupted envelope rejection */
    /* Bad outer CRC */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 7, 1, 0, 1, (const uint8_t *)"BAD_CRC", 7, 1, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_ERR_ENV_CRC);

    /* Bad tag CRC */
    mock_rx_reset();
    make_test_data_envelope(0, 2, 7, 1, 0, 1, (const uint8_t *)"BAD_TAG", 7, 0, 1, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_BAD_TAG);

    /* 11. Metadata violations:
     * - idx >= total
     * - non-final fragment len < 16
     * - total changed mid-message
     */
    /* idx >= total (idx=2, total=2) */
    mock_rx_reset();
    make_test_data_envelope(0, 3, 8, 2, 2, 1, (const uint8_t *)"BAD_IDX", 7, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_METADATA);

    /* Non-final fragment with len < 16 (e.g. 10B on frag 0/2) */
    mock_rx_reset();
    make_test_data_envelope(0, 4, 8, 2, 0, 0, (const uint8_t *)"SHORT_PART", 10, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_METADATA);

    /* Total changed mid-message */
    mock_rx_reset();
    make_test_data_envelope(0, 5, 8, 2, 0, 0, (const uint8_t *)"VALID_PART_ONE__", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_PASS_FRAG);

    mock_rx_reset();
    make_test_data_envelope(0, 6, 8, 3 /* changed to 3! */, 1, 0, (const uint8_t *)"VALID_PART_TWO__", 16, 0, 0, env, &env_len);
    mock_rx_push(env, env_len);
    avail = (uint16_t)((s_mock_rx_head - s_mock_rx_tail) & 255);
    outcome = custom_rx_process_incoming(&h, avail, mock_rx_peek, mock_rx_advance, &ack, &tx_ch, &rx_pkt);
    assert(outcome == CUSTOM_RX_REJECT_METADATA);

    printf("[HOST_TEST] test_custom_rx: PASS\n");
}

int main(void) {
    printf("--- Running Node B Host Tests (Step B0, B2, Step 1, Stage 2, Stage 3, Stage 4, Stage 5, Stage 6 & Stage 7) ---\n");
    test_status_strings();
    test_channel_plan();
    test_log_formatter();
    test_hal_null_safety();
    test_sx1278_spi();
    test_step1_framing();
    test_step2_protocol();
    test_step3_state_machine();
    test_step4_spi_diag();
    test_step5_fault_test();
    test_step6_data_mock();
    test_step7_arq_mock();
    test_custom_rx();
    printf("--- All Host Tests Passed ---\n");
    return 0;
}
