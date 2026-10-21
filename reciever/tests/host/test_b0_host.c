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

int main(void) {
    printf("--- Running Node B Host Tests (Step B0, B2, Step 1, Stage 2, Stage 3 & Stage 4) ---\n");
    test_status_strings();
    test_channel_plan();
    test_log_formatter();
    test_hal_null_safety();
    test_sx1278_spi();
    test_step1_framing();
    test_step2_protocol();
    test_step3_state_machine();
    test_step4_spi_diag();
    printf("--- All Host Tests Passed ---\n");
    return 0;
}
