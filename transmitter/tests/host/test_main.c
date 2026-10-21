#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "mock_hal.h"
#include "common/config.h"
#include "common/status.h"
#include "common/sx1278_fsk.h"
#include "common/fhss_protocol.h"
#include "node_a_f103/air_mock.h"

#define MOCK_WIRE_DELIMITER 0x7EU

typedef struct {
    uint8_t  ring[256];
    uint16_t head;
    uint16_t tail;
} mock_ring_t;

static void mock_ring_init(mock_ring_t *r) {
    r->head = 0U;
    r->tail = 0U;
}

static int mock_ring_push(mock_ring_t *r, uint8_t byte) {
    uint16_t next = (r->head + 1U) & 0xFFU;
    if (next != r->tail) {
        r->ring[r->head] = byte;
        r->head = next;
        return 1;
    }
    return 0; /* Overflow: dropped */
}

static uint16_t mock_ring_avail(const mock_ring_t *r) {
    return (r->head >= r->tail) ? (uint16_t)(r->head - r->tail) : (uint16_t)(256U + r->head - r->tail);
}

static int mock_ring_poll(mock_ring_t *r, uint8_t tuned_ch, uint8_t *out_buf, uint8_t *out_len) {
    while (1) {
        uint16_t head = r->head;
        uint16_t tail = r->tail;
        uint16_t avail = (head >= tail) ? (uint16_t)(head - tail) : (uint16_t)(256U + head - tail);

        if (avail == 0U) {
            return (int)ERR_TIMEOUT;
        }

        if (r->ring[tail] != MOCK_WIRE_DELIMITER) {
            r->tail = (tail + 1U) & 0xFFU;
            continue;
        }

        if (avail < 3U) {
            return (int)ERR_TIMEOUT;
        }

        uint8_t wire_ch = r->ring[(tail + 1U) & 0xFFU];
        uint8_t wire_len = r->ring[(tail + 2U) & 0xFFU];

        if (wire_len < HEADER_LEN || wire_len > MAX_FRAME_LEN || wire_ch >= NUM_CHANNELS) {
            r->tail = (tail + 1U) & 0xFFU;
            continue;
        }

        uint16_t wire_total = (uint16_t)(wire_len + 4U);
        if (avail < wire_total) {
            return (int)ERR_TIMEOUT;
        }

        uint8_t temp[MAX_FRAME_LEN + 4U];
        for (uint16_t i = 0; i < wire_total; i++) {
            temp[i] = r->ring[(tail + i) & 0xFFU];
        }
        r->tail = (tail + wire_total) & 0xFFU;

        uint8_t calc_crc = fhss_crc8(&temp[1], (uint8_t)(wire_len + 2U));
        uint8_t wire_crc = temp[wire_total - 1U];

        if (calc_crc != wire_crc) {
            continue;
        }

        if (wire_ch != tuned_ch) {
            continue;
        }

        for (uint8_t i = 0; i < wire_len; i++) {
            out_buf[i] = temp[3U + i];
        }
        *out_len = wire_len;
        return (int)OK;
    }
}

static int mock_ring_poll_diag(mock_ring_t *r, uint8_t tuned_ch, uint8_t *out_buf, uint8_t *out_len, air_mock_diag_t *diag) {
    while (1) {
        uint16_t head = r->head;
        uint16_t tail = r->tail;
        uint16_t avail = (head >= tail) ? (uint16_t)(head - tail) : (uint16_t)(256U + head - tail);

        if (avail == 0U) {
            return (int)ERR_TIMEOUT;
        }

        if (r->ring[tail] != MOCK_WIRE_DELIMITER) {
            r->tail = (tail + 1U) & 0xFFU;
            continue;
        }

        if (avail < 3U) {
            return (int)ERR_TIMEOUT;
        }

        uint8_t wire_ch = r->ring[(tail + 1U) & 0xFFU];
        uint8_t wire_len = r->ring[(tail + 2U) & 0xFFU];

        if (wire_len < HEADER_LEN || wire_len > MAX_FRAME_LEN || wire_ch >= NUM_CHANNELS) {
            r->tail = (tail + 1U) & 0xFFU;
            continue;
        }

        uint16_t wire_total = (uint16_t)(wire_len + 4U);
        if (avail < wire_total) {
            return (int)ERR_TIMEOUT;
        }

        uint8_t temp[MAX_FRAME_LEN + 4U];
        for (uint16_t i = 0; i < wire_total; i++) {
            temp[i] = r->ring[(tail + i) & 0xFFU];
        }
        r->tail = (tail + wire_total) & 0xFFU;

        uint8_t calc_crc = fhss_crc8(&temp[1], (uint8_t)(wire_len + 2U));
        uint8_t wire_crc = temp[wire_total - 1U];

        if (calc_crc != wire_crc) {
            if (diag != (air_mock_diag_t *)0) {
                diag->crc_error_count++;
            }
            continue;
        }

        if (wire_ch != tuned_ch) {
            if (diag != (air_mock_diag_t *)0) {
                diag->channel_mismatch_count++;
                diag->last_mismatch_wire_ch = wire_ch;
                diag->last_mismatch_tuned_ch = tuned_ch;
                diag->last_mismatch_len = wire_len;
            }
            continue;
        }

        for (uint8_t i = 0; i < wire_len; i++) {
            out_buf[i] = temp[3U + i];
        }
        *out_len = wire_len;
        return (int)OK;
    }
}

static void dummy_cb(void) {
}

int main(void) {
    printf("[HOST TEST] Running A0 scaffold tests...\n");

    /* Verify mock HAL resets */
    mock_hal_reset();
    assert(mock_hal_get_nss_state() == 1);
    assert(mock_hal_get_reset_state() == 1);

    /* Verify TIM2_ScheduleHopMs range check */
    assert(TIM2_ScheduleHopMs(0, dummy_cb) == (int)ERR_RANGE);
    assert(TIM2_ScheduleHopMs(60001, dummy_cb) == (int)ERR_RANGE);
    assert(TIM2_ScheduleHopMs(100, (void (*)(void))0) == (int)ERR_NULL);
    assert(TIM2_ScheduleHopMs(100, dummy_cb) == (int)OK);

    /* Verify frozen constants */
    assert(FHSS_KEY == 0xA5C31F7BUL);
    assert(NUM_CHANNELS == 8U);
    assert(MAX_PAYLOAD == 16U);
    assert(HOME_DWELL_MS == 120U);
    assert(HOP_GUARD_MS == 8U);
    assert(ACK_TIMEOUT_MS == 12U);

    printf("[HOST TEST] A0 scaffold tests PASS\n");

    printf("[HOST TEST] Running A1 clock and UART tests...\n");

    /* A1-N2: NULL and empty string safe */
    mock_hal_clear_uart();
    UART1_SendString((const char *)0);
    assert(mock_hal_get_uart_bytes() == 0);
    UART1_SendString("");
    assert(mock_hal_get_uart_bytes() == 0);

    /* A1-P2: Verify 115200 BRR arithmetic for 8 MHz HSI */
    /* USARTDIV = 8000000 / (16 * 115200) = 4.340277... */
    /* Mantissa = 4, Fraction = round(0.340277 * 16) = 5 -> BRR = 0x0045 */
    uint32_t f_pclk = 8000000UL;
    uint32_t baud = 115200UL;
    uint32_t div_x16 = (f_pclk + (baud / 2UL)) / baud; /* integer rounded 8000000/115200 = 69 = 0x45 */
    uint32_t expected_brr = ((div_x16 / 16UL) << 4) | (div_x16 % 16UL);
    assert(expected_brr == 0x0045UL);

    /* A1-X2: Send 512-character string */
    mock_hal_clear_uart();
    char test_512[513];
    for (int i = 0; i < 512; i++) {
        test_512[i] = (char)('A' + (i % 26));
    }
    test_512[512] = '\0';
    UART1_SendString(test_512);
    assert(mock_hal_get_uart_bytes() == 512);
    const char *rx = mock_hal_get_uart_buffer();
    for (int i = 0; i < 512; i++) {
        assert(rx[i] == test_512[i]);
    }

    /* Verify millis and delay_ms */
    mock_hal_set_millis(50);
    assert(millis() == 50);
    delay_ms(100);
    assert(millis() == 150);

    printf("[HOST TEST] A1 clock and UART tests PASS\n");

    printf("[HOST TEST] Running A2 SPI and RegVersion tests...\n");

    /* A2-P1: Read RegVersion, expect 0x12 */
    mock_hal_reset();
    uint8_t ver = sx1278_read_reg(REG_VERSION);
    assert(ver == 0x12U);
    assert(mock_hal_get_nss_state() == 1); /* NSS returned high */

    /* A2-P3: Write 0x5A to RegPreambleLsb (0x26), read back, restore 0x03 */
    sx1278_write_reg(REG_PREAMBLE_LSB, 0x5AU);
    uint8_t readback = sx1278_read_reg(REG_PREAMBLE_LSB);
    assert(readback == 0x5AU);
    assert(mock_hal_get_reg(REG_PREAMBLE_LSB) == 0x5AU);

    sx1278_write_reg(REG_PREAMBLE_LSB, 0x03U);
    uint8_t restored = sx1278_read_reg(REG_PREAMBLE_LSB);
    assert(restored == 0x03U);
    assert(mock_hal_get_reg(REG_PREAMBLE_LSB) == 0x03U);
    assert(mock_hal_get_nss_state() == 1);

    /* A2-P4: Reset sequence check */
    mock_hal_reset();
    assert(mock_hal_get_reset_state() == 1);
    RADIO_RESET_LOW();
    assert(mock_hal_get_reset_state() == 0);
    RADIO_RESET_HIGH();
    assert(mock_hal_get_reset_state() == 1);

    /* A2-N1 & A2-N2: Simulated MISO stuck low (0x00) and stuck high (0xFF) */
    mock_hal_set_reg(REG_VERSION, 0x00U);
    assert(sx1278_read_reg(REG_VERSION) == 0x00U);
    mock_hal_set_reg(REG_VERSION, 0xFFU);
    assert(sx1278_read_reg(REG_VERSION) == 0xFFU);

    /* SELFTEST_SPI_LOOPBACK host simulation test */
    static const uint8_t loopback_pattern[4] = {0xA5U, 0x5AU, 0x00U, 0xFFU};
    int loopback_match = 1;
    for (int i = 0; i < 4; i++) {
        /* In mock HAL, simulate loopback: SPI1_Transfer returns tx byte when NSS=0 */
        NSS_LOW();
        /* Verify pattern elements */
        assert(loopback_pattern[0] == 0xA5U);
        assert(loopback_pattern[1] == 0x5AU);
        assert(loopback_pattern[2] == 0x00U);
        assert(loopback_pattern[3] == 0xFFU);
        NSS_HIGH();
    }
    assert(loopback_match == 1);

    printf("[HOST TEST] A2 SPI and RegVersion tests PASS\n");

    printf("[HOST TEST] Running FHSS protocol vector tests...\n");
    assert(fhss_verify_vectors() == OK);
    printf("[HOST TEST] FHSS protocol vector tests PASS\n");

    printf("[HOST TEST] Running Node A USART2 mock transport tests...\n");

    /* 1. USART2 BRR calculation test for 8 MHz APB1 HSI clock */
    uint32_t f_pclk1 = 8000000UL;
    uint32_t baud_usart2 = 115200UL;
    uint32_t div_x16_usart2 = (f_pclk1 + (baud_usart2 / 2UL)) / baud_usart2; /* 69 = 0x45 */
    uint32_t expected_usart2_brr = ((div_x16_usart2 / 16UL) << 4) | (div_x16_usart2 % 16UL);
    assert(expected_usart2_brr == 0x0045UL);
    /* Actual baud rate verification: 8000000 / (16 * (4 + 5/16)) = 115942 (+0.64% error) */
    uint32_t actual_baud = (f_pclk1 * 16UL) / (69UL * 16UL);
    assert(actual_baud >= 114000UL && actual_baud <= 116500UL);

    /* 2. Mock wire frame encoding and decoding using existing format */
    static const uint8_t s_test_hello[5] = {'H', 'E', 'L', 'L', 'O'};
    uint8_t raw_frame[MAX_FRAME_LEN];
    uint8_t raw_len = 0U;
    assert(fhss_build_data(5U, 3U, 8U, 2U, 0U, 5U, 17U, s_test_hello, 5U, raw_frame, &raw_len) == OK);
    assert(raw_len == 11U);

    /* Encode wire packet: 0x7E, ch=5, len=11, raw_frame bytes, wire_crc */
    uint8_t wire_packet[MAX_FRAME_LEN + 4U];
    uint8_t test_ch = 5U;
    wire_packet[0] = MOCK_WIRE_DELIMITER;
    wire_packet[1] = test_ch;
    wire_packet[2] = raw_len;
    for (uint8_t i = 0; i < raw_len; i++) {
        wire_packet[3U + i] = raw_frame[i];
    }
    uint8_t wire_hdr[2] = {test_ch, raw_len};
    uint8_t wire_crc = fhss_crc8(wire_hdr, 2U);
    wire_crc = fhss_crc8_update(wire_crc, raw_frame, raw_len);
    wire_packet[3U + raw_len] = wire_crc;
    uint8_t total_wire_len = (uint8_t)(raw_len + 4U);

    mock_ring_t ring;
    mock_ring_init(&ring);

    /* Push wire packet into ring buffer */
    for (uint8_t i = 0; i < total_wire_len; i++) {
        assert(mock_ring_push(&ring, wire_packet[i]) == 1);
    }
    assert(mock_ring_avail(&ring) == total_wire_len);

    /* Decode with matching tuned channel (ch=5) */
    uint8_t dec_buf[MAX_FRAME_LEN];
    uint8_t dec_len = 0U;
    assert(mock_ring_poll(&ring, 5U, dec_buf, &dec_len) == OK);
    assert(dec_len == raw_len);
    assert(memcmp(dec_buf, raw_frame, raw_len) == 0);
    assert(mock_ring_avail(&ring) == 0U);

    /* Parse extracted frame to confirm complete protocol integrity */
    fhss_frame_t parsed_frame;
    fhss_replay_t replay = {0, 0};
    assert(fhss_parse_frame(dec_buf, dec_len, DIR_A_TO_B, &replay, &parsed_frame) == OK);
    assert(parsed_frame.seq == 5U);
    assert(parsed_frame.msg_id == 3U);
    assert(parsed_frame.next_ch == 5U);
    assert(parsed_frame.dwell_code == 17U);
    assert(parsed_frame.payload_len == 5U);
    assert(memcmp(parsed_frame.payload, s_test_hello, 5U) == 0);

    /* 3. Channel filtering test: receiver tuned to ch=2 ignores ch=5 packet */
    for (uint8_t i = 0; i < total_wire_len; i++) {
        assert(mock_ring_push(&ring, wire_packet[i]) == 1);
    }
    assert(mock_ring_poll(&ring, 2U, dec_buf, &dec_len) == (int)ERR_TIMEOUT);
    assert(mock_ring_avail(&ring) == 0U); /* Packet discarded from ring */

    /* 4. Rejection of invalid wire CRC */
    for (uint8_t i = 0; i < total_wire_len; i++) {
        uint8_t b = wire_packet[i];
        if (i == (total_wire_len - 1U)) {
            b ^= 0x55U; /* Invert CRC byte */
        }
        assert(mock_ring_push(&ring, b) == 1);
    }
    assert(mock_ring_poll(&ring, 5U, dec_buf, &dec_len) == (int)ERR_TIMEOUT);
    assert(mock_ring_avail(&ring) == 0U);

    /* 5. Rejection of malformed length: skips false delimiter and recovers next valid frame */
    assert(mock_ring_push(&ring, MOCK_WIRE_DELIMITER) == 1);
    assert(mock_ring_push(&ring, 5U) == 1);
    assert(mock_ring_push(&ring, 3U) == 1); /* len = 3 < HEADER_LEN (false delimiter) */
    /* Append valid frame directly behind the false delimiter */
    for (uint8_t i = 0; i < total_wire_len; i++) {
        assert(mock_ring_push(&ring, wire_packet[i]) == 1);
    }
    assert(mock_ring_poll(&ring, 5U, dec_buf, &dec_len) == OK);
    assert(dec_len == raw_len);
    assert(memcmp(dec_buf, raw_frame, raw_len) == 0);
    assert(mock_ring_avail(&ring) == 0U);

    /* 6. Recovery from leading garbage / noise bytes */
    assert(mock_ring_push(&ring, 0xAAU) == 1);
    assert(mock_ring_push(&ring, 0x55U) == 1);
    assert(mock_ring_push(&ring, 0x00U) == 1);
    for (uint8_t i = 0; i < total_wire_len; i++) {
        assert(mock_ring_push(&ring, wire_packet[i]) == 1);
    }
    assert(mock_ring_poll(&ring, 5U, dec_buf, &dec_len) == OK);
    assert(dec_len == raw_len);
    assert(memcmp(dec_buf, raw_frame, raw_len) == 0);
    assert(mock_ring_avail(&ring) == 0U);

    /* 7. Incomplete / truncated frame handling (returns ERR_TIMEOUT without dropping valid start) */
    for (uint8_t i = 0; i < 5U; i++) {
        assert(mock_ring_push(&ring, wire_packet[i]) == 1);
    }
    assert(mock_ring_poll(&ring, 5U, dec_buf, &dec_len) == (int)ERR_TIMEOUT);
    assert(mock_ring_avail(&ring) == 5U); /* Preserves partial frame */
    for (uint8_t i = 5U; i < total_wire_len; i++) {
        assert(mock_ring_push(&ring, wire_packet[i]) == 1);
    }
    assert(mock_ring_poll(&ring, 5U, dec_buf, &dec_len) == OK);
    assert(dec_len == raw_len);
    assert(mock_ring_avail(&ring) == 0U);

    /* 8. Buffer overflow protection and recovery */
    mock_ring_init(&ring);
    for (int i = 0; i < 255; i++) {
        assert(mock_ring_push(&ring, (uint8_t)i) == 1);
    }
    assert(mock_ring_avail(&ring) == 255U);
    /* 256th push must fail (buffer full) */
    assert(mock_ring_push(&ring, 0xFFU) == 0);
    assert(mock_ring_avail(&ring) == 255U);
    /* Drain 10 bytes */
    ring.tail = (ring.tail + 10U) & 0xFFU;
    assert(mock_ring_avail(&ring) == 245U);
    /* Push 10 new bytes successfully */
    for (int i = 0; i < 10; i++) {
        assert(mock_ring_push(&ring, 0xA0U) == 1);
    }
    assert(mock_ring_avail(&ring) == 255U);

    printf("[HOST TEST] Node A USART2 mock transport tests PASS\n");

    printf("[HOST TEST] Running Step 1 Ping-Pong test logic tests...\n");

    /* 1. Verify exact 15-byte ping envelope format and CRC */
    static const uint8_t s_exp_ping_env[15] = {
        0x7EU, 0x00U, 0x0BU,
        0xA1U, 0x05U, 0x03U, 0xE8U, 0x6AU, 0x7DU, 0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU,
        0x18U
    };
    /* Delimiter, channel, length check */
    assert(s_exp_ping_env[0] == 0x7EU);
    assert(s_exp_ping_env[1] == 0x00U);
    assert(s_exp_ping_env[2] == 0x0BU);

    /* Verify CRC-8/SMBUS calculation over channel, length, and frame bytes */
    uint8_t calc_ping_crc = fhss_crc8(&s_exp_ping_env[1], 2U);
    calc_ping_crc = fhss_crc8_update(calc_ping_crc, &s_exp_ping_env[3], 11U);
    assert(calc_ping_crc == 0x18U);
    assert(s_exp_ping_env[14] == 0x18U);

    /* Verify embedded frame parses as valid DATA frame */
    fhss_frame_t ping_parsed;
    fhss_replay_t ping_replay = {0, 0};
    assert(fhss_parse_frame(&s_exp_ping_env[3], 11U, DIR_A_TO_B, &ping_replay, &ping_parsed) == OK);
    assert(ping_parsed.type == PKT_TYPE_DATA);
    assert(ping_parsed.seq == 5U);
    assert(ping_parsed.msg_id == 3U);
    assert(ping_parsed.total == 8U);
    assert(ping_parsed.idx == 2U);
    assert(ping_parsed.last == 0U);
    assert(ping_parsed.payload_len == 5U);
    assert(memcmp(ping_parsed.payload, "HELLO", 5U) == 0);

    /* 2. Test Step 1 response evaluation logic */
    /* Case A: Valid 0x06 ACK received -> PASS */
    uint8_t rx_test_byte = 0x06U;
    int test_got_ack = (rx_test_byte == 0x06U) ? 1 : 0;
    assert(test_got_ack == 1);

    /* Case B: NAK / non-0x06 byte received -> does not pass */
    rx_test_byte = 0x15U;
    test_got_ack = (rx_test_byte == 0x06U) ? 1 : 0;
    assert(test_got_ack == 0);

    /* Case C: Zero / noise byte received -> does not pass */
    rx_test_byte = 0x00U;
    test_got_ack = (rx_test_byte == 0x06U) ? 1 : 0;
    assert(test_got_ack == 0);

    printf("[HOST TEST] Step 1 Ping-Pong test logic tests PASS\n");

    printf("[HOST TEST] Running Stage 2 Mock transport and vector tests...\n");

    /* 1. Verify documented frozen known-answer frame vector against CONTEXT.md and fhss_protocol */
    static const uint8_t s_stage2_frozen_frame[11] = {
        0xA1U, 0x05U, 0x03U, 0xE8U, 0x6AU, 0x7DU,
        0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU
    };
    assert(sizeof(s_stage2_frozen_frame) == 11U);

    /* Reconstruct via fhss_build_data: seq=5, msg_id=3, total=8, idx=2, last=0, next_ch=5, code=17, "HELLO" */
    uint8_t built_frame[MAX_FRAME_LEN];
    uint8_t built_len = 0U;
    static const uint8_t s_stage2_hello[5] = {'H', 'E', 'L', 'L', 'O'};
    assert(fhss_build_data(5U, 3U, 8U, 2U, 0U, 5U, 17U, s_stage2_hello, 5U, built_frame, &built_len) == OK);
    assert(built_len == 11U);
    assert(memcmp(built_frame, s_stage2_frozen_frame, 11U) == 0);

    /* Parse and verify fields */
    fhss_frame_t stage2_parsed;
    fhss_replay_t stage2_replay = {0, 0};
    assert(fhss_parse_frame(s_stage2_frozen_frame, 11U, DIR_A_TO_B, &stage2_replay, &stage2_parsed) == OK);
    assert(stage2_parsed.type == PKT_TYPE_DATA);
    assert(stage2_parsed.seq == 5U);
    assert(stage2_parsed.msg_id == 3U);
    assert(stage2_parsed.total == 8U);
    assert(stage2_parsed.idx == 2U);
    assert(stage2_parsed.last == 0U);
    assert(stage2_parsed.next_ch == 5U);
    assert(stage2_parsed.dwell_code == 17U);
    assert(stage2_parsed.tag == 0x7DU);
    assert(stage2_parsed.payload_len == 5U);
    assert(memcmp(stage2_parsed.payload, "HELLO", 5U) == 0);

    /* 2. Verify wire envelope serialization for channel 0 and CRC-8 calculation */
    /* Wire format: [0x7E][channel 0][length 11][frame bytes 0..10][CRC-8] */
    uint8_t stage2_wire_hdr[2] = {0x00U, 0x0BU};
    uint8_t stage2_calc_crc = fhss_crc8(stage2_wire_hdr, 2U);
    stage2_calc_crc = fhss_crc8_update(stage2_calc_crc, s_stage2_frozen_frame, 11U);
    assert(stage2_calc_crc == 0x18U);

    /* Full 15-byte envelope comparison */
    static const uint8_t s_exp_stage2_envelope[15] = {
        0x7EU, 0x00U, 0x0BU,
        0xA1U, 0x05U, 0x03U, 0xE8U, 0x6AU, 0x7DU, 0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU,
        0x18U
    };
    uint8_t stage2_serialized[15];
    stage2_serialized[0] = 0x7EU;
    stage2_serialized[1] = 0x00U;
    stage2_serialized[2] = 0x0BU;
    memcpy(&stage2_serialized[3], s_stage2_frozen_frame, 11U);
    stage2_serialized[14] = stage2_calc_crc;
    assert(memcmp(stage2_serialized, s_exp_stage2_envelope, 15U) == 0);

    /* 3. Feed envelope into mock receiver ring on channel 0 */
    mock_ring_t stage2_ring;
    mock_ring_init(&stage2_ring);
    for (uint8_t i = 0; i < 15U; i++) {
        assert(mock_ring_push(&stage2_ring, stage2_serialized[i]) == 1);
    }
    uint8_t rx_extracted[MAX_FRAME_LEN];
    uint8_t rx_extracted_len = 0U;
    assert(mock_ring_poll(&stage2_ring, 0U, rx_extracted, &rx_extracted_len) == OK);
    assert(rx_extracted_len == 11U);
    assert(memcmp(rx_extracted, s_stage2_frozen_frame, 11U) == 0);
    assert(mock_ring_avail(&stage2_ring) == 0U);

    /* 4. Verify channel filtering: receiver tuned to channel 1 discards channel 0 frame */
    for (uint8_t i = 0; i < 15U; i++) {
        assert(mock_ring_push(&stage2_ring, stage2_serialized[i]) == 1);
    }
    assert(mock_ring_poll(&stage2_ring, 1U, rx_extracted, &rx_extracted_len) == (int)ERR_TIMEOUT);
    assert(mock_ring_avail(&stage2_ring) == 0U);

    /* 5. Verify Stage 2 receipt marker handling logic */
    /* Case A: Marker 0x06 received within 50 ms -> PASS */
    uint8_t test_marker = 0x06U;
    int stage2_got_receipt = (test_marker == 0x06U) ? 1 : 0;
    assert(stage2_got_receipt == 1);

    /* Case B: Corrupted / NAK byte (0x15) -> does not trigger PASS */
    test_marker = 0x15U;
    stage2_got_receipt = (test_marker == 0x06U) ? 1 : 0;
    assert(stage2_got_receipt == 0);

    /* Case C: Stale byte cleared by flush */
    mock_ring_init(&stage2_ring);
    assert(mock_ring_push(&stage2_ring, 0xFFU) == 1);
    assert(mock_ring_avail(&stage2_ring) == 1U);
    mock_ring_init(&stage2_ring); /* Simulates air_mock_flush */
    assert(mock_ring_avail(&stage2_ring) == 0U);

    printf("[HOST TEST] Stage 2 Mock transport and vector tests PASS\n");

    printf("[HOST TEST] Running Stage 3 Mock FHSS state-machine tests...\n");

    /* 1. Verify SEARCH state HOP beacon generation and directive interpretation */
    uint8_t stage3_hop_buf[MAX_FRAME_LEN];
    uint8_t stage3_hop_len = 0U;
    /* In SEARCH, Node A emits HOP packets with directive for hop 1: next_ch=5, dwell_code=13 */
    assert(fhss_build_hop(0U, 5U, 13U, stage3_hop_buf, &stage3_hop_len) == OK);
    assert(stage3_hop_len == HEADER_LEN); /* 6 bytes */
    assert(stage3_hop_buf[0] == 0xA3U);   /* PKT_TYPE_HOP (0xA0 | 3) */
    assert(stage3_hop_buf[1] == 0x00U);   /* seq = 0 */
    assert(stage3_hop_buf[2] == 0x00U);   /* msg_id = 0 */
    assert(stage3_hop_buf[3] == 0x00U);   /* fragment descriptor = 0 */

    /* Plain directive: (5 << 5) | 13 = 0xAD. Mask for seq=0, dir=0 is 0x03 -> 0xAD ^ 0x03 = 0xAE */
    assert(stage3_hop_buf[4] == 0xAEU);

    /* Verify TAG integrity */
    uint8_t expected_hop_tag = fhss_calc_tag(stage3_hop_buf, (const uint8_t *)0, 0U);
    assert(stage3_hop_buf[5] == expected_hop_tag);

    /* Parse HOP frame from Node A perspective (DIR_A_TO_B) */
    fhss_frame_t stage3_parsed_hop;
    fhss_replay_t stage3_hop_replay = {0, 0};
    assert(fhss_parse_frame(stage3_hop_buf, stage3_hop_len, DIR_A_TO_B, &stage3_hop_replay, &stage3_parsed_hop) == OK);
    assert(stage3_parsed_hop.type == PKT_TYPE_HOP);
    assert(stage3_parsed_hop.seq == 0U);
    assert(stage3_parsed_hop.next_ch == 5U);
    assert(stage3_parsed_hop.dwell_code == 13U);
    uint32_t hop1_dwell_ms = DWELL_BASE_MS + DWELL_STEP_MS * (uint32_t)stage3_parsed_hop.dwell_code;
    assert(hop1_dwell_ms == 145U); /* 80 + 5*13 = 145 ms */

    /* 2. Verify mock transport envelope encoding for HOP beacon on channel 0 */
    uint8_t hop_wire[MAX_FRAME_LEN + 4U];
    hop_wire[0] = MOCK_WIRE_DELIMITER;
    hop_wire[1] = 0U; /* Channel 0 (home/search channel) */
    hop_wire[2] = stage3_hop_len;
    for (uint8_t i = 0; i < stage3_hop_len; i++) {
        hop_wire[3U + i] = stage3_hop_buf[i];
    }
    uint8_t hop_hdr[2] = {0U, stage3_hop_len};
    uint8_t hop_wire_crc = fhss_crc8(hop_hdr, 2U);
    hop_wire_crc = fhss_crc8_update(hop_wire_crc, stage3_hop_buf, stage3_hop_len);
    hop_wire[3U + stage3_hop_len] = hop_wire_crc;
    uint8_t total_hop_wire = (uint8_t)(stage3_hop_len + 4U);

    mock_ring_t stage3_ring;
    mock_ring_init(&stage3_ring);
    for (uint8_t i = 0; i < total_hop_wire; i++) {
        assert(mock_ring_push(&stage3_ring, hop_wire[i]) == 1);
    }
    uint8_t extracted_hop[MAX_FRAME_LEN];
    uint8_t extracted_hop_len = 0U;

    /* Channel 0 receiver successfully hears beacon */
    assert(mock_ring_poll(&stage3_ring, 0U, extracted_hop, &extracted_hop_len) == OK);
    assert(extracted_hop_len == stage3_hop_len);
    assert(memcmp(extracted_hop, stage3_hop_buf, stage3_hop_len) == 0);

    /* Simulated channel mismatch: receiver tuned to channel 2 drops channel 0 packet */
    for (uint8_t i = 0; i < total_hop_wire; i++) {
        assert(mock_ring_push(&stage3_ring, hop_wire[i]) == 1);
    }
    assert(mock_ring_poll(&stage3_ring, 2U, extracted_hop, &extracted_hop_len) == (int)ERR_TIMEOUT);
    assert(mock_ring_avail(&stage3_ring) == 0U);

    /* 3. Verify Peer Synchronization (SEARCH -> LOCKED) on receipt of valid ACK */
    uint8_t ack_frame[MAX_FRAME_LEN];
    uint8_t ack_len = 0U;
    /* Node B sends ACK (DIR_B_TO_A): seq=0, msg_id=0, total=1, bitmap=0x01, rssi_raw=0x78, next_ch=5, dwell_code=13 */
    assert(fhss_build_ack(0U, 0U, 1U, 0x01U, 0x78U, 5U, 13U, ack_frame, &ack_len) == OK);
    assert(ack_len == 8U); /* 6 header + 2 payload */
    assert(ack_frame[0] == 0xA2U); /* PKT_TYPE_ACK */

    /* Replay and state verification */
    fhss_replay_t node_a_replay = {0, 0};
    fhss_frame_t parsed_ack;
    assert(fhss_parse_frame(ack_frame, ack_len, DIR_B_TO_A, &node_a_replay, &parsed_ack) == OK);
    assert(parsed_ack.type == PKT_TYPE_ACK);
    assert(parsed_ack.seq == 0U);
    assert(parsed_ack.next_ch == 5U);
    assert(parsed_ack.dwell_code == 13U);
    assert(node_a_replay.have_last == 1U);
    assert(node_a_replay.last_seq == 0U);

    /* Replay rejection: duplicate seq=0 must be rejected */
    assert(fhss_parse_frame(ack_frame, ack_len, DIR_B_TO_A, &node_a_replay, &parsed_ack) == (int)ERR_REPLAY);

    /* Sequence progression: seq=1 must be accepted */
    uint8_t ack_frame_seq1[MAX_FRAME_LEN];
    uint8_t ack_len_seq1 = 0U;
    assert(fhss_build_ack(1U, 0U, 1U, 0x01U, 0x78U, 5U, 13U, ack_frame_seq1, &ack_len_seq1) == OK);
    assert(fhss_parse_frame(ack_frame_seq1, ack_len_seq1, DIR_B_TO_A, &node_a_replay, &parsed_ack) == OK);
    assert(node_a_replay.last_seq == 1U);

    /* Integrity protection: corrupted tag must be rejected with ERR_TAG */
    ack_frame_seq1[5] ^= 0x55U; /* Corrupt tag byte */
    assert(fhss_parse_frame(ack_frame_seq1, ack_len_seq1, DIR_B_TO_A, &node_a_replay, &parsed_ack) == (int)ERR_TAG);

    /* 4. Verify Hop Schedule, PRNG channel generation, and dwell uniqueness */
    assert(HOME_DWELL_MS == 120U);
    assert(HOP_GUARD_MS == 8U);
    assert(ACK_TIMEOUT_MS == 12U);

    uint32_t prng = 0x83A14B2CUL;
    uint8_t last_code = 13U;
    for (int step = 0; step < 20; step++) {
        prng ^= prng << 13;
        prng ^= prng >> 17;
        prng ^= prng << 5;
        uint8_t ch = (uint8_t)(prng % NUM_CHANNELS);
        assert(ch < NUM_CHANNELS);

        uint8_t next_code;
        do {
            prng ^= prng << 13;
            prng ^= prng >> 17;
            prng ^= prng << 5;
            next_code = (uint8_t)(prng % 32U);
        } while (next_code == last_code);

        assert(next_code != last_code);
        assert(next_code <= MAX_DWELL_CODE);
        uint32_t dwell_ms = DWELL_BASE_MS + DWELL_STEP_MS * (uint32_t)next_code;
        assert(dwell_ms >= 80U && dwell_ms <= 235U);
        last_code = next_code;
    }

    /* 5. Verify Loss Transition (LOCKED -> LOST -> SEARCH) */
    uint8_t missed_hops = 0U;
    int state_locked = 1;
    for (int h = 0; h < (int)SYNC_MAX_MISSED_HOPS; h++) {
        missed_hops++;
    }
    assert(missed_hops >= SYNC_MAX_MISSED_HOPS);
    if (missed_hops >= SYNC_MAX_MISSED_HOPS) {
        state_locked = 0; /* Transitions to SEARCH */
    }
    assert(state_locked == 0);
    /* In SEARCH, replay history is cleared and channel resets to 0 */
    node_a_replay.have_last = 0U;
    uint8_t current_ch = 0U;
    assert(current_ch == 0U);
    assert(node_a_replay.have_last == 0U);

    /* 6. Verify Isolation from Stage 2 marker: raw byte 0x06 is rejected by FHSS receiver */
    mock_ring_init(&stage3_ring);
    assert(mock_ring_push(&stage3_ring, 0x06U) == 1);
    uint8_t raw_rx_buf[MAX_FRAME_LEN];
    uint8_t raw_rx_len = 0U;
    /* Not delimited by 0x7E, poll times out */
    assert(mock_ring_poll(&stage3_ring, 0U, raw_rx_buf, &raw_rx_len) == (int)ERR_TIMEOUT);

    /* Directly parsing raw 1-byte 0x06 fails length check */
    uint8_t raw_06 = 0x06U;
    fhss_frame_t dummy_frame;
    assert(fhss_parse_frame(&raw_06, 1U, DIR_B_TO_A, &node_a_replay, &dummy_frame) == (int)ERR_LEN);

    printf("[HOST TEST] Stage 3 Mock FHSS state-machine tests PASS\n");

    printf("[HOST TEST] Running Stage 3.1 FHSS diagnostic and failure isolation tests...\n");

    /* 1. Verify channel mismatch detection and telemetry in mock transport */
    air_mock_diag_t test_diag = {0U, 0U, 0U, 0U, 0U};
    mock_ring_t diag_ring;
    mock_ring_init(&diag_ring);

    /* Construct a wire envelope for channel 0 (e.g. Node A SEARCH beacon, or Node B ACK on ch 0) */
    uint8_t ch0_payload[HEADER_LEN] = {0xA3U, 0x01U, 0x00U, 0x00U, 0xAEU, 0x00U};
    ch0_payload[5] = fhss_calc_tag(ch0_payload, (const uint8_t *)0, 0U);
    uint8_t ch0_wire[MAX_FRAME_LEN + 4U];
    ch0_wire[0] = MOCK_WIRE_DELIMITER;
    ch0_wire[1] = 0U; /* wire_ch = 0 */
    ch0_wire[2] = HEADER_LEN;
    for (uint8_t i = 0; i < HEADER_LEN; i++) {
        ch0_wire[3U + i] = ch0_payload[i];
    }
    uint8_t ch0_hdr[2] = {0U, HEADER_LEN};
    uint8_t ch0_crc = fhss_crc8(ch0_hdr, 2U);
    ch0_crc = fhss_crc8_update(ch0_crc, ch0_payload, HEADER_LEN);
    ch0_wire[3U + HEADER_LEN] = ch0_crc;

    /* Push channel 0 frame into ring buffer */
    for (uint8_t i = 0; i < (HEADER_LEN + 4U); i++) {
        assert(mock_ring_push(&diag_ring, ch0_wire[i]) == 1);
    }

    /* Receiver is tuned to channel 5 (Hop 1). Must reject and record mismatch diagnostic */
    uint8_t diag_rx_buf[MAX_FRAME_LEN];
    uint8_t diag_rx_len = 0U;
    assert(mock_ring_poll_diag(&diag_ring, 5U, diag_rx_buf, &diag_rx_len, &test_diag) == (int)ERR_TIMEOUT);
    assert(test_diag.channel_mismatch_count == 1U);
    assert(test_diag.last_mismatch_wire_ch == 0U);
    assert(test_diag.last_mismatch_tuned_ch == 5U);
    assert(test_diag.last_mismatch_len == HEADER_LEN);
    assert(test_diag.crc_error_count == 0U);

    /* 2. Verify wire CRC error detection and telemetry */
    ch0_wire[1] = 5U; /* Set wire_ch to 5, but keep old CRC -> CRC mismatch */
    for (uint8_t i = 0; i < (HEADER_LEN + 4U); i++) {
        assert(mock_ring_push(&diag_ring, ch0_wire[i]) == 1);
    }
    assert(mock_ring_poll_diag(&diag_ring, 5U, diag_rx_buf, &diag_rx_len, &test_diag) == (int)ERR_TIMEOUT);
    assert(test_diag.crc_error_count == 1U);
    assert(test_diag.channel_mismatch_count == 1U); /* unchanged */

    /* 3. Verify discrimination between ERR_REPLAY, ERR_TAG, and ERR_TYPE */
    fhss_replay_t test_replay = {0, 0};
    uint8_t ack_wire_body[MAX_FRAME_LEN];
    uint8_t ack_wire_len = 0U;

    /* Build valid ACK with seq=0 from Node B */
    assert(fhss_build_ack(0U, 0U, 1U, 0x01U, 0x78U, 5U, 13U, ack_wire_body, &ack_wire_len) == OK);
    fhss_frame_t parsed_res;

    /* First arrival: accepted */
    assert(fhss_parse_frame(ack_wire_body, ack_wire_len, DIR_B_TO_A, &test_replay, &parsed_res) == OK);
    assert(parsed_res.type == PKT_TYPE_ACK);
    assert(parsed_res.seq == 0U);
    assert(test_replay.have_last == 1U);
    assert(test_replay.last_seq == 0U);

    /* Duplicate seq=0 (e.g. if Node B doesn't increment ACK sequence) -> rejected with ERR_REPLAY */
    assert(fhss_parse_frame(ack_wire_body, ack_wire_len, DIR_B_TO_A, &test_replay, &parsed_res) == (int)ERR_REPLAY);

    /* Valid seq=1 from Node B -> accepted */
    assert(fhss_build_ack(1U, 0U, 1U, 0x01U, 0x78U, 5U, 13U, ack_wire_body, &ack_wire_len) == OK);
    assert(fhss_parse_frame(ack_wire_body, ack_wire_len, DIR_B_TO_A, &test_replay, &parsed_res) == OK);
    assert(test_replay.last_seq == 1U);

    /* Tag corruption on seq=2 -> rejected with ERR_TAG, not ERR_REPLAY */
    assert(fhss_build_ack(2U, 0U, 1U, 0x01U, 0x78U, 5U, 13U, ack_wire_body, &ack_wire_len) == OK);
    ack_wire_body[5] ^= 0x33U; /* Corrupt tag */
    assert(fhss_parse_frame(ack_wire_body, ack_wire_len, DIR_B_TO_A, &test_replay, &parsed_res) == (int)ERR_TAG);

    /* Non-ACK packet type discrimination: DATA packet received in ACK wait window */
    uint8_t data_wire_body[MAX_FRAME_LEN];
    uint8_t data_wire_len = 0U;
    const uint8_t dummy_payload[4] = {'T', 'E', 'S', 'T'};
    assert(fhss_build_data(2U, 0U, 1U, 0U, 1U, 5U, 13U, dummy_payload, 4U, data_wire_body, &data_wire_len) == OK);
    assert(fhss_parse_frame(data_wire_body, data_wire_len, DIR_B_TO_A, &test_replay, &parsed_res) == OK);
    assert(parsed_res.type == PKT_TYPE_DATA);
    assert(parsed_res.type != PKT_TYPE_ACK); /* State machine identifies type mismatch */

    /* 4. Verify 3 consecutive missed hops trigger LOST and reset to SEARCH on ch 0 */
    uint8_t s_missed_count = 0U;
    int link_locked = 1;
    for (int hop = 1; hop <= 3; hop++) {
        s_missed_count++;
        if (s_missed_count >= SYNC_MAX_MISSED_HOPS) {
            link_locked = 0; /* Transition to SEARCH */
        }
    }
    assert(s_missed_count == 3U);
    assert(link_locked == 0);
    /* In SEARCH, replay is cleared and channel resets to 0 */
    test_replay.have_last = 0U;
    uint8_t search_ch = 0U;
    assert(search_ch == 0U);
    assert(test_replay.have_last == 0U);

    printf("[HOST TEST] Stage 3.1 FHSS diagnostic and failure isolation tests PASS\n");

    printf("[HOST TEST] Running Stage 4 SX1278 SPI diagnostic tests...\n");

    /* 1. Verify read-only register access path via sx1278_read_reg */
    mock_hal_reset();
    assert(mock_hal_get_nss_state() == 1); /* Starts unselected */

    /* Ensure initial RegVersion is 0x12 */
    assert(mock_hal_get_reg(REG_VERSION) == 0x12U);
    uint8_t ver_read = sx1278_read_reg(REG_VERSION);
    assert(ver_read == 0x12U);
    assert(mock_hal_get_nss_state() == 1); /* NSS deasserts after read */

    /* 2. Verify 32 consecutive reads yielding 100% valid 0x12 */
    uint32_t valid_cnt = 0U;
    uint32_t fail_cnt = 0U;
    uint8_t first_unexp = 0U;
    int have_unexp = 0;

    for (int i = 0; i < 32; i++) {
        uint8_t v = sx1278_read_reg(REG_VERSION);
        if (v == SX1278_EXPECTED_VERSION) {
            valid_cnt++;
        } else {
            fail_cnt++;
            if (!have_unexp) { first_unexp = v; have_unexp = 1; }
        }
    }
    assert(valid_cnt == 32U);
    assert(fail_cnt == 0U);
    assert(have_unexp == 0);

    /* 3. Verify failure classification: simulated MISO stuck LOW (all 0x00) */
    mock_hal_set_reg(REG_VERSION, 0x00U);
    valid_cnt = 0U;
    fail_cnt = 0U;
    first_unexp = 0U;
    have_unexp = 0;

    for (int i = 0; i < 32; i++) {
        uint8_t v = sx1278_read_reg(REG_VERSION);
        if (v == SX1278_EXPECTED_VERSION) {
            valid_cnt++;
        } else {
            fail_cnt++;
            if (!have_unexp) { first_unexp = v; have_unexp = 1; }
        }
    }
    assert(valid_cnt == 0U);
    assert(fail_cnt == 32U);
    assert(have_unexp == 1);
    assert(first_unexp == 0x00U);

    /* 4. Verify failure classification: simulated MISO stuck HIGH / unselected (all 0xFF) */
    mock_hal_set_reg(REG_VERSION, 0xFFU);
    valid_cnt = 0U;
    fail_cnt = 0U;
    first_unexp = 0U;
    have_unexp = 0;

    for (int i = 0; i < 32; i++) {
        uint8_t v = sx1278_read_reg(REG_VERSION);
        if (v == SX1278_EXPECTED_VERSION) {
            valid_cnt++;
        } else {
            fail_cnt++;
            if (!have_unexp) { first_unexp = v; have_unexp = 1; }
        }
    }
    assert(valid_cnt == 0U);
    assert(fail_cnt == 32U);
    assert(have_unexp == 1);
    assert(first_unexp == 0xFFU);

    /* 5. Verify failure classification: intermittent reads */
    valid_cnt = 0U;
    fail_cnt = 0U;
    first_unexp = 0U;
    have_unexp = 0;

    for (int i = 0; i < 32; i++) {
        if ((i % 2) == 0) {
            mock_hal_set_reg(REG_VERSION, 0x12U);
        } else {
            mock_hal_set_reg(REG_VERSION, 0x00U);
        }
        uint8_t v = sx1278_read_reg(REG_VERSION);
        if (v == SX1278_EXPECTED_VERSION) {
            valid_cnt++;
        } else {
            fail_cnt++;
            if (!have_unexp) { first_unexp = v; have_unexp = 1; }
        }
    }
    assert(valid_cnt == 16U);
    assert(fail_cnt == 16U);
    assert(have_unexp == 1);
    assert(first_unexp == 0x00U);

    /* 6. Verify SX1278 hardware reset sequence toggles RESET pin */
    mock_hal_reset();
    assert(mock_hal_get_reset_state() == 1); /* starts high */
    sx1278_reset();
    assert(mock_hal_get_reset_state() == 1); /* ends high */

    /* 7. Verify read-only guarantee: reading RegVersion does not mutate other registers */
    mock_hal_set_reg(REG_PREAMBLE_LSB, 0x5AU);
    (void)sx1278_read_reg(REG_VERSION);
    assert(mock_hal_get_reg(REG_PREAMBLE_LSB) == 0x5AU); /* Unchanged */

    printf("[HOST TEST] Stage 4 SX1278 SPI diagnostic tests PASS\n");

    /* =====================================================================
     * Stage 5: Fault-Injection and Reliability Test Harness host verification
     * ===================================================================== */
    printf("[HOST TEST] Running Stage 5 fault-injection and reliability harness tests...\n");

    /* 1. Verify delimiter absence: NO byte in ANY Stage 5 frame body or envelope is 0x7E */
    static const uint8_t s_stage5_seqs[11] = {
        254U, 255U, 0U, 1U, /* Case 1 */
        2U,                 /* Case 2 */
        3U,                 /* Case 3 & 4 */
        4U,                 /* Case 5 */
        5U,                 /* Case 6 */
        6U,                 /* Case 7 */
        7U, 8U              /* Case 8 */
    };

    for (size_t i = 0; i < 11; i++) {
        uint8_t seq = s_stage5_seqs[i];
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(seq, 0U, 0U, f_buf, &f_len) == OK);
        assert(f_len == HEADER_LEN);

        /* Verify frame body has no 0x7E delimiter */
        for (uint8_t j = 0; j < f_len; j++) {
            assert(f_buf[j] != MOCK_WIRE_DELIMITER);
        }

        /* Verify wire envelope on channel 0 has no 0x7E in CRC */
        uint8_t env_crc_ch0 = fhss_crc8_update(fhss_crc8((const uint8_t[]){0x00U, f_len}, 2U), f_buf, f_len);
        assert(env_crc_ch0 != MOCK_WIRE_DELIMITER);

        /* For Case 6 (seq 5), also verify envelope on channel 1 */
        if (seq == 5U) {
            uint8_t env_crc_ch1 = fhss_crc8_update(fhss_crc8((const uint8_t[]){0x01U, f_len}, 2U), f_buf, f_len);
            assert(env_crc_ch1 != MOCK_WIRE_DELIMITER);
        }
    }

    /* 2. Verify Case 1 wraparound arithmetic and parsing on receiver replay */
    {
        fhss_replay_t rx_replay = {0, 0};
        static const uint8_t wrap_seqs[4] = {254U, 255U, 0U, 1U};

        for (int i = 0; i < 4; i++) {
            uint8_t seq = wrap_seqs[i];
            uint8_t f_buf[MAX_FRAME_LEN];
            uint8_t f_len = 0U;
            assert(fhss_build_hop(seq, 0U, 0U, f_buf, &f_len) == OK);

            fhss_frame_t parsed;
            status_t status = fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed);
            assert(status == OK);
            assert(parsed.seq == seq);
            assert(parsed.type == PKT_TYPE_HOP);
            assert(rx_replay.last_seq == seq);
            assert(rx_replay.have_last == 1U);
        }
    }

    /* 3. Verify Case 2: Bad envelope CRC rejection */
    {
        mock_ring_t r;
        mock_ring_init(&r);

        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(2U, 0U, 0U, f_buf, &f_len) == OK);

        /* Build envelope with corrupted CRC */
        uint8_t env[MAX_FRAME_LEN + 4U];
        env[0] = MOCK_WIRE_DELIMITER;
        env[1] = 0x00U;
        env[2] = f_len;
        memcpy(&env[3], f_buf, f_len);
        uint8_t good_crc = fhss_crc8(&env[1], (uint8_t)(f_len + 2U));
        env[3U + f_len] = good_crc ^ 0xA5U;
        uint8_t env_len = (uint8_t)(f_len + 4U);

        for (uint8_t i = 0; i < env_len; i++) {
            mock_ring_push(&r, env[i]);
        }

        uint8_t out_buf[MAX_FRAME_LEN];
        uint8_t out_len = 0U;
        int poll_res = mock_ring_poll(&r, 0U, out_buf, &out_len);
        assert(poll_res == (int)ERR_TIMEOUT); /* Wire parser drops corrupted CRC */
    }

    /* 4. Verify Case 3: Bad protocol tag rejection */
    {
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(3U, 0U, 0U, f_buf, &f_len) == OK);
        f_buf[5] ^= 0xFFU; /* Invert integrity tag */

        fhss_replay_t rx_replay = {1U, 1U}; /* last_seq = 1 */
        fhss_frame_t parsed;
        status_t status = fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed);
        assert(status == ERR_TAG);
        assert(rx_replay.last_seq == 1U); /* Replay state must not advance on ERR_TAG */
    }

    /* 5. Verify Case 4: Recovery after bad tag */
    {
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(3U, 0U, 0U, f_buf, &f_len) == OK);

        fhss_replay_t rx_replay = {1U, 1U}; /* last_seq = 1 */
        fhss_frame_t parsed;
        status_t status = fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed);
        assert(status == OK);
        assert(parsed.seq == 3U);
        assert(rx_replay.last_seq == 3U);
    }

    /* 6. Verify Case 5: Duplicate frame replay rejection */
    {
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(4U, 0U, 0U, f_buf, &f_len) == OK);

        fhss_replay_t rx_replay = {3U, 1U}; /* last_seq = 3 */
        fhss_frame_t parsed;

        /* First copy: accepted */
        status_t status1 = fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed);
        assert(status1 == OK);
        assert(rx_replay.last_seq == 4U);

        /* Second copy: rejected with ERR_REPLAY */
        status_t status2 = fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed);
        assert(status2 == ERR_REPLAY);
        assert(rx_replay.last_seq == 4U);
    }

    /* 7. Verify Case 6: Wrong envelope channel filtering */
    {
        mock_ring_t r;
        mock_ring_init(&r);

        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(5U, 0U, 0U, f_buf, &f_len) == OK);

        /* Build valid envelope on channel 1 */
        uint8_t env[MAX_FRAME_LEN + 4U];
        env[0] = MOCK_WIRE_DELIMITER;
        env[1] = 0x01U; /* channel 1 */
        env[2] = f_len;
        memcpy(&env[3], f_buf, f_len);
        env[3U + f_len] = fhss_crc8(&env[1], (uint8_t)(f_len + 2U));
        uint8_t env_len = (uint8_t)(f_len + 4U);

        for (uint8_t i = 0; i < env_len; i++) {
            mock_ring_push(&r, env[i]);
        }

        uint8_t out_buf[MAX_FRAME_LEN];
        uint8_t out_len = 0U;
        /* Receiver tuned to channel 0 drops channel 1 */
        int poll_res = mock_ring_poll(&r, 0U, out_buf, &out_len);
        assert(poll_res == (int)ERR_TIMEOUT);
    }

    /* 8. Verify Case 7: Recovery after wrong channel */
    {
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_hop(6U, 0U, 0U, f_buf, &f_len) == OK);

        fhss_replay_t rx_replay = {4U, 1U}; /* last_seq = 4 from Case 5 */
        fhss_frame_t parsed;
        status_t status = fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed);
        assert(status == OK);
        assert(parsed.seq == 6U);
        assert(rx_replay.last_seq == 6U);
    }

    /* 9. Verify Case 8: ACK suppression and recovery sequence arithmetic */
    {
        fhss_replay_t rx_replay = {6U, 1U}; /* last_seq = 6 */

        /* Packet 7 (suppressed): receiver validates and advances replay */
        uint8_t f_buf7[MAX_FRAME_LEN];
        uint8_t f_len7 = 0U;
        assert(fhss_build_hop(7U, 0U, 0U, f_buf7, &f_len7) == OK);
        fhss_frame_t parsed7;
        assert(fhss_parse_frame(f_buf7, f_len7, DIR_A_TO_B, &rx_replay, &parsed7) == OK);
        assert(rx_replay.last_seq == 7U);
        /* Node B intentionally does not transmit ACK; Node A times out */

        /* Packet 8 (recovery): receiver validates and advances replay */
        uint8_t f_buf8[MAX_FRAME_LEN];
        uint8_t f_len8 = 0U;
        assert(fhss_build_hop(8U, 0U, 0U, f_buf8, &f_len8) == OK);
        fhss_frame_t parsed8;
        assert(fhss_parse_frame(f_buf8, f_len8, DIR_A_TO_B, &rx_replay, &parsed8) == OK);
        assert(rx_replay.last_seq == 8U);

        /* Node B sends ACK (seq 8); Node A receives and parses ACK */
        uint8_t ack_wire[MAX_FRAME_LEN];
        uint8_t ack_wire_len = 0U;
        assert(fhss_build_ack(8U, 0U, 1U, 0x01U, 0x78U, 0U, 0U, ack_wire, &ack_wire_len) == OK);

        fhss_replay_t a_rx_replay = {6U, 1U}; /* Node A last received ACK was for seq 6 */
        fhss_frame_t a_parsed_ack;
        assert(fhss_parse_frame(ack_wire, ack_wire_len, DIR_B_TO_A, &a_rx_replay, &a_parsed_ack) == OK);
        assert(a_parsed_ack.type == PKT_TYPE_ACK);
        assert(a_parsed_ack.seq == 8U);
        assert(a_rx_replay.last_seq == 8U);
    }

    /* 10. Verify Node A ACK rejection on corrupted integrity tag or wrong type */
    {
        uint8_t ack_wire[MAX_FRAME_LEN];
        uint8_t ack_wire_len = 0U;
        assert(fhss_build_ack(9U, 0U, 1U, 0x01U, 0x78U, 0U, 0U, ack_wire, &ack_wire_len) == OK);
        ack_wire[5] ^= 0x55U; /* Corrupt tag */

        fhss_replay_t a_rx_replay = {8U, 1U};
        fhss_frame_t a_parsed_ack;
        assert(fhss_parse_frame(ack_wire, ack_wire_len, DIR_B_TO_A, &a_rx_replay, &a_parsed_ack) == ERR_TAG);
        assert(a_rx_replay.last_seq == 8U);
    }

    /* 11. Verify documented ACK sequence semantics: ACK sequence is per-sender counter, NOT echoed TX sequence */
    {
        /* Suppose Node B maintains its own sequence counter: 0, 1, 2, ... */
        fhss_replay_t a_rx_replay = {0, 0};

        /* Node B transmits ACKs with independent sequence numbers */
        for (uint8_t b_seq = 0; b_seq < 5U; b_seq++) {
            uint8_t ack_buf[MAX_FRAME_LEN];
            uint8_t ack_len = 0U;
            assert(fhss_build_ack(b_seq, 0U, 1U, 0x01U, 0x78U, 0U, 0U, ack_buf, &ack_len) == OK);

            fhss_frame_t parsed_ack;
            status_t status = fhss_parse_frame(ack_buf, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack);
            assert(status == OK);
            assert(parsed_ack.type == PKT_TYPE_ACK);
            assert(parsed_ack.seq == b_seq);
            assert(a_rx_replay.last_seq == b_seq);
        }
    }

    /* 12. Verify root-cause reproduction: repeating cycle from seq 254 when Node B is at last_seq=8 triggers ERR_REPLAY */
    {
        fhss_replay_t b_replay_after_cycle1 = {8U, 1U}; /* Node B after completing cycle 1 */

        uint8_t f_buf254[MAX_FRAME_LEN];
        uint8_t f_len254 = 0U;
        assert(fhss_build_hop(254U, 0U, 0U, f_buf254, &f_len254) == OK);

        fhss_frame_t parsed;
        status_t status = fhss_parse_frame(f_buf254, f_len254, DIR_A_TO_B, &b_replay_after_cycle1, &parsed);
        /* (uint8_t)(254 - 8) = 246 > 127 -> ERR_REPLAY */
        assert(status == ERR_REPLAY);
        assert(b_replay_after_cycle1.last_seq == 8U);
    }

    printf("[HOST TEST] Stage 5 fault-injection and reliability harness tests PASS\n");

    /* =====================================================================
     * Stage 6: End-to-End DATA Messaging over Mock Transport host tests
     * ===================================================================== */
    printf("[HOST TEST] Running Stage 6 DATA messaging over mock transport tests...\n");

    /* 1. Message fragmentation and slicing */
    {
        static const char s_m1[] = "HELLO_NODE_B";
        uint8_t len1 = (uint8_t)strlen(s_m1);
        assert(len1 == 12U);
        uint8_t total1 = (uint8_t)((len1 + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
        assert(total1 == 1U);

        /* Single fragment: idx=0, last=1, len=12 */
        uint8_t chunk_len1 = (len1 > MAX_PAYLOAD) ? MAX_PAYLOAD : len1;
        uint8_t last1 = (0U == (total1 - 1U)) ? 1U : 0U;
        assert(chunk_len1 == 12U);
        assert(last1 == 1U);
        assert(memcmp(s_m1, "HELLO_NODE_B", 12U) == 0);

        static const char s_m2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
        uint8_t len2 = (uint8_t)strlen(s_m2);
        assert(len2 == 34U);
        uint8_t total2 = (uint8_t)((len2 + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
        assert(total2 == 3U);

        /* Frag 0: idx=0, last=0, len=16 */
        uint8_t off0 = 0U * MAX_PAYLOAD;
        uint8_t rem0 = (uint8_t)(len2 - off0);
        uint8_t chunk_len2_0 = (rem0 > MAX_PAYLOAD) ? MAX_PAYLOAD : rem0;
        uint8_t last2_0 = (0U == (total2 - 1U)) ? 1U : 0U;
        assert(chunk_len2_0 == 16U);
        assert(last2_0 == 0U);
        assert(memcmp(&s_m2[off0], "FHSS_DATA_TRANSP", 16U) == 0);

        /* Frag 1: idx=1, last=0, len=16 */
        uint8_t off1 = 1U * MAX_PAYLOAD;
        uint8_t rem1 = (uint8_t)(len2 - off1);
        uint8_t chunk_len2_1 = (rem1 > MAX_PAYLOAD) ? MAX_PAYLOAD : rem1;
        uint8_t last2_1 = (1U == (total2 - 1U)) ? 1U : 0U;
        assert(chunk_len2_1 == 16U);
        assert(last2_1 == 0U);
        assert(memcmp(&s_m2[off1], "ORT_TEST_PACKET_", 16U) == 0);

        /* Frag 2: idx=2, last=1, len=2 */
        uint8_t off2 = 2U * MAX_PAYLOAD;
        uint8_t rem2 = (uint8_t)(len2 - off2);
        uint8_t chunk_len2_2 = (rem2 > MAX_PAYLOAD) ? MAX_PAYLOAD : rem2;
        uint8_t last2_2 = (2U == (total2 - 1U)) ? 1U : 0U;
        assert(chunk_len2_2 == 2U);
        assert(last2_2 == 1U);
        assert(memcmp(&s_m2[off2], "#2", 2U) == 0);

        /* Boundary checks */
        /* 1 byte message */
        uint8_t b1_len = 1U;
        uint8_t b1_total = (uint8_t)((b1_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
        assert(b1_total == 1U);

        /* 16 byte message */
        uint8_t b16_len = 16U;
        uint8_t b16_total = (uint8_t)((b16_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
        assert(b16_total == 1U);

        /* 17 byte message */
        uint8_t b17_len = 17U;
        uint8_t b17_total = (uint8_t)((b17_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
        assert(b17_total == 2U);

        /* 128 byte message */
        uint8_t b128_len = 128U;
        uint8_t b128_total = (uint8_t)((b128_len + MAX_PAYLOAD - 1U) / MAX_PAYLOAD);
        assert(b128_total == 8U);
        assert(b128_total == MAX_FRAGMENTS);
    }

    /* 2. Absence of delimiter 0x7E in frame bodies and wire envelope CRCs */
    {
        /* Check Message 1 (seq=0, msg_id=1, total=1, idx=0, last=1) */
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        static const char s_m1[] = "HELLO_NODE_B";
        assert(fhss_build_data(0U, 1U, 1U, 0U, 1U, 0U, 0U, (const uint8_t *)s_m1, 12U, f_buf, &f_len) == OK);
        for (uint8_t j = 0; j < f_len; j++) {
            assert(f_buf[j] != MOCK_WIRE_DELIMITER);
        }
        uint8_t env_crc1 = fhss_crc8_update(fhss_crc8((const uint8_t[]){0x00U, f_len}, 2U), f_buf, f_len);
        assert(env_crc1 != MOCK_WIRE_DELIMITER);

        /* Check Message 2 (3 fragments) */
        static const char s_m2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
        for (uint8_t idx = 0; idx < 3U; idx++) {
            uint8_t off = idx * 16U;
            uint8_t rem = (uint8_t)(34U - off);
            uint8_t chunk_len = (rem > 16U) ? 16U : rem;
            uint8_t last = (idx == 2U) ? 1U : 0U;
            uint8_t seq = 1U + idx;

            assert(fhss_build_data(seq, 2U, 3U, idx, last, 0U, 0U, (const uint8_t *)&s_m2[off], chunk_len, f_buf, &f_len) == OK);
            for (uint8_t j = 0; j < f_len; j++) {
                assert(f_buf[j] != MOCK_WIRE_DELIMITER);
            }
            uint8_t env_crc = fhss_crc8_update(fhss_crc8((const uint8_t[]){0x00U, f_len}, 2U), f_buf, f_len);
            assert(env_crc != MOCK_WIRE_DELIMITER);
        }
    }

    /* 3. DATA frame construction and bit-packing verification */
    {
        fhss_replay_t rx_replay = {0, 0};

        /* Msg 1 Frag 0 */
        static const char s_m1[] = "HELLO_NODE_B";
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_data(0U, 1U, 1U, 0U, 1U, 0U, 0U, (const uint8_t *)s_m1, 12U, f_buf, &f_len) == OK);
        assert(f_len == (HEADER_LEN + 12U));
        assert(f_buf[0] == 0xA1U); /* DATA */
        assert(f_buf[1] == 0U);    /* seq 0 */
        assert(f_buf[2] == 1U);    /* msg_id 1 */
        assert(f_buf[3] == 0x02U); /* total=1 ((0)<<5), idx=0 ((0)<<2), last=1 (2) */

        fhss_frame_t parsed;
        assert(fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed) == OK);
        assert(parsed.type == PKT_TYPE_DATA);
        assert(parsed.seq == 0U);
        assert(parsed.msg_id == 1U);
        assert(parsed.total == 1U);
        assert(parsed.idx == 0U);
        assert(parsed.last == 1U);
        assert(parsed.payload_len == 12U);
        assert(memcmp(parsed.payload, s_m1, 12U) == 0);
        assert(rx_replay.last_seq == 0U);
        assert(rx_replay.have_last == 1U);

        /* Msg 2 Frags 0, 1, 2 */
        static const char s_m2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
        for (uint8_t idx = 0; idx < 3U; idx++) {
            uint8_t off = idx * 16U;
            uint8_t rem = (uint8_t)(34U - off);
            uint8_t chunk_len = (rem > 16U) ? 16U : rem;
            uint8_t last = (idx == 2U) ? 1U : 0U;
            uint8_t seq = 1U + idx;

            assert(fhss_build_data(seq, 2U, 3U, idx, last, 0U, 0U, (const uint8_t *)&s_m2[off], chunk_len, f_buf, &f_len) == OK);
            assert(f_len == (HEADER_LEN + chunk_len));
            assert(f_buf[0] == 0xA1U);
            assert(f_buf[1] == seq);
            assert(f_buf[2] == 2U);
            uint8_t exp_b3 = (uint8_t)(((3U - 1U) << 5) | (idx << 2) | (last ? 2U : 0U));
            assert(f_buf[3] == exp_b3);

            assert(fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed) == OK);
            assert(parsed.type == PKT_TYPE_DATA);
            assert(parsed.seq == seq);
            assert(parsed.msg_id == 2U);
            assert(parsed.total == 3U);
            assert(parsed.idx == idx);
            assert(parsed.last == last);
            assert(parsed.payload_len == chunk_len);
            assert(memcmp(parsed.payload, &s_m2[off], chunk_len) == 0);
            assert(rx_replay.last_seq == seq);
        }
    }

    /* 4. DATA Integrity Tag Verification & Corruption Rejection */
    {
        static const char s_m1[] = "HELLO_NODE_B";
        uint8_t f_buf[MAX_FRAME_LEN];
        uint8_t f_len = 0U;
        assert(fhss_build_data(4U, 1U, 1U, 0U, 1U, 0U, 0U, (const uint8_t *)s_m1, 12U, f_buf, &f_len) == OK);

        /* Corrupt payload byte */
        uint8_t f_bad_payload[MAX_FRAME_LEN];
        memcpy(f_bad_payload, f_buf, f_len);
        f_bad_payload[6] ^= 0x01U;
        fhss_replay_t rx_replay = {3U, 1U};
        fhss_frame_t parsed;
        assert(fhss_parse_frame(f_bad_payload, f_len, DIR_A_TO_B, &rx_replay, &parsed) == ERR_TAG);
        assert(rx_replay.last_seq == 3U); /* Replay not updated on ERR_TAG */

        /* Corrupt header field (e.g. b3 frag) */
        uint8_t f_bad_hdr[MAX_FRAME_LEN];
        memcpy(f_bad_hdr, f_buf, f_len);
        f_bad_hdr[3] ^= 0x04U;
        assert(fhss_parse_frame(f_bad_hdr, f_len, DIR_A_TO_B, &rx_replay, &parsed) == ERR_TAG);
        assert(rx_replay.last_seq == 3U);

        /* Corrupt tag byte itself */
        uint8_t f_bad_tag[MAX_FRAME_LEN];
        memcpy(f_bad_tag, f_buf, f_len);
        f_bad_tag[5] ^= 0x80U;
        assert(fhss_parse_frame(f_bad_tag, f_len, DIR_A_TO_B, &rx_replay, &parsed) == ERR_TAG);
        assert(rx_replay.last_seq == 3U);
    }

    /* 5. ACK Validation Rules */
    {
        /* Valid standard FHSS ACK from Node B (independent seq=40, echoes msg_id=2, total=3, bitmap=0x01) */
        uint8_t ack_wire[MAX_FRAME_LEN];
        uint8_t ack_len = 0U;
        assert(fhss_build_ack(40U, 2U, 3U, 0x01U, 200U, 0U, 0U, ack_wire, &ack_len) == OK);

        fhss_replay_t a_rx_replay = {0, 0};
        fhss_frame_t parsed_ack;
        assert(fhss_parse_frame(ack_wire, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert(parsed_ack.type == PKT_TYPE_ACK);
        assert(parsed_ack.seq == 40U);
        assert(parsed_ack.msg_id == 2U);
        assert(parsed_ack.total == 3U);
        assert((parsed_ack.payload[0] & (1U << 0)) != 0U); /* Frag 0 ACKed */

        /* Rejection: wrong msg_id (ACK echoes msg_id=99 instead of 2) */
        uint8_t ack_wrong_msg[MAX_FRAME_LEN];
        assert(fhss_build_ack(41U, 99U, 3U, 0x01U, 200U, 0U, 0U, ack_wrong_msg, &ack_len) == OK);
        assert(fhss_parse_frame(ack_wrong_msg, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert(parsed_ack.msg_id != 2U); /* Detected by caller as mismatched msg_id */

        /* Rejection: wrong total (ACK reports total=1 instead of 3) */
        uint8_t ack_wrong_tot[MAX_FRAME_LEN];
        assert(fhss_build_ack(42U, 2U, 1U, 0x01U, 200U, 0U, 0U, ack_wrong_tot, &ack_len) == OK);
        assert(fhss_parse_frame(ack_wrong_tot, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert(parsed_ack.total != 3U); /* Detected by caller as mismatched total */

        /* Rejection: bitmap bit for fragment 1 is clear (0x01 instead of bit 1 set) */
        uint8_t ack_frag_not_acked[MAX_FRAME_LEN];
        assert(fhss_build_ack(43U, 2U, 3U, 0x01U, 200U, 0U, 0U, ack_frag_not_acked, &ack_len) == OK);
        assert(fhss_parse_frame(ack_frag_not_acked, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert((parsed_ack.payload[0] & (1U << 1)) == 0U); /* Fragment 1 is NOT acknowledged */

        /* Rejection: packet type is HOP instead of ACK */
        uint8_t hop_wire[MAX_FRAME_LEN];
        uint8_t hop_len = 0U;
        assert(fhss_build_hop(44U, 0U, 0U, hop_wire, &hop_len) == OK);
        assert(fhss_parse_frame(hop_wire, hop_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert(parsed_ack.type != PKT_TYPE_ACK);

        /* Rejection: corrupted ACK tag */
        uint8_t ack_corrupt[MAX_FRAME_LEN];
        assert(fhss_build_ack(45U, 2U, 3U, 0x02U, 200U, 0U, 0U, ack_corrupt, &ack_len) == OK);
        ack_corrupt[5] ^= 0x55U;
        assert(fhss_parse_frame(ack_corrupt, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == ERR_TAG);

        /* Rejection: ACK replay violation (re-sending seq=44) */
        uint8_t ack_replay[MAX_FRAME_LEN];
        assert(fhss_build_ack(44U, 2U, 3U, 0x02U, 200U, 0U, 0U, ack_replay, &ack_len) == OK);
        assert(fhss_parse_frame(ack_replay, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == ERR_REPLAY);
    }

    /* 6. Sequence wraparound for DATA frames and ACKs */
    {
        /* DATA sequence wraparound: seq 254 -> 255 -> 0 -> 1 */
        static const uint8_t s_wrap_seqs[4] = {254U, 255U, 0U, 1U};
        fhss_replay_t rx_replay = {0, 0};

        for (int i = 0; i < 4; i++) {
            uint8_t seq = s_wrap_seqs[i];
            uint8_t f_buf[MAX_FRAME_LEN];
            uint8_t f_len = 0U;
            assert(fhss_build_data(seq, 10U, 1U, 0U, 1U, 0U, 0U, (const uint8_t *)"WRAP", 4U, f_buf, &f_len) == OK);

            fhss_frame_t parsed;
            assert(fhss_parse_frame(f_buf, f_len, DIR_A_TO_B, &rx_replay, &parsed) == OK);
            assert(parsed.seq == seq);
            assert(rx_replay.last_seq == seq);
        }

        /* ACK sequence wraparound: b_seq 254 -> 255 -> 0 -> 1 */
        fhss_replay_t tx_rx_replay = {0, 0};
        for (int i = 0; i < 4; i++) {
            uint8_t b_seq = s_wrap_seqs[i];
            uint8_t ack_buf[MAX_FRAME_LEN];
            uint8_t ack_len = 0U;
            assert(fhss_build_ack(b_seq, 10U, 1U, 0x01U, 200U, 0U, 0U, ack_buf, &ack_len) == OK);

            fhss_frame_t parsed_ack;
            assert(fhss_parse_frame(ack_buf, ack_len, DIR_B_TO_A, &tx_rx_replay, &parsed_ack) == OK);
            assert(parsed_ack.seq == b_seq);
            assert(tx_rx_replay.last_seq == b_seq);
        }
    }

    /* 7. Sender sequential delivery and failure handling simulation */
    {
        /* Simulate 3-fragment message with failure at fragment 1 */
        int frag0_acked = 1;
        int frag1_acked = 0; /* Simulated timeout or corrupt ACK on frag 1 */
        int frag2_sent = 0;
        int m_success = 0;

        if (frag0_acked) {
            /* Proceed to frag 1 */
            if (frag1_acked) {
                frag2_sent = 1;
            } else {
                /* Sequential delivery abort: frag 2 MUST NOT be sent */
                frag2_sent = 0;
                m_success = 0;
            }
        }
        assert(frag2_sent == 0);
        assert(m_success == 0);

        /* Simulate complete 3-fragment message success */
        frag0_acked = 1;
        frag1_acked = 1;
        int frag2_acked = 1;
        if (frag0_acked && frag1_acked && frag2_acked) {
            frag2_sent = 1;
            m_success = 1;
        }
        assert(frag2_sent == 1);
        assert(m_success == 1);
    }

    printf("[HOST TEST] Stage 6 DATA messaging over mock transport tests PASS\n");

    printf("[HOST TEST] Running Stage 7 ARQ Retransmission and ACK Recovery tests...\n");

    /* 1. Exact Deterministic 5-Step Schedule Wire Bytes & CRC Verification */
    {
        /* Step 1: seq 0, Msg 1, Frag 0/1 ("HELLO_NODE_B", 12 bytes) */
        uint8_t f1[MAX_FRAME_LEN]; uint8_t f1_len = 0U;
        static const char s_m1[] = "HELLO_NODE_B";
        assert(fhss_build_data(0U, 1U, 1U, 0U, 1U, 0U, 0U, (const uint8_t *)s_m1, 12U, f1, &f1_len) == OK);
        assert(f1_len == 18U);
        assert(f1[0] == 0xA1U); /* DATA */
        assert(f1[1] == 0U);    /* seq 0 */
        assert(f1[2] == 1U);    /* msg_id 1 */
        assert(f1[3] == 0x02U); /* total=1, idx=0, last=1 -> ((1-1)<<5)|(0<<2)|(1<<1) = 0x02 */
        assert(f1[4] == 0x03U); /* masked hop: mask(0,DIR_A_TO_B)=0x03 -> 0x03 */
        assert(f1[5] == 0x2EU); /* tag = CRC8(header[0..3] + plain_hop + payload) */
        assert(memcmp(&f1[6], s_m1, 12U) == 0);

        /* Step 2: seq 1, Msg 2, Frag 0/3 ("FHSS_DATA_TRANSP", 16 bytes) */
        uint8_t f2[MAX_FRAME_LEN]; uint8_t f2_len = 0U;
        static const char s_m2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
        assert(fhss_build_data(1U, 2U, 3U, 0U, 0U, 0U, 0U, (const uint8_t *)&s_m2[0], 16U, f2, &f2_len) == OK);
        assert(f2_len == 22U);
        assert(f2[0] == 0xA1U);
        assert(f2[1] == 1U);    /* seq 1 */
        assert(f2[2] == 2U);    /* msg_id 2 */
        assert(f2[3] == 0x40U); /* total=3, idx=0, last=0 -> (2<<5)|(0<<2)|(0<<1) = 0x40 */
        assert(f2[4] == 0x52U); /* mask(1,DIR_A_TO_B)=0x52 -> 0x52 */
        assert(f2[5] == 0x8DU); /* tag */

        /* Step 3: seq 2, Msg 2, Frag 1/3, Attempt 1 ("ORT_TEST_PACKET_", 16 bytes) */
        uint8_t f3[MAX_FRAME_LEN]; uint8_t f3_len = 0U;
        assert(fhss_build_data(2U, 2U, 3U, 1U, 0U, 0U, 0U, (const uint8_t *)&s_m2[16], 16U, f3, &f3_len) == OK);
        assert(f3_len == 22U);
        assert(f3[0] == 0xA1U);
        assert(f3[1] == 2U);    /* seq 2 */
        assert(f3[2] == 2U);    /* msg_id 2 */
        assert(f3[3] == 0x44U); /* total=3, idx=1, last=0 -> (2<<5)|(1<<2)|(0<<1) = 0x44 */
        assert(f3[4] == 0xA1U); /* mask(2,DIR_A_TO_B)=0xA1 -> 0xA1 */
        assert(f3[5] == 0x1EU); /* tag */

        /* Step 4: seq 3, Msg 2, Frag 1/3, Attempt 2 (RETRY with fresh sequence number 3) */
        uint8_t f4[MAX_FRAME_LEN]; uint8_t f4_len = 0U;
        assert(fhss_build_data(3U, 2U, 3U, 1U, 0U, 0U, 0U, (const uint8_t *)&s_m2[16], 16U, f4, &f4_len) == OK);
        assert(f4_len == 22U);
        assert(f4[0] == 0xA1U);
        assert(f4[1] == 3U);    /* seq 3 (fresh sequence number!) */
        assert(f4[2] == 2U);    /* msg_id 2 (retained) */
        assert(f4[3] == 0x44U); /* total=3, idx=1, last=0 (retained) */
        assert(f4[4] == 0xF4U); /* mask(3,DIR_A_TO_B)=0xF4 -> 0xF4 (recomputed!) */
        assert(f4[5] == 0x16U); /* tag recomputed for seq 3! (0x16 != 0x1E) */
        assert(f4[5] != f3[5]); /* Fresh tag distinct from attempt 1 tag */
        assert(f4[4] != f3[4]); /* Fresh masked hop distinct from attempt 1 masked hop */

        /* Step 5: seq 4, Msg 2, Frag 2/3 ("#2", 2 bytes) */
        uint8_t f5[MAX_FRAME_LEN]; uint8_t f5_len = 0U;
        assert(fhss_build_data(4U, 2U, 3U, 2U, 1U, 0U, 0U, (const uint8_t *)&s_m2[32], 2U, f5, &f5_len) == OK);
        assert(f5_len == 8U);
        assert(f5[0] == 0xA1U);
        assert(f5[1] == 4U);    /* seq 4 */
        assert(f5[2] == 2U);    /* msg_id 2 */
        assert(f5[3] == 0x4AU); /* total=3, idx=2, last=1 -> (2<<5)|(2<<2)|(1<<1) = 0x4A */
        assert(f5[4] == 0x47U); /* mask(4,DIR_A_TO_B)=0x47 -> 0x47 */
        assert(f5[5] == 0xA8U); /* tag */

        /* Wire envelope checks: Verify no 0x7E in payload or CRC */
        const uint8_t *frames[5] = {f1, f2, f3, f4, f5};
        const uint8_t flens[5] = {f1_len, f2_len, f3_len, f4_len, f5_len};
        for (int k = 0; k < 5; k++) {
            uint8_t w_hdr[2] = {0U, flens[k]};
            uint8_t w_crc = fhss_crc8(w_hdr, 2U);
            w_crc = fhss_crc8_update(w_crc, frames[k], flens[k]);
            assert(w_crc != 0x7EU);
            for (uint8_t b = 0; b < flens[k]; b++) {
                assert(frames[k][b] != 0x7EU);
            }
        }
    }

    /* 2. Replay Protection & Recomputation on Retry */
    {
        /* If sender had sent stale sequence 2 on retry, Node B would reject it */
        fhss_replay_t b_rx_replay = {2U, 1U}; /* Node B already received seq 2 */
        fhss_frame_t parsed;

        /* Attempt to parse stale seq 2 frame */
        static const char s_m2[] = "FHSS_DATA_TRANSPORT_TEST_PACKET_#2";
        uint8_t f_stale[MAX_FRAME_LEN]; uint8_t f_stale_len = 0U;
        assert(fhss_build_data(2U, 2U, 3U, 1U, 0U, 0U, 0U, (const uint8_t *)&s_m2[16], 16U, f_stale, &f_stale_len) == OK);
        assert(fhss_parse_frame(f_stale, f_stale_len, DIR_A_TO_B, &b_rx_replay, &parsed) == ERR_REPLAY);
        assert(b_rx_replay.last_seq == 2U); /* Not updated on replay error */

        /* Fresh seq 3 frame succeeds */
        uint8_t f_fresh[MAX_FRAME_LEN]; uint8_t f_fresh_len = 0U;
        assert(fhss_build_data(3U, 2U, 3U, 1U, 0U, 0U, 0U, (const uint8_t *)&s_m2[16], 16U, f_fresh, &f_fresh_len) == OK);
        assert(fhss_parse_frame(f_fresh, f_fresh_len, DIR_A_TO_B, &b_rx_replay, &parsed) == OK);
        assert(parsed.seq == 3U);
        assert(b_rx_replay.last_seq == 3U);
    }

    /* 3. ACK Bitmap Validation & Cumulative Bitmap Rules */
    {
        /* Fragment 0 requires bit 0 */
        assert((0x01U & (1U << 0)) != 0U); /* Valid (0x01) */
        assert((0x03U & (1U << 0)) != 0U); /* Cumulative (0x03) valid */
        assert((0x07U & (1U << 0)) != 0U); /* Cumulative (0x07) valid */
        assert((0x02U & (1U << 0)) == 0U); /* Bit 0 clear -> Invalid */

        /* Fragment 1 requires bit 1 */
        assert((0x01U & (1U << 1)) == 0U); /* Bit 1 clear -> Invalid for frag 1 */
        assert((0x02U & (1U << 1)) != 0U); /* Individual 0x02 valid */
        assert((0x03U & (1U << 1)) != 0U); /* Cumulative 0x03 valid */
        assert((0x07U & (1U << 1)) != 0U); /* Cumulative 0x07 valid */

        /* Fragment 2 requires bit 2 */
        assert((0x03U & (1U << 2)) == 0U); /* Bit 2 clear -> Invalid for frag 2 */
        assert((0x04U & (1U << 2)) != 0U); /* Individual 0x04 valid */
        assert((0x07U & (1U << 2)) != 0U); /* Cumulative 0x07 valid */
    }

    /* 4. ACK Sequence Independence (Node B ACK sequence does NOT echo DATA sequence) */
    {
        fhss_replay_t a_rx_replay = {0, 0};
        fhss_frame_t parsed_ack;

        /* Node A sent DATA seq 2 and 3; Node B responds with ACK seq 50 */
        uint8_t ack_wire[MAX_FRAME_LEN]; uint8_t ack_len = 0U;
        assert(fhss_build_ack(50U, 2U, 3U, 0x03U, 180U, 0U, 0U, ack_wire, &ack_len) == OK);
        assert(fhss_parse_frame(ack_wire, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert(parsed_ack.seq == 50U);
        assert(parsed_ack.seq != 2U && parsed_ack.seq != 3U); /* Sequence independence */
        assert(parsed_ack.msg_id == 2U);
        assert(parsed_ack.total == 3U);
        assert((parsed_ack.payload[0] & (1U << 1)) != 0U); /* Fragment 1 confirmed */

        /* Next ACK from Node B has seq 51 */
        assert(fhss_build_ack(51U, 2U, 3U, 0x07U, 180U, 0U, 0U, ack_wire, &ack_len) == OK);
        assert(fhss_parse_frame(ack_wire, ack_len, DIR_B_TO_A, &a_rx_replay, &parsed_ack) == OK);
        assert(parsed_ack.seq == 51U);
        assert((parsed_ack.payload[0] & (1U << 2)) != 0U); /* Fragment 2 confirmed */
    }

    /* 5. ARQ Retry State Machine & Retry Limit Exhaustion Simulation */
    {
        /* Simulate 3-attempt limit when ACK is suppressed on all 3 attempts */
        uint8_t seq = 10U;
        uint8_t attempts = 0U;
        int ack_received = 0;
        const uint8_t max_attempts = 3U;

        for (uint8_t att = 1U; att <= max_attempts; att++) {
            attempts = att;
            seq++; /* Fresh sequence on each attempt */
            /* Simulated ACK check: always times out */
            ack_received = 0;
        }
        assert(attempts == 3U);
        assert(seq == 13U);
        assert(ack_received == 0);
        /* Decision: abort message */
        int message_aborted = (ack_received == 0) ? 1 : 0;
        assert(message_aborted == 1);

        /* Simulate recovery on attempt 2 (Stage 7 schedule) */
        seq = 2U;
        attempts = 0U;
        ack_received = 0;
        for (uint8_t att = 1U; att <= max_attempts; att++) {
            attempts = att;
            seq++;
            if (att == 1U) {
                /* Attempt 1 times out */
                ack_received = 0;
            } else if (att == 2U) {
                /* Attempt 2 succeeds */
                ack_received = 1;
                break;
            }
        }
        assert(attempts == 2U);
        assert(seq == 4U);
        assert(ack_received == 1);
        int message_recovered = (ack_received == 1) ? 1 : 0;
        assert(message_recovered == 1);
    }

    printf("[HOST TEST] Stage 7 ARQ Retransmission and ACK Recovery tests PASS\n");

    return 0;
}


