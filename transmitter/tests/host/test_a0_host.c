#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "status.h"
#include "config.h"
#include "log.h"
#include "mock_hal.h"
#include "sx1278_fsk.h"

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
    Log_Test("A0-01", 1);
    log = mock_hal_get_uart_log();
    assert(strcmp(log, "[TEST] A0-01 PASS\r\n") == 0);

    mock_hal_reset();
    Log_Test("A0-02", 0);
    log = mock_hal_get_uart_log();
    assert(strcmp(log, "[TEST] A0-02 FAIL\r\n") == 0);

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

int main(void) {
    printf("--- Running Node A Host Tests (Step A0 & A2) ---\n");
    test_status_strings();
    test_channel_plan();
    test_log_formatter();
    test_hal_null_safety();
    test_sx1278_spi();
    printf("--- All Host Tests Passed ---\n");
    return 0;
}
