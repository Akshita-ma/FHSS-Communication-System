#include "mock_hal.h"
#include <string.h>

#define UART_BUF_SIZE 4096

static char s_uart_buf[UART_BUF_SIZE];
static size_t s_uart_buf_len = 0;
static uint8_t s_nss_state = 1;
static uint8_t s_reset_pin_state = 1;
static uint32_t s_mock_millis = 0;
static void (*s_tim2_cb)(void) = NULL;
static void (*s_exti0_cb)(void) = NULL;

static uint8_t s_mock_regs[128];
static uint8_t s_spi_cmd = 0;
static uint8_t s_spi_byte_idx = 0;

void mock_hal_reset(void) {
    s_uart_buf[0] = '\0';
    s_uart_buf_len = 0;
    s_nss_state = 1;
    s_reset_pin_state = 1;
    s_mock_millis = 0;
    s_tim2_cb = NULL;
    s_exti0_cb = NULL;
    memset(s_mock_regs, 0, sizeof(s_mock_regs));
    s_mock_regs[0x42] = 0x12; /* RegVersion default */
    s_spi_cmd = 0;
    s_spi_byte_idx = 0;
}

const char *mock_hal_get_uart_log(void) {
    return s_uart_buf;
}

size_t mock_hal_get_uart_log_len(void) {
    return s_uart_buf_len;
}

uint8_t mock_hal_get_nss_state(void) {
    return s_nss_state;
}

uint8_t mock_hal_get_reset_pin_state(void) {
    return s_reset_pin_state;
}

void mock_hal_set_reg(uint8_t addr, uint8_t val) {
    if (addr < 128) {
        s_mock_regs[addr] = val;
    }
}

uint8_t mock_hal_get_reg(uint8_t addr) {
    return (addr < 128) ? s_mock_regs[addr] : 0;
}

/* FROZEN HAL implementations for host testing */
uint8_t SPI1_Transfer(uint8_t tx) {
    if (s_spi_byte_idx == 0) {
        s_spi_cmd = tx;
        s_spi_byte_idx++;
        return 0x00;
    } else {
        uint8_t addr = (uint8_t)(s_spi_cmd & 0x7F);
        if (s_spi_cmd & 0x80) {
            /* Write */
            s_mock_regs[addr] = tx;
            return 0x00;
        } else {
            /* Read */
            return s_mock_regs[addr];
        }
    }
}

void NSS_LOW(void) {
    s_nss_state = 0;
    s_spi_byte_idx = 0;
}

void NSS_HIGH(void) {
    s_nss_state = 1;
    s_spi_byte_idx = 0;
}

int TIM2_ScheduleHopMs(uint32_t ms, void (*cb)(void)) {
    if (ms < 1 || ms > 60000 || cb == NULL) {
        return (int)ERR_RANGE;
    }
    s_tim2_cb = cb;
    return (int)OK;
}

void UART1_SendString(const char *s) {
    if (s == NULL) {
        return;
    }
    size_t len = strlen(s);
    if (s_uart_buf_len + len < UART_BUF_SIZE - 1) {
        memcpy(&s_uart_buf[s_uart_buf_len], s, len);
        s_uart_buf_len += len;
        s_uart_buf[s_uart_buf_len] = '\0';
    }
}

void Radio_Reset_Pin(uint8_t state) {
    s_reset_pin_state = state ? 1 : 0;
}

uint32_t millis(void) {
    return s_mock_millis;
}

void delay_ms(uint32_t ms) {
    s_mock_millis += ms;
}

void delay_us(uint32_t us) {
    (void)us;
}

status_t EXTI0_RegisterCallback(void (*cb)(void)) {
    if (cb == NULL) {
        return ERR_NULL;
    }
    s_exti0_cb = cb;
    return OK;
}
