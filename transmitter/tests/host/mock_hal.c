#include <stdio.h>
#include "mock_hal.h"

static int g_nss_state = 1;
static int g_reset_state = 1;
static uint32_t g_mock_millis = 0;
static uint32_t g_mock_micros = 0;
static void (*g_hop_callback)(void) = (void (*)(void))0;
static void (*g_exti0_callback)(void) = (void (*)(void))0;

static uint8_t g_mock_registers[256];
static int     g_spi_phase = 0;
static uint8_t g_spi_addr = 0;
static int     g_spi_is_write = 0;

void mock_hal_reset(void) {
    g_nss_state = 1;
    g_reset_state = 1;
    g_mock_millis = 0;
    g_mock_micros = 0;
    g_hop_callback = (void (*)(void))0;
    g_exti0_callback = (void (*)(void))0;
    g_spi_phase = 0;
    mock_hal_clear_uart();

    for (int i = 0; i < 256; i++) {
        g_mock_registers[i] = 0x00U;
    }
    g_mock_registers[0x42] = 0x12U; /* RegVersion */
    g_mock_registers[0x26] = 0x03U; /* RegPreambleLsb */
}

void mock_hal_set_reg(uint8_t addr, uint8_t val) {
    g_mock_registers[addr] = val;
}

uint8_t mock_hal_get_reg(uint8_t addr) {
    return g_mock_registers[addr];
}

void mock_hal_set_millis(uint32_t ms) {
    g_mock_millis = ms;
}

void mock_hal_set_micros(uint32_t us) {
    g_mock_micros = us;
}

int mock_hal_get_nss_state(void) {
    return g_nss_state;
}

int mock_hal_get_reset_state(void) {
    return g_reset_state;
}

uint8_t SPI1_Transfer(uint8_t tx) {
    if (g_nss_state != 0) {
        return 0xFFU;
    }

    if (g_spi_phase == 0) {
        g_spi_is_write = ((tx & 0x80U) != 0U);
        g_spi_addr = tx & 0x7FU;
        g_spi_phase = 1;
        return 0x00U;
    } else {
        if (g_spi_is_write) {
            g_mock_registers[g_spi_addr] = tx;
            return 0x00U;
        } else {
            return g_mock_registers[g_spi_addr];
        }
    }
}

void NSS_LOW(void) {
    g_nss_state = 0;
    g_spi_phase = 0;
}

void NSS_HIGH(void) {
    g_nss_state = 1;
    g_spi_phase = 0;
}

int TIM2_ScheduleHopMs(uint32_t ms, void (*cb)(void)) {
    if (ms < 1U || ms > 60000U) {
        return (int)ERR_RANGE;
    }
    if (cb == (void (*)(void))0) {
        return (int)ERR_NULL;
    }
    g_hop_callback = cb;
    (void)g_hop_callback;
    return (int)OK;
}

void TIM2_Poll(void) {
}

static char g_uart_buf[2048];
static uint32_t g_uart_bytes = 0;

const char *mock_hal_get_uart_buffer(void) {
    return g_uart_buf;
}

uint32_t mock_hal_get_uart_bytes(void) {
    return g_uart_bytes;
}

void mock_hal_clear_uart(void) {
    g_uart_bytes = 0;
    g_uart_buf[0] = '\0';
}

void UART1_SendString(const char *s) {
    if (s == (const char *)0 || *s == '\0') {
        return;
    }
    while (*s != '\0' && g_uart_bytes < (sizeof(g_uart_buf) - 1U)) {
        g_uart_buf[g_uart_bytes++] = *s++;
    }
    g_uart_buf[g_uart_bytes] = '\0';
}

void RADIO_RESET_LOW(void) {
    g_reset_state = 0;
}

void RADIO_RESET_HIGH(void) {
    g_reset_state = 1;
}

uint32_t millis(void) {
    return g_mock_millis;
}

void delay_ms(uint32_t ms) {
    g_mock_millis += ms;
}

status_t EXTI0_RegisterCallback(void (*cb)(void)) {
    if (cb == (void (*)(void))0) {
        return ERR_NULL;
    }
    g_exti0_callback = cb;
    (void)g_exti0_callback;
    return OK;
}

uint32_t micros(void) {
    return g_mock_micros;
}
