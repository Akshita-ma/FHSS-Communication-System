#include "air_usart2.h"
#include "stm32f4_regs.h"
#include "status.h"
#include "config.h"
#include "fhss_protocol.h"
#include "hal.h"
#include <stddef.h>

#if defined(DEMO_RF_KEYING)
#include "sx1278_fsk.h"
#endif

#define MOCK_RING_SIZE 256
static volatile uint8_t  g_usart2_ring[MOCK_RING_SIZE];
static volatile uint16_t g_usart2_head = 0;
static volatile uint16_t g_usart2_tail = 0;

static uint32_t g_mock_rng = 0x98765432UL;
static uint32_t g_first_locked_ms = 0;

static uint32_t mock_rand(void) {
    uint32_t x = g_mock_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_mock_rng = x;
    return x;
}

void air_set_locked_timestamp(uint32_t t_ms) {
    if (g_first_locked_ms == 0) {
        g_first_locked_ms = t_ms;
    }
}

uint8_t air_is_channel_jammed(uint8_t ch) {
    if (g_first_locked_ms == 0) {
        return 0;
    }
    uint32_t now = millis();
    uint32_t elapsed = now - g_first_locked_ms;
    if (elapsed >= DEMO_JAM_START_MS && elapsed < DEMO_JAM_END_MS) {
        if ((DEMO_JAM_MASK & (1UL << ch)) != 0) {
            return 1;
        }
    }
    return 0;
}

void air_usart2_init(void) {
    /* 1. Clocks: Enable GPIOA in AHB1, USART2 in APB1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* 2. Configure PA2 (TX) and PA3 (RX): AF7, High speed, Pull-up on RX */
    GPIOA->MODER &= ~((3UL << (2 * 2)) | (3UL << (3 * 2)));
    GPIOA->MODER |=  ((2UL << (2 * 2)) | (2UL << (3 * 2)));   /* Alternate function */
    GPIOA->OSPEEDR |= ((3UL << (2 * 2)) | (3UL << (3 * 2)));  /* High speed */
    GPIOA->PUPDR &= ~((3UL << (2 * 2)) | (3UL << (3 * 2)));
    GPIOA->PUPDR |=  (1UL << (3 * 2));                         /* Pull-up on PA3 RX */

    GPIOA->AFR[0] &= ~((0xFUL << (2 * 4)) | (0xFUL << (3 * 4)));
    GPIOA->AFR[0] |=  ((0x7UL << (2 * 4)) | (0x7UL << (3 * 4))); /* AF7 = USART2 */

    /* 3. Configure USART2: 115200 8N1 @ 16 MHz APB1 (BRR = 0x008B) */
    USART2->BRR = 0x008B;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* 4. Enable NVIC IRQ 38 for USART2 */
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
}

void air_trace_pins_init(void) {
    /* Enable GPIOA and GPIOB clocks */
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);

    /* Configure trace pins on GPIOB:
     * PB1  : Hop toggle
     * PB5  : Locked state
     * PB10 : TX active
     * PB12 : ACK pulse
     * PB13, PB14, PB15 : Channel number bits 0, 1, 2
     */
    GPIOB->MODER &= ~( (3UL << (1 * 2)) | (3UL << (5 * 2)) | (3UL << (10 * 2)) |
                       (3UL << (12 * 2)) | (3UL << (13 * 2)) | (3UL << (14 * 2)) | (3UL << (15 * 2)) );
    GPIOB->MODER |=  ( (1UL << (1 * 2)) | (1UL << (5 * 2)) | (1UL << (10 * 2)) |
                       (1UL << (12 * 2)) | (1UL << (13 * 2)) | (1UL << (14 * 2)) | (1UL << (15 * 2)) );

    GPIOB->OSPEEDR |= ( (3UL << (1 * 2)) | (3UL << (5 * 2)) | (3UL << (10 * 2)) |
                        (3UL << (12 * 2)) | (3UL << (13 * 2)) | (3UL << (14 * 2)) | (3UL << (15 * 2)) );

    /* Configure PA8: Jam status */
    GPIOA->MODER &= ~(3UL << (8 * 2));
    GPIOA->MODER |=  (1UL << (8 * 2));
    GPIOA->OSPEEDR |= (3UL << (8 * 2));

    /* Reset all trace outputs to low */
    GPIOB->BSRR = (1UL << (1 + 16)) | (1UL << (5 + 16)) | (1UL << (10 + 16)) |
                  (1UL << (12 + 16)) | (7UL << (13 + 16));
    GPIOA->BSRR = (1UL << (8 + 16));
}

static volatile uint32_t g_usart2_ore_count = 0;
static volatile uint32_t g_usart2_overflow_count = 0;

void USART2_IRQHandler(void) {
    uint32_t sr = USART2->SR;
    if (sr & USART_SR_ORE) {
        /* Reading SR followed by DR clears ORE and prevents interrupt storm */
        (void)USART2->DR;
        g_usart2_ore_count++;
    }
    if (sr & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(USART2->DR);
        uint16_t next = (uint16_t)((g_usart2_head + 1) & (MOCK_RING_SIZE - 1));
        if (next != g_usart2_tail) {
            g_usart2_ring[g_usart2_head] = byte;
            g_usart2_head = next;
        } else {
            g_usart2_overflow_count++;
        }
    }
}

void air_usart2_write_byte(uint8_t b) {
    uint32_t timeout = 200000UL;
    while (!(USART2->SR & USART_SR_TXE)) {
        if (--timeout == 0) return;
    }
    USART2->DR = b;
    timeout = 200000UL;
    while (!(USART2->SR & USART_SR_TC)) {
        if (--timeout == 0) return;
    }
}

static void usart2_write_byte(uint8_t b) {
    air_usart2_write_byte(b);
}

uint16_t air_usart2_rx_available(void) {
    uint16_t head = g_usart2_head;
    uint16_t tail = g_usart2_tail;
    return (uint16_t)((head - tail) & (MOCK_RING_SIZE - 1));
}

int air_usart2_rx_peek(uint16_t offset, uint8_t *b) {
    if (b == NULL) return (int)ERR_NULL;
    uint16_t head = g_usart2_head;
    uint16_t tail = g_usart2_tail;
    uint16_t available = (uint16_t)((head - tail) & (MOCK_RING_SIZE - 1));
    if (offset >= available) {
        return (int)ERR_RANGE;
    }
    *b = g_usart2_ring[(tail + offset) & (MOCK_RING_SIZE - 1)];
    return (int)OK;
}

void air_usart2_rx_advance(uint16_t count) {
    uint16_t head = g_usart2_head;
    uint16_t tail = g_usart2_tail;
    uint16_t available = (uint16_t)((head - tail) & (MOCK_RING_SIZE - 1));
    if (count > available) {
        count = available;
    }
    g_usart2_tail = (uint16_t)((tail + count) & (MOCK_RING_SIZE - 1));
}

uint32_t air_usart2_get_ore_count(void) {
    return g_usart2_ore_count;
}

uint32_t air_usart2_get_overflow_count(void) {
    return g_usart2_overflow_count;
}

void air_usart2_clear_errors(void) {
    g_usart2_ore_count = 0;
    g_usart2_overflow_count = 0;
}

uint32_t air_airtime_us(uint8_t len) {
    /* (5 preamble + 2 sync + 1 len + len + 2 crc bytes) * 160 us */
    return (uint32_t)(10 + len) * 160UL;
}

int air_tx(uint8_t ch, const uint8_t *frame, uint8_t len) {
    if (frame == NULL || len == 0 || len > MAX_FRAME_LEN) {
        return (int)ERR_LEN;
    }

#if defined(DEMO_TRACE)
    /* PB10 high while transmitting frame */
    GPIOB->BSRR = (1UL << 10);
#endif

    uint8_t crc = crc8_smbus_cont(&ch, 1, CRC8_SMBUS_INIT);
    crc = crc8_smbus_cont(&len, 1, crc);
    crc = crc8_smbus_cont(frame, len, crc);

    usart2_write_byte(0x7E);
    usart2_write_byte(ch);
    usart2_write_byte(len);
    for (uint8_t i = 0; i < len; i++) {
        usart2_write_byte(frame[i]);
    }
    usart2_write_byte(crc);

#if defined(DEMO_RF_KEYING)
    /* Write frame into radio FIFO and set RegOpMode to FSK transmit */
    NSS_LOW();
    delay_us(2);
    SPI1_Transfer(0x00 | 0x80); /* REG_FIFO */
    SPI1_Transfer(len);
    for (uint8_t i = 0; i < len; i++) {
        SPI1_Transfer(frame[i]);
    }
    delay_us(2);
    NSS_HIGH();
    delay_us(2);
    sx1278_write_reg(0x01, 0x03); /* FSK TX */
#endif

    /* Wait for frame airtime */
    uint32_t airtime_us = air_airtime_us(len);
    delay_us(airtime_us);

#if defined(DEMO_RF_KEYING)
    sx1278_write_reg(0x01, 0x01); /* Return to FSK standby */
#endif

#if defined(DEMO_TRACE)
    GPIOB->BSRR = (1UL << (10 + 16)); /* PB10 low */
#endif

    return (int)OK;
}

int air_rx_poll(uint8_t ch, uint8_t *buf, uint8_t *len) {
    if (buf == NULL || len == NULL) {
        return (int)ERR_NULL;
    }

    uint16_t head = g_usart2_head;
    uint16_t tail = g_usart2_tail;

    while (tail != head) {
        /* Search for sync byte 0x7E */
        if (g_usart2_ring[tail] != 0x7E) {
            tail = (uint16_t)((tail + 1) & (MOCK_RING_SIZE - 1));
            continue;
        }

        uint16_t available = (uint16_t)((head - tail) & (MOCK_RING_SIZE - 1));
        if (available < 4) {
            /* Need at least 0x7E, ch, len, crc */
            g_usart2_tail = tail;
            return (int)ERR_TIMEOUT;
        }

        uint8_t pkt_ch  = g_usart2_ring[(tail + 1) & (MOCK_RING_SIZE - 1)];
        uint8_t pkt_len = g_usart2_ring[(tail + 2) & (MOCK_RING_SIZE - 1)];

        if (pkt_len < HEADER_LEN || pkt_len > MAX_FRAME_LEN) {
            /* Corrupted header: discard sync byte and continue */
            tail = (uint16_t)((tail + 1) & (MOCK_RING_SIZE - 1));
            continue;
        }

        uint16_t total_wire_len = (uint16_t)(1 + 1 + 1 + pkt_len + 1);
        if (available < total_wire_len) {
            /* Whole frame not yet arrived */
            g_usart2_tail = tail;
            return (int)ERR_TIMEOUT;
        }

        /* Calculate CRC over pkt_ch, pkt_len, and frame payload */
        uint8_t calc_crc = crc8_smbus_cont(&pkt_ch, 1, CRC8_SMBUS_INIT);
        calc_crc = crc8_smbus_cont(&pkt_len, 1, calc_crc);

        uint8_t temp_frame[MAX_FRAME_LEN];
        for (uint8_t i = 0; i < pkt_len; i++) {
            temp_frame[i] = g_usart2_ring[(tail + 3 + i) & (MOCK_RING_SIZE - 1)];
        }
        calc_crc = crc8_smbus_cont(temp_frame, pkt_len, calc_crc);

        uint8_t wire_crc = g_usart2_ring[(tail + 3 + pkt_len) & (MOCK_RING_SIZE - 1)];

        /* Advance tail past this complete frame */
        tail = (uint16_t)((tail + total_wire_len) & (MOCK_RING_SIZE - 1));
        g_usart2_tail = tail;

        if (calc_crc != wire_crc) {
            /* Wire CRC failed: frame dropped */
            continue;
        }

        /* Delivery rule: channel must match current receiver channel */
        if (pkt_ch != ch) {
            continue;
        }

        /* Mock channel model */
        uint8_t jammed = air_is_channel_jammed(ch);
        if (jammed) {
            /* 97% loss on jammed channel */
            if ((mock_rand() % 100) < 97) {
                return (int)ERR_TIMEOUT;
            }
            /* 15% of frames that survive get one random bit flipped */
            if ((mock_rand() % 100) < 15 && pkt_len > 0) {
                uint8_t byte_idx = (uint8_t)(mock_rand() % pkt_len);
                uint8_t bit_idx  = (uint8_t)(mock_rand() % 8);
                temp_frame[byte_idx] ^= (uint8_t)(1 << bit_idx);
            }
        } else {
            /* 3% baseline loss on clean channel */
            if ((mock_rand() % 100) < 3) {
                return (int)ERR_TIMEOUT;
            }
        }

        /* Deliver frame */
        for (uint8_t i = 0; i < pkt_len; i++) {
            buf[i] = temp_frame[i];
        }
        *len = pkt_len;
        return (int)OK;
    }

    g_usart2_tail = tail;
    return (int)ERR_TIMEOUT;
}

uint8_t air_rssi_raw(uint8_t ch) {
    if (air_is_channel_jammed(ch)) {
        /* Jammed channel: about -40 dBm -> raw = -2 * (-40) = 80 */
        int32_t jitter = (int32_t)(mock_rand() % 5) - 2;
        int32_t raw = 80 + jitter;
        if (raw < 0) raw = 0;
        if (raw > 255) raw = 255;
        return (uint8_t)raw;
    } else {
        /* Clean channel: about -100 dBm +/- 3 -> raw = 200 +/- 6 */
        int32_t jitter = (int32_t)(mock_rand() % 7) - 3;
        int32_t raw = 200 + (jitter * 2);
        if (raw < 0) raw = 0;
        if (raw > 255) raw = 255;
        return (uint8_t)raw;
    }
}
