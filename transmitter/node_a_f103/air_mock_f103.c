#include "common/air.h"
#include "common/config.h"
#include "common/hal.h"
#include "common/status.h"
#include "common/fhss_protocol.h"
#include "stm32f1_regs.h"
#include "air_mock.h"

#if defined(DEMO_USART_LINK) || defined(STEP1_UART_TEST) || defined(STEP2_MOCK_TEST) || defined(STEP3_FHSS_MOCK) || defined(STEP5_FAULT_TEST)

/*
 * USART2 Mock Air Link for Node A (STM32F103)
 * Wire framing: 0x7E, ch, len, [frame bytes], crc8 (poly 0x07 over ch,len,frame bytes)
 * USART2: 115200 8N1 on PA2 (TX AF PP) and PA3 (RX Input Floating)
 */

#define WIRE_DELIMITER 0x7EU

/* Diagnostic counters for Stage 3.1 analysis */
static air_mock_diag_t s_mock_diag = {0U, 0U, 0U, 0U, 0U};

void air_mock_get_diag(air_mock_diag_t *diag) {
    if (diag != (air_mock_diag_t *)0) {
        diag->channel_mismatch_count = s_mock_diag.channel_mismatch_count;
        diag->crc_error_count = s_mock_diag.crc_error_count;
        diag->last_mismatch_wire_ch = s_mock_diag.last_mismatch_wire_ch;
        diag->last_mismatch_tuned_ch = s_mock_diag.last_mismatch_tuned_ch;
        diag->last_mismatch_len = s_mock_diag.last_mismatch_len;
    }
}

void air_mock_clear_diag(void) {
    s_mock_diag.channel_mismatch_count = 0U;
    s_mock_diag.crc_error_count = 0U;
    s_mock_diag.last_mismatch_wire_ch = 0U;
    s_mock_diag.last_mismatch_tuned_ch = 0U;
    s_mock_diag.last_mismatch_len = 0U;
}

/* Ring buffer for USART2 RX */
static volatile uint8_t  s_rx_ring[256];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/*
 * USART2 RXNE interrupt: pushes bytes into ring buffer.
 * Conforms to Hard Rule 5: ISR only pushes byte into volatile ring buffer.
 * Reads DR on RXNE or ORE to ensure overrun flag is always cleared.
 */
void USART2_IRQHandler(void) {
    uint32_t sr = USART2->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        uint8_t byte = (uint8_t)(USART2->DR & 0xFFU);
        if ((sr & USART_SR_RXNE) != 0U) {
            uint16_t next = (s_rx_head + 1U) & 0xFFU;
            if (next != s_rx_tail) {
                s_rx_ring[s_rx_head] = byte;
                s_rx_head = next;
            }
        }
    }
}

/*
 * Initialize USART2 on PA2 (TX) and PA3 (RX) for Node A (F103)
 */
void air_mock_init(void) {
    /* Initialize ring buffer pointers and diagnostic counters */
    s_rx_head = 0U;
    s_rx_tail = 0U;
    air_mock_clear_diag();

    /* Enable USART2 clock in RCC->APB1ENR bit 17 */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /*
     * Configure GPIOA:
     * PA2: Alternate Function Push-Pull 50 MHz (MODE=11, CNF=10 -> 0xB in CRL bits [11:8])
     * PA3: Input Floating (MODE=00, CNF=01 -> 0x4 in CRL bits [15:12])
     */
    GPIOA->CRL &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOA->CRL |=  ((0xBUL << 8) | (0x4UL << 12));

    /* USART2 BRR = 0x0045 (115200 baud @ 8 MHz APB1) */
    USART2->BRR = 0x0045UL;

    /* Clear any pending status flags and data */
    (void)USART2->SR;
    (void)USART2->DR;

    /* Enable USART2, Transmitter, Receiver, and RXNE Interrupt */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* Enable USART2 IRQ 38 in NVIC ISER[1] bit 6 (38 - 32) */
    NVIC->ISER[1] = (1UL << 6);
}

void air_mock_flush(void) {
    s_rx_tail = s_rx_head;
}

int air_mock_read_byte(uint8_t *byte) {
    if (byte == (uint8_t *)0) {
        return (int)ERR_NULL;
    }
    if (s_rx_head == s_rx_tail) {
        return (int)ERR_TIMEOUT;
    }
    *byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1U) & 0xFFU;
    return (int)OK;
}

int air_mock_send_raw(const uint8_t *data, uint8_t len) {
    if (data == (const uint8_t *)0) {
        return (int)ERR_NULL;
    }
    for (uint8_t i = 0; i < len; i++) {
        uint32_t timeout = 100000UL;
        while ((USART2->SR & USART_SR_TXE) == 0U) {
            if (--timeout == 0U) {
                return (int)ERR_TIMEOUT;
            }
        }
        USART2->DR = data[i];
    }
    uint32_t tc_timeout = 100000UL;
    while ((USART2->SR & USART_SR_TC) == 0U) {
        if (--tc_timeout == 0U) {
            return (int)ERR_TIMEOUT;
        }
    }
    return (int)OK;
}

#ifdef DEMO_USART_LINK

/* Jam state tracking */
static uint32_t s_first_locked_ms = 0U;
static uint8_t  s_has_locked = 0U;

/* Local xorshift32 PRNG for channel model randomness */
static uint32_t s_xorshift_state = 0x1A2B3C4DUL;

static uint32_t mock_rand32(void) {
    uint32_t x = s_xorshift_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_xorshift_state = x;
    return x;
}

void mock_air_notify_locked(void) {
    if (s_has_locked == 0U) {
        s_has_locked = 1U;
        s_first_locked_ms = millis();
    }
}

static int mock_is_jammed(uint8_t ch) {
    if (s_has_locked == 0U) {
        return 0;
    }
    if ((DEMO_JAM_MASK & (1U << ch)) == 0U) {
        return 0;
    }
    uint32_t elapsed = millis() - s_first_locked_ms;
    if (elapsed >= DEMO_JAM_START_MS && elapsed < DEMO_JAM_END_MS) {
        return 1;
    }
    return 0;
}

uint32_t air_airtime_us(uint8_t len) {
    /* (5 preamble + 2 sync + 1 chip len + len frame + 2 chip crc) * 160 us */
    return (10UL + (uint32_t)len) * 160UL;
}

int air_tx(uint8_t ch, const uint8_t *frame, uint8_t len) {
    if (frame == (const uint8_t *)0) {
        return (int)ERR_NULL;
    }
    if (len == 0U || len > MAX_FRAME_LEN) {
        return (int)ERR_LEN;
    }
    if (ch >= NUM_CHANNELS) {
        return (int)ERR_RANGE;
    }

#ifdef DEMO_TRACE
    /* Drive PB10 HIGH while transmitting */
    GPIOB->BSRR = (1UL << 10);
#endif

    uint32_t start_us = micros();

    /* Calculate wire CRC-8 over ch, len, and frame bytes */
    uint8_t wire_hdr[2];
    wire_hdr[0] = ch;
    wire_hdr[1] = len;
    uint8_t crc = fhss_crc8(wire_hdr, 2U);
    crc = fhss_crc8_update(crc, frame, len);

    /* Send wire packet: 0x7E, ch, len, frame bytes, crc */
    uint8_t wire_buf[MAX_FRAME_LEN + 4U];
    wire_buf[0] = WIRE_DELIMITER;
    wire_buf[1] = ch;
    wire_buf[2] = len;
    for (uint8_t i = 0; i < len; i++) {
        wire_buf[3U + i] = frame[i];
    }
    wire_buf[3U + len] = crc;

    uint8_t total_wire = (uint8_t)(len + 4U);
    for (uint8_t i = 0; i < total_wire; i++) {
        uint32_t timeout = 100000UL;
        while ((USART2->SR & USART_SR_TXE) == 0U) {
            if (--timeout == 0U) {
                return (int)ERR_TIMEOUT;
            }
        }
        USART2->DR = wire_buf[i];
    }

    /* Wait for transmission complete on wire */
    uint32_t tc_timeout = 100000UL;
    while ((USART2->SR & USART_SR_TC) == 0U) {
        if (--tc_timeout == 0U) {
            return (int)ERR_TIMEOUT;
        }
    }

    /* Wait for RF airtime duration so physical channel timing is modeled accurately */
    uint32_t airtime = air_airtime_us(len);
    while ((micros() - start_us) < airtime) {
        TIM2_Poll();
        __asm__ volatile ("nop");
    }

#ifdef DEMO_TRACE
    /* Drive PB10 LOW after airtime elapsed */
    GPIOB->BRR = (1UL << 10);
#endif

    return (int)OK;
}

int air_rx_poll(uint8_t ch, uint8_t *buf, uint8_t *len) {
    if (buf == (uint8_t *)0 || len == (uint8_t *)0) {
        return (int)ERR_NULL;
    }
    if (ch >= NUM_CHANNELS) {
        return (int)ERR_RANGE;
    }

    while (1) {
        uint16_t head = s_rx_head;
        uint16_t tail = s_rx_tail;
        uint16_t avail = (head >= tail) ? (uint16_t)(head - tail) : (uint16_t)(256U + head - tail);

        if (avail == 0U) {
            return (int)ERR_TIMEOUT;
        }

        /* Scan for wire delimiter 0x7E */
        if (s_rx_ring[tail] != WIRE_DELIMITER) {
            s_rx_tail = (tail + 1U) & 0xFFU;
            continue;
        }

        /* Need at least 3 bytes: 0x7E, ch, len */
        if (avail < 3U) {
            return (int)ERR_TIMEOUT;
        }

        uint8_t wire_ch = s_rx_ring[(tail + 1U) & 0xFFU];
        uint8_t wire_len = s_rx_ring[(tail + 2U) & 0xFFU];

        /* Sanity check frame length and channel */
        if (wire_len < HEADER_LEN || wire_len > MAX_FRAME_LEN || wire_ch >= NUM_CHANNELS) {
            /* Not a valid wire frame header; skip false delimiter */
            s_rx_tail = (tail + 1U) & 0xFFU;
            continue;
        }

        uint16_t wire_total = (uint16_t)(wire_len + 4U);
        if (avail < wire_total) {
            /* Incomplete wire frame; wait for remaining bytes */
            return (int)ERR_TIMEOUT;
        }

        /* Extract complete wire frame into temporary buffer */
        uint8_t temp[MAX_FRAME_LEN + 4U];
        for (uint16_t i = 0; i < wire_total; i++) {
            temp[i] = s_rx_ring[(tail + i) & 0xFFU];
        }

        /* Advance tail past this frame */
        s_rx_tail = (tail + wire_total) & 0xFFU;

        /* Verify wire CRC-8 over ch, len, and frame bytes */
        uint8_t calc_crc = fhss_crc8(&temp[1], (uint8_t)(wire_len + 2U));
        uint8_t wire_crc = temp[wire_total - 1U];

        if (calc_crc != wire_crc) {
            /* Corrupted wire packet; record diagnostic and skip */
            s_mock_diag.crc_error_count++;
            continue;
        }

        /* Frequency hopping / synchronization check:
         * Receiver only hears sender if sender's channel equals receiver's current channel
         */
        if (wire_ch != ch) {
            /* Different channel, not heard; record mismatch diagnostic */
            s_mock_diag.channel_mismatch_count++;
            s_mock_diag.last_mismatch_wire_ch = wire_ch;
            s_mock_diag.last_mismatch_tuned_ch = ch;
            s_mock_diag.last_mismatch_len = wire_len;
            continue;
        }

#if !defined(STEP5_FAULT_TEST)
        /* Channel error and loss model */
        int jammed = mock_is_jammed(ch);
        uint32_t r_loss = mock_rand32() % 100U;

        if (jammed != 0) {
            /* 97% loss on jammed channel */
            if (r_loss < 97U) {
                /* Packet dropped by jammer */
                continue;
            }

            /* 15% of surviving frames get one random bit flipped */
            uint32_t r_flip = mock_rand32() % 100U;
            if (r_flip < 15U) {
                uint32_t r = mock_rand32();
                uint8_t byte_idx = (uint8_t)(r % wire_len);
                uint8_t bit_idx = (uint8_t)((r >> 8) % 8U);
                temp[3U + byte_idx] ^= (uint8_t)(1U << bit_idx);
            }
        } else {
            /* 3% baseline packet loss on clean channel */
            if (r_loss < 3U) {
                /* Packet dropped by baseline loss */
                continue;
            }
        }
#endif

        /* Deliver frame to application */
        for (uint8_t i = 0; i < wire_len; i++) {
            buf[i] = temp[3U + i];
        }
        *len = wire_len;
        return (int)OK;
    }
}

uint8_t air_rssi_raw(uint8_t ch) {
    if (ch >= NUM_CHANNELS) {
        return 0xFFU;
    }

    int jammed = mock_is_jammed(ch);
    int32_t dbm;

    if (jammed != 0) {
        /* Jammed: about -40 dBm +/- 3 dBm */
        int32_t jitter = (int32_t)(mock_rand32() % 7U) - 3;
        dbm = -40 + jitter;

#ifdef DEMO_TRACE
        /* High while current channel is jammed */
        GPIOA->BSRR = (1UL << 8);
#endif
    } else {
        /* Clean: about -100 dBm +/- 3 dBm */
        int32_t jitter = (int32_t)(mock_rand32() % 7U) - 3;
        dbm = -100 + jitter;

#ifdef DEMO_TRACE
        GPIOA->BRR = (1UL << 8);
#endif
    }

    /* raw = -2 * dBm */
    int32_t raw = -2 * dbm;
    if (raw < 0) {
        raw = 0;
    } else if (raw > 255) {
        raw = 255;
    }
    return (uint8_t)raw;
}

#endif /* DEMO_USART_LINK */

#endif /* DEMO_USART_LINK || STEP1_UART_TEST || STEP2_MOCK_TEST || STEP3_FHSS_MOCK || STEP5_FAULT_TEST */

