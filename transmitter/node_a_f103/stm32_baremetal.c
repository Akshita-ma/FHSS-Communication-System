#include "stm32f1_regs.h"
#include "common/hal.h"
#include "common/status.h"

/* Monotonic millisecond counter driven by SysTick interrupt */
static volatile uint32_t g_systick_ms = 0;

/*
 * SysTick ISR: increments millisecond counter.
 * Conforms to Hard Rule 5: ISRs only modify volatile flags/counters.
 */
void SysTick_Handler(void) {
    g_systick_ms++;
}

/*
 * HAL: Monotonic millisecond timestamp.
 */
uint32_t millis(void) {
    return g_systick_ms;
}

/*
 * HAL: Synchronous millisecond busy wait using monotonic timer.
 * Cooperatively dispatches TIM2 hop timer callback.
 */
void delay_ms(uint32_t ms) {
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        TIM2_Poll();
        __asm__ volatile ("nop");
    }
}

/* TIM2 Hop Schedule callback and flag */
static void (*s_tim2_hop_cb)(void) = (void (*)(void))0;
static volatile uint8_t s_tim2_hop_pending = 0;

void TIM2_IRQHandler(void) {
    TIM2->SR = 0;
    s_tim2_hop_pending = 1;
}

void TIM2_Poll(void) {
    if (s_tim2_hop_pending != 0U) {
        s_tim2_hop_pending = 0U;
        if (s_tim2_hop_cb != (void (*)(void))0) {
            s_tim2_hop_cb();
        }
    }
}

int TIM2_ScheduleHopMs(uint32_t ms, void (*cb)(void)) {
    if (ms < 1U || ms > 60000U) {
        return (int)ERR_RANGE;
    }
    if (cb == (void (*)(void))0) {
        return (int)ERR_NULL;
    }

    TIM2->CR1 &= ~TIM_CR1_CEN;
    s_tim2_hop_cb = cb;
    s_tim2_hop_pending = 0U;
    TIM2->SR = 0U;
    TIM2->CNT = 0U;
    TIM2->ARR = ms;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_CEN;

    return (int)OK;
}

static void (*s_exti0_cb)(void) = (void (*)(void))0;
static volatile uint8_t s_exti0_flag = 0;

status_t EXTI0_RegisterCallback(void (*cb)(void)) {
    if (cb == (void (*)(void))0) {
        return ERR_NULL;
    }
    s_exti0_cb = cb;
    (void)s_exti0_flag;
    (void)s_exti0_cb;
    AFIO->EXTICR[0] &= ~(0xFUL << 0);
    EXTI->IMR |= (1UL << 0);
    EXTI->RTSR |= (1UL << 0);
    EXTI->FTSR &= ~(1UL << 0);
    NVIC->ISER[0] = (1UL << 6);
    return OK;
}

void EXTI0_IRQHandler(void) {
    if ((EXTI->PR & (1UL << 0)) != 0U) {
        EXTI->PR = (1UL << 0);
        s_exti0_flag = 1U;
    }
}

/*
 * HAL: Microsecond timestamp helper.
 * 8 MHz HSI clock: 8 ticks per microsecond.
 */
uint32_t micros(void) {
    uint32_t ms1 = g_systick_ms;
    uint32_t val = SysTick->VAL;
    uint32_t ms2 = g_systick_ms;

    if (ms2 != ms1) {
        /* SysTick wrapped during read; take new value */
        val = SysTick->VAL;
        ms1 = ms2;
    }

    uint32_t ticks_elapsed = (8000UL - 1UL) - val;
    return (ms1 * 1000UL) + (ticks_elapsed / 8UL);
}

/*
 * HAL: NULL-safe UART1 string transmission on PA9 (115200 8N1).
 * Bounded timeout ensures firmware never hangs if peripheral faults.
 */
void UART1_SendString(const char *s) {
    if (s == (const char *)0 || *s == '\0') {
        return;
    }

    while (*s != '\0') {
        uint32_t timeout = 100000UL;
        while ((USART1->SR & USART_SR_TXE) == 0) {
            if (--timeout == 0) {
                return;
            }
        }
        USART1->DR = (uint8_t)(*s++);
    }

    uint32_t tc_timeout = 100000UL;
    while ((USART1->SR & USART_SR_TC) == 0) {
        if (--tc_timeout == 0) {
            break;
        }
    }
}

/*
 * Small nanosecond-scale guard delay to satisfy SX1278 SPI setup/hold timings:
 * t_NSS,setup >= 30 ns, t_NSS,hold >= 100 ns, t_NSS,high >= 40 ns.
 */
static inline void spi_guard_delay(void) {
    for (volatile int i = 0; i < 30; i++) {
        __asm__ volatile ("nop");
    }
}

/*
 * HAL: Software NSS control on PB0 (and PA4).
 * Driving both PB0 (CONTEXT.md frozen pin) and PA4 (default hardware SPI1 NSS)
 * guarantees active select regardless of whether the physical jumper is connected
 * to PB0 or PA4.
 */
void NSS_LOW(void) {
    GPIOB->BRR = (1UL << 0);
    GPIOA->BRR = (1UL << 4);
    spi_guard_delay();
}

void NSS_HIGH(void) {
    /* Wait for transmission complete on wire before raising NSS */
    uint32_t timeout = 100000UL;
    while (SPI1->SR & SPI_SR_BSY) {
        if (--timeout == 0) {
            break;
        }
    }
    spi_guard_delay();
    GPIOB->BSRR = (1UL << 0);
    GPIOA->BSRR = (1UL << 4);
    spi_guard_delay();
}

/*
 * HAL: Radio hardware reset pin control on PA1.
 * Active-low: Open-Drain output.
 * RADIO_RESET_LOW pulls PA1 low to GND (assert reset).
 * RADIO_RESET_HIGH releases PA1 to Hi-Z (let SX1278 internal pull-up pull high).
 */
void RADIO_RESET_LOW(void) {
    GPIOA->BRR = (1UL << 1);
}

void RADIO_RESET_HIGH(void) {
    GPIOA->BSRR = (1UL << 1);
}

/*
 * HAL: SPI1 8-bit full-duplex transfer (Master, Mode 0, ~1 MHz).
 * Bounded loops ensure no hang even if SPI hardware or pins are faulted.
 * Full 32-bit register access prevents STM32F103 AHB-to-APB bridge byte duplication.
 */
uint8_t SPI1_Transfer(uint8_t tx) {
    uint32_t timeout = 100000UL;

    /* Drain any stale RX data from previous transactions */
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }

    while ((SPI1->SR & SPI_SR_TXE) == 0) {
        if (--timeout == 0) {
            return 0xFFU;
        }
    }
    SPI1->DR = (uint32_t)tx;

    timeout = 100000UL;
    while ((SPI1->SR & SPI_SR_RXNE) == 0) {
        if (--timeout == 0) {
            return 0xFFU;
        }
    }
    uint8_t rx = (uint8_t)(SPI1->DR & 0xFFU);

    timeout = 100000UL;
    while (SPI1->SR & SPI_SR_BSY) {
        if (--timeout == 0) {
            break;
        }
    }
    return rx;
}

/*
 * Hardware initialization for Node A (STM32F103C8T6):
 * - 8 MHz HSI system clock
 * - SysTick 1 ms tick (8000 - 1)
 * - GPIOA, GPIOB, AFIO, SPI1, USART1 clocks enabled
 * - USART1: PA9 AF Push-Pull (50 MHz), BRR = 0x0045 (115200 baud)
 * - SPI1: PA5 SCK (AF PP), PA6 MISO (Input with pull-up), PA7 MOSI (AF PP)
 * - PB0 + PA4: Software NSS (GP Output Push-Pull, default high)
 * - PA1: SX1278 RESET (GP Output Push-Pull, default high)
 * - SPI1: Master Mode 0 (CPOL=0, CPHA=0), Prescaler /8 (1 MHz), SSM=1, SSI=1
 */
void baremetal_init(void) {
    /* 1. Ensure 8 MHz HSI clock is active and stable */
    RCC->CR |= (1UL << 0); /* HSION */
    while ((RCC->CR & (1UL << 1)) == 0) {
        /* Wait until HSI is ready */
    }
    RCC->CFGR &= ~(0x3UL); /* Select HSI as SYSCLK */

    /* 2. Configure SysTick for 1 ms tick @ 8 MHz (8000 - 1) */
    SysTick->LOAD = 8000UL - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_ENABLE | SysTick_CTRL_TICKINT | SysTick_CTRL_CLKSOURCE;

    /* 3. Enable peripheral clocks in RCC->APB2ENR: IOPA, IOPB, AFIO, SPI1, USART1 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN |
                    RCC_APB2ENR_USART1EN;

    /* 4. Configure GPIOA pins:
     * PA1:  RESET   - Output GP Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     * PA4:  NSS_ALT - Output GP Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     * PA5:  SCK     - Output AF Push-Pull 50 MHz (MODE=11, CNF=10 -> 0xB)
     * PA6:  MISO    - Input with Pull-Up (MODE=00, CNF=10 -> 0x8, ODR=1)
     * PA7:  MOSI    - Output AF Push-Pull 50 MHz (MODE=11, CNF=10 -> 0xB)
     * PA9:  TX      - Output AF Push-Pull 50 MHz (MODE=11, CNF=10 -> 0xB)
     */
    /* PA1 in CRL bits [7:4]: Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3) */
    GPIOA->CRL &= ~(0xFUL << 4);
    GPIOA->CRL |= (0x3UL << 4);
    GPIOA->BSRR = (1UL << 1); /* RESET starts HIGH (3.3 V) */

    /* PA4 in CRL bits [19:16] */
    GPIOA->CRL &= ~(0xFUL << 16);
    GPIOA->CRL |= (0x3UL << 16);
    GPIOA->BSRR = (1UL << 4); /* PA4 starts HIGH */

    /* PA5 in CRL bits [23:20] */
    GPIOA->CRL &= ~(0xFUL << 20);
    GPIOA->CRL |= (0xBUL << 20);

    /* PA6 in CRL bits [27:24]: Input with Pull-Up (CNF=10, MODE=00 -> 0x8) */
    GPIOA->CRL &= ~(0xFUL << 24);
    GPIOA->CRL |= (0x8UL << 24);
    GPIOA->BSRR = (1UL << 6); /* Enable pull-up */

    /* PA7 in CRL bits [31:28] */
    GPIOA->CRL &= ~(0xFUL << 28);
    GPIOA->CRL |= (0xBUL << 28);

    /* PA9 in CRH bits [7:4] */
    GPIOA->CRH &= ~(0xFUL << 4);
    GPIOA->CRH |= (0xBUL << 4);

    /* 5. Configure GPIOB pins:
     * PB0: NSS - Output GP Push-Pull 50 MHz (MODE=11, CNF=00 -> 0x3)
     */
    GPIOB->CRL &= ~(0xFUL << 0);
    GPIOB->CRL |= (0x3UL << 0);
    GPIOB->BSRR = (1UL << 0); /* NSS starts HIGH (unselected) */

    /* 6. Configure USART1: BRR = 0x0045 (115200 baud @ 8 MHz PCLK2) */
    USART1->BRR = 0x0045UL;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* 7. Configure SPI1:
     * - Master mode: MSTR = 1
     * - Baud rate: f_PCLK2 / 8 = 1 MHz (BR[2:0] = 0b010)
     * - Mode 0: CPOL = 0, CPHA = 0
     * - Software NSS: SSM = 1, SSI = 1
     * - 8-bit, MSB first
     */
    RCC->APB2RSTR |= (1UL << 12);
    RCC->APB2RSTR &= ~(1UL << 12);

    /* Clear any pending flags */
    (void)SPI1->SR;
    (void)SPI1->DR;

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_DIV8 |
                SPI_CR1_SSM  | SPI_CR1_SSI;
    SPI1->CR2 = 0U;
    SPI1->CR1 |= SPI_CR1_SPE; /* Enable SPI1 */

    /* 8. Configure TIM2 for 1 ms tick */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 8000UL - 1UL; /* 1 ms tick @ 8 MHz APB1 */
    TIM2->CR1 = TIM_CR1_OPM;
    NVIC->ISER[0] = (1UL << 28); /* IRQ 28 (TIM2) */

#ifdef DEMO_TRACE
    /*
     * Configure Trace Pins for logic analyzer (DEMO_TRACE):
     * PB1:  Hop toggle (CRL [7:4] -> 0x3)
     * PB5:  LOCKED (CRL [23:20] -> 0x3)
     * PB10: TX active (CRH [11:8] -> 0x3)
     * PB12: ACK pulse (CRH [19:16] -> 0x3)
     * PB13: CH bit 0 (CRH [23:20] -> 0x3)
     * PB14: CH bit 1 (CRH [27:24] -> 0x3)
     * PB15: CH bit 2 (CRH [31:28] -> 0x3)
     * PA8:  Jammed flag (CRH [3:0] -> 0x3)
     * Caution: do not touch PB3, PB4, PA13, PA14, PA15!
     */
    GPIOB->CRL &= ~((0xFUL << 4) | (0xFUL << 20));
    GPIOB->CRL |=  ((0x3UL << 4) | (0x3UL << 20));

    GPIOB->CRH &= ~((0xFUL << 8) | (0xFUL << 16) | (0xFUL << 20) | (0xFUL << 24) | (0xFUL << 28));
    GPIOB->CRH |=  ((0x3UL << 8) | (0x3UL << 16) | (0x3UL << 20) | (0x3UL << 24) | (0x3UL << 28));

    GPIOA->CRH &= ~(0xFUL << 0);
    GPIOA->CRH |=  (0x3UL << 0);

    /* Initialize trace pins LOW */
    GPIOB->BRR = (1UL << 1) | (1UL << 5) | (1UL << 10) | (1UL << 12) |
                 (1UL << 13) | (1UL << 14) | (1UL << 15);
    GPIOA->BRR = (1UL << 8);
#endif

#if defined(DEMO_USART_LINK) || defined(STEP1_UART_TEST) || defined(STEP2_MOCK_TEST) || defined(STEP3_FHSS_MOCK) || defined(STEP5_FAULT_TEST)
    void air_mock_init(void);
    air_mock_init();
#endif
}
