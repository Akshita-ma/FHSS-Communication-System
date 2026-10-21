#include "stm32f4_regs.h"
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
    SYSCFG->EXTICR[0] &= ~(0xFUL << 0);
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
 * 16 MHz HSI clock: 16 ticks per microsecond.
 */
uint32_t micros(void) {
    uint32_t ms1 = g_systick_ms;
    uint32_t val = SysTick->VAL;
    uint32_t ms2 = g_systick_ms;

    if (ms2 != ms1) {
        val = SysTick->VAL;
        ms1 = ms2;
    }

    uint32_t ticks_elapsed = (16000UL - 1UL) - val;
    return (ms1 * 1000UL) + (ticks_elapsed / 16UL);
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
    for (volatile int i = 0; i < 40; i++) {
        __asm__ volatile ("nop");
    }
}

/*
 * HAL: Software NSS control on PB0.
 */
void NSS_LOW(void) {
    GPIOB->BSRR = (1UL << (0 + 16)); /* PB0 LOW */
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
    GPIOB->BSRR = (1UL << 0); /* PB0 HIGH */
    spi_guard_delay();
}

/*
 * HAL: Radio hardware reset pin control on PA1.
 * Active-low: Open-Drain output.
 * RADIO_RESET_LOW pulls PA1 low to GND (assert reset).
 * RADIO_RESET_HIGH releases PA1 to Hi-Z (let SX1278 internal pull-up pull high).
 */
void RADIO_RESET_LOW(void) {
    GPIOA->BSRR = (1UL << (1 + 16));
}

void RADIO_RESET_HIGH(void) {
    GPIOA->BSRR = (1UL << 1);
}

/*
 * HAL: SPI1 8-bit full-duplex transfer (Master, Mode 0, ~1 MHz).
 * Bounded loops ensure no hang even if SPI hardware or pins are faulted.
 * Checklist: waits for TXE, writes DR, waits for RXNE, reads DR, and waits for BSY to clear.
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
 * Hardware initialization for Node B (STM32F401 / STM32F411 Black Pill):
 * - 16 MHz HSI system clock
 * - SysTick 1 ms tick (16000 - 1)
 * - GPIOA, GPIOB clocks enabled in RCC->AHB1ENR
 * - USART1, SPI1, SYSCFG clocks enabled in RCC->APB2ENR
 * - USART1: PA9 AF7 Push-Pull, BRR = 0x008B (115200 baud @ 16 MHz PCLK2)
 * - SPI1: PA5 SCK (AF5), PA6 MISO (AF5, pull-up), PA7 MOSI (AF5)
 * - PB0: Software NSS (GP Output Push-Pull, default high)
 * - PA1: SX1278 RESET (GP Output Open-Drain, default high)
 * - SPI1: Master Mode 0 (CPOL=0, CPHA=0), Prescaler /16 (1 MHz), SSM=1, SSI=1, SPE set last
 */
void baremetal_init(void) {
    /* 1. Ensure 16 MHz HSI clock is active and stable */
    RCC->CR |= (1UL << 0); /* HSION */
    while ((RCC->CR & (1UL << 1)) == 0) {
        /* Wait until HSI is ready */
    }
    RCC->CFGR &= ~(0x3UL); /* Select HSI as SYSCLK */

    /* 2. Configure SysTick for 1 ms tick @ 16 MHz (16000 - 1) */
    SysTick->LOAD = 16000UL - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_ENABLE | SysTick_CTRL_TICKINT | SysTick_CTRL_CLKSOURCE;

    /* 3. Enable peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;

    /* 4. Configure GPIOA pins:
     * PA1:  RESET - Output GP Open-Drain (MODER=01, OTYPER=1, OSPEEDR=10)
     * PA5:  SCK   - Alternate Function AF5 (MODER=10, OSPEEDR=10, AFR[0]=5)
     * PA6:  MISO  - Alternate Function AF5 with Pull-Up (MODER=10, OSPEEDR=10, AFR[0]=5, PUPDR=01)
     * PA7:  MOSI  - Alternate Function AF5 (MODER=10, OSPEEDR=10, AFR[0]=5)
     * PA9:  TX    - Alternate Function AF7 (MODER=10, OSPEEDR=10, AFR[1]=7)
     */
    /* PA1: Output GP Open-Drain */
    GPIOA->MODER &= ~(3UL << 2);
    GPIOA->MODER |=  (1UL << 2); /* 01 = Output */
    GPIOA->OTYPER &= ~(1UL << 1); /* Push-Pull */
    GPIOA->OSPEEDR &= ~(3UL << 2);
    GPIOA->OSPEEDR |=  (2UL << 2); /* High speed */
    GPIOA->BSRR = (1UL << 1); /* Starts HIGH (3.3 V) */

    /* PA5, PA6, PA7: AF mode (MODER=10) with AF5 */
    GPIOA->MODER &= ~((3UL << 10) | (3UL << 12) | (3UL << 14));
    GPIOA->MODER |=  ((2UL << 10) | (2UL << 12) | (2UL << 14));
    GPIOA->OSPEEDR &= ~((3UL << 10) | (3UL << 12) | (3UL << 14));
    GPIOA->OSPEEDR |=  ((2UL << 10) | (2UL << 12) | (2UL << 14));
    /* AFR[0]: PA5 bits [23:20], PA6 bits [27:24], PA7 bits [31:28] = 5 */
    GPIOA->AFR[0] &= ~((0xFUL << 20) | (0xFUL << 24) | (0xFUL << 28));
    GPIOA->AFR[0] |=  ((5UL << 20) | (5UL << 24) | (5UL << 28));
    /* PA6 MISO pull-up */
    GPIOA->PUPDR &= ~(3UL << 12);
    GPIOA->PUPDR |=  (1UL << 12); /* 01 = Pull-Up */

    /* PA9: USART1 TX AF mode (MODER=10) with AF7 */
    GPIOA->MODER &= ~(3UL << 18);
    GPIOA->MODER |=  (2UL << 18);
    GPIOA->OSPEEDR &= ~(3UL << 18);
    GPIOA->OSPEEDR |=  (2UL << 18);
    /* AFR[1]: PA9 bits [7:4] = 7 */
    GPIOA->AFR[1] &= ~(0xFUL << 4);
    GPIOA->AFR[1] |=  (7UL << 4);

    /* 5. Configure GPIOB pins:
     * PB0: NSS - Output GP Push-Pull (MODER=01, OTYPER=0, OSPEEDR=10)
     */
    GPIOB->MODER &= ~(3UL << 0);
    GPIOB->MODER |=  (1UL << 0); /* 01 = Output */
    GPIOB->OTYPER &= ~(1UL << 0); /* Push-Pull */
    GPIOB->OSPEEDR &= ~(3UL << 0);
    GPIOB->OSPEEDR |=  (2UL << 0);
    GPIOB->BSRR = (1UL << 0); /* NSS starts HIGH (unselected) */

    /* 6. Configure USART1: BRR = 0x008B (115200 baud @ 16 MHz PCLK2) */
    USART1->BRR = 0x008BUL;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* 7. Configure SPI1:
     * - Master mode: MSTR = 1
     * - Baud rate: f_PCLK2 / 16 = 1.0 MHz (BR[2:0] = 0b011)
     * - Mode 0: CPOL = 0, CPHA = 0
     * - Software NSS: SSM = 1, SSI = 1
     * - 8-bit, MSB first
     */
    RCC->APB2RSTR |= (1UL << 12);
    RCC->APB2RSTR &= ~(1UL << 12);

    /* Clear any pending flags */
    (void)SPI1->SR;
    (void)SPI1->DR;

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_DIV16 |
                SPI_CR1_SSM  | SPI_CR1_SSI;
    SPI1->CR2 = 0U;
    SPI1->CR1 |= SPI_CR1_SPE; /* Enable SPI1 last */

    /* 8. Configure TIM2 for 1 ms tick */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 16000UL - 1UL; /* 1 ms tick @ 16 MHz APB1 */
    TIM2->CR1 = TIM_CR1_OPM;
    NVIC->ISER[0] = (1UL << 28); /* IRQ 28 (TIM2) */

#ifdef DEMO_TRACE
    /*
     * Configure Trace Pins for logic analyzer (DEMO_TRACE) on STM32F4:
     * PB1, PB5, PB10, PB12, PB13, PB14, PB15: GP Output (MODER=01, OSPEEDR=10)
     * PA8: GP Output (MODER=01, OSPEEDR=10)
     */
    GPIOB->MODER &= ~((3UL << 2)  | (3UL << 10) | (3UL << 20) | (3UL << 24) |
                      (3UL << 26) | (3UL << 28) | (3UL << 30));
    GPIOB->MODER |=  ((1UL << 2)  | (1UL << 10) | (1UL << 20) | (1UL << 24) |
                      (1UL << 26) | (1UL << 28) | (1UL << 30));

    GPIOB->OSPEEDR &= ~((3UL << 2)  | (3UL << 10) | (3UL << 20) | (3UL << 24) |
                        (3UL << 26) | (3UL << 28) | (3UL << 30));
    GPIOB->OSPEEDR |=  ((2UL << 2)  | (2UL << 10) | (2UL << 20) | (2UL << 24) |
                        (2UL << 26) | (2UL << 28) | (2UL << 30));

    GPIOA->MODER &= ~(3UL << 16);
    GPIOA->MODER |=  (1UL << 16);
    GPIOA->OSPEEDR &= ~(3UL << 16);
    GPIOA->OSPEEDR |=  (2UL << 16);

    /* Initialize trace pins LOW */
    GPIOB->BSRR = (1UL << (1 + 16))  | (1UL << (5 + 16))  | (1UL << (10 + 16)) |
                  (1UL << (12 + 16)) | (1UL << (13 + 16)) | (1UL << (14 + 16)) |
                  (1UL << (15 + 16));
    GPIOA->BSRR = (1UL << (8 + 16));
#endif

#ifdef DEMO_USART_LINK
    void air_mock_init(void);
    air_mock_init();
#endif
}
