#include "stm32_baremetal.h"
#include <stddef.h>

/* Global millisecond tick from SysTick */
static volatile uint32_t g_ms_ticks = 0;

/* Callback pointers for interrupts */
static void (*g_exti0_cb)(void) = NULL;
static void (*g_tim2_hop_cb)(void) = NULL;

/* Volatile ISR event flags (ISRs only set volatile flags/timestamps - Hard Rule 5) */
static volatile uint8_t  g_exti0_flag = 0;
static volatile uint32_t g_exti0_timestamp = 0;

#define UART_TIMEOUT_CYCLES  200000UL
#define SPI_TIMEOUT_CYCLES   200000UL

/* Core SysTick base address for 1 ms tick */
#define SYSTICK_BASE         (0xE000E010UL)
typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_TypeDef;
#define SysTick              ((SysTick_TypeDef *)SYSTICK_BASE)

void baremetal_init(void) {
    /* 1. Clocks: Enable GPIOA, GPIOB, AFIO, SPI1, USART1 in APB2ENR (8 MHz HSI default) */
    RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                     RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN);

    /* 2. Clocks: Enable TIM2 in APB1ENR */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* 3. Configure PA9 as USART1 TX: Alternate function push-pull 50 MHz (MODE=11, CNF=10 -> 0xB) */
    GPIOA->CRH &= ~(0xFUL << ((9 - 8) * 4));
    GPIOA->CRH |=  (0xBUL << ((9 - 8) * 4));

    /* 4. Configure USART1: 115200 8N1 @ 8 MHz HSI (BRR = 0x0045) */
    USART1->BRR = 0x0045;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    /* 5. Configure PB0 as SX1278 NSS: General purpose output push-pull 50 MHz (0x3), initial HIGH */
    GPIOB->CRL &= ~(0xFUL << (0 * 4));
    GPIOB->CRL |=  (0x3UL << (0 * 4));
    GPIOB->BSRR = (1UL << 0);            /* NSS HIGH */

    /* 6. Configure PA1 as SX1278 RESET: General purpose output push-pull 50 MHz (0x3), initial HIGH */
    GPIOA->CRL &= ~(0xFUL << (1 * 4));
    GPIOA->CRL |=  (0x3UL << (1 * 4));
    GPIOA->BSRR = (1UL << 1);            /* RESET HIGH (inactive) */

    /* 7. Configure PA5 (SCK) and PA7 (MOSI) as Alternate function push-pull 50 MHz (0xB) */
    GPIOA->CRL &= ~((0xFUL << (5 * 4)) | (0xFUL << (7 * 4)));
    GPIOA->CRL |=  ((0xBUL << (5 * 4)) | (0xBUL << (7 * 4)));

    /* 8. Configure PA6 (MISO) as Input with pull-up (MODE=00, CNF=10 -> 0x8, ODR=1) */
    GPIOA->CRL &= ~(0xFUL << (6 * 4));
    GPIOA->CRL |=  (0x8UL << (6 * 4));
    GPIOA->ODR |=  (1UL << 6);           /* Pull-up */

    /* 9. Configure SPI1: Master, Mode 0 (CPOL=0, CPHA=0), Prescaler /8 (1 MHz @ 8 MHz HSI), MSB first, 8-bit */
    SPI1->CR1 = 0; /* Reset */
    (void)SPI1->SR;
    (void)SPI1->DR;
    /* Set Baud rate /8, Software slave management (SSM=1), Internal slave select (SSI=1), Master (MSTR=1) */
    SPI1->CR1 = SPI_CR1_BR_DIV8 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
    /* Set SPE last */
    SPI1->CR1 |= SPI_CR1_SPE;
    (void)SPI1->SR;
    (void)SPI1->DR;

    /* 10. Setup SysTick for 1 ms tick (8 MHz / 1000 = 8000) */
    SysTick->LOAD = 8000UL - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07; /* Enable, TickInt enable, Core clock source */

    /* Enable global interrupts */
    __asm__ volatile ("cpsie i" : : : "memory");
}

/* FROZEN HAL: UART1_SendString - NULL-safe */
void UART1_SendString(const char *s) {
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
        uint32_t timeout = UART_TIMEOUT_CYCLES;
        while (!(USART1->SR & USART_SR_TXE)) {
            if (--timeout == 0) {
                return;
            }
        }
        USART1->DR = (uint8_t)(*s++);
    }
    uint32_t timeout = UART_TIMEOUT_CYCLES;
    while (!(USART1->SR & USART_SR_TC)) {
        if (--timeout == 0) {
            return;
        }
    }
}

/* FROZEN HAL: NSS Control */
void NSS_LOW(void) {
    GPIOB->BSRR = (1UL << (0 + 16)); /* Reset PB0 */
}

void NSS_HIGH(void) {
    GPIOB->BSRR = (1UL << 0);        /* Set PB0 */
}

/* FROZEN HAL: SPI1_Transfer */
uint8_t SPI1_Transfer(uint8_t tx) {
    /* If MODF error cleared MSTR, restore Master mode and SPE */
    if (!(SPI1->CR1 & SPI_CR1_MSTR)) {
        (void)SPI1->SR;
        (void)SPI1->DR;
        SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;
    }
    if (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }
    uint32_t timeout = SPI_TIMEOUT_CYCLES;
    while (!(SPI1->SR & SPI_SR_TXE)) {
        if (--timeout == 0) {
            return 0xFF;
        }
    }
    SPI1->DR = tx;
    timeout = SPI_TIMEOUT_CYCLES;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (--timeout == 0) {
            return 0xFF;
        }
    }
    uint8_t rx = (uint8_t)(SPI1->DR);
    timeout = SPI_TIMEOUT_CYCLES;
    while (SPI1->SR & SPI_SR_BSY) {
        if (--timeout == 0) {
            break;
        }
    }
    return rx;
}

/* FROZEN HAL: TIM2_ScheduleHopMs - one-shot, 1..60000 ms */
int TIM2_ScheduleHopMs(uint32_t ms, void (*cb)(void)) {
    if (ms < 1 || ms > 60000 || cb == NULL) {
        return (int)ERR_RANGE;
    }
    g_tim2_hop_cb = cb;

    /* Stop TIM2 while reconfiguring */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM2_IRQn);

    /* TIM2: 1 ms tick -> PSC = 8000-1 @ 8 MHz HSI */
    TIM2->PSC = 8000UL - 1;
    TIM2->ARR = ms;
    TIM2->CNT = 0;
    TIM2->SR = 0;              /* Clear pending flags */
    TIM2->CR1 |= TIM_CR1_OPM;  /* One-pulse mode */
    TIM2->DIER |= TIM_DIER_UIE;/* Update interrupt enable */

    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;  /* Start counter */
    return (int)OK;
}

/* Extra helpers declared in hal.h */
void Radio_Reset_Pin(uint8_t state) {
    if (state) {
        GPIOA->BSRR = (1UL << 1);        /* Set PA1 HIGH */
    } else {
        GPIOA->BSRR = (1UL << (1 + 16)); /* Reset PA1 LOW */
    }
}

uint32_t millis(void) {
    return g_ms_ticks;
}

void delay_us(uint32_t us) {
    while (us--) {
        __asm__ volatile (
            "nop; nop; nop; nop; nop; nop;"
        );
    }
}

void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}

status_t EXTI0_RegisterCallback(void (*cb)(void)) {
    if (cb == NULL) {
        return ERR_NULL;
    }
    g_exti0_cb = cb;
    return OK;
}

/* Interrupt Service Handlers (Hard Rule 5: No SPI or UART in ISR) */
void SysTick_Handler(void) {
    g_ms_ticks++;
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = ~TIM_SR_UIF;
        if (g_tim2_hop_cb != NULL) {
            void (*cb)(void) = g_tim2_hop_cb;
            g_tim2_hop_cb = NULL;
            cb();
        }
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1UL << 0)) {
        EXTI->PR = (1UL << 0); /* Clear pending bit by writing 1 */
        g_exti0_flag = 1;
        g_exti0_timestamp = g_ms_ticks;
        if (g_exti0_cb != NULL) {
            g_exti0_cb();
        }
    }
}
