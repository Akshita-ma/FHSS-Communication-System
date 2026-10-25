#ifndef STM32F1_REGS_H
#define STM32F1_REGS_H

#include <stdint.h>

/*
 * Minimal Register Definitions for STM32F103C8T6 (RM0008)
 * 100% Bare-metal C, direct register access only (Hard Rule 1)
 */

#define PERIPH_BASE            (0x40000000UL)
#define APB1PERIPH_BASE        (PERIPH_BASE)
#define APB2PERIPH_BASE        (PERIPH_BASE + 0x10000UL)
#define AHBPERIPH_BASE         (PERIPH_BASE + 0x20000UL)

/* RCC */
#define RCC_BASE               (AHBPERIPH_BASE + 0x1000UL)

typedef struct {
    volatile uint32_t CR;            /* 0x00 */
    volatile uint32_t CFGR;          /* 0x04 */
    volatile uint32_t CIR;           /* 0x08 */
    volatile uint32_t APB2RSTR;      /* 0x0C */
    volatile uint32_t APB1RSTR;      /* 0x10 */
    volatile uint32_t AHBENR;        /* 0x14 */
    volatile uint32_t APB2ENR;       /* 0x18 */
    volatile uint32_t APB1ENR;       /* 0x1C */
    volatile uint32_t BDCR;          /* 0x20 */
    volatile uint32_t CSR;           /* 0x24 */
} RCC_TypeDef;

#define RCC                    ((RCC_TypeDef *)RCC_BASE)

#define RCC_APB2ENR_AFIOEN     (1UL << 0)
#define RCC_APB2ENR_IOPAEN     (1UL << 2)
#define RCC_APB2ENR_IOPBEN     (1UL << 3)
#define RCC_APB2ENR_SPI1EN     (1UL << 12)
#define RCC_APB2ENR_USART1EN   (1UL << 14)
#define RCC_APB1ENR_TIM2EN     (1UL << 0)

/* GPIO */
#define GPIOA_BASE             (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE             (APB2PERIPH_BASE + 0x0C00UL)

typedef struct {
    volatile uint32_t CRL;           /* 0x00 */
    volatile uint32_t CRH;           /* 0x04 */
    volatile uint32_t IDR;           /* 0x08 */
    volatile uint32_t ODR;           /* 0x0C */
    volatile uint32_t BSRR;          /* 0x10 */
    volatile uint32_t BRR;           /* 0x14 */
    volatile uint32_t LCKR;          /* 0x18 */
} GPIO_TypeDef;

#define GPIOA                  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                  ((GPIO_TypeDef *)GPIOB_BASE)

/* AFIO */
#define AFIO_BASE              (APB2PERIPH_BASE + 0x0000UL)

typedef struct {
    volatile uint32_t EVCR;          /* 0x00 */
    volatile uint32_t MAPR;          /* 0x04 */
    volatile uint32_t EXTICR[4];     /* 0x08 - 0x14 */
    volatile uint32_t MAPR2;         /* 0x18 */
} AFIO_TypeDef;

#define AFIO                   ((AFIO_TypeDef *)AFIO_BASE)

/* USART */
#define USART1_BASE            (APB2PERIPH_BASE + 0x3800UL)

typedef struct {
    volatile uint32_t SR;            /* 0x00 */
    volatile uint32_t DR;            /* 0x04 */
    volatile uint32_t BRR;           /* 0x08 */
    volatile uint32_t CR1;           /* 0x0C */
    volatile uint32_t CR2;           /* 0x10 */
    volatile uint32_t CR3;           /* 0x14 */
    volatile uint32_t GTPR;          /* 0x18 */
} USART_TypeDef;

#define USART1                 ((USART_TypeDef *)USART1_BASE)

#define USART_SR_TXE           (1UL << 7)
#define USART_SR_TC            (1UL << 6)
#define USART_SR_RXNE          (1UL << 5)

#define USART_CR1_RE           (1UL << 2)
#define USART_CR1_TE           (1UL << 3)
#define USART_CR1_UE           (1UL << 13)

/* SPI */
#define SPI1_BASE              (APB2PERIPH_BASE + 0x3000UL)

typedef struct {
    volatile uint32_t CR1;           /* 0x00 */
    volatile uint32_t CR2;           /* 0x04 */
    volatile uint32_t SR;            /* 0x08 */
    volatile uint32_t DR;            /* 0x0C */
    volatile uint32_t CRCPR;         /* 0x10 */
    volatile uint32_t RXCRCR;        /* 0x14 */
    volatile uint32_t TXCRCR;        /* 0x18 */
    volatile uint32_t I2SCFGR;       /* 0x1C */
    volatile uint32_t I2SPR;         /* 0x20 */
} SPI_TypeDef;

#define SPI1                   ((SPI_TypeDef *)SPI1_BASE)

#define SPI_CR1_CPHA           (1UL << 0)
#define SPI_CR1_CPOL           (1UL << 1)
#define SPI_CR1_MSTR           (1UL << 2)
#define SPI_CR1_BR_DIV8        (2UL << 3)  /* Prescaler /8: bits [5:3] = 010 (1 MHz @ 8 MHz HSI) */
#define SPI_CR1_SPE            (1UL << 6)
#define SPI_CR1_LSBFIRST       (1UL << 7)
#define SPI_CR1_SSI            (1UL << 8)
#define SPI_CR1_SSM            (1UL << 9)

#define SPI_SR_RXNE            (1UL << 0)
#define SPI_SR_TXE             (1UL << 1)
#define SPI_SR_BSY             (1UL << 7)

/* EXTI */
#define EXTI_BASE              (APB2PERIPH_BASE + 0x0400UL)

typedef struct {
    volatile uint32_t IMR;           /* 0x00 */
    volatile uint32_t EMR;           /* 0x04 */
    volatile uint32_t RTSR;          /* 0x08 */
    volatile uint32_t FTSR;          /* 0x0C */
    volatile uint32_t SWIER;         /* 0x10 */
    volatile uint32_t PR;            /* 0x14 */
} EXTI_TypeDef;

#define EXTI                   ((EXTI_TypeDef *)EXTI_BASE)

/* TIM2 */
#define TIM2_BASE              (APB1PERIPH_BASE + 0x0000UL)

typedef struct {
    volatile uint32_t CR1;           /* 0x00 */
    volatile uint32_t CR2;           /* 0x04 */
    volatile uint32_t SMCR;          /* 0x08 */
    volatile uint32_t DIER;          /* 0x0C */
    volatile uint32_t SR;            /* 0x10 */
    volatile uint32_t EGR;           /* 0x14 */
    volatile uint32_t CCMR1;         /* 0x18 */
    volatile uint32_t CCMR2;         /* 0x1C */
    volatile uint32_t CCER;          /* 0x20 */
    volatile uint32_t CNT;           /* 0x24 */
    volatile uint32_t PSC;           /* 0x28 */
    volatile uint32_t ARR;           /* 0x2C */
    volatile uint32_t RCR;           /* 0x30 */
    volatile uint32_t CCR1;          /* 0x34 */
    volatile uint32_t CCR2;          /* 0x38 */
    volatile uint32_t CCR3;          /* 0x3C */
    volatile uint32_t CCR4;          /* 0x40 */
    volatile uint32_t BDTR;          /* 0x44 */
    volatile uint32_t DCR;           /* 0x48 */
    volatile uint32_t DMAR;          /* 0x4C */
} TIM_TypeDef;

#define TIM2                   ((TIM_TypeDef *)TIM2_BASE)

#define TIM_CR1_CEN            (1UL << 0)
#define TIM_CR1_OPM            (1UL << 3)
#define TIM_DIER_UIE           (1UL << 0)
#define TIM_SR_UIF             (1UL << 0)

/* NVIC */
#define NVIC_ISER_BASE         (0xE000E100UL)
#define NVIC_ICER_BASE         (0xE000E180UL)
#define NVIC_ICPR_BASE         (0xE000E280UL)

#define EXTI0_IRQn             6
#define TIM2_IRQn              28

static inline void NVIC_EnableIRQ(uint8_t irqn) {
    volatile uint32_t *iser = (volatile uint32_t *)(NVIC_ISER_BASE + ((irqn >> 5) * 4));
    *iser = (1UL << (irqn & 0x1F));
}

static inline void NVIC_DisableIRQ(uint8_t irqn) {
    volatile uint32_t *icer = (volatile uint32_t *)(NVIC_ICER_BASE + ((irqn >> 5) * 4));
    *icer = (1UL << (irqn & 0x1F));
}

static inline void NVIC_ClearPendingIRQ(uint8_t irqn) {
    volatile uint32_t *icpr = (volatile uint32_t *)(NVIC_ICPR_BASE + ((irqn >> 5) * 4));
    *icpr = (1UL << (irqn & 0x1F));
}

#endif /* STM32F1_REGS_H */
