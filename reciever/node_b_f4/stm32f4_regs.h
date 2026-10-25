#ifndef STM32F4_REGS_H
#define STM32F4_REGS_H

#include <stdint.h>

/*
 * Minimal Bare-Metal Register Header for STM32F401 / STM32F411 (RM0368 / RM0383)
 * No vendor headers used (Hard Rule 1).
 */

#define PERIPH_BASE            (0x40000000UL)
#define APB1PERIPH_BASE        (PERIPH_BASE)
#define APB2PERIPH_BASE        (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

/* RCC */
#define RCC_BASE               (AHB1PERIPH_BASE + 0x3800UL)

typedef struct {
    volatile uint32_t CR;            /* 0x00 */
    volatile uint32_t PLLCFGR;       /* 0x04 */
    volatile uint32_t CFGR;          /* 0x08 */
    volatile uint32_t CIR;           /* 0x0C */
    volatile uint32_t AHB1RSTR;      /* 0x10 */
    volatile uint32_t AHB2RSTR;      /* 0x14 */
    volatile uint32_t Reserved1[2];  /* 0x18 - 0x1C */
    volatile uint32_t APB1RSTR;      /* 0x20 */
    volatile uint32_t APB2RSTR;      /* 0x24 */
    volatile uint32_t Reserved2[2];  /* 0x28 - 0x2C */
    volatile uint32_t AHB1ENR;       /* 0x30 */
    volatile uint32_t AHB2ENR;       /* 0x34 */
    volatile uint32_t Reserved3[2];  /* 0x38 - 0x3C */
    volatile uint32_t APB1ENR;       /* 0x40 */
    volatile uint32_t APB2ENR;       /* 0x44 */
    volatile uint32_t Reserved4[2];  /* 0x48 - 0x4C */
    volatile uint32_t AHB1LPENR;     /* 0x50 */
    volatile uint32_t AHB2LPENR;     /* 0x54 */
    volatile uint32_t Reserved5[2];  /* 0x58 - 0x5C */
    volatile uint32_t APB1LPENR;     /* 0x60 */
    volatile uint32_t APB2LPENR;     /* 0x64 */
    volatile uint32_t Reserved6[2];  /* 0x68 - 0x6C */
    volatile uint32_t BDCR;          /* 0x70 */
    volatile uint32_t CSR;           /* 0x74 */
    volatile uint32_t Reserved7[2];  /* 0x78 - 0x7C */
    volatile uint32_t SSCGR;         /* 0x80 */
    volatile uint32_t PLLI2SCFGR;    /* 0x84 */
} RCC_TypeDef;

#define RCC                    ((RCC_TypeDef *)RCC_BASE)

#define RCC_AHB1ENR_GPIOAEN    (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN    (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN    (1UL << 2)

#define RCC_APB1ENR_TIM2EN     (1UL << 0)
#define RCC_APB1ENR_USART2EN   (1UL << 17)

#define RCC_APB2ENR_USART1EN   (1UL << 4)
#define RCC_APB2ENR_SPI1EN     (1UL << 12)
#define RCC_APB2ENR_SYSCFGEN   (1UL << 14)

/* GPIO */
#define GPIOA_BASE             (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE             (AHB1PERIPH_BASE + 0x0400UL)

typedef struct {
    volatile uint32_t MODER;         /* 0x00 */
    volatile uint32_t OTYPER;        /* 0x04 */
    volatile uint32_t OSPEEDR;       /* 0x08 */
    volatile uint32_t PUPDR;         /* 0x0C */
    volatile uint32_t IDR;           /* 0x10 */
    volatile uint32_t ODR;           /* 0x14 */
    volatile uint32_t BSRR;          /* 0x18 */
    volatile uint32_t LCKR;          /* 0x1C */
    volatile uint32_t AFR[2];        /* 0x20: AFR[0]=AFRL (pins 0..7), AFR[1]=AFRH (pins 8..15) */
} GPIO_TypeDef;

#define GPIOA                  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                  ((GPIO_TypeDef *)GPIOB_BASE)

/* USART */
#define USART1_BASE            (APB2PERIPH_BASE + 0x1000UL)

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
#define USART2_BASE            (APB1PERIPH_BASE + 0x4400UL)
#define USART2                 ((USART_TypeDef *)USART2_BASE)

#define USART_SR_ORE           (1UL << 3)
#define USART_SR_RXNE          (1UL << 5)
#define USART_SR_TC            (1UL << 6)
#define USART_SR_TXE           (1UL << 7)

#define USART_CR1_RE           (1UL << 2)
#define USART_CR1_TE           (1UL << 3)
#define USART_CR1_RXNEIE       (1UL << 5)
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
#define SPI_CR1_BR_DIV16       (3UL << 3)  /* Prescaler /16: bits [5:3] = 011 */
#define SPI_CR1_SPE            (1UL << 6)
#define SPI_CR1_LSBFIRST       (1UL << 7)
#define SPI_CR1_SSI            (1UL << 8)
#define SPI_CR1_SSM            (1UL << 9)

#define SPI_SR_RXNE            (1UL << 0)
#define SPI_SR_TXE             (1UL << 1)
#define SPI_SR_BSY             (1UL << 7)

/* SYSCFG */
#define SYSCFG_BASE            (APB2PERIPH_BASE + 0x3800UL)

typedef struct {
    volatile uint32_t MEMRMP;        /* 0x00 */
    volatile uint32_t PMC;           /* 0x04 */
    volatile uint32_t EXTICR[4];     /* 0x08 - 0x14 */
    volatile uint32_t Reserved[2];
    volatile uint32_t CMPCR;         /* 0x20 */
} SYSCFG_TypeDef;

#define SYSCFG                 ((SYSCFG_TypeDef *)SYSCFG_BASE)

/* EXTI */
#define EXTI_BASE              (APB2PERIPH_BASE + 0x3C00UL)

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
    volatile uint32_t Reserved1;     /* 0x30 */
    volatile uint32_t CCR1;          /* 0x34 */
    volatile uint32_t CCR2;          /* 0x38 */
    volatile uint32_t CCR3;          /* 0x3C */
    volatile uint32_t CCR4;          /* 0x40 */
} TIM_TypeDef;

#define TIM2                   ((TIM_TypeDef *)TIM2_BASE)

#define TIM_CR1_CEN            (1UL << 0)
#define TIM_CR1_OPM            (1UL << 3)
#define TIM_CR1_ARPE           (1UL << 7)
#define TIM_DIER_UIE           (1UL << 0)
#define TIM_SR_UIF             (1UL << 0)
#define TIM_EGR_UG             (1UL << 0)

/* Cortex-M4 Core Peripherals */
#define NVIC_BASE              (0xE000E100UL)

typedef struct {
    volatile uint32_t ISER[8];
    uint32_t Reserved1[24];
    volatile uint32_t ICER[8];
    uint32_t Reserved2[24];
    volatile uint32_t ISPR[8];
    uint32_t Reserved3[24];
    volatile uint32_t ICPR[8];
    uint32_t Reserved4[24];
    volatile uint32_t IABR[8];
    uint32_t Reserved5[56];
    volatile uint8_t  IP[240];
} NVIC_TypeDef;

#define NVIC                   ((NVIC_TypeDef *)NVIC_BASE)

#define EXTI0_IRQn             6
#define TIM2_IRQn              28
#define USART1_IRQn            37
#define USART2_IRQn            38

static inline void NVIC_EnableIRQ(uint32_t irq) {
    NVIC->ISER[irq >> 5] = (1UL << (irq & 0x1FUL));
}

static inline void NVIC_DisableIRQ(uint32_t irq) {
    NVIC->ICER[irq >> 5] = (1UL << (irq & 0x1FUL));
}

static inline void NVIC_ClearPendingIRQ(uint32_t irq) {
    NVIC->ICPR[irq >> 5] = (1UL << (irq & 0x1FUL));
}

#endif /* STM32F4_REGS_H */
