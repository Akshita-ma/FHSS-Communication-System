#ifndef STM32F1_REGS_H
#define STM32F1_REGS_H

#include <stdint.h>

/*
 * Minimal bare-metal register definitions for STM32F103 (RM0008).
 * 100% direct register access without third-party headers.
 */

/* Memory map base addresses */
#define FLASH_BASE          0x08000000UL
#define SRAM_BASE           0x20000000UL
#define PERIPH_BASE         0x40000000UL

#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x00018000UL)

/* APB1 Peripherals */
#define TIM2_BASE           (APB1PERIPH_BASE + 0x00000000UL)
#define USART2_BASE         (APB1PERIPH_BASE + 0x00004400UL)

/* APB2 Peripherals */
#define AFIO_BASE           (APB2PERIPH_BASE + 0x00000000UL)
#define EXTI_BASE           (APB2PERIPH_BASE + 0x00000400UL)
#define GPIOA_BASE          (APB2PERIPH_BASE + 0x00000800UL)
#define GPIOB_BASE          (APB2PERIPH_BASE + 0x00000C00UL)
#define GPIOC_BASE          (APB2PERIPH_BASE + 0x00001000UL)
#define SPI1_BASE           (APB2PERIPH_BASE + 0x00003000UL)
#define USART1_BASE         (APB2PERIPH_BASE + 0x00003800UL)

/* AHB Peripherals */
#define RCC_BASE            (AHBPERIPH_BASE + 0x00009000UL)

/* System Control Space (ARM Cortex-M3 Core) */
#define SCS_BASE            0xE000E000UL
#define SysTick_BASE        (SCS_BASE + 0x0010UL)
#define NVIC_BASE           (SCS_BASE + 0x0100UL)

/* Register structures */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR[4];
    uint32_t          RESERVED0;
    volatile uint32_t MAPR2;
} AFIO_TypeDef;

typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
} SysTick_TypeDef;

typedef struct {
    volatile uint32_t ISER[8];
    uint32_t          RESERVED0[24];
    volatile uint32_t ICER[8];
    uint32_t          RESERVED1[24];
    volatile uint32_t ISPR[8];
    uint32_t          RESERVED2[24];
    volatile uint32_t ICPR[8];
    uint32_t          RESERVED3[24];
    volatile uint32_t IABR[8];
    uint32_t          RESERVED4[56];
    volatile uint8_t  IP[240];
} NVIC_TypeDef;

/* Peripheral instance pointers */
#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)
#define AFIO                ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE)
#define USART1              ((USART_TypeDef *)USART1_BASE)
#define USART2              ((USART_TypeDef *)USART2_BASE)
#define SPI1                ((SPI_TypeDef *)SPI1_BASE)
#define TIM2                ((TIM_TypeDef *)TIM2_BASE)
#define SysTick             ((SysTick_TypeDef *)SysTick_BASE)
#define NVIC                ((NVIC_TypeDef *)NVIC_BASE)

/* RCC APB2 bit definitions */
#define RCC_APB2ENR_AFIOEN  (1UL << 0)
#define RCC_APB2ENR_IOPAEN  (1UL << 2)
#define RCC_APB2ENR_IOPBEN  (1UL << 3)
#define RCC_APB2ENR_IOPCEN  (1UL << 4)
#define RCC_APB2ENR_SPI1EN  (1UL << 12)
#define RCC_APB2ENR_USART1EN (1UL << 14)

/* RCC APB1 bit definitions */
#define RCC_APB1ENR_TIM2EN   (1UL << 0)
#define RCC_APB1ENR_USART2EN (1UL << 17)

/* USART bit definitions */
#define USART_SR_TXE        (1UL << 7)
#define USART_SR_TC         (1UL << 6)
#define USART_SR_RXNE       (1UL << 5)
#define USART_SR_ORE        (1UL << 3)
#define USART_CR1_UE        (1UL << 13)
#define USART_CR1_RXNEIE    (1UL << 5)
#define USART_CR1_TE        (1UL << 3)
#define USART_CR1_RE        (1UL << 2)

/* TIM bit definitions */
#define TIM_CR1_CEN         (1UL << 0)
#define TIM_CR1_OPM         (1UL << 3)
#define TIM_DIER_UIE        (1UL << 0)
#define TIM_SR_UIF          (1UL << 0)

/* SysTick bit definitions */
#define SysTick_CTRL_ENABLE     (1UL << 0)
#define SysTick_CTRL_TICKINT    (1UL << 1)
#define SysTick_CTRL_CLKSOURCE  (1UL << 2)
#define SysTick_CTRL_COUNTFLAG  (1UL << 16)

/* SPI bit definitions */
#define SPI_CR1_CPHA        (1UL << 0)
#define SPI_CR1_CPOL        (1UL << 1)
#define SPI_CR1_MSTR        (1UL << 2)
#define SPI_CR1_BR_DIV8     (2UL << 3)
#define SPI_CR1_SPE         (1UL << 6)
#define SPI_CR1_LSBFIRST    (1UL << 7)
#define SPI_CR1_SSI         (1UL << 8)
#define SPI_CR1_SSM         (1UL << 9)

#define SPI_SR_RXNE         (1UL << 0)
#define SPI_SR_TXE          (1UL << 1)
#define SPI_SR_BSY          (1UL << 7)

#endif /* STM32F1_REGS_H */
