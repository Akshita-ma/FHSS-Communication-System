#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

int main(void);

void Reset_Handler(void);
void Default_Handler(void);

/* Cortex-M4 Core Exceptions */
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

/* Peripheral Interrupts */
void WWDG_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void USART6_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void SPI4_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector"), used))
const void *g_pfnVectors[] = {
    &_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    /* Peripheral Interrupts */
    WWDG_IRQHandler,               /* 0 */
    PVD_IRQHandler,                /* 1 */
    TAMP_STAMP_IRQHandler,         /* 2 */
    RTC_WKUP_IRQHandler,           /* 3 */
    FLASH_IRQHandler,              /* 4 */
    RCC_IRQHandler,                /* 5 */
    EXTI0_IRQHandler,              /* 6 */
    EXTI1_IRQHandler,              /* 7 */
    EXTI2_IRQHandler,              /* 8 */
    EXTI3_IRQHandler,              /* 9 */
    EXTI4_IRQHandler,              /* 10 */
    DMA1_Stream0_IRQHandler,       /* 11 */
    DMA1_Stream1_IRQHandler,       /* 12 */
    DMA1_Stream2_IRQHandler,       /* 13 */
    DMA1_Stream3_IRQHandler,       /* 14 */
    DMA1_Stream4_IRQHandler,       /* 15 */
    DMA1_Stream5_IRQHandler,       /* 16 */
    DMA1_Stream6_IRQHandler,       /* 17 */
    ADC_IRQHandler,                /* 18 */
    0, 0, 0, 0,                    /* 19 - 22 */
    EXTI9_5_IRQHandler,            /* 23 */
    TIM1_BRK_TIM9_IRQHandler,      /* 24 */
    TIM1_UP_TIM10_IRQHandler,      /* 25 */
    TIM1_TRG_COM_TIM11_IRQHandler, /* 26 */
    TIM1_CC_IRQHandler,            /* 27 */
    TIM2_IRQHandler,               /* 28 */
    TIM3_IRQHandler,               /* 29 */
    TIM4_IRQHandler,               /* 30 */
    I2C1_EV_IRQHandler,            /* 31 */
    I2C1_ER_IRQHandler,            /* 32 */
    I2C2_EV_IRQHandler,            /* 33 */
    I2C2_ER_IRQHandler,            /* 34 */
    SPI1_IRQHandler,               /* 35 */
    SPI2_IRQHandler,               /* 36 */
    USART1_IRQHandler,             /* 37 */
    USART2_IRQHandler,             /* 38 */
    0,                             /* 39 */
    EXTI15_10_IRQHandler,          /* 40 */
    RTC_Alarm_IRQHandler,          /* 41 */
    OTG_FS_WKUP_IRQHandler,        /* 42 */
    0, 0, 0, 0,                    /* 43 - 46 */
    DMA1_Stream7_IRQHandler,       /* 47 */
    0,                             /* 48 */
    SDIO_IRQHandler,               /* 49 */
    TIM5_IRQHandler,               /* 50 */
    SPI3_IRQHandler,               /* 51 */
    0, 0, 0, 0,                    /* 52 - 55 */
    DMA2_Stream0_IRQHandler,       /* 56 */
    DMA2_Stream1_IRQHandler,       /* 57 */
    DMA2_Stream2_IRQHandler,       /* 58 */
    DMA2_Stream3_IRQHandler,       /* 59 */
    DMA2_Stream4_IRQHandler,       /* 60 */
    0, 0,                          /* 61 - 62 */
    0, 0, 0, 0,                    /* 63 - 66 */
    OTG_FS_IRQHandler,             /* 67 */
    DMA2_Stream5_IRQHandler,       /* 68 */
    DMA2_Stream6_IRQHandler,       /* 69 */
    DMA2_Stream7_IRQHandler,       /* 70 */
    USART6_IRQHandler,             /* 71 */
    I2C3_EV_IRQHandler,            /* 72 */
    I2C3_ER_IRQHandler,            /* 73 */
    0, 0, 0, 0, 0, 0, 0,           /* 74 - 80 */
    FPU_IRQHandler,                /* 81 */
    0, 0,                          /* 82 - 83 */
    SPI4_IRQHandler                /* 84 */
};

void Reset_Handler(void) {
    uint32_t *p_src = &_sidata;
    uint32_t *p_dst = &_sdata;

    /* Copy .data segment to RAM */
    while (p_dst < &_edata) {
        *p_dst++ = *p_src++;
    }

    /* Zero initialize .bss segment */
    p_dst = &_sbss;
    while (p_dst < &_ebss) {
        *p_dst++ = 0;
    }

    /* Jump to main */
    main();

    /* Infinite loop if main returns */
    while (1) {
    }
}

void Default_Handler(void) {
    while (1) {
    }
}
