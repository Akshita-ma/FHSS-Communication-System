#include <stdint.h>

extern uint32_t _stack_top;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

int main(void);

void Reset_Handler(void);
void Default_Handler(void);

/* Core exception handlers */
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

/* External interrupt handlers */
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

/* Vector Table */
__attribute__((section(".isr_vector"), used))
const void * const g_pfnVectors[] = {
    (const void *)&_stack_top,
    (const void *)Reset_Handler,
    (const void *)NMI_Handler,
    (const void *)HardFault_Handler,
    (const void *)MemManage_Handler,
    (const void *)BusFault_Handler,
    (const void *)UsageFault_Handler,
    0, 0, 0, 0,
    (const void *)SVC_Handler,
    (const void *)DebugMon_Handler,
    0,
    (const void *)PendSV_Handler,
    (const void *)SysTick_Handler,
    /* External interrupts */
    (const void *)WWDG_IRQHandler,
    (const void *)PVD_IRQHandler,
    (const void *)TAMP_STAMP_IRQHandler,
    (const void *)RTC_WKUP_IRQHandler,
    (const void *)FLASH_IRQHandler,
    (const void *)RCC_IRQHandler,
    (const void *)EXTI0_IRQHandler,
    (const void *)EXTI1_IRQHandler,
    (const void *)EXTI2_IRQHandler,
    (const void *)EXTI3_IRQHandler,
    (const void *)EXTI4_IRQHandler,
    (const void *)DMA1_Stream0_IRQHandler,
    (const void *)DMA1_Stream1_IRQHandler,
    (const void *)DMA1_Stream2_IRQHandler,
    (const void *)DMA1_Stream3_IRQHandler,
    (const void *)DMA1_Stream4_IRQHandler,
    (const void *)DMA1_Stream5_IRQHandler,
    (const void *)DMA1_Stream6_IRQHandler,
    (const void *)ADC_IRQHandler,
    0, 0, 0, 0,
    (const void *)EXTI9_5_IRQHandler,
    (const void *)TIM1_BRK_TIM9_IRQHandler,
    (const void *)TIM1_UP_TIM10_IRQHandler,
    (const void *)TIM1_TRG_COM_TIM11_IRQHandler,
    (const void *)TIM1_CC_IRQHandler,
    (const void *)TIM2_IRQHandler,
    (const void *)TIM3_IRQHandler,
    (const void *)TIM4_IRQHandler,
    (const void *)I2C1_EV_IRQHandler,
    (const void *)I2C1_ER_IRQHandler,
    (const void *)I2C2_EV_IRQHandler,
    (const void *)I2C2_ER_IRQHandler,
    (const void *)SPI1_IRQHandler,
    (const void *)SPI2_IRQHandler,
    (const void *)USART1_IRQHandler,
    (const void *)USART2_IRQHandler,
    0,
    (const void *)EXTI15_10_IRQHandler,
    (const void *)RTC_Alarm_IRQHandler,
    (const void *)OTG_FS_WKUP_IRQHandler,
    0, 0, 0, 0,
    (const void *)DMA1_Stream7_IRQHandler,
    0,
    (const void *)SDIO_IRQHandler,
    (const void *)TIM5_IRQHandler,
    (const void *)SPI3_IRQHandler,
    0, 0, 0, 0,
    (const void *)DMA2_Stream0_IRQHandler,
    (const void *)DMA2_Stream1_IRQHandler,
    (const void *)DMA2_Stream2_IRQHandler,
    (const void *)DMA2_Stream3_IRQHandler,
    (const void *)DMA2_Stream4_IRQHandler,
    0, 0, 0, 0, 0, 0,
    (const void *)OTG_FS_IRQHandler,
    (const void *)DMA2_Stream5_IRQHandler,
    (const void *)DMA2_Stream6_IRQHandler,
    (const void *)DMA2_Stream7_IRQHandler,
    (const void *)USART6_IRQHandler,
    (const void *)I2C3_EV_IRQHandler,
    (const void *)I2C3_ER_IRQHandler,
    0, 0, 0, 0, 0, 0, 0,
    (const void *)FPU_IRQHandler,
    0, 0,
    (const void *)SPI4_IRQHandler
};

void Reset_Handler(void) {
    uint32_t *p_src = &_sidata;
    uint32_t *p_dst = &_sdata;

    while (p_dst < &_edata) {
        *p_dst++ = *p_src++;
    }

    p_dst = &_sbss;
    while (p_dst < &_ebss) {
        *p_dst++ = 0U;
    }

    main();

    while (1) {
        __asm__ volatile ("nop");
    }
}

void Default_Handler(void) {
    while (1) {
        __asm__ volatile ("nop");
    }
}
