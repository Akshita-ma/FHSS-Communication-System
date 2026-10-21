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

/* Declare core system exception handlers */
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

/* Declare peripheral interrupt handlers (including EXTI0, TIM2, USART1) */
void WWDG_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TAMPER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void USB_HP_CAN1_TX_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void USB_LP_CAN1_RX0_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
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
void USART3_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void RTCAlarm_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));

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
    (const void *)TAMPER_IRQHandler,
    (const void *)RTC_IRQHandler,
    (const void *)FLASH_IRQHandler,
    (const void *)RCC_IRQHandler,
    (const void *)EXTI0_IRQHandler,
    (const void *)EXTI1_IRQHandler,
    (const void *)EXTI2_IRQHandler,
    (const void *)EXTI3_IRQHandler,
    (const void *)EXTI4_IRQHandler,
    (const void *)DMA1_Channel1_IRQHandler,
    (const void *)DMA1_Channel2_IRQHandler,
    (const void *)DMA1_Channel3_IRQHandler,
    (const void *)DMA1_Channel4_IRQHandler,
    (const void *)DMA1_Channel5_IRQHandler,
    (const void *)DMA1_Channel6_IRQHandler,
    (const void *)DMA1_Channel7_IRQHandler,
    (const void *)ADC1_2_IRQHandler,
    (const void *)USB_HP_CAN1_TX_IRQHandler,
    (const void *)USB_LP_CAN1_RX0_IRQHandler,
    (const void *)CAN1_RX1_IRQHandler,
    (const void *)CAN1_SCE_IRQHandler,
    (const void *)EXTI9_5_IRQHandler,
    (const void *)TIM1_BRK_IRQHandler,
    (const void *)TIM1_UP_IRQHandler,
    (const void *)TIM1_TRG_COM_IRQHandler,
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
    (const void *)USART3_IRQHandler,
    (const void *)EXTI15_10_IRQHandler,
    (const void *)RTCAlarm_IRQHandler,
    (const void *)USBWakeUp_IRQHandler
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
