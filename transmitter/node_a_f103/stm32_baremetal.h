#ifndef STM32_BAREMETAL_H
#define STM32_BAREMETAL_H

#include "stm32f1_regs.h"
#include "hal.h"
#include "status.h"

/* Initialize bare-metal clocks and core peripherals for Node A (STM32F103) */
void baremetal_init(void);

#endif /* STM32_BAREMETAL_H */
