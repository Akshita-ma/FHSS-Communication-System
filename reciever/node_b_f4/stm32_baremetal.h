#ifndef STM32_BAREMETAL_H
#define STM32_BAREMETAL_H

#include "stm32f4_regs.h"
#include "hal.h"
#include "status.h"

/* Initialize bare-metal clocks and core peripherals for Node B */
void baremetal_init(void);

#endif /* STM32_BAREMETAL_H */
