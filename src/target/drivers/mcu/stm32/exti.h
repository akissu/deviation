#ifndef _DTX_STM32_EXTI_H_
#define _DTX_STM32_EXTI_H_

#include <libopencm3/stm32/exti.h>

INLINE static inline uint32_t EXTIx(struct mcu_pin pin)
{
    switch (pin.pin) {
        case GPIO1: return EXTI1;
        case GPIO2: return EXTI2;
        case GPIO3: return EXTI3;
        case GPIO4: return EXTI4;
        case GPIO5: return EXTI5;
        case GPIO6: return EXTI6;
        case GPIO7: return EXTI7;
        case GPIO8: return EXTI8;
        case GPIO9: return EXTI9;
        case GPIO10: return EXTI10;
        case GPIO11: return EXTI11;
        case GPIO12: return EXTI12;
        case GPIO13: return EXTI13;
        case GPIO14: return EXTI14;
        case GPIO15: return EXTI15;
        default: ltassert(); return 0;
    }
}

INLINE static inline uint32_t NVIC_EXTIx_IRQ(struct mcu_pin pin)
{
    switch (pin.pin) {
        case GPIO1: return NVIC_EXTI1_IRQ;
        case GPIO2: return NVIC_EXTI2_IRQ;
        case GPIO3: return NVIC_EXTI3_IRQ;
        case GPIO4: return NVIC_EXTI4_IRQ;
        case GPIO5: return NVIC_EXTI9_5_IRQ;
        case GPIO6: return NVIC_EXTI9_5_IRQ;
        case GPIO7: return NVIC_EXTI9_5_IRQ;
        case GPIO8: return NVIC_EXTI9_5_IRQ;
        case GPIO9: return NVIC_EXTI9_5_IRQ;
        case GPIO10: return NVIC_EXTI15_10_IRQ;
        case GPIO11: return NVIC_EXTI15_10_IRQ;
        case GPIO12: return NVIC_EXTI15_10_IRQ;
        case GPIO13: return NVIC_EXTI15_10_IRQ;
        case GPIO14: return NVIC_EXTI15_10_IRQ;
        case GPIO15: return NVIC_EXTI15_10_IRQ;
        default: ltassert(); return 0;
    }
}

#endif  // _DTX_STM32_EXTI_H_
