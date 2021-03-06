#ifndef _DEVO7E256_HARDWARE_H_
#define _DEVO7E256_HARDWARE_H_

#include "target/drivers/mcu/stm32/gpio.h"
#include "../common/hardware_t8sg_buttonmatrix.h"

// Analog inputs
#define ADC_CHANNELS { \
    ADC_CHAN(GPIOC, GPIO2, CHAN_INVERT),  /* ADC123_12 - INP_AIL */ \
    ADC_CHAN(GPIOC, GPIO1, CHAN_NONINV),  /* ADC123_11 - INP ELE */ \
    ADC_CHAN(GPIOC, GPIO0, CHAN_INVERT),  /* ADC123_10 - INP_THR */ \
    ADC_CHAN(GPIOC, GPIO3, CHAN_NONINV),  /* ADC123_13 - INP_RUD */ \
    ADC_CHAN(GPIOA, GPIO0, CHAN_INVERT),  /* ADC123_0  - INP_AUX4 */ \
    ADC_CHAN(GPIOA, GPIO4, CHAN_INVERT),  /* ADC12_4   - INP_AUX5 */ \
    ADC_CHAN(0, 16, CHAN_NONINV),         /* TEMPERATURE */ \
    ADC_CHAN(GPIOC, GPIO4, CHAN_NONINV),  /* ADC12_14  */ \
    }

#define SWITCHES \
    TWO_WAY(INP_HOLD, (GPIOC, GPIO11), CHAN_INVERT) \
    TWO_WAY(INP_FMOD, (GPIOC, GPIO10), CHAN_INVERT)

#define EXTRA_SWITCHES \
    EXTRA_3WAY(INP_SWA, 0x04, 0x08, CHAN_NONINV, SWITCH_3x2) \
    EXTRA_3WAY(INP_SWB, 0x01, 0x02, CHAN_NONINV, SWITCH_3x2) \
    EXTRA_2WAY(INP_SWG, 0x04, 0x08, CHAN_INVERT, SWITCH_2x2) \
    EXTRA_2WAY(INP_SWH, 0x01, 0x02, CHAN_INVERT, SWITCH_2x2)

#define REPLACEMENT_SWITCHES \
    EXTRA_3WAY(INP_SWA, CH(24), CH(23), CHAN_NONINV, SWITCH_3x1) \
    EXTRA_3WAY(INP_SWB, CH(26), CH(25), CHAN_NONINV, SWITCH_3x2) \
    EXTRA_3WAY(INP_SWC, CH(28), CH(27), CHAN_NONINV, SWITCH_3x3) \
    EXTRA_3WAY(INP_SWD, CH(30), CH(29), CHAN_NONINV, SWITCH_3x4) \
    EXTRA_2WAY(INP_SWA, CH(25), CH(25), CHAN_NONINV, SWITCH_2x8) \
    EXTRA_2WAY(INP_SWB, CH(26), CH(26), CHAN_NONINV, SWITCH_2x7) \
    EXTRA_2WAY(INP_SWC, CH(27), CH(27), CHAN_NONINV, SWITCH_2x6) \
    EXTRA_2WAY(INP_SWD, CH(28), CH(28), CHAN_NONINV, SWITCH_2x5) \
    EXTRA_2WAY(INP_SWE, CH(29), CH(29), CHAN_NONINV, SWITCH_2x4) \
    EXTRA_2WAY(INP_SWF, CH(30), CH(30), CHAN_NONINV, SWITCH_2x3) \
    EXTRA_2WAY(INP_SWG, CH(31), CH(31), CHAN_NONINV, SWITCH_2x2) \
    EXTRA_2WAY(INP_SWH, CH(32), CH(32), CHAN_NONINV, SWITCH_2x1) \

#include "target/tx/devo/common/extra_switch_7e.h"

#endif  // _DEVO7E256_HARDWARE_H_
