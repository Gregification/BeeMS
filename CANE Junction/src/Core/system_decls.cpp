/*
 * system_decls.cpp
 *
 *  Created on: Oct 3, 2025
 *      Author: turtl
 */

/*
 * thisfile hides the definitions so that code in system.cpp cant access it. the footprint headers only disable the header file deceleraitons
 */

#ifndef SRC_CORE_SYSTEM_DECLS_CPP_
#define SRC_CORE_SYSTEM_DECLS_CPP_

#include "system.hpp"

#include <ti/driverlib/driverlib.h>

namespace System {

    namespace GPIO {
        // this may look redundant but the IOMUX and PIN numbers don't necessarily match and
        //      I got tired of proding the data sheet everytime I want a new pin

        // Port A (PA) pins
        const GPIO PA0  = { .port = GPIOA, .pin = DL_GPIO_PIN_0,  .iomux = IOMUX_PINCM1  };
        const GPIO PA1  = { .port = GPIOA, .pin = DL_GPIO_PIN_1,  .iomux = IOMUX_PINCM2  };
        const GPIO PA2  = { .port = GPIOA, .pin = DL_GPIO_PIN_2,  .iomux = IOMUX_PINCM7  };
        const GPIO PA3  = { .port = GPIOA, .pin = DL_GPIO_PIN_3,  .iomux = IOMUX_PINCM8  };
        const GPIO PA4  = { .port = GPIOA, .pin = DL_GPIO_PIN_4,  .iomux = IOMUX_PINCM9  };
        const GPIO PA5  = { .port = GPIOA, .pin = DL_GPIO_PIN_5,  .iomux = IOMUX_PINCM10 };
        const GPIO PA6  = { .port = GPIOA, .pin = DL_GPIO_PIN_6,  .iomux = IOMUX_PINCM11 };
        const GPIO PA7  = { .port = GPIOA, .pin = DL_GPIO_PIN_7,  .iomux = IOMUX_PINCM14 };
        const GPIO PA8  = { .port = GPIOA, .pin = DL_GPIO_PIN_8,  .iomux = IOMUX_PINCM19 };
        const GPIO PA9  = { .port = GPIOA, .pin = DL_GPIO_PIN_9,  .iomux = IOMUX_PINCM20 };
        const GPIO PA10 = { .port = GPIOA, .pin = DL_GPIO_PIN_10, .iomux = IOMUX_PINCM21 };
        const GPIO PA11 = { .port = GPIOA, .pin = DL_GPIO_PIN_11, .iomux = IOMUX_PINCM22 };
        const GPIO PA12 = { .port = GPIOA, .pin = DL_GPIO_PIN_12, .iomux = IOMUX_PINCM34 };
        const GPIO PA13 = { .port = GPIOA, .pin = DL_GPIO_PIN_13, .iomux = IOMUX_PINCM35 };
        const GPIO PA14 = { .port = GPIOA, .pin = DL_GPIO_PIN_14, .iomux = IOMUX_PINCM36 };
        const GPIO PA15 = { .port = GPIOA, .pin = DL_GPIO_PIN_15, .iomux = IOMUX_PINCM37 };
        const GPIO PA16 = { .port = GPIOA, .pin = DL_GPIO_PIN_16, .iomux = IOMUX_PINCM38 };
        const GPIO PA17 = { .port = GPIOA, .pin = DL_GPIO_PIN_17, .iomux = IOMUX_PINCM39 };
        const GPIO PA18 = { .port = GPIOA, .pin = DL_GPIO_PIN_18, .iomux = IOMUX_PINCM40 };
//        const GPIO PA19 = { .port = GPIOA, .pin = DL_GPIO_PIN_19, .iomux = IOMUX_PINCM41 }; // reserved for SWD
//        const GPIO PA20 = { .port = GPIOA, .pin = DL_GPIO_PIN_20, .iomux = IOMUX_PINCM42 }; // reserved for SWD
        const GPIO PA21 = { .port = GPIOA, .pin = DL_GPIO_PIN_21, .iomux = IOMUX_PINCM46 };
        const GPIO PA22 = { .port = GPIOA, .pin = DL_GPIO_PIN_22, .iomux = IOMUX_PINCM47 };
        const GPIO PA23 = { .port = GPIOA, .pin = DL_GPIO_PIN_23, .iomux = IOMUX_PINCM53 };
        const GPIO PA24 = { .port = GPIOA, .pin = DL_GPIO_PIN_24, .iomux = IOMUX_PINCM54 };
        const GPIO PA25 = { .port = GPIOA, .pin = DL_GPIO_PIN_25, .iomux = IOMUX_PINCM55 };
        const GPIO PA26 = { .port = GPIOA, .pin = DL_GPIO_PIN_26, .iomux = IOMUX_PINCM59 };
        const GPIO PA27 = { .port = GPIOA, .pin = DL_GPIO_PIN_27, .iomux = IOMUX_PINCM60 };
        const GPIO PA28 = { .port = GPIOA, .pin = DL_GPIO_PIN_28, .iomux = IOMUX_PINCM3  };
        const GPIO PA29 = { .port = GPIOA, .pin = DL_GPIO_PIN_29, .iomux = IOMUX_PINCM4  };
        const GPIO PA30 = { .port = GPIOA, .pin = DL_GPIO_PIN_30, .iomux = IOMUX_PINCM5  };
        const GPIO PA31 = { .port = GPIOA, .pin = DL_GPIO_PIN_31, .iomux = IOMUX_PINCM6  };

        // Port B (PB) pins
        const GPIO PB0  = { .port = GPIOB, .pin = DL_GPIO_PIN_0,  .iomux = IOMUX_PINCM12 };
        const GPIO PB1  = { .port = GPIOB, .pin = DL_GPIO_PIN_1,  .iomux = IOMUX_PINCM13 };
        const GPIO PB2  = { .port = GPIOB, .pin = DL_GPIO_PIN_2,  .iomux = IOMUX_PINCM15 };
        const GPIO PB3  = { .port = GPIOB, .pin = DL_GPIO_PIN_3,  .iomux = IOMUX_PINCM16 };
        const GPIO PB4  = { .port = GPIOB, .pin = DL_GPIO_PIN_4,  .iomux = IOMUX_PINCM17 };
        const GPIO PB5  = { .port = GPIOB, .pin = DL_GPIO_PIN_5,  .iomux = IOMUX_PINCM18 };
        const GPIO PB6  = { .port = GPIOB, .pin = DL_GPIO_PIN_6,  .iomux = IOMUX_PINCM23 };
        const GPIO PB7  = { .port = GPIOB, .pin = DL_GPIO_PIN_7,  .iomux = IOMUX_PINCM24 };
        const GPIO PB8  = { .port = GPIOB, .pin = DL_GPIO_PIN_8,  .iomux = IOMUX_PINCM25 };
        const GPIO PB9  = { .port = GPIOB, .pin = DL_GPIO_PIN_9,  .iomux = IOMUX_PINCM26 };
        const GPIO PB10 = { .port = GPIOB, .pin = DL_GPIO_PIN_10, .iomux = IOMUX_PINCM27 };
        const GPIO PB11 = { .port = GPIOB, .pin = DL_GPIO_PIN_11, .iomux = IOMUX_PINCM28 };
        const GPIO PB12 = { .port = GPIOB, .pin = DL_GPIO_PIN_12, .iomux = IOMUX_PINCM29 };
        const GPIO PB13 = { .port = GPIOB, .pin = DL_GPIO_PIN_13, .iomux = IOMUX_PINCM30 };
        const GPIO PB14 = { .port = GPIOB, .pin = DL_GPIO_PIN_14, .iomux = IOMUX_PINCM31 };
        const GPIO PB15 = { .port = GPIOB, .pin = DL_GPIO_PIN_15, .iomux = IOMUX_PINCM32 };
        const GPIO PB16 = { .port = GPIOB, .pin = DL_GPIO_PIN_16, .iomux = IOMUX_PINCM33 };
        const GPIO PB17 = { .port = GPIOB, .pin = DL_GPIO_PIN_17, .iomux = IOMUX_PINCM43 };
        const GPIO PB18 = { .port = GPIOB, .pin = DL_GPIO_PIN_18, .iomux = IOMUX_PINCM44 };
        const GPIO PB19 = { .port = GPIOB, .pin = DL_GPIO_PIN_19, .iomux = IOMUX_PINCM45 };
        const GPIO PB20 = { .port = GPIOB, .pin = DL_GPIO_PIN_20, .iomux = IOMUX_PINCM48 };
        const GPIO PB21 = { .port = GPIOB, .pin = DL_GPIO_PIN_21, .iomux = IOMUX_PINCM49 };
        const GPIO PB22 = { .port = GPIOB, .pin = DL_GPIO_PIN_22, .iomux = IOMUX_PINCM50 };
        const GPIO PB23 = { .port = GPIOB, .pin = DL_GPIO_PIN_23, .iomux = IOMUX_PINCM51 };
        const GPIO PB24 = { .port = GPIOB, .pin = DL_GPIO_PIN_24, .iomux = IOMUX_PINCM52 };
        const GPIO PB25 = { .port = GPIOB, .pin = DL_GPIO_PIN_25, .iomux = IOMUX_PINCM56 };
        const GPIO PB26 = { .port = GPIOB, .pin = DL_GPIO_PIN_26, .iomux = IOMUX_PINCM57 };
        const GPIO PB27 = { .port = GPIOB, .pin = DL_GPIO_PIN_27, .iomux = IOMUX_PINCM58 };
        const GPIO PB28 = { .port = GPIOB, .pin = DL_GPIO_PIN_28, .iomux = IOMUX_PINCM65 };
        const GPIO PB29 = { .port = GPIOB, .pin = DL_GPIO_PIN_29, .iomux = IOMUX_PINCM66 };
        const GPIO PB30 = { .port = GPIOB, .pin = DL_GPIO_PIN_30, .iomux = IOMUX_PINCM67 };
        const GPIO PB31 = { .port = GPIOB, .pin = DL_GPIO_PIN_31, .iomux = IOMUX_PINCM68 };

        // Port C (PC) pins
        const GPIO PC0  = { .port = GPIOC, .pin = DL_GPIO_PIN_0,  .iomux = IOMUX_PINCM74 };
        const GPIO PC1  = { .port = GPIOC, .pin = DL_GPIO_PIN_1,  .iomux = IOMUX_PINCM75 };
        const GPIO PC2  = { .port = GPIOC, .pin = DL_GPIO_PIN_2,  .iomux = IOMUX_PINCM76 };
        const GPIO PC3  = { .port = GPIOC, .pin = DL_GPIO_PIN_3,  .iomux = IOMUX_PINCM77 };
        const GPIO PC4  = { .port = GPIOC, .pin = DL_GPIO_PIN_4,  .iomux = IOMUX_PINCM78 };
        const GPIO PC5  = { .port = GPIOC, .pin = DL_GPIO_PIN_5,  .iomux = IOMUX_PINCM79 };
        const GPIO PC6  = { .port = GPIOC, .pin = DL_GPIO_PIN_6,  .iomux = IOMUX_PINCM84 };
        const GPIO PC7  = { .port = GPIOC, .pin = DL_GPIO_PIN_7,  .iomux = IOMUX_PINCM85 };
        const GPIO PC8  = { .port = GPIOC, .pin = DL_GPIO_PIN_8,  .iomux = IOMUX_PINCM86 };
        const GPIO PC9  = { .port = GPIOC, .pin = DL_GPIO_PIN_9,  .iomux = IOMUX_PINCM87 };
        const GPIO PC10 = { .port = GPIOC, .pin = DL_GPIO_PIN_10, .iomux = IOMUX_PINCM88 };
        const GPIO PC11 = { .port = GPIOC, .pin = DL_GPIO_PIN_11, .iomux = IOMUX_PINCM89 };
        const GPIO PC12 = { .port = GPIOC, .pin = DL_GPIO_PIN_12, .iomux = IOMUX_PINCM61 };
        const GPIO PC13 = { .port = GPIOC, .pin = DL_GPIO_PIN_13, .iomux = IOMUX_PINCM62 };
        const GPIO PC14 = { .port = GPIOC, .pin = DL_GPIO_PIN_14, .iomux = IOMUX_PINCM63 };
        const GPIO PC15 = { .port = GPIOC, .pin = DL_GPIO_PIN_15, .iomux = IOMUX_PINCM64 };
        const GPIO PC16 = { .port = GPIOC, .pin = DL_GPIO_PIN_16, .iomux = IOMUX_PINCM69 };
        const GPIO PC17 = { .port = GPIOC, .pin = DL_GPIO_PIN_17, .iomux = IOMUX_PINCM70 };
        const GPIO PC18 = { .port = GPIOC, .pin = DL_GPIO_PIN_18, .iomux = IOMUX_PINCM71 };
        const GPIO PC19 = { .port = GPIOC, .pin = DL_GPIO_PIN_19, .iomux = IOMUX_PINCM72 };
        const GPIO PC20 = { .port = GPIOC, .pin = DL_GPIO_PIN_20, .iomux = IOMUX_PINCM73 };
        const GPIO PC21 = { .port = GPIOC, .pin = DL_GPIO_PIN_21, .iomux = IOMUX_PINCM80 };
        const GPIO PC22 = { .port = GPIOC, .pin = DL_GPIO_PIN_22, .iomux = IOMUX_PINCM81 };
        const GPIO PC23 = { .port = GPIOC, .pin = DL_GPIO_PIN_23, .iomux = IOMUX_PINCM82 };
        const GPIO PC24 = { .port = GPIOC, .pin = DL_GPIO_PIN_24, .iomux = IOMUX_PINCM83 };
        const GPIO PC25 = { .port = GPIOC, .pin = DL_GPIO_PIN_25, .iomux = IOMUX_PINCM90 };
        const GPIO PC26 = { .port = GPIOC, .pin = DL_GPIO_PIN_26, .iomux = IOMUX_PINCM91 };
        const GPIO PC27 = { .port = GPIOC, .pin = DL_GPIO_PIN_27, .iomux = IOMUX_PINCM92 };
        const GPIO PC28 = { .port = GPIOC, .pin = DL_GPIO_PIN_28, .iomux = IOMUX_PINCM93 };
        const GPIO PC29 = { .port = GPIOC, .pin = DL_GPIO_PIN_29, .iomux = IOMUX_PINCM94 };
    }
}


#endif /* SRC_CORE_SYSTEM_DECLS_CPP_ */
