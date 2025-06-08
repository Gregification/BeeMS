/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "system.hpp"

namespace System {
    OCCUPY(PINCM21) //PA10
    OCCUPY(PINCM22) //PA11
}

//void fiddle_task(void *){
//
//
//    vTaskDelete(NULL);
//}

//void fiddle_task(void *){
//    /* Power on GPIO, initialize pins as digital outputs */
//        DL_GPIO_reset(GPIOA);
//        DL_GPIO_reset(GPIOB);
//
//        DL_GPIO_enablePower(GPIOA);
//        DL_GPIO_enablePower(GPIOB);
//        delay_cycles(POWER_STARTUP_DELAY);
//
//
//        DL_GPIO_initDigitalOutput((IOMUX_PINCM50));
//
//        DL_GPIO_initDigitalOutput((IOMUX_PINCM57));
//
//        DL_GPIO_initDigitalOutput((IOMUX_PINCM58));
//
//        DL_GPIO_initDigitalOutput((IOMUX_PINCM33));
//
//        DL_GPIO_clearPins((GPIOB), (DL_GPIO_PIN_22) |
//                          (DL_GPIO_PIN_26) |
//                          (DL_GPIO_PIN_27) |
//                          (DL_GPIO_PIN_16));
//        DL_GPIO_enableOutput((GPIOB),
//                             (DL_GPIO_PIN_22) |
//            (DL_GPIO_PIN_26) |
//            (DL_GPIO_PIN_27) |
//            (DL_GPIO_PIN_16)
//            );
//
//
//        DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
//
//
//        DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
//        DL_SYSCTL_disableHFXT();
//        DL_SYSCTL_disableSYSPLL();
//
//
//
//        /* Default: LED1 and LED3 ON, LED2 OFF */
//        DL_GPIO_clearPins((GPIOB), (DL_GPIO_PIN_26));
//        DL_GPIO_setPins((GPIOB), (DL_GPIO_PIN_22) |
//                                            (DL_GPIO_PIN_27) |
//                                            (DL_GPIO_PIN_16));
//
//        while (1) {
//            /*
//             * Call togglePins API to flip the current value of LEDs 1-3. This
//             * API causes the corresponding HW bits to be flipped by the GPIO HW
//             * without need for additional R-M-W cycles by the processor.
//             */
//            delay_cycles(16);
//            DL_GPIO_togglePins((GPIOB),
//                (DL_GPIO_PIN_22) | (DL_GPIO_PIN_26) |
//                    (DL_GPIO_PIN_27) | (DL_GPIO_PIN_16));
//        }
//    vTaskDelete(NULL);
//}

void fiddle_task(void *){

//    System::UART::partialInit(UART0);
//    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX); // PA10
//    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX); // PA11
//    DL_UART_enable(UART0);

    uint8_t a = 0;
    char str[20] = "no work" NEWLINE;

    for(;;){
        a++;
        System::UART::nputs(UARTUI, str, sizeof(str));
        snprintf(str, sizeof(str), "%d" NEWLINE, a);
        System::UART::nputs(UARTUI, STRANDN("llllllllll"));
        System::UART::nputs(UARTUI, STRANDN(NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelete(NULL);
}
