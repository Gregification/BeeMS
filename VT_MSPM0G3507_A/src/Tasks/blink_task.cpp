/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#include "blink_task.hpp"

#include <FreeRTOS.h>
#include <task.h>

#define GPIO_PAIR GPIOB, DL_GPIO_PIN_12

void Task::blink_task(void*) {
    DL_GPIO_initDigitalOutputFeatures(
            IOMUX_PINCM29,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_initDigitalOutput(IOMUX_PINCM29);
    DL_GPIO_clearPins(GPIO_PAIR);
    DL_GPIO_enableOutput(GPIO_PAIR);

    for(;;){
        DL_GPIO_togglePins(GPIO_PAIR);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
