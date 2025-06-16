/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#include "blink_task.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include "Core/system.hpp"

void Task::blink_task(void*) {
    auto &led = System::GPIO::PB26;

    DL_GPIO_initDigitalOutputFeatures(
            led.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );

    DL_GPIO_initDigitalOutput(led.iomux);
    DL_GPIO_clearPins(GPIOPINPUX(led));
    DL_GPIO_enableOutput(GPIOPINPUX(led));

    for(;;){
        DL_GPIO_togglePins(GPIOPINPUX(led));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
