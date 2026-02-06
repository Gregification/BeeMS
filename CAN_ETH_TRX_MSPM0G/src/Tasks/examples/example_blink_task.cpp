/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#include <Core/CETRX.hpp>
#include "Core/system.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include "Tasks/examples/example_blink_task.hpp"


auto &led = CEB::Indi::LED::scheduler;
//auto &led = System::GPIO::PA22;

void Task::blink_task(void*) {

//    DL_GPIO_initDigitalOutputFeatures(
//            led.iomux,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
//            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
//            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//        );
//    DL_GPIO_clearPins(GPIOPINPUX(led));
//    DL_GPIO_enableOutput(GPIOPINPUX(led));

    for(;;){
        DL_GPIO_togglePins(GPIOPINPUX(led));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
