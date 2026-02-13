/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#include "Core/system.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include "Tasks/examples/example_blink_task.hpp"

#include "Core/MasterBoard.hpp"

void Task::blink_task(void*) {

    /* different iterations of the board use different pins for the blink led
     * see schematic of exact version for correct pin.
     */
//    auto &led = System::GPIO::PA14; // EVAL purple board
//    auto &led = System::GPIO::PB27; // EVAL blue board
//    auto &led = System::GPIO::PA14; // EVAL green board
//    auto &led = System::GPIO::PB26; // LP RED
//    auto &led = System::GPIO::PB27; // LP GREEN
//    auto &led = System::GPIO::PA7; // EVAL v3
//    auto &led = MstrB::Indi::LED::i1;
//    auto &led = MstrB::Indi::LED::i2;
//    auto &led = MstrB::Indi::LED::fault;
    auto &led = MstrB::Indi::LED::scheduler;

//    auto &led = System::GPIO::PA26;

    DL_GPIO_initDigitalOutputFeatures(
            led.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(led));
    DL_GPIO_enableOutput(GPIOPINPUX(led));

    for(;;){
        DL_GPIO_togglePins(GPIOPINPUX(led));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }


}
