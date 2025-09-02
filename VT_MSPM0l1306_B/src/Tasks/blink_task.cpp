/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#include "blink_task.hpp"

#include "Core/system.hpp"

auto &led = System::GPIO::PA0;
void Task::blink_task(void*) {
    /* different itterations of the board use different pins for the blink led
     * see schematic of exact version for correct pin.
     */

    DL_GPIO_initDigitalOutputFeatures(
            led.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );

    DL_GPIO_clearPins(GPIOPINPUX(led));
    DL_GPIO_enableOutput(GPIOPINPUX(led));

    for(;;){
        DL_GPIO_togglePins(GPIOPINPUX(led));
        delay_cycles(32e6);
    }
}
