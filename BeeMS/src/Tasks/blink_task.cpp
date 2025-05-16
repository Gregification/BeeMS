/*
 * blink.cpp
 *
 *  Created on: May 14, 2025
 *      Author: turtl
 */



#include "blink_task.hpp"

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

void Task::Blink::main(void*)
{
    LED.defaultInitAsOutput();

    period_ms = 1000 / portTICK_PERIOD_MS;

    for(;;){
        LED.setValue(0);
        vTaskDelay(period_ms);
        LED.setValue(1);
        vTaskDelay(period_ms);
    }
}
