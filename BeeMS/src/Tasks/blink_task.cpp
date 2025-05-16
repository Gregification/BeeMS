/*
 * blink.cpp
 *
 *  Created on: May 14, 2025
 *      Author: turtl
 */



#include "blink_task.hpp"

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

// default
uint16_t Task::Blink::period_ms = Task::Blink::PERIOD_NORMAL;

void Task::Blink::main(void*)
{
    LED.defaultInitAsOutput();

    for(;;){
        LED.setValue(0);
        vTaskDelay(period_ms);
        LED.setValue(1);
        vTaskDelay(period_ms);
    }
}
