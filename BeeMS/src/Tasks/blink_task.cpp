/*
 * blink.cpp
 *
 *  Created on: May 14, 2025
 *      Author: turtl
 */



#include "blink_task.hpp"

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

void Task::Blink::task(void * v)
{
    /*--- parameter validation -----------------------------*/

    if(!v)
        return;
    Args * args = (Args *)v;

    /*------------------------------------------------------*/

    args->pin.defaultInitAsOutput();

    for(;;){
        args->pin.setValue(0);
        vTaskDelay(args->period_ms);
        args->pin.setValue(1);
        vTaskDelay(args->period_ms);
    }
}
