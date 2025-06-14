/*
 * blink.cpp
 *
 *  Created on: May 14, 2025
 *      Author: turtl
 */



#include "blink_task.hpp"

#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

void Task::Blink::main(void * v)
{
    /*--- parameter validation -----------------------------*/

    if(!v)
        return;
    Args * args = (Args *)v;

    /*------------------------------------------------------*/

    args->pin.defaultInitAsOutput();

    for(;;){
//        System::nputsUIUART(STRANDN("blink" NEWLINE));
        args->pin.setValue(0);
        vTaskDelay(args->period_ms);
        args->pin.setValue(1);
        vTaskDelay(args->period_ms);
    }
}
