/*
 * scheduler_watchdog_task.cpp
 *
 *  Created on: Jun 15, 2025
 *      Author: FSAE
 */

#include "scheduler_watchdog_task.hpp"
#include <ti/driverlib/driverlib.h>

void Task::scheduler_watchdog_task(void *){
    DL_TimerG_enablePower(TIMG0);
    DL_WWDT_enablePower(WWDT0);
    delay_cycles(POWER_STARTUP_DELAY);


}
