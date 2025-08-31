/*
 * scheduler_watchdog_task.cpp
 *
 *  Created on: Jun 15, 2025
 *      Author: FSAE
 */

#include "scheduler_watchdog_task.hpp"
#include <ti/driverlib/driverlib.h>


/*
 * i couldnt get it to work, dont use it until you fix it
 */
void Task::scheduler_watchdog_task(void *){
//    DL_WWDT_enablePower(WWDT1);
//    delay_cycles(POWER_STARTUP_DELAY);
//
//    DL_WWDT_initWatchdogMode(
//            WWDT1,
//            DL_WWDT_CLOCK_DIVIDE::DL_WWDT_CLOCK_DIVIDE_4,
//            DL_WWDT_TIMER_PERIOD::DL_WWDT_TIMER_PERIOD_21_BITS,
//            DL_WWDT_SLEEP_MODE::DL_WWDT_RUN_IN_SLEEP,
//            DL_WWDT_WINDOW_PERIOD::DL_WWDT_WINDOW_PERIOD_87,
//            DL_WWDT_WINDOW_PERIOD::DL_WWDT_WINDOW_PERIOD_0
//        );
//    DL_WWDT_setActiveWindow(WWDT1, DL_WWDT_WINDOW0);
//
//    // 4 * 2^21 / 4e6 = 2.097s
//    // %87 ~= 1.82s
//
//    while(true){
//        DL_WWDT_restart(WWDT1);
//        vTaskDelay(pdMS_TO_TICKS(1e3));
//    }

    System::FailHard("scheduler watch dog task ended");
    vTaskDelete(NULL);
}
