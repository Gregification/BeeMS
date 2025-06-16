/*
 * scheduler_watchdog_task.hpp
 *
 *  Created on: Jun 15, 2025
 *      Author: FSAE
 */


#ifndef SRC_TASKS_SCHEDULER_WATCHDOG_TASK_CPP_
#define SRC_TASKS_SCHEDULER_WATCHDOG_TASK_CPP_

#include "Core/system.hpp"

namespace System {
    /* Windowed WatchDog Timer 1 does what we need.
     * WWDT1 will trigger a software and core reset.
     * not using WWDT0 because it's able to trigger the BCR. WWDT0 will be used elsewhere but not for this.
     * - WWDT0/1 difference : FDS.30.1/2229
     * - Boot Configuration Routine : FDS.1.4.2/20
     */
    OCCUPY(WWDT1)
}

namespace Task {
    /** if the FreeRTOS scheduler dies then the board is reset */
    void scheduler_watchdog_task(void *);
}

#endif /* SRC_TASKS_SCHEDULER_WATCHDOG_TASK_CPP_ */
