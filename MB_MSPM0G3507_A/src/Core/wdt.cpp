/*
 * Watchdog for multiple tasks
 * Only the monitor task should call kick(). Individual tasks call checkin().
 */

#include "watchdog.hpp"
#include <ti/driverlib/driverlib.h>
#include "Core/system.hpp"

namespace WDT
{

    volatile uint32_t _checkinMask = 0;

    void init()
    {
        DL_WWDT_initFreeRunningMode(WWDT0, DL_WWDT_CLOCK_DIVIDE_1,
                                    DL_WWDT_TIMER_PERIOD_15_TO_16_BITS, // ~1000ms timeout
                                    DL_WWDT_SLEEP_MODE_RUN
                                    );

        DL_WWDT_enableModule(WWDT0, DL_WWDT_MODE_WATCHDOG);
    }

    void kick()
    {
        DL_WWDT_restart(WWDT0);
    }

    void checkin(TaskBit bit)
    {
        taskENTER_CRITICAL();
        _checkinMask |= bit;
        taskEXIT_CRITICAL();
    }

    bool allCheckedIn()
    {
        return (_checkinMask & TaskBit::ALL_TASKS) == TaskBit::ALL_TASKS;
    }

    void resetCheckins()
    {
        taskENTER_CRITICAL();
        _checkinMask = 0;
        taskEXIT_CRITICAL();
    }

}
