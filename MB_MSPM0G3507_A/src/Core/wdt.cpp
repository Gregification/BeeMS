/*
 * Watchdog for multiple tasks
 * Only the monitor task should call kick(). Individual tasks call checkin().
 */

#include "wdt.hpp"
#include <ti/driverlib/dl_wwdt.h>
#include "Core/system.hpp"

namespace WDT
{

    volatile uint32_t _checkinMask = 0;

    void init()
    {
        DL_WWDT_enablePower(WWDT0);

        DL_WWDT_initWatchdogMode(WWDT0, DL_WWDT_CLOCK_DIVIDE_1,
                                    DL_WWDT_TIMER_PERIOD_15_BITS, // ~1s timeout
                                    DL_WWDT_STOP_IN_SLEEP, // double check- i dont think we want in sleep (right ??)
                                    DL_WWDT_WINDOW_PERIOD_0, DL_WWDT_WINDOW_PERIOD_0
                                    );
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
