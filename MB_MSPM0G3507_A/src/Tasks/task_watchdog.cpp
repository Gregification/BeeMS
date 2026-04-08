#include "Core/wdt.hpp"
#include <Tasks/task_watchdog.hpp>

void Task::watchdog_task(void *)
{
    const TickType_t period = pdMS_TO_TICKS(200); // check every .2 sec, so 5 times per window

    while (1)
    {
        vTaskDelay(period);

        if (WDT::allCheckedIn()) // if all tasks checked in, reset wdt
        {
            WDT::kick();
            WDT::resetCheckins(); // clear checkins for the next
        }
    }
}
