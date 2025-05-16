/*
 * blink_task.hpp
 *
 *  Created on: May 14, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_BLINK_TASK_HPP_
#define SRC_TASKS_BLINK_TASK_HPP_

#include <FreeRTOS.h>
#include <task.h>

#include "Core/system.hpp"

namespace System {
    OCCUPY(PN0) // LED / LP D2
}

namespace Task {
    namespace Blink {
        constexpr uint16_t PERIOD_NORMAL    = 1000  / portTICK_PERIOD_MS;
        constexpr uint16_t PERIOD_WARNING   = 500   / portTICK_PERIOD_MS;
        constexpr uint16_t PERIOD_FAULT     = 100   / portTICK_PERIOD_MS;

        constexpr System::GPIO::GPIO_REG LED = {
                .GPIO_PORTn_BASE    = GPIO_PORTN_BASE,
                .GPIO_PIN_n         = GPIO_PIN_0
            };

        /* blink period time in milliseconds */
        extern uint16_t period_ms;

        void main(void *);
    }
}


#endif /* SRC_TASKS_BLINK_TASK_HPP_ */
