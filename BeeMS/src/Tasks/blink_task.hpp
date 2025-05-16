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

        constexpr System::GPIO::GPIO_REG LED = {
                .GPIO_PORTn_BASE    = GPIO_PORTN_BASE,
                .GPIO_PIN_n         = GPIO_PIN_0
            };

        /* period time in milliseconds */
        uint16_t period_ms; // e.g: period = 100 / portTICK_PERIOD_MS;  // Delay for 100 ms

        void main(void *);
    }
}


#endif /* SRC_TASKS_BLINK_TASK_HPP_ */
