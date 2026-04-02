/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#ifndef SRC_TASKS_BLINK_TASK_HPP_
#define SRC_TASKS_BLINK_TASK_HPP_

#include "Core/system.hpp"

namespace Task {
    /** purpose is to blink a LED to visually indicate that the FreeRTOS scheduler is running properly */
    void blink_task(void *);
}

#endif /* SRC_TASKS_BLINK_TASK_CPP_ */
