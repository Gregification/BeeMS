/*
 * blink_task.hpp
 *
 *  Created on: May 14, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_BLINK_TASK_HPP_
#define SRC_TASKS_BLINK_TASK_HPP_

#include "Core/system.hpp"

namespace System {
    OCCUPY(PA1)
}

namespace Task {
    void blink(void*);
}


#endif /* SRC_TASKS_BLINK_TASK_HPP_ */
