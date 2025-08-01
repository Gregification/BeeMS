/*
 * CAN_Task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#ifndef SRC_TASKS_CAN_TASK_CPP_
#define SRC_TASKS_CAN_TASK_CPP_

#include "Core/system.hpp"

namespace System {
    OCCUPY(PA12)
    OCCUPY(PA13)
}

namespace Task {
    /** purpose is to communicate with the CAN on the PCB */
    void CAN_test(void *);
}

#endif /* SRC_TASKS_CAN_TASK_CPP_ */
