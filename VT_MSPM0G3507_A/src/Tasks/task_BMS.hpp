/*
 * task_DAQ.hpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_TASK_BMS_HPP_
#define SRC_TASKS_TASK_BMS_HPP_

namespace Task {
    /**
     * performs the DAQ and comms required to operate as part of a BMS.
     *
     * - does NOT handle board functions
     * - uses CAN FIFO 0
     */
    void BMS_task(void *);
}

#endif /* SRC_TASKS_TASK_BMS_HPP_ */
