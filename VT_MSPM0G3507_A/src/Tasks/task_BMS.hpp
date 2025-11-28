/*
 * task_DAQ.hpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_TASK_BMS_HPP_
#define SRC_TASKS_TASK_BMS_HPP_

#include "Middleware/BQ769x2/BQ76952.hpp"

namespace Task {
    /**
     * performs the DAQ and comms required to operate as part of a BMS.
     *
     * - handles minimal board functions
     * - uses CAN FIFO 0
     */
    void BMS_task(void *);

    extern BQ76952 bq;
}

#endif /* SRC_TASKS_TASK_BMS_HPP_ */
