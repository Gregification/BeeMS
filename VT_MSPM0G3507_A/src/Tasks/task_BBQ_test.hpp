/*
 * task_DAQ.hpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_TASK_BBQ_TEST_HPP_
#define SRC_TASKS_TASK_BBQ_TEST_HPP_

#include "Middleware/BQ769x2/BQ76952.hpp"

namespace Task {
    /**
     * for determining if interface with the bbq is cooked
     */
    void BBQ_test_task(void *);

    //extern BQ76952 bq;
}

#endif /* SRC_TASKS_TASK_BBQ_TEST_HPP_ */
