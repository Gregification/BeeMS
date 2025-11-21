/*
 * SPI_test.hpp
 *
 *  Created on: Aug 26, 2025
 *      Author: turtl
 */

#ifndef SRC_TASKS_EXAMPLE_SPI_TASK_HPP_
#define SRC_TASKS_EXAMPLE_SPI_TASK_HPP_

#include "Core/system.hpp"

namespace System {
    OCCUPY(i2c1);
    OCCUPY(PA15);
    OCCUPY(PA16);
}

namespace Task {
    void SPI_example_task(void *);
}



#endif /* SRC_TASKS_EXAMPLE_SPI_TASK_HPP_ */
