/*
 * Test_UART_Task.hpp
 *
 *  Created on: Jul 5, 2025
 *      Author: fsae
 */

#ifndef SRC_TASKS_EXAMPLE_UART_TASK_HPP_
#define SRC_TASKS_EXAMPLE_UART_TASK_HPP_


#include "Core/system.hpp"


namespace Task {
    /** purpose is to blink a LED to visually indicate that the FreeRTOS scheduler is running properly */
    void UART_Task(void *);
}

#endif /* SRC_TASKS_EXAMPLE_UART_TASK_HPP_ */
