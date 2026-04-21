/*
 * task_test.cpp
 *
 *  Created on: Apr 20, 2026
 *      Author: turtl
 */

#include "task_test.hpp"

#include "Core/system.hpp"
#include "Core/Board.hpp"

void Task::test(void*) {

    while(1) {
        Board::LED::indicators[0].toggle();
        vTaskDelay(pdMS_TO_TICKS(1e3));
    }

    vTaskDelete( NULL );
}
