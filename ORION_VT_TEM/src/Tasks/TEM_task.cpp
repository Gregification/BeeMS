/*
 * TEM.cpp
 *
 *  Created on: Mar 25, 2026
 *      Author: turtl
 */

#ifndef SRC_TASKS_TEM_CPP_
#define SRC_TASKS_TEM_CPP_

#include "TEM_task.hpp"
#include "Core/system.hpp"
#include "Core/Board.hpp"


void Task::TEM_task(void*) {
    using namespace BOARD;

    while(1){
        //uint16_t val = 0xbeef;
        UI::SWITCHES::cm.sample_blocking();
        uint16_t val = UI::SWITCHES::cm.getResult();

        System::UART::uart_ui.nputs(ARRANDN("adc: "));
        System::UART::uart_ui.put32d(val);
        System::UART::uart_ui.nputs(ARRANDN(NEWLINE));

        vTaskDelay(pdMS_TO_TICKS(1e3));
    }
}


#endif /* SRC_TASKS_TEM_CPP_ */
