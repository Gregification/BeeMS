/*
 * task_ui_check.cpp
 *
 *  Created on: Apr 1, 2026
 *      Author: turtl
 */

#include "Core/system.hpp"
#include "task_ui_check.hpp"
#include "Core/Board.hpp"

void Task::task_check_UI(void *) {
    using namespace BOARD;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5e3));

        id = UI::SWITCHES::getUID();

        CAN_MODE cm = UI::SWITCHES::getCANmode();
        if(cm != can_mode) {
            setCanMode(cm);
            DL_GPIO_togglePins(GPIOPINPUX(UI::LED::scheduler));
        }
    }
}
