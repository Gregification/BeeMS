/*
 * blink_task.cpp
 *
 *  Created on: Jun 8, 2025
 *      Author: FSAE
 */

#include <Core/Board.hpp>
#include "Core/system.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include "Tasks/examples/example_blink_task.hpp"


auto &led = BOARD::UI::LED::scheduler;
//auto &led = CEB::Indi::LED::i1;
//auto &led = System::GPIO::PA26;

void Task::blink_task(void*) {

    for(;;){
        DL_GPIO_setPins(GPIOPINPUX(BOARD::UI::LED::scheduler));
        vTaskDelay(pdMS_TO_TICKS(10));
        DL_GPIO_togglePins(GPIOPINPUX(BOARD::UI::LED::scheduler));
        vTaskDelay(pdMS_TO_TICKS(10));
        DL_GPIO_togglePins(GPIOPINPUX(BOARD::UI::LED::scheduler));
        vTaskDelay(pdMS_TO_TICKS(10));
        DL_GPIO_togglePins(GPIOPINPUX(BOARD::UI::LED::scheduler));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
