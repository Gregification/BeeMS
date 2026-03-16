/*
 * task_BMS.cpp
 *
 *  Created on: Mar 16, 2026
 *      Author: turtl
 */

#include "task_BMS.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"
#include "Core/system.hpp"
#include "Core/VT.hpp"

using namespace System;

void Task::BMS(void *) {
    using namespace VT;

    auto & uart = System::uart_ui;

    uart.nputs(ARRANDN("BMS_task start" NEWLINE));

    uart.nputs(ARRANDN("postScheduler init ..." NEWLINE));
    VT::postScheduler_init();

    while(1);

    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}
