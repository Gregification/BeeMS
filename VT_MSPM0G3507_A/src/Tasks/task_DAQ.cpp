/*
 * task_DAQ.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include "task_DAQ.hpp"
#include "Core/system.hpp"

Task::DAQ_task(void *){

    System::FailHard("DAQ_task ended" NEWLINE);
    vTaskDelete(NULL);
}
