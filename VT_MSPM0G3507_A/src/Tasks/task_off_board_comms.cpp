/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf
 */

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/task_off_board_comms.hpp>
#include "Core/system.hpp"


void Task::external_communications(void *){

    System::FailHard("DAQ_task ended" NEWLINE);
    vTaskDelete(NULL);
}
