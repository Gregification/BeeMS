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
#include <Tasks/task_non_BMS.hpp>
#include "Core/system.hpp"

void Task::non_BMS_functions_task(void *){

    /* pesudo code
     * {
     *      while(1){
     *
     *          - check hardware configurations ...
     *              - nothing for now
     *
     *          - TX up-time over can
     *
     *          - check and respond to ...
     *              - UART UI
     *              - CAN
     *                  - negotiate a new CAN ID if theres a conflict
     *
     *      }
     * }
     */

    System::FailHard("non_BMS_functions_task ended" NEWLINE);
    vTaskDelete(NULL);
}
