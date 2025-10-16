/*
 * task_DAQ.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include <Tasks/task_BMS.hpp>
#include "Core/system.hpp"

void Task::BMS_task(void *){

    /* pesudo code
     * {
     *      - POST
     *
     *      - use existing model if available
     *
     *      while(1){
     *
     *          - DAQ
     *
     *          - safety check
     *              - cell balance
     *              - fan pwm
     *
     *          - iterate model
     *              - rebase model if needed
     *              - periodically save model, like ~1h
     *
     *          - periodically/as-necessary TX to CAN ...
     *              - ~1s, DAQ
     *              - ~5s, safe operating ranges
     *
     *          - check and respond to ...
     *              - CAN
     *      }
     * }
     */
    while(1){};
    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}
