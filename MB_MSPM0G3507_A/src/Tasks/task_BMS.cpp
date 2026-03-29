/*
 * task_BMS.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 *
 *  All voltage tap logic is in here
 */

#include <Tasks/task_BMS.hpp>
#include "Core/system.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"
#include "Core/common.h"

void processCAN();


void Task::BMS_task(void *){
    System::UART::uart_ui.nputs(ARRANDN("BMS_task start" NEWLINE));

    while(true) {
        { // every X mS
            constexpr int32_t dt = 10;
            static TickType_t former = xTaskGetTickCount();
            TickType_t dt_mS = (xTaskGetTickCount() - former) * portTICK_PERIOD_MS;
            if(dt_mS < dt)
                vTaskDelay(pdMS_TO_TICKS(dt - dt_mS));
            former = xTaskGetTickCount();
        }
        processCAN();
    }

    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}

void processCAN() {
    auto & can = System::CANFD::canFD0;

    if(!can.takeResource(pdMS_TO_TICKS(10)))
        return;

    do {
        DL_MCAN_RxFIFOStatus rf;
        rf.num = System::CANFD::OP_RXFIFO;
        DL_MCAN_getRxFIFOStatus(can.reg, &rf);

        if(rf.fillLvl == 0)
            break;

        DL_MCAN_RxBufElement e;
        DL_MCAN_readMsgRam(can.reg, DL_MCAN_MEM_TYPE_FIFO, 0, rf.num, &e);
        DL_MCAN_writeRxFIFOAck(can.reg, rf.num, rf.getIdx);

        uint32_t id = System::CANFD::getID(e);
//        System::UART::uart_ui.nputs(ARRANDN("ID: "));
//        System::UART::uart_ui.putu32d(id);
//        System::UART::uart_ui.nputs(ARRANDN("" NEWLINE));
//
//        for(int i = 0; i < System::CANFD::DLC2Len(&e); i++) {
//            System::UART::uart_ui.putu32h(e.data[i]);
//            System::UART::uart_ui.nputs(ARRANDN("\t"));
//        }
//        System::UART::uart_ui.nputs(ARRANDN(NEWLINE));

    } while(false);

    System::CANFD::canFD0.giveResource();
}
