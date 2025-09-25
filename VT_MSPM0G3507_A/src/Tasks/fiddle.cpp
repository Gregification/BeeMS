/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf?ts=1749927951492&ref_url=https%253A%252F%252Fwww.google.com%252F
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "Core/system.hpp"


void Task::fiddle_task(void *){
    System::uart_ui.nputs(ARRANDN("fiddle task start" NEWLINE));

    //--- TX --------------------------------------------------
    while(1) {
        DL_MCAN_TxBufElement txmsg = {
                .id     = BV(1),    // CAN id
                .rtr    = 0,        // 0: data frame, 1: remote frame
                .xtd    = 0,        // 0: 11b id, 1: 29b id
                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                .dlc    = 9,        // data byte count, see DL comments
                .brs    = 0,        // 0: no bit rate switching, 1: yes brs
                .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
                .efc    = 0,        // 0: dont store Tx events, 1: store
                .mm     = 0x1,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
                .data   = {1,2,3,4,5,6,7,8,9,10}
            };

        uint32_t bufferIndex = 0;
        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_BUF, bufferIndex, &txmsg);
        DL_MCAN_TXBufAddReq(CANFD0, bufferIndex);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //--- RX --------------------------------------------------



    System::uart_ui.nputs(ARRANDN("fiddle task end" NEWLINE));
    vTaskDelete(NULL);
}
