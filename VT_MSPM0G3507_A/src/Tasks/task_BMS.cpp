/*
 * task_DAQ.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include <Tasks/task_BMS.hpp>
#include "Core/system.hpp"
#include "Tasks/examples/example_BQ769x2_PROTOCOL_V.hpp"

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
    while(1)
    {
        BQ769x2_PROTOCOL_Test_V_Task(NULL);

        // Canned CAN TX
        DL_MCAN_TxBufElement txmsg = {
                .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
                .rtr    = 0,        // 0: data frame, 1: remote frame
                .xtd    = 1,        // 0: 11b id, 1: 29b id
                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                .dlc    = 3,        // data byte count, see DL comments
                .brs    = 0,        // 0: no bit rate switching, 1: yes brs
                .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
                .efc    = 0,        // 0: dont store Tx events, 1: store
                .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
            };
        txmsg.data[0] = 6;
        txmsg.data[1] = 7;
        txmsg.data[2] = 8;

        DL_MCAN_TxFIFOStatus tf;
        DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

        uint32_t bufferIndex = tf.putIdx;
        System::uart_ui.nputs(ARRANDN("TX from buffer "));
        System::uart_ui.putu32d(bufferIndex);
        System::uart_ui.nputs(ARRANDN("" NEWLINE));

        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
        DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);

        vTaskDelay(pdMS_TO_TICKS(400));
    };
    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}
