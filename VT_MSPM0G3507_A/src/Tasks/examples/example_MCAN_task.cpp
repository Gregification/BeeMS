/*
 * MCAN_test_task.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include <Tasks/examples/example_MCAN_task.hpp>
#include "Core/system.hpp"

void Task::MCAN_test_task(void *){
    System::uart_ui.nputs(ARRANDN("MCAN_test_task start" NEWLINE));

    //--- TX --------------------------------------------------
    do {
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
    } while(0);

    //--- RX --------------------------------------------------

    System::uart_ui.nputs(ARRANDN(CLIHIGHLIGHT "RX start" NEWLINE CLIRESET));
    do {
        static DL_MCAN_RxFIFOStatus rf;
        rf.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(CANFD0, &rf);

        if(rf.fillLvl != 0) { // if not empty
            DL_MCAN_RxBufElement e;
            DL_MCAN_readMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, 0, rf.num, &e);
            DL_MCAN_writeRxFIFOAck(CANFD0, rf.num, rf.getIdx);

            uint32_t id;
            if(e.xtd)   id = e.id;
            else        id = (e.id & 0x1FFC'0000) >> 18;

            System::uart_ui.nputs(ARRANDN("ID: "));
            System::uart_ui.putu32d(id);
            System::uart_ui.nputs(ARRANDN("" NEWLINE));
        }

        vTaskDelay(pdMS_TO_TICKS(400));
    } while(1);

    System::uart_ui.nputs(ARRANDN("MCAN_test_task end" NEWLINE));
    vTaskDelete(NULL);
}
