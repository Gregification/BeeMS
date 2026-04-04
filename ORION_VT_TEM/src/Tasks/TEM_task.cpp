/*
 * TEM.cpp
 *
 *  Created on: Mar 25, 2026
 *      Author: turtl
 */

#ifndef SRC_TASKS_TEM_CPP_
#define SRC_TASKS_TEM_CPP_

#include "TEM_task.hpp"
#include "Core/system.hpp"
#include "Core/Board.hpp"
#include "Core/std alternatives/string.hpp"


void sendCan29b(uint32_t canid, uint8_t datalen, void * data);

struct GeneralBroadcast {
    uint8_t temID;
    int8_t min_C;
    int8_t max_C;
    int8_t avg_C;
    struct __attribute__((packed)) {
        bool thermError     : 1;
        uint8_t numTherms   : 7;
    };
    uint8_t highestThermID;
    uint8_t lowestThermID;
    uint8_t chksum;
};
static_assert(sizeof(GeneralBroadcast) == 8);

void Task::TEM_task(void*) {
    using namespace BOARD;

    DL_GPIO_setAnalogInternalResistor(System::GPIO::PA18.iomux,
                      DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN);

//    while(1) {
//        vTaskDelay(pdMS_TO_TICKS(100));
//        UI::SWITCHES::cm.sample_blocking();
//        uint16_t data = UI::SWITCHES::cm.getResult();
//        sendCan29b(0xbeef, sizeof(data), &data);
//    }

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        auto & tb = Therm::TB[2];
        tb.update();

        uint16_t data = tb.degcC[6];
//        if(tb.error[6])
//            data = 0xff;
        sendCan29b(0xbeef, sizeof(data), &data);
    }

    while(1){
        vTaskDelay(pdMS_TO_TICKS(80));

        UI::SWITCHES::cm.sample_blocking();
        uint16_t val = UI::SWITCHES::cm.getResult();

        GeneralBroadcast gb;
        gb.numTherms = 23;
        gb.highestThermID = 0;
        gb.lowestThermID = 0;
        gb.max_C = -128;
        gb.min_C = 127;

        for(int i = 0; i < Therm::THERMB_N; i++) {
            auto & tb = Therm::TB[i];
            tb.update();

            for(int j = 0; j < sizeof(tb.degcC)/sizeof(tb.degcC[0]); j++) {
                uint8_t tid = Therm::getThermID(i, j);
                if(tid == 0xFF) // is not a cell therm
                    continue;
                if(tb.degcC[j] / 10 < gb.min_C) {
                    gb.min_C = tb.degcC[j] / 10;
                    gb.lowestThermID =  tid;
                }
                if(tb.degcC[j] / 10 > gb.max_C) {
                    gb.max_C = tb.degcC[j] / 10;
                    gb.highestThermID =  tid;
                }
                if(tb.error[j])
                    gb.numTherms |= 0x80;
            }
        }

        {
            gb.temID = id;

            gb.chksum = 0x39 + 8;
            for(int i = 0 ; i < sizeof(gb); i++)
                gb.chksum += ((uint8_t *)&gb)[i];
        }

        sendCan29b(0x1838'F380, sizeof(gb), &gb);
    }
}

void sendCan29b(uint32_t canid, uint8_t datalen, void * data) {
    if(datalen > 8)
        datalen = 8;
    DL_MCAN_TxBufElement txmsg = {
            .id     = 0,  // CAN id, 11b->[28:18], 29b->[] when using 11b can id
            .rtr    = 0,        // 0: data frame, 1: remote frame
            .xtd    = 1,        // 0: 11b id, 1: 29b id
            .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
            .dlc    = 3,        // data byte count, see DL comments
            .brs    = 0,        // 0: no bit rate switching, 1: yes brs
            .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
            .efc    = 0,        // 0: dont store Tx events, 1: store
            .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
        };

    txmsg.id = canid;
    ALT::memcpy(data, txmsg.data, datalen);
    txmsg.dlc = datalen;

    switch(BOARD::can_mode) {
        case BOARD::CAN_MODE::A1M_D4M:
        case BOARD::CAN_MODE::A500k_D2M:
            txmsg.brs = 1;
            break;
        default:
            break;
    };

    BOARD::can.takeResource(pdMS_TO_TICKS(10));

    DL_MCAN_TxFIFOStatus tf;
    DL_MCAN_getTxFIFOQueStatus(BOARD::can.reg, &tf);
    if(tf.fifoFull){
        BOARD::can.giveResource();
        return;
    }

    DL_MCAN_writeMsgRam(BOARD::can.reg, DL_MCAN_MEM_TYPE_BUF, tf.putIdx, &txmsg);
    DL_MCAN_TXBufAddReq(BOARD::can.reg, tf.putIdx);
    BOARD::can.giveResource();
}

#endif /* SRC_TASKS_TEM_CPP_ */
