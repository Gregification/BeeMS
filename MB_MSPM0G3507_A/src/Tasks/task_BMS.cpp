/*
 * task_BMS.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 *
 *  All bms logic is in here
 */

#include <Tasks/task_BMS.hpp>
#include "Core/system.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"
#include "Core/common.h"
#include "Core/BMS/BMSComms.hpp"
#include "Core/Networking/CAN.hpp"
#include "Core/Networking/ModbusRegisters.hpp"
#include "Core/MasterBoard.hpp"


void processCAN();
void process29bCANPacket(DL_MCAN_RxBufElement &, TickType_t);
void process11bCANPacket(DL_MCAN_RxBufElement &, TickType_t);
BMSCommon::Module * getModule(uint8_t id);

void Task::BMS_task(void *){
    System::UART::uart_ui.nputs(ARRANDN("BMS_task start" NEWLINE));

    vTaskDelay(pdMS_TO_TICKS(1500));

    while(true) {
        { // every X mS
            constexpr int32_t dt = 5;
            static TickType_t former = xTaskGetTickCount();
            TickType_t dt_mS = (xTaskGetTickCount() - former) * portTICK_PERIOD_MS;
            if(dt_mS < dt)
                vTaskDelay(pdMS_TO_TICKS(dt - dt_mS));
            former = xTaskGetTickCount();
        }

        processCAN();

        for(uint8_t i = 0; i < BMSCommon::Module::MAX_MODULES; i++) {
            BMSCommon::Module & m = MstrB::opVars.modules[i];

            if(!m.enabled) continue;

            // error if module reports error
            for(uint8_t j = 0; j < BMSCommon::Module::MAX_ICs; j++) {
                if(m.safetyStatus[j]) {
                    MstrB::opVars.masterSafteyStatus.module = i;
                    MstrB::opVars.masterSafteyStatus.pack_module_error = true;
                    MstrB::logSnapshot(false);
                    break;
                }
            }

            if(MstrB::opVars.masterSafteyStatus.Raw) break;

            // error on timeout
            if(m.lastSafteyStatusUpdate > pdMS_TO_TICKS(MstrB::opProfile.maxModuleUpdatePeriod_mS)) {
                MstrB::opVars.masterSafteyStatus.module = i;
                MstrB::opVars.masterSafteyStatus.pack_module_timeout = true;
                MstrB::logSnapshot(false);
                break;
            }
        }

        if(MstrB::opVars.masterSafteyStatus.Raw != 0) {
            MstrB::Indi::LED::fault.set();
            MstrB::IL::setEnable(false);
        } else {
            MstrB::Indi::LED::fault.clear();
            MstrB::IL::setEnable(true);
        }
    }

    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}

void processCAN() {
    auto & can = System::CANFD::canFD0;

    if(!can.takeResource(pdMS_TO_TICKS(10)))
        return;

    TickType_t timestamp = xTaskGetTickCount();
    DL_MCAN_RxFIFOStatus rf;
    rf.num = System::CANFD::OP_RXFIFO;
    DL_MCAN_getRxFIFOStatus(can.reg, &rf);

    while(rf.fillLvl > 0) {
        DL_MCAN_RxBufElement re;
        DL_MCAN_readMsgRam(can.reg, DL_MCAN_MEM_TYPE_FIFO, 0, rf.num, &re);
        DL_MCAN_writeRxFIFOAck(can.reg, rf.num, rf.getIdx);
        rf.fillLvl--;

        if(!BMSComms::isValidPacketID(re))
            continue;

        // seperate function to declutter
        if(re.xtd)
            process29bCANPacket(re, timestamp);
        else
            process11bCANPacket(re, timestamp);

        uint32_t id = System::CANFD::getID(re);

    } while(false);

    System::CANFD::canFD0.giveResource();
}

void process29bCANPacket(DL_MCAN_RxBufElement & rx, TickType_t timestamp) {
    using namespace BMSComms;
    using namespace Networking;

    if(!rx.xtd) return;

    CAN::J1939::ID id = {
         .raw = System::CANFD::getID(rx)
    };


    switch(id.pdu_format) {
        case J1939_PF_e::B : {
            System::UART::uart_ui.nputs(ARRANDN("process29bCANPacket B _114" NEWLINE));
        } break;

        case J1939_PF_e::SM : {
//            System::UART::uart_ui.nputs(ARRANDN("process29bCANPacket SM _118" NEWLINE));

            BMSCommon::Module * m = getModule(id.src_addr);
            if(!m) {
//                System::UART::uart_ui.nputs(ARRANDN("received SM from invalid module  :"));
//                System::UART::uart_ui.put32d(id.src_addr);
//                System::UART::uart_ui.nputs(ARRANDN(NEWLINE));
                break;
            }

//            System::UART::uart_ui.nputs(ARRANDN("received from valid module : " ));
//            System::UART::uart_ui.put32d(id.src_addr);
//            System::UART::uart_ui.nputs(ARRANDN(NEWLINE));

            switch(id.pdu_specific) {
                case PktSM_JS_e::CELLT : {
                    auto d = reinterpret_cast<BMSComms::SM_CELLT_t *>(rx.data);
                    if(System::CANFD::DLC2Len(&rx) < sizeof(*d)) break;

                    for(uint8_t i = 0; i < d->cellCount; i++){
                        uint8_t cell = d->base_cell + i;

                        if(cell > BMSCommon::Module::MAX_CELLS)
                            break;

                        m->cells_dDegC[cell] = d->dDegC[i];
                    }
                }break;

                case PktSM_JS_e::CELLV : {
                    auto d = reinterpret_cast<BMSComms::SM_CELLV_t *>(rx.data);
                    if(System::CANFD::DLC2Len(&rx) < sizeof(*d)) break;

                    for(uint8_t i = 0; i < d->cellCount; i++){
                        uint8_t cell = d->base_cell + i;

                        if(cell > BMSCommon::Module::MAX_CELLS)
                            break;

                        m->cells_mV[cell] = d->mV[i];
                    }
                }break;

                case PktSM_JS_e::STATUS1: {
                    auto d = reinterpret_cast<BMSComms::SM_STATUS1_t *>(rx.data);
                    if(System::CANFD::DLC2Len(&rx) < sizeof(*d)) break;

                    if(d->error)
                        MstrB::IL::setEnable(false);

                    m->state = d->state;
                    m->stack_cV = d->stack_cV;
                    for(uint8_t i = 0; i <  BMSCommon::Module::MAX_ICs; i++) {
                        m->safetyStatus[i] = d->ICSafetyStatus[i];
                        if(m->safetyStatus[i])
                            MstrB::IL::setEnable(false);
                    }

                    m->lastSafteyStatusUpdate = timestamp;
                }break;

                case PktSM_JS_e::STATUS2: {
//                    auto d = reinterpret_cast<BMSComms::SM_STATUS2_t *>(rx.data);
//                    if(System::CANFD::DLC2Len(&rx) < sizeof(*d)) break;
//
//                    m->temp.ambient = d->ambient;
//                    m->temp.board_avg = d->board.avg_dDegC;
//                    m->temp.board_min = d->board.min_dDegC;
//                    m->temp.board_max = d->board.max_dDegC;
                }break;

                default: break;
            };
        } break;

        case J1939_PF_e::MS : {
            System::UART::uart_ui.nputs(ARRANDN("process29bCANPacket MS _192" NEWLINE));
            break;
        }
        case J1939_PF_e::MOD : {
            System::UART::uart_ui.nputs(ARRANDN("process29bCANPacket MOD _196" NEWLINE));
            break; // should NOT receive anything here. otherwise means can filter is broken
        }
        default : break;
    }
}

void process11bCANPacket(DL_MCAN_RxBufElement & rx, TickType_t timestamp) {
    if(rx.xtd) return;
    uint16_t id = System::CANFD::getID(rx);
}

BMSCommon::Module * getModule(uint8_t id) {
    if(id == BMSCommon::Module::BAD_MODULE_ID)
        return NULL;

    for(uint8_t i = 0; i < sizeof(MstrB::opVars.modules)/sizeof(MstrB::opVars.modules[0]); i++) {
        BMSCommon::Module * m = &MstrB::opVars.modules[i];
        if(m->enabled && m->unitID == id)
            return m;
    }

    return NULL;
}
